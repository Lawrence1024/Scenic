"""Racing track representation extending the driving domain's road network.

This module extends the road network classes from :obj:`scenic.domains.driving.roads`
with racing-specific concepts like pit lanes, sectors, racing lines, and track direction.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple
import attr

from scenic.domains.driving.roads import (
    Network, Road, Lane, LaneSection, 
    ManeuverType, NetworkElement
)
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import Vector, OrientedVector
from scenic.core.distributions import RejectionException


@attr.s(auto_attribs=True, kw_only=True, eq=False)
class Sector:
    """A sector of a racing track for timing purposes.
    
    Racing tracks are typically divided into 2-3 sectors to measure
    lap times and performance through different parts of the circuit.
    
    Attributes:
        number: Sector number (1, 2, 3, etc.)
        startDistance: Distance along track where sector starts (in meters)
        endDistance: Distance along track where sector ends (in meters)
        region: The geographic region covered by this sector
        name: Optional name for the sector (e.g., "Corkscrew", "Hairpin")
    """
    number: int
    startDistance: float
    endDistance: float
    region: PolygonalRegion
    name: Optional[str] = None
    
    @property
    def length(self) -> float:
        """Length of the sector in meters."""
        return self.endDistance - self.startDistance


@attr.s(auto_attribs=True, kw_only=True, eq=False)
class PitLane:
    """A pit lane on a racing track.
    
    Pit lanes are special lanes where cars can stop for service (tire changes,
    refueling, repairs). They typically have:
    - Speed limits (enforced)
    - Entry and exit points
    - Pit boxes for each team
    - Separate from main racing lanes
    
    Attributes:
        lane: The underlying Lane object representing the pit lane
        speedLimit: Speed limit in m/s (typically 60-80 km/h = 16-22 m/s)
        entryPoint: Point where cars can enter the pit lane
        exitPoint: Point where cars rejoin the racing line
        pitBoxes: List of regions representing individual pit boxes
    """
    lane: Lane
    speedLimit: float = 22.0  # ~80 km/h default
    entryPoint: Optional[Vector] = None
    exitPoint: Optional[Vector] = None
    pitBoxes: List[PolygonalRegion] = attr.Factory(list)
    
    @property
    def region(self) -> PolygonalRegion:
        """The region covered by the pit lane."""
        return self.lane.polygon
    
    @property
    def centerline(self) -> PolylineRegion:
        """The centerline of the pit lane."""
        return self.lane.centerline
    
    def isPitBox(self, position: Vector) -> bool:
        """Check if a position is in any pit box."""
        for box in self.pitBoxes:
            if position in box:
                return True
        return False


@attr.s(auto_attribs=True, kw_only=True, eq=False)
class RacingLine:
    """The optimal racing line through a section of track.
    
    The racing line is the fastest path through a corner or section of track,
    typically using the full width of the track to maximize corner speed.
    
    Attributes:
        path: The polyline representing the racing line
        section: The section of track this racing line covers
        speedProfile: Optional list of (distance, recommended_speed) tuples
    """
    path: PolylineRegion
    section: str = "general"  # e.g., "turn_1", "main_straight", etc.
    speedProfile: List[Tuple[float, float]] = attr.Factory(list)
    
    def recommendedSpeedAt(self, distance: float) -> Optional[float]:
        """Get the recommended speed at a given distance along the racing line."""
        if not self.speedProfile:
            return None
        
        # Find the closest point in the speed profile
        closest_dist = min(self.speedProfile, key=lambda x: abs(x[0] - distance))
        return closest_dist[1]


class RacingTrack:
    """A racing track (closed-loop circuit).
    
    Extends the Network concept from the driving domain with racing-specific features:
    - Enforced track direction (one-way)
    - Pit lane identification
    - Sector divisions
    - Racing line
    - Start/finish line
    - Starting grid positions
    
    Attributes:
        network: The underlying road Network
        direction: 'clockwise' or 'counterclockwise'
        pitLane: The pit lane, if any
        sectors: List of track sectors
        racingLine: The optimal racing line
        startFinishLine: Position of the start/finish line
        trackLength: Total length of the racing circuit in meters
        startingGrid: List of positions for race starts
    """
    
    def __init__(
        self, 
        network: Network,
        direction: str = 'clockwise',
        trackLength: Optional[float] = None,
        pitLaneRoadId: Optional[str] = None,
        pitLaneRoadName: Optional[str] = None,
        mainLineRoadId: Optional[str] = None
    ):
        """Initialize a racing track from a road network.
        
        Args:
            network: Road network (typically from OpenDRIVE file)
            direction: 'clockwise' or 'counterclockwise'
            trackLength: Total track length in meters (auto-detected if None)
            pitLaneRoadId: OpenDRIVE road ID for pit lane (optional)
            pitLaneRoadName: Pattern to match pit lane name (e.g., "Pit Lane", "pit")
            mainLineRoadId: OpenDRIVE road ID for main racing line (optional)
        """
        self.network = network
        self.direction = direction
        self.trackLength = trackLength or self._calculateTrackLength()
        
        # User-specified road identifiers
        self.pitLaneRoadId = pitLaneRoadId
        self.pitLaneRoadName = pitLaneRoadName or "pit"  # Default pattern
        self.mainLineRoadId = mainLineRoadId
        
        # Racing-specific features (to be populated)
        self.pitLane: Optional[PitLane] = None
        self.sectors: List[Sector] = []
        self.racingLine: Optional[RacingLine] = None
        self.startFinishLine: Optional[Vector] = None
        self.startingGrid: List[Vector] = []
        
        # Track segments: Two mutually exclusive regions
        # mainRacingRoad = union of all non-pit roads (main line + parallel roads)
        # pitLaneRoad = pit lane only
        self.mainRacingRoad: Optional[Union[Road, Region]] = None
        self.pitLaneRoad: Optional[Road] = None
        self._mainRacingRoads: List[Road] = []  # Individual roads that make up main racing circuit
        
        # Analyze the network to identify racing features
        self._identifyRacingFeatures()
    
    def _calculateTrackLength(self) -> float:
        """Calculate total track length by following the main racing line."""
        # Find the longest continuous path (main track)
        max_length = 0.0
        for road in self.network.roads:
            road_length = sum(lane.centerline.length for lane in road.lanes)
            max_length = max(max_length, road_length)
        return max_length
    
    def _identifyRacingFeatures(self):
        """Identify pit lanes, sectors, and other racing features from the network."""
        
        print(f"\n[RacingTrack] Identifying track features...")
        print(f"  Total roads in network: {len(self.network.roads)}")
        
        # Step 1: Identify road segments (main line, pit lane, parallel roads)
        self._identifyRoadSegments()
        
        # Step 2: Create pit lane if identified
        if self.pitLaneRoad is not None:
            # Use the first lane of the pit lane road
            if self.pitLaneRoad.lanes:
                self.pitLane = PitLane(lane=self.pitLaneRoad.lanes[0])
                print(f"  [OK] Pit lane identified: {self.pitLaneRoad}")
        else:
            print(f"  [INFO] No pit lane identified (will use all roads as racing line)")
        
        # Step 3: Auto-generate sectors if not specified
        # Common practice: divide track into 3 equal sectors
        if not self.sectors and self.trackLength > 0:
            sector_length = self.trackLength / 3.0
            for i in range(3):
                self.sectors.append(Sector(
                    number=i + 1,
                    startDistance=i * sector_length,
                    endDistance=(i + 1) * sector_length,
                    region=self.network.drivableRegion,  # Simplified
                    name=f"Sector {i + 1}"
                ))
            print(f"  [OK] Generated {len(self.sectors)} sectors")
    
    def _identifyRoadSegments(self):
        """Identify pit lane and main racing roads from the network.
        
        Creates two mutually exclusive segments:
        - pitLaneRoad: The pit lane
        - mainRacingRoad: Union of all non-pit roads (main line + parallel roads)
        
        Uses multiple strategies:
        1. User-specified road IDs (pitLaneRoadId)
        2. Road name patterns (e.g., "Pit Lane")
        3. Remaining roads become part of mainRacingRoad
        """
        
        print(f"\n  [Track Segments] Analyzing road structure...")
        
        # Analyze each road
        road_info = []
        for road in self.network.roads:
            # Get road properties
            road_id = getattr(road, 'id', None)
            road_name = getattr(road, 'name', str(road))
            
            # Use actual road length (not sum of all lanes)
            if hasattr(road, 'length'):
                road_length = road.length
            elif road.lanes:
                road_length = road.lanes[0].centerline.length
            else:
                road_length = 0
            
            road_info.append({
                'road': road,
                'id': road_id,
                'name': road_name,
                'length': road_length
            })
            
            print(f"    Road: {road_name[:50]}")
            print(f"      ID: {road_id}, Length: {road_length:.1f}m, Lanes: {len(road.lanes)}")
        
        # Sort by length (longest first)
        road_info.sort(key=lambda x: x['length'], reverse=True)
        
        # Step 1: Identify pit lane
        # Strategy 1a: User-specified pit lane ID
        if self.pitLaneRoadId:
            for info in road_info:
                if str(info['id']) == str(self.pitLaneRoadId):
                    self.pitLaneRoad = info['road']
                    print(f"\n  [OK] Pit lane identified by ID: {info['name']} ({info['length']:.1f}m)")
                    break
        
        # Strategy 1b: Name pattern matching for pit lane
        if self.pitLaneRoad is None and self.pitLaneRoadName:
            pattern = self.pitLaneRoadName.lower()
            for info in road_info:
                if pattern in info['name'].lower():
                    self.pitLaneRoad = info['road']
                    print(f"\n  [OK] Pit lane identified by name: {info['name']} ({info['length']:.1f}m)")
                    break
        
        # Step 2: All non-pit roads become mainRacingRoad
        for info in road_info:
            road = info['road']
            if road != self.pitLaneRoad:
                self._mainRacingRoads.append(road)
                print(f"  [INFO] Main racing road: {info['name']} ({info['length']:.1f}m)")
        
        # Step 3: Create union region for mainRacingRoad
        if self._mainRacingRoads:
            from scenic.core.regions import UnionRegion
            # Combine all main racing roads into one region
            road_regions = []
            for road in self._mainRacingRoads:
                # Each road is a region
                road_regions.append(road)
            
            if len(road_regions) == 1:
                self.mainRacingRoad = road_regions[0]
            else:
                # Create union of all racing roads
                self.mainRacingRoad = UnionRegion(*road_regions)
            
            print(f"\n  [OK] Main racing road: Union of {len(self._mainRacingRoads)} road(s)")
        
        # Summary
        if self.pitLaneRoad and self.mainRacingRoad:
            print(f"  [OK] Two mutually exclusive segments defined:")
            print(f"       - Pit lane: 1 road")
            print(f"       - Main racing: {len(self._mainRacingRoads)} road(s)")
    
    def _isPitLane(self, lane: Lane) -> bool:
        """Determine if a lane belongs to the pit lane road."""
        # Check if this lane's parent road is the pit lane road
        if self.pitLaneRoad and hasattr(lane, 'road'):
            return lane.road == self.pitLaneRoad
        return False
    
    def generateStartingGrid(
        self, 
        numPositions: int = 20,
        spacing: float = 8.0,
        offset: float = 0.0
    ) -> List:
        """Generate starting grid positions.
        
        Args:
            numPositions: Number of grid positions to generate
            spacing: Distance between grid positions (meters)
            offset: Distance from start/finish line (meters)
            
        Returns:
            List of lane regions for the starting grid (Scenic will sample positions from these)
        """
        if self.startFinishLine is None:
            # Use the beginning of the longest road as start/finish
            longest_road = max(
                self.network.roads,
                key=lambda r: sum(lane.centerline.length for lane in r.lanes)
            )
            self.startFinishLine = longest_road.lanes[0].centerline.start
        
        # Generate grid positions along the main straight
        # For now, return the main lane region and let Scenic sample positions from it
        # This ensures cars are always placed within valid lane boundaries
        positions = []
        main_lane = self._getMainRacingLane()
        
        if main_lane:
            # Return the lane region for each grid position
            # Scenic will sample actual positions from this region
            for i in range(numPositions):
                positions.append(main_lane)
        
        self.startingGrid = positions
        return positions
    
    def _getMainRacingLane(self) -> Optional[Lane]:
        """Get the main racing lane (typically the widest or most central lane)."""
        # Find the longest lane that's not a pit lane
        main_lane = None
        max_length = 0.0
        
        for road in self.network.roads:
            for lane in road.lanes:
                if not self._isPitLane(lane):
                    if lane.centerline.length > max_length:
                        max_length = lane.centerline.length
                        main_lane = lane
        
        return main_lane
    
    def distanceAlongTrack(self, position: Vector) -> Optional[float]:
        """Calculate distance along the track from start/finish line.
        
        Args:
            position: Position on track
            
        Returns:
            Distance in meters from start/finish line, or None if not on track
        """
        # Project position onto main racing lane
        main_lane = self._getMainRacingLane()
        if main_lane is None:
            return None
        
        # Find closest point on centerline
        centerline = main_lane.centerline
        # This would need the actual implementation of finding distance along a polyline
        # For now, return a placeholder
        return 0.0  # TODO: Implement proper distance calculation
    
    def getSectorAt(self, position: Vector) -> Optional[Sector]:
        """Get the sector containing the given position."""
        distance = self.distanceAlongTrack(position)
        if distance is None:
            return None
        
        for sector in self.sectors:
            if sector.startDistance <= distance < sector.endDistance:
                return sector
        
        return None
    
    def isOnPitLane(self, position: Vector) -> bool:
        """Check if a position is on the pit lane."""
        if self.pitLane is None:
            return False
        return position in self.pitLane.region
    
    def enforceTrackDirection(self, heading: float, position: Vector) -> bool:
        """Check if a heading at a position matches the track direction.
        
        Args:
            heading: Heading in radians
            position: Position on track
            
        Returns:
            True if heading matches track direction, False otherwise
        """
        # Get expected heading from road direction
        expected_heading = self.network.roadDirection[position]
        
        # Compare headings (allow some tolerance)
        heading_diff = abs(heading - expected_heading)
        if heading_diff > math.pi:
            heading_diff = 2 * math.pi - heading_diff
        
        # If going the wrong way, reject
        tolerance = math.pi / 4  # 45 degrees
        if heading_diff > tolerance:
            if self.direction == 'clockwise':
                raise RejectionException(
                    "Vehicle heading opposite to clockwise track direction"
                )
            else:
                raise RejectionException(
                    "Vehicle heading opposite to counterclockwise track direction"
                )
        
        return True


def createRacingTrack(
    mapFile: str,
    direction: str = 'counterclockwise',
    pitLaneRoadId: Optional[str] = None,
    pitLaneRoadName: Optional[str] = None,
    mainLineRoadId: Optional[str] = None,
    **map_options
) -> RacingTrack:
    """Create a RacingTrack from a map file.
    
    Args:
        mapFile: Path to OpenDRIVE (.xodr) file
        direction: Track direction ('clockwise' or 'counterclockwise')
        pitLaneRoadId: OpenDRIVE road ID for pit lane (optional, e.g., "1545702203")
        pitLaneRoadName: Pattern to match pit lane name (optional, e.g., "Pit Lane", "pit")
        mainLineRoadId: OpenDRIVE road ID for main racing line (optional, e.g., "2117817291")
        **map_options: Additional options passed to Network.fromFile()
        
    Returns:
        RacingTrack object with identified segments
        
    Example::
    
        # Auto-detect by name pattern (default)
        track = createRacingTrack(
            'laguna_seca.xodr',
            direction='counterclockwise'
        )
        
        # Explicit pit lane specification
        track = createRacingTrack(
            'laguna_seca.xodr',
            direction='counterclockwise',
            pitLaneRoadId='1545702203',
            mainLineRoadId='2117817291'
        )
        
        track.generateStartingGrid(numPositions=20)
    """
    network = Network.fromFile(mapFile, **map_options)
    track = RacingTrack(
        network, 
        direction=direction,
        pitLaneRoadId=pitLaneRoadId,
        pitLaneRoadName=pitLaneRoadName,
        mainLineRoadId=mainLineRoadId
    )
    return track

