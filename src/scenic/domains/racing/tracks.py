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
from scenic.core.vectors import Vector
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
        trackLength: Optional[float] = None
    ):
        """Initialize a racing track from a road network.
        
        Args:
            network: Road network (typically from OpenDRIVE file)
            direction: 'clockwise' or 'counterclockwise'
            trackLength: Total track length in meters (auto-detected if None)
        """
        self.network = network
        self.direction = direction
        self.trackLength = trackLength or self._calculateTrackLength()
        
        # Racing-specific features (to be populated)
        self.pitLane: Optional[PitLane] = None
        self.sectors: List[Sector] = []
        self.racingLine: Optional[RacingLine] = None
        self.startFinishLine: Optional[Vector] = None
        self.startingGrid: List[Vector] = []
        
        # Analyze the network to identify racing features
        self._identifyRacingFeatures()
    
    def _calculateTrackLength(self) -> float:
        """Calculate total track length by following the main racing line."""
        # Find the longest continuous path (main track)
        max_length = 0.0
        for road in self.network.roads.values():
            road_length = sum(lane.centerline.length for lane in road.lanes)
            max_length = max(max_length, road_length)
        return max_length
    
    def _identifyRacingFeatures(self):
        """Identify pit lanes, sectors, and other racing features from the network."""
        # Identify pit lane (look for lanes with "pit" in type or specific lane types)
        for road in self.network.roads.values():
            for lane in road.lanes:
                # Check if this is a pit lane
                # In OpenDRIVE, pit lanes often have specific types
                if self._isPitLane(lane):
                    if self.pitLane is None:
                        self.pitLane = PitLane(lane=lane)
                    break
        
        # Auto-generate sectors if not specified
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
    
    def _isPitLane(self, lane: Lane) -> bool:
        """Determine if a lane is a pit lane based on its properties."""
        # Check lane type (implementation depends on OpenDRIVE format)
        # Common indicators:
        # - Lane has "pit" in name or type
        # - Lane is parallel to main track but separated
        # - Lane has lower speed limit
        
        # For now, use a simple heuristic:
        # Pit lanes are typically narrower and off to the side
        return False  # TODO: Implement proper pit lane detection
    
    def generateStartingGrid(
        self, 
        numPositions: int = 20,
        spacing: float = 8.0,
        offset: float = 0.0
    ) -> List[Vector]:
        """Generate starting grid positions.
        
        Args:
            numPositions: Number of grid positions to generate
            spacing: Distance between grid positions (meters)
            offset: Distance from start/finish line (meters)
            
        Returns:
            List of positions for the starting grid
        """
        if self.startFinishLine is None:
            # Use the beginning of the longest road as start/finish
            longest_road = max(
                self.network.roads.values(),
                key=lambda r: sum(lane.centerline.length for lane in r.lanes)
            )
            self.startFinishLine = longest_road.lanes[0].centerline.start
        
        # Generate grid positions along the main straight
        # Typically 2 cars per row, staggered
        positions = []
        main_lane = self._getMainRacingLane()
        
        if main_lane:
            centerline = main_lane.centerline
            current_distance = offset
            row = 0
            
            for i in range(numPositions):
                # Alternate left and right (staggered grid)
                lateral_offset = 1.5 if i % 2 == 0 else -1.5
                
                # Get point along centerline
                point = centerline.pointAlongBy(current_distance)
                
                # Offset laterally for staggered grid
                direction = centerline.heading[point]
                left_direction = direction + math.pi / 2
                grid_pos = point + Vector(
                    lateral_offset * math.cos(left_direction),
                    lateral_offset * math.sin(left_direction)
                )
                
                positions.append(grid_pos)
                
                # Move to next row every 2 cars
                if i % 2 == 1:
                    current_distance += spacing
        
        self.startingGrid = positions
        return positions
    
    def _getMainRacingLane(self) -> Optional[Lane]:
        """Get the main racing lane (typically the widest or most central lane)."""
        # Find the longest lane that's not a pit lane
        main_lane = None
        max_length = 0.0
        
        for road in self.network.roads.values():
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
    **map_options
) -> RacingTrack:
    """Create a RacingTrack from a map file.
    
    Args:
        mapFile: Path to OpenDRIVE (.xodr) file
        direction: Track direction ('clockwise' or 'counterclockwise')
        **map_options: Additional options passed to Network.fromFile()
        
    Returns:
        RacingTrack object
        
    Example::
    
        track = createRacingTrack(
            'laguna_seca.xodr',
            direction='counterclockwise'
        )
        track.generateStartingGrid(numPositions=20)
    """
    network = Network.fromFile(mapFile, **map_options)
    track = RacingTrack(network, direction=direction)
    return track

