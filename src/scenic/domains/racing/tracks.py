"""Racing track representation extending the driving domain's road network.

This module extends the road network classes from :obj:`scenic.domains.driving.roads`
with racing-specific concepts like pit lanes, sectors, racing lines, and track direction.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple, Union
import attr

from scenic.domains.driving.roads import (
    Network, Road, Lane, LaneSection,
    Intersection,
    ManeuverType, NetworkElement
)
from scenic.core.regions import PolygonalRegion, PolylineRegion
from scenic.core.vectors import Vector, OrientedVector
from scenic.core.distributions import RejectionException


def _road_centerline(road: Road):
    """First lane centerline of a road, or None."""
    if not getattr(road, "lanes", None) or len(road.lanes) == 0:
        return None
    return getattr(road.lanes[0], "centerline", None)


def _road_endpoint_and_heading(road: Road, at_start: bool) -> Optional[Tuple[float, float, float]]:
    """Return (x, y, heading_rad) at the start or end of the road's centerline. None if not available."""
    cl = _road_centerline(road)
    if cl is None or len(cl) < 2:
        return None
    if at_start:
        p0, p1 = cl[0], cl[1]
    else:
        p0, p1 = cl[-2], cl[-1]
    dx = float(p1.x - p0.x)
    dy = float(p1.y - p0.y)
    heading = math.atan2(dy, dx)
    pt = cl[0] if at_start else cl[-1]
    return (float(pt.x), float(pt.y), heading)


def _angle_diff(a_rad: float, b_rad: float) -> float:
    """Smallest angle difference in [-pi, pi] (absolute value)."""
    d = a_rad - b_rad
    while d > math.pi:
        d -= 2 * math.pi
    while d < -math.pi:
        d += 2 * math.pi
    return abs(d)


# Helper wrappers: use the module-level geometry helpers (PolylineRegion points with .x/.y).
def _road_endpoint_and_heading_local(road: Road, at_start: bool):
    return _road_endpoint_and_heading(road, at_start)


def _angle_diff_local(a: float, b: float) -> float:
    return _angle_diff(a, b)


def _smoothness_conn_to_mains(
    conn_road: Road,
    main_roads: List[Road],
) -> Optional[float]:
    """
    Smoothness score for a connecting road that links two main roads: sum of angle differences
    at the two connection points. Lower = smoother. Returns None if geometry is missing.
    """
    cl = _road_centerline(conn_road)
    if cl is None or len(cl) < 2:
        return None
    conn_start_pt = (float(cl[0].x), float(cl[0].y))
    conn_end_pt = (float(cl[-1].x), float(cl[-1].y))
    conn_start_heading = math.atan2(float(cl[1].y - cl[0].y), float(cl[1].x - cl[0].x))
    conn_end_heading = math.atan2(float(cl[-1].y - cl[-2].y), float(cl[-1].x - cl[-2].x))

    def dist_pt(a, b):
        return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2

    best = None
    for ma in main_roads:
        for mb in main_roads:
            if ma is mb:
                continue
            ha_start = _road_endpoint_and_heading(ma, at_start=True)
            ha_end = _road_endpoint_and_heading(ma, at_start=False)
            hb_start = _road_endpoint_and_heading(mb, at_start=True)
            hb_end = _road_endpoint_and_heading(mb, at_start=False)
            if ha_start is None or ha_end is None or hb_start is None or hb_end is None:
                continue
            d_cs_ma_end = dist_pt(conn_start_pt, (ha_end[0], ha_end[1]))
            d_cs_ma_start = dist_pt(conn_start_pt, (ha_start[0], ha_start[1]))
            d_ce_mb_end = dist_pt(conn_end_pt, (hb_end[0], hb_end[1]))
            d_ce_mb_start = dist_pt(conn_end_pt, (hb_start[0], hb_start[1]))
            # Conn start meets one main's endpoint; conn end meets the other main's endpoint
            if d_cs_ma_end <= d_cs_ma_start and d_ce_mb_start <= d_ce_mb_end:
                score = _angle_diff(ha_end[2], conn_start_heading) + _angle_diff(conn_end_heading, hb_start[2])
            elif d_cs_ma_start <= d_cs_ma_end and d_ce_mb_end <= d_ce_mb_start:
                score = _angle_diff(ha_start[2], conn_start_heading) + _angle_diff(conn_end_heading, hb_end[2])
            elif d_cs_ma_end <= d_cs_ma_start and d_ce_mb_end <= d_ce_mb_start:
                score = _angle_diff(ha_end[2], conn_start_heading) + _angle_diff(conn_end_heading, hb_end[2])
            elif d_cs_ma_start <= d_cs_ma_end and d_ce_mb_start <= d_ce_mb_end:
                score = _angle_diff(ha_start[2], conn_start_heading) + _angle_diff(conn_end_heading, hb_start[2])
            else:
                continue
            if best is None or score < best:
                best = score
    return best


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
        mainLineRoadId: Optional[str] = None,
        main_loop_connecting_road_ids: Optional[Tuple[Union[int, str], ...]] = None,
        pit_connecting_road_ids: Optional[Tuple[Union[int, str], ...]] = None,
    ):
        """Initialize a racing track from a road network.

        Args:
            network: Road network (typically from OpenDRIVE file)
            direction: 'clockwise' or 'counterclockwise'
            trackLength: Total track length in meters (auto-detected if None)
            pitLaneRoadId: OpenDRIVE road ID for pit lane (optional)
            pitLaneRoadName: Pattern to match pit lane name (e.g., "Pit Lane", "pit")
            mainLineRoadId: OpenDRIVE road ID for main racing line (optional)
            main_loop_connecting_road_ids: OpenDRIVE road IDs of junction connecting roads
                for the outer loop (e.g. (24, 34)). If None, at each junction the link that
                smoothly connects the two main roads (smallest angle change) is used.
            pit_connecting_road_ids: OpenDRIVE road IDs of junction connecting roads to the
                pit (e.g. (25, 30)). If None, pit links are auto-detected (conn that has
                the pit road at the junction).
        """
        self.network = network
        self.direction = direction
        self.trackLength = trackLength or self._calculateTrackLength()

        # User-specified road identifiers
        self.pitLaneRoadId = pitLaneRoadId
        self.pitLaneRoadName = pitLaneRoadName or "pit"  # Default pattern
        self.mainLineRoadId = mainLineRoadId
        self.main_loop_connecting_road_ids = main_loop_connecting_road_ids
        self.pit_connecting_road_ids = pit_connecting_road_ids or ()

        # Racing-specific features (to be populated)
        self.pitLane: Optional[PitLane] = None
        self.sectors: List[Sector] = []
        self.racingLine: Optional[RacingLine] = None
        self.startFinishLine: Optional[Vector] = None
        self.startingGrid: List[Vector] = []

        # Track segments: Two mutually exclusive regions
        # mainRacingRoad = union of all main loop roads (main line + chosen junction links)
        # pitLaneRoad = primary pit road; _pitRoads = all pit roads (primary + junction links)
        self.mainRacingRoad: Optional[Union[Road, Region]] = None
        self.pitLaneRoad: Optional[Road] = None
        self._mainRacingRoads: List[Road] = []
        self._pitRoads: List[Road] = []
        
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
        
        # Step 2: Create pit lane if identified (_pitRoads may include junction links)
        if self.pitLaneRoad is not None:
            if self.pitLaneRoad.lanes:
                self.pitLane = PitLane(lane=self.pitLaneRoad.lanes[0])
                print(f"  [OK] Pit lane identified: {self.pitLaneRoad} ({len(self._pitRoads)} road(s) total)")
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
        
        # Sort roads by length (descending)
        road_info.sort(key=lambda x: x['length'], reverse=True)
        
        # Print road info for debugging
        print(f"  [INFO] Found {len(road_info)} roads in network:")
        for i, info in enumerate(road_info[:10]):  # Show top 10 by length
            print(f"    {i+1}. Road {info['id']}: {info['name']} ({info['length']:.1f}m)")
        if len(road_info) > 10:
            print(f"    ... and {len(road_info)-10} more roads")
        
        # 1. Identify pit lane road
        self.pitLaneRoad = None
        
        # Strategy 1: Use specified pit lane road ID
        if getattr(self, 'pitLaneRoadId', None) is not None:
            print(f"  [INFO] Looking for specified pit lane road ID: {self.pitLaneRoadId}")
            for info in road_info:
                if info['id'] == self.pitLaneRoadId:
                    self.pitLaneRoad = info['road']
                    print(f"  [FOUND] Pit lane road by ID: {info['name']} ({info['length']:.1f}m)")
                    break
        
        # Strategy 2: Look for pit lane by name pattern
        if self.pitLaneRoad is None:
            pit_patterns = ['pit', 'box', 'pits', 'pitlane', 'pit lane']
            for info in road_info:
                name_lower = str(info['name']).lower()
                if any(pattern in name_lower for pattern in pit_patterns):
                    self.pitLaneRoad = info['road']
                    print(f"  [FOUND] Pit lane road by name: {info['name']} ({info['length']:.1f}m)")
                    break
        
        # Strategy 3: If still not found, look for a shorter parallel road (pit lane is often shorter than main straight)
        if self.pitLaneRoad is None and len(road_info) >= 2:
            # Sometimes pit lane is the 2nd/3rd longest road, check top few for reasonable pit lane characteristics
            for info in road_info[1:5]:  # Check roads 2-5 by length
                # Pit lane is often 30-80% length of main road
                if 0.3 * road_info[0]['length'] < info['length'] < 0.8 * road_info[0]['length']:
                    # Check if it has similar start/end points to main road (parallel)
                    self.pitLaneRoad = info['road']
                    print(f"  [GUESS] Pit lane road by length: {info['name']} ({info['length']:.1f}m)")
                    break
        
        if self.pitLaneRoad is None:
            print("  [WARNING] Could not identify pit lane road!")
        else:
            print(f"  [SUCCESS] Identified pit lane road: {getattr(self.pitLaneRoad, 'name', str(self.pitLaneRoad))}")
        
        # 2. Identify pit roads and main racing roads
        self._pitRoads = []
        self._mainRacingRoads = []
        
        # Add pit lane road to pit roads
        if self.pitLaneRoad is not None:
            self._pitRoads.append(self.pitLaneRoad)
        
        # Helper functions for road connectivity analysis
        def _road_length(road: Road) -> float:
            if hasattr(road, 'length'):
                return float(road.length)
            if road.lanes and road.lanes[0].centerline is not None:
                return float(road.lanes[0].centerline.length)
            return 0.0
        
        def _junction_center(junc):
            """(x, y) of junction polygon centroid, or None."""
            try:
                poly = getattr(junc, "polygon", None)
                if poly is not None:
                    c = poly.centroid
                    return (float(c.x), float(c.y))
            except Exception:
                pass
            return None

        def _conn_belongs_to_junction(conn: Road, junc: Intersection, intersections: List[Intersection]) -> bool:
            """True if `conn` is a connecting road belonging to `junc` (both endpoints at junction, this junction closest)."""
            jroads = set(getattr(junc, "roads", ()))
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            # Topology: connector belongs to a junction if both endpoints (pred, succ) are in that junction's roads
            pred_at = pred == junc or (pred in jroads)
            succ_at = succ == junc or (succ in jroads)
            if not (pred_at and succ_at):
                return False

            # When multiple junctions share the same roads, assign conn to the junction whose center is closest to conn
            conn_pt = _road_endpoint_and_heading_local(conn, at_start=True)
            conn_pt2 = _road_endpoint_and_heading_local(conn, at_start=False)
            if conn_pt is not None and conn_pt2 is not None:
                cx = (conn_pt[0] + conn_pt2[0]) * 0.5
                cy = (conn_pt[1] + conn_pt2[1]) * 0.5
            else:
                return True
            junc_pt = _junction_center(junc)
            if junc_pt is None:
                return True
            best_dist_sq = (cx - junc_pt[0]) ** 2 + (cy - junc_pt[1]) ** 2
            junc_idx = intersections.index(junc) if junc in intersections else -1
            for other in intersections:
                if other is junc:
                    continue
                oroads = set(getattr(other, "roads", ()))
                opred = pred == other or (pred in oroads)
                osucc = succ == other or (succ in oroads)
                if not (opred and osucc):
                    continue
                other_pt = _junction_center(other)
                if other_pt is None:
                    continue
                d_sq = (cx - other_pt[0]) ** 2 + (cy - other_pt[1]) ** 2
                other_idx = intersections.index(other)
                if d_sq < best_dist_sq or (d_sq == best_dist_sq and other_idx < junc_idx):
                    return False
            return True

        def _conn_connects_to_road(conn: Road, road: Road) -> bool:
            """
            Topology-only connection check (NO geometry fallback).

            Geometry-based fallback is what causes your “wrong 2 green junction segments”:
            dense junctions have multiple connectors passing close to endpoints.
            """
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            return (pred == road) or (succ == road)

        def _conn_other_end_road(conn: Road, road: Road):
            """If `conn` connects to `road`, return the object on the other end; else None."""
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            if pred == road:
                return succ
            if succ == road:
                return pred
            return None

        def _conn_connects_two_roads_in_set(conn: Road, road_set: set) -> bool:
            """True iff `conn` connects two roads in `road_set` by topology."""
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            return (pred in road_set) and (succ in road_set)

        # --- NEW: smoothness scoring for PIT connectors (pit↔conn↔main) ---

        def _heading_of_road_at_connection(road: Road, conn: Road) -> Optional[float]:
            """Heading of `road` at the endpoint closest to either end of `conn`."""
            rs = _road_endpoint_and_heading_local(road, at_start=True)
            re_ = _road_endpoint_and_heading_local(road, at_start=False)
            cs = _road_endpoint_and_heading_local(conn, at_start=True)
            ce = _road_endpoint_and_heading_local(conn, at_start=False)
            if rs is None or re_ is None or cs is None or ce is None:
                return None

            def dist_sq(ax, ay, bx, by):
                dx = ax - bx
                dy = ay - by
                return dx * dx + dy * dy

            d_rs = min(dist_sq(rs[0], rs[1], cs[0], cs[1]), dist_sq(rs[0], rs[1], ce[0], ce[1]))
            d_re = min(dist_sq(re_[0], re_[1], cs[0], cs[1]), dist_sq(re_[0], re_[1], ce[0], ce[1]))
            return rs[2] if d_rs <= d_re else re_[2]

        def _heading_of_conn_toward_road(conn: Road, road: Road) -> Optional[float]:
            """Heading of `conn` at the end which connects to `road` (topology)."""
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            cs = _road_endpoint_and_heading_local(conn, at_start=True)
            ce = _road_endpoint_and_heading_local(conn, at_start=False)
            if cs is None or ce is None:
                return None
            # Convention: start ↔ predecessor, end ↔ successor
            if pred == road:
                return cs[2]
            if succ == road:
                return ce[2]
            return None

        def _smoothness_between_roads_via_conn(a: Road, conn: Road, b: Road) -> Optional[float]:
            """Sum of angle diffs at a↔conn and conn↔b. Lower = smoother."""
            ha = _heading_of_road_at_connection(a, conn)
            hb = _heading_of_road_at_connection(b, conn)
            hca = _heading_of_conn_toward_road(conn, a)
            hcb = _heading_of_conn_toward_road(conn, b)
            if ha is None or hb is None or hca is None or hcb is None:
                return None
            return _angle_diff_local(ha, hca) + _angle_diff_local(hcb, hb)

        # Find intersections/junctions in the network
        intersections = list(getattr(self.network, "intersections", ()))
        if not intersections:
            intersections = [el for el in self.network.elements.values() if isinstance(el, Intersection)]
        connecting_roads = getattr(self.network, "connectingRoads", [])
        
        print(f"  [INFO] Found {len(intersections)} intersections and {len(connecting_roads)} connecting roads")

        # Start with all non-pit-lane roads as main racing roads
        for info in road_info:
            road = info['road']
            if road is not self.pitLaneRoad:
                self._mainRacingRoads.append(road)

        def _pit_conn_endpoint_fit_to_main(conn: Road, pit_road: Road, main_road: Road) -> float:
            """Distance from the connector's main-road end to the main road's nearest endpoint. Lower = pit link meets main at the same place as the main-loop geometry at that junction."""
            pred = getattr(conn, "_predecessor", None)
            succ = getattr(conn, "_successor", None)
            conn_at_main = None
            if pred == main_road:
                conn_at_main = _road_endpoint_and_heading_local(conn, at_start=True)
            elif succ == main_road:
                conn_at_main = _road_endpoint_and_heading_local(conn, at_start=False)
            if conn_at_main is None:
                return float("inf")
            c_pt = (conn_at_main[0], conn_at_main[1])
            main_start = _road_endpoint_and_heading_local(main_road, at_start=True)
            main_end = _road_endpoint_and_heading_local(main_road, at_start=False)
            def dist(a, b):
                return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
            d_start = dist(c_pt, (main_start[0], main_start[1])) if main_start else float("inf")
            d_end = dist(c_pt, (main_end[0], main_end[1])) if main_end else float("inf")
            return min(d_start, d_end)

        # 2a. Identify pit junction links (connectors that attach pit lane to main)
        if self.pitLaneRoad is not None and intersections and connecting_roads:
            main_racing_set_pre = set(self._mainRacingRoads)

            for junc in intersections:
                pit_conns_here = []
                for c in connecting_roads:
                    if c in self._pitRoads:
                        continue
                    if not _conn_belongs_to_junction(c, junc, intersections):
                        continue
                    if not _conn_connects_to_road(c, self.pitLaneRoad):
                        continue

                    # require the other end to attach to a main racing road
                    other = _conn_other_end_road(c, self.pitLaneRoad)
                    if other is None or other not in main_racing_set_pre:
                        continue

                    pit_conns_here.append(c)

                if not pit_conns_here:
                    continue

                # Prefer pit connector that meets main road at main's endpoint (so pit junction aligns with main-loop geometry)
                scored = []
                for c in pit_conns_here:
                    other = _conn_other_end_road(c, self.pitLaneRoad)
                    if other is None:
                        continue
                    s = _smoothness_between_roads_via_conn(self.pitLaneRoad, c, other)
                    if s is None:
                        continue
                    endpoint_fit = _pit_conn_endpoint_fit_to_main(c, self.pitLaneRoad, other)
                    PIT_ENDPOINT_FIT_WEIGHT = 0.02
                    combined = s + PIT_ENDPOINT_FIT_WEIGHT * endpoint_fit
                    scored.append((combined, -_road_length(c), c))

                if scored:
                    scored.sort(key=lambda x: (x[0], x[1]))
                    best = scored[0][2]
                else:
                    # Fallback: longest (legacy behavior)
                    best = max(pit_conns_here, key=_road_length)

                self._pitRoads.append(best)
                conn_name = getattr(best, 'name', str(best))[:50]
                conn_len = _road_length(best)
                print(f"  [INFO] Pit (junction link, auto): {conn_name} ({conn_len:.1f}m)")

        def _road_id_match(road, id_list):
            if not id_list:
                return False
            rid = getattr(road, "id", None)
            if rid is None:
                return False
            rid = int(rid) if rid is not None else None
            for i in id_list:
                if rid == (int(i) if i is not None else None):
                    return True
            return False

        def _pit_conn_at_junction(junc):
            """Return the pit junction link at this junction (from _pitRoads), or None."""
            for c in self._pitRoads:
                if c not in connecting_roads:
                    continue
                if _conn_belongs_to_junction(c, junc, intersections):
                    return c
            return None

        def _divergence_angle_with_pit(main_conn: Road, pit_conn: Road) -> float:
            """Angle (radians) between main and pit connector centerlines at the junction. Higher = more natural fork."""
            def dist_sq(a, b):
                return (a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2
            ms = _road_endpoint_and_heading_local(main_conn, at_start=True)
            me = _road_endpoint_and_heading_local(main_conn, at_start=False)
            ps = _road_endpoint_and_heading_local(pit_conn, at_start=True)
            pe = _road_endpoint_and_heading_local(pit_conn, at_start=False)
            if ms is None or me is None or ps is None or pe is None:
                return 0.0
            pairs = [
                ((ms[0], ms[1]), ms[2], (ps[0], ps[1]), ps[2]),
                ((ms[0], ms[1]), ms[2], (pe[0], pe[1]), pe[2]),
                ((me[0], me[1]), me[2], (ps[0], ps[1]), ps[2]),
                ((me[0], me[1]), me[2], (pe[0], pe[1]), pe[2]),
            ]
            best = float("inf")
            main_heading, pit_heading = None, None
            for mx, mh, px, ph in pairs:
                d = dist_sq(mx, px)
                if d < best:
                    best = d
                    main_heading = mh
                    pit_heading = ph
            if main_heading is None or pit_heading is None:
                return 0.0
            return _angle_diff_local(main_heading, pit_heading)

        def _endpoint_fit_to_mains(conn: Road, main_roads: List[Road]) -> float:
            """Sum of distances from conn endpoints to main road endpoints when best-matched (conn start to one road, end to other). Lower = better alignment with main road geometry (e.g. Corkscrew)."""
            cs = _road_endpoint_and_heading_local(conn, at_start=True)
            ce = _road_endpoint_and_heading_local(conn, at_start=False)
            if cs is None or ce is None:
                return float("inf")
            c_start = (cs[0], cs[1])
            c_end = (ce[0], ce[1])
            main_pts = []
            for r in main_roads:
                rs = _road_endpoint_and_heading_local(r, at_start=True)
                re = _road_endpoint_and_heading_local(r, at_start=False)
                if rs is not None:
                    main_pts.append((rs[0], rs[1]))
                if re is not None:
                    main_pts.append((re[0], re[1]))
            if len(main_pts) < 2:
                return 0.0
            def dist(a, b):
                return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5
            best = float("inf")
            for i, pi in enumerate(main_pts):
                for j, pj in enumerate(main_pts):
                    if i == j:
                        continue
                    total = dist(c_start, pi) + dist(c_end, pj)
                    if total < best:
                        best = total
            return best

        # Step 2b: Add junction connecting roads to main loop — either explicit IDs or smoothness-based
        n_main = len(self._mainRacingRoads)
        main_racing_set = set(self._mainRacingRoads)
        main_loop_ids = self.main_loop_connecting_road_ids

        if main_loop_ids is not None:
            for conn_road in connecting_roads:
                if conn_road in main_racing_set:
                    continue
                if _road_id_match(conn_road, main_loop_ids):
                    self._mainRacingRoads.append(conn_road)
                    main_racing_set.add(conn_road)
                    conn_name = getattr(conn_road, 'name', str(conn_road))[:50]
                    conn_len = _road_length(conn_road)
                    print(f"  [INFO] Main racing (junction link): {conn_name} ({conn_len:.1f}m)")
        else:
            # Auto-select main-loop link per junction: prefer smooth main–main link that flows naturally with the pit (diverges at junction), then longer
            for junc in intersections:
                main_roads_here = [r for r in junc.roads if r in main_racing_set]
                if len(main_roads_here) < 2:
                    continue
                conn_roads_here = [
                    c for c in connecting_roads
                    if c not in main_racing_set and _conn_belongs_to_junction(c, junc, intersections)
                ]
                if not conn_roads_here:
                    continue
                pit_conn_here = _pit_conn_at_junction(junc)
                # Only consider full ramps (avoid lane-merge stubs); 25m threshold
                MIN_MAIN_LOOP_LINK_LENGTH = 25.0
                candidates = []
                main_roads_set = set(main_roads_here)

                for conn in conn_roads_here:
                    if self.pitLaneRoad is not None and _conn_connects_to_road(conn, self.pitLaneRoad):
                        continue
                    if _road_length(conn) < MIN_MAIN_LOOP_LINK_LENGTH:
                        continue

                    # NEW: connector must connect two main roads (topology), not just be near them geometrically
                    if not _conn_connects_two_roads_in_set(conn, main_roads_set):
                        continue

                    score = _smoothness_conn_to_mains(conn, main_roads_here)
                    if score is not None:
                        div = _divergence_angle_with_pit(conn, pit_conn_here) if pit_conn_here else 0.0
                        length = _road_length(conn)
                        # Prefer connector whose endpoints align with main road endpoints (so junction meets Corkscrew/Andretti at right place)
                        endpoint_fit = _endpoint_fit_to_mains(conn, main_roads_here)
                        DIVERGENCE_WEIGHT = 0.2
                        ENDPOINT_FIT_WEIGHT = 0.02  # 1m misalignment ≈ 0.02 added to score
                        div_penalty = DIVERGENCE_WEIGHT * (math.pi - div) if pit_conn_here else 0.0
                        combined = score + div_penalty + ENDPOINT_FIT_WEIGHT * endpoint_fit
                        candidates.append((combined, -length, conn))
                if not candidates:
                    continue
                candidates.sort(key=lambda x: (x[0], x[1]))
                best_conn = candidates[0][2]
                self._mainRacingRoads.append(best_conn)
                main_racing_set.add(best_conn)
                conn_name = getattr(best_conn, 'name', str(best_conn))[:50]
                conn_len = _road_length(best_conn)
                print(f"  [INFO] Main racing (junction link, smooth): {conn_name} ({conn_len:.1f}m)")

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
            
            n_junction_links = len(self._mainRacingRoads) - n_main
            print(f"\n  [OK] Main racing road: Union of {len(self._mainRacingRoads)} road(s)" + (
                f" ({n_junction_links} junction link(s))" if n_junction_links else ""
            ))

        # Summary
        if self._pitRoads and self.mainRacingRoad:
            print(f"  [OK] Two mutually exclusive segments defined:")
            print(f"       - Pit lane: {len(self._pitRoads)} road(s)")
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
    main_loop_connecting_road_ids: Optional[Tuple[Union[int, str], ...]] = None,
    pit_connecting_road_ids: Optional[Tuple[Union[int, str], ...]] = None,
    **map_options
) -> RacingTrack:
    """Create a RacingTrack from a map file.

    Args:
        mapFile: Path to OpenDRIVE (.xodr) file
        direction: Track direction ('clockwise' or 'counterclockwise')
        pitLaneRoadId: OpenDRIVE road ID for pit lane (optional, e.g., "1545702203")
        pitLaneRoadName: Pattern to match pit lane name (optional, e.g., "Pit Lane", "pit")
        mainLineRoadId: OpenDRIVE road ID for main racing line (optional, e.g., "2117817291")
        main_loop_connecting_road_ids: OpenDRIVE road IDs of junction links for outer loop (e.g. (24, 34))
        pit_connecting_road_ids: OpenDRIVE road IDs of junction links to pit (e.g. (25, 30))
        **map_options: Additional options passed to Network.fromFile()

    Returns:
        RacingTrack object with identified segments

    Example::

        # With explicit junction assignment (outer loop vs pit at two junctions)
        track = createRacingTrack(
            'laguna_seca.xodr',
            direction='counterclockwise',
            main_loop_connecting_road_ids=(24, 34),
            pit_connecting_road_ids=(25, 30),
        )
    """
    network = Network.fromFile(mapFile, **map_options)
    track = RacingTrack(
        network,
        direction=direction,
        pitLaneRoadId=pitLaneRoadId,
        pitLaneRoadName=pitLaneRoadName,
        mainLineRoadId=mainLineRoadId,
        main_loop_connecting_road_ids=main_loop_connecting_road_ids,
        pit_connecting_road_ids=pit_connecting_road_ids,
    )
    return track

