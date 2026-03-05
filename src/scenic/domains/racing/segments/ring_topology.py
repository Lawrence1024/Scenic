"""Extract main and pit ring topology from OpenDRIVE track.

Builds ordered lists of roads (main ring and pit ring) by traversing the road
graph using successor/predecessor and junction connecting roads. Used when
segmenting roads to produce explicit progression sequences (e.g. 1->2->...->15
->20->16->...->19->21->1 and 26->22->23->24->25->27->1).
"""

from typing import Any, List, Optional, Set, Tuple

from scenic.domains.driving.roads import Road, Intersection


def _conn_connects_to_road(conn: Road, road: Road) -> bool:
    """True if connector links to road (topology or lane-level)."""
    pred = getattr(conn, "_predecessor", None)
    succ = getattr(conn, "_successor", None)
    if pred is road or succ is road:
        return True
    # Lane-level: connector lane's predecessor/successor lane belongs to road
    for lane in getattr(conn, "lanes", ()):
        pred_lane = getattr(lane, "_predecessor", None)
        succ_lane = getattr(lane, "_successor", None)
        if pred_lane is not None and getattr(pred_lane, "road", None) is road:
            return True
        if succ_lane is not None and getattr(succ_lane, "road", None) is road:
            return True
    return False


def _conn_other_end_road(conn: Road, road: Road) -> Optional[Road]:
    """Other road that connector links to (topology or lane-level)."""
    pred = getattr(conn, "_predecessor", None)
    succ = getattr(conn, "_successor", None)
    if pred is road:
        return succ if isinstance(succ, Road) else None
    if succ is road:
        return pred if isinstance(pred, Road) else None
    # Lane-level: from a lane on conn that connects to road, get the other end's road
    for lane in getattr(conn, "lanes", ()):
        pred_lane = getattr(lane, "_predecessor", None)
        succ_lane = getattr(lane, "_successor", None)
        if pred_lane is not None and getattr(pred_lane, "road", None) is road and succ_lane is not None:
            other = getattr(succ_lane, "road", None)
            if other is not None and other is not road:
                return other
        if succ_lane is not None and getattr(succ_lane, "road", None) is road and pred_lane is not None:
            other = getattr(pred_lane, "road", None)
            if other is not None and other is not road:
                return other
    return None


def _get_next_roads(
    road: Road,
    road_set: Set[Road],
    connecting_roads: Tuple[Road, ...],
) -> List[Road]:
    """Ordered list of next road(s) when leaving `road` (connector then target, or direct)."""
    succ = getattr(road, "_successor", None)
    if succ is None:
        return []
    if isinstance(succ, Road) and succ in road_set:
        return [succ]
    # Successor is junction: find connector in road_set that links road to another road in road_set
    if isinstance(succ, Intersection) or succ not in road_set:
        for conn in connecting_roads:
            if conn not in road_set:
                continue
            if not _conn_connects_to_road(conn, road):
                continue
            other = _conn_other_end_road(conn, road)
            if other is not None and other in road_set:
                return [conn, other]
    return []


def build_ring_topology(track: Any) -> Tuple[List[Road], List[Road]]:
    """Extract main ring and pit ring as ordered lists of roads from the track.

    Traverses the road graph (successor and junction connecting roads) to build
    the driving order for the main loop and the pit path. Main ring is a closed
    loop; pit ring is the sequence through the pit (entry connector -> pit road(s)
    -> exit connector).

    Args:
        track: RacingTrack with _mainRacingRoads, _pitRoads, and network.connectingRoads.

    Returns:
        (main_ring_roads, pit_ring_roads):
        - main_ring_roads: ordered list of Road objects (main line + main-loop connectors).
        - pit_ring_roads: ordered list of Road objects (pit connectors + pit lane).
    """
    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])
    connecting_roads = tuple(getattr(getattr(track, "network", None), "connectingRoads", ()) or ())

    main_set = set(main_roads)
    pit_set = set(pit_roads)

    main_ring: List[Road] = []
    pit_ring: List[Road] = []

    # Build main ring: start from first main road, follow successors until we close the loop
    if main_roads:
        start = main_roads[0]
        path: List[Road] = [start]
        visited: Set[Road] = {start}
        current = start
        max_steps = len(main_set) * 2 + 10
        for _ in range(max_steps):
            next_list = _get_next_roads(current, main_set, connecting_roads)
            if not next_list:
                break
            for r in next_list:
                if r is start and len(path) > 1:
                    main_ring = path
                    break
                if r not in visited:
                    path.append(r)
                    visited.add(r)
                current = r
            if main_ring:
                break
            current = path[-1]
        if not main_ring and path:
            main_ring = path

    # Build pit ring: start from pit lane road, follow back to entry connector then forward to exit
    if pit_roads and connecting_roads:
        pit_lane_road = getattr(track, "pitLaneRoad", None)
        if pit_lane_road is not None and pit_lane_road in pit_set:
            # Find entry: connector that has pit_lane as successor (we approach from main, then connector, then pit)
            entry_conn = None
            for c in connecting_roads:
                if c not in pit_set:
                    continue
                other = _conn_other_end_road(c, pit_lane_road)
                if other is not None and other in main_set:
                    entry_conn = c
                    break
            # Find exit: connector that has pit_lane as predecessor
            exit_conn = None
            for c in connecting_roads:
                if c not in pit_set or c is entry_conn:
                    continue
                if _conn_connects_to_road(c, pit_lane_road):
                    other = _conn_other_end_road(c, pit_lane_road)
                    if other is not None and other in main_set and other is not (getattr(entry_conn, "_predecessor", None) if entry_conn else None):
                        exit_conn = c
                        break
            if entry_conn is not None:
                pit_ring = [entry_conn, pit_lane_road]
                if exit_conn is not None and exit_conn is not entry_conn:
                    pit_ring.append(exit_conn)
            elif pit_roads:
                pit_ring = list(pit_roads)
        else:
            pit_ring = list(pit_roads)

    if not main_ring and main_roads:
        main_ring = list(main_roads)
    if not pit_ring and pit_roads:
        pit_ring = list(pit_roads)

    return (main_ring, pit_ring)


def get_pit_entry_main_road(track: Any) -> Optional[Road]:
    """Return the main road that precedes the pit entry (junction) so pit-enter transition can be derived.

    Used to compute pit_enter_transitions: (last_segment_of_this_road, first_pit_segment).
    """
    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])
    connecting_roads = tuple(getattr(getattr(track, "network", None), "connectingRoads", ()) or ())
    main_set = set(main_roads)
    pit_lane_road = getattr(track, "pitLaneRoad", None)
    if not pit_roads or not pit_lane_road or pit_lane_road not in set(pit_roads):
        return None
    for r in pit_roads:
        if r not in connecting_roads:
            continue
        other = _conn_other_end_road(r, pit_lane_road)
        if other is not None and other in main_set:
            return other
    return None
