"""OpenDRIVE-based and TTL-based segment mapping for racing evaluation and logging.

Two entry points (same return type so downstream code is unchanged):
- build_waypoint_segment_map(waypoints, track, ...) — segments from OpenDRIVE track.
- build_waypoint_segment_map_from_ttl(main_ttl_waypoints, pit_ttl_waypoints=None, waypoints=None, ...)
  — segments from TTL waypoints only. Uses the same pipeline as the visualization
  (polyline from waypoints, _build_curve_straight_segments with CURVATURE_THRESHOLD,
  MIN_SEGMENT_LENGTH, MAX_STRAIGHT_ABSORB_INTO_CURVE). In overlap (e.g. Andretti,
  Corkscrew) waypoints are assigned to main (TTL_OVERLAP_MAIN_WINS_M) so what you
  see in visualize_racing_segments --ttl-folder matches what Scenic uses.

Segments can be:
- Curve/straight (default): derived from centerline curvature. Where curvature
  exceeds a threshold the track is a "curve" segment; where it is below, "straight".
  Alternating curve/straight segments give many segments for fine-grained analysis.
- Conventional (Laguna Seca): fixed named sections (OpenDRIVE path only).
- Coarse: one segment per road when curvature/conventional not used.

Waypoints (x, y) are projected onto the nearest centerline (track road or TTL) to get s.
Segments are deterministic for the same geometry and CURVATURE_THRESHOLD.
Use scenic.domains.racing.segments.visualize_racing_segments to visualize.
"""

import math
from typing import Any, List, Optional, Sequence, Tuple

from scenic.core.geometry import makeShapelyPoint
from scenic.core.regions import PolylineRegion
from scenic.core.vectors import Vector

from scenic.domains.racing.segments.ring_topology import build_ring_topology, get_pit_entry_main_road

# Curvature threshold (1/m): above this = curve, below = straight.
# Lower value = gentler bends count as curve (curve segments extend further into approaches).
# 0.011 ≈ 91 m radius so curve 10 extends a bit; 0.012 ≈ 83 m radius.
CURVATURE_THRESHOLD = 0.011

# Minimum segment length (m). Shorter segments are merged into an adjacent segment
# to avoid tiny curve/straight chunks from geometry noise or dense sampling.
MIN_SEGMENT_LENGTH = 15.0

# Short straights (below this length) adjacent to a curve are merged into the curve
# so that the curve segment covers the full bend (entry/exit transitions).
# 30 m merges Corkscrew straights (e.g. segment 15 ~25.6 m) into adjacent curves so
# 14+15+16 become one big curve; also lets curve 10 extend a bit further.
MAX_STRAIGHT_ABSORB_INTO_CURVE = 30.0

# Minimum run length (waypoints) to treat as a real main/pit stretch; shorter runs
# are merged into the adjacent stretch so we get a single path (main -> pit -> main).
MIN_STRETCH_WAYPOINTS = 40

# TTL only: waypoints within this distance (m) of the main polyline are assigned to
# main (overlap). Matches visualization OVERLAP_THRESHOLD_M so Scenic segment map
# matches what you see (main includes Andretti/Corkscrew; pit is pit-only).
TTL_OVERLAP_MAIN_WINS_M = 2.0


# -----------------------------------------------------------------------------
# CC-2 (2026-04-26): LAGUNA_SECA_SEGMENTS deleted. It was a fallback for the
# OLD LagunaSeca.xodr (2 roads, named sections by s-fraction); the live path
# is XODR-derived curve/straight from `_build_curve_straight_segments`. The
# `use_conventional_laguna=True` mode of `build_waypoint_segment_map` is
# retained for API compatibility but always falls through to the curvature
# path or the coarse-one-segment path.


def _midline_from_edges(left_edge: Any, right_edge: Any, num_points: int = 500) -> Optional[List[Tuple[float, float]]]:
    """Compute midline as average of left and right boundary polylines.

    Samples both edges at the same normalized arc length (0..1) and averages (x,y).
    This gives the true geometric center of the track regardless of XODR reference/lanes.
    """
    try:
        left_ls = getattr(left_edge, "lineString", left_edge)
        right_ls = getattr(right_edge, "lineString", right_edge)
        if not hasattr(left_ls, "interpolate") or not hasattr(right_ls, "interpolate"):
            return None
        if left_ls.is_empty or right_ls.is_empty or len(left_ls.coords) < 2 or len(right_ls.coords) < 2:
            return None
        pts = []
        for i in range(num_points):
            t = i / max(1, num_points - 1)
            try:
                l_pt = left_ls.interpolate(t, normalized=True)
                r_pt = right_ls.interpolate(t, normalized=True)
            except Exception:
                return None
            lx, ly = float(l_pt.x), float(l_pt.y)
            rx, ry = float(r_pt.x), float(r_pt.y)
            pts.append(((lx + rx) * 0.5, (ly + ry) * 0.5))
        return pts
    except Exception:
        return None


def _get_road_centerline(road: Any) -> Optional[Any]:
    """Get the road centerline for track regions.

    Prefer the geometric midline (average of left and right boundaries) when both
    exist; that is the true track center regardless of XODR lanes/reference.
    Fallback to Road.centerline (reference line), then lane 0 centerline.
    """
    left_edge = getattr(road, "leftEdge", None)
    right_edge = getattr(road, "rightEdge", None)
    if left_edge is not None and right_edge is not None:
        mid_pts = _midline_from_edges(left_edge, right_edge)
        if mid_pts and len(mid_pts) >= 2:
            from scenic.core.regions import PolylineRegion
            return PolylineRegion(points=mid_pts)
    centerline = getattr(road, "centerline", None)
    if centerline is not None:
        return centerline
    lanes = getattr(road, "lanes", None)
    if not lanes or len(lanes) == 0:
        return None
    return getattr(lanes[0], "centerline", None)


def _dist2(p: Tuple[float, ...], q: Tuple[float, ...]) -> float:
    """Euclidean distance between two 2D/3D points."""
    dx = float(q[0]) - float(p[0])
    dy = float(q[1]) - float(p[1])
    return math.hypot(dx, dy)


def _curvature_at_vertex(
    p_prev: Tuple[float, ...],
    p_curr: Tuple[float, ...],
    p_next: Tuple[float, ...],
) -> float:
    """Curvature (1/m) at the middle vertex; turn angle / arc length."""
    ax = float(p_curr[0]) - float(p_prev[0])
    ay = float(p_curr[1]) - float(p_prev[1])
    bx = float(p_next[0]) - float(p_curr[0])
    by = float(p_next[1]) - float(p_curr[1])
    len_a = math.hypot(ax, ay)
    len_b = math.hypot(bx, by)
    if len_a < 1e-9 or len_b < 1e-9:
        return 0.0
    # Turn angle (radians) between segment a and b
    angle_a = math.atan2(ay, ax)
    angle_b = math.atan2(by, bx)
    turn = angle_b - angle_a
    while turn > math.pi:
        turn -= 2 * math.pi
    while turn < -math.pi:
        turn += 2 * math.pi
    ds = 0.5 * (len_a + len_b)
    if ds < 1e-9:
        return 0.0
    return abs(turn) / ds


def _merge_short_segments(
    segments: List[Tuple[float, float, str]],
    min_length: float,
) -> List[Tuple[float, float, str]]:
    """Merge segments shorter than min_length (m) into an adjacent segment."""
    if min_length <= 0 or not segments:
        return segments
    out: List[Tuple[float, float, str]] = []
    for s_start, s_end, seg_type in segments:
        length = s_end - s_start
        if length < min_length and out:
            # Merge into previous segment
            prev_start, prev_end, prev_type = out[-1]
            out[-1] = (prev_start, s_end, prev_type)
        elif length < min_length and segments:
            # First segment is short; will be merged when we see next (leave for now)
            out.append((s_start, s_end, seg_type))
        else:
            out.append((s_start, s_end, seg_type))
    # Merge any short leading segments (e.g. first was short, rest weren't)
    i = 0
    while i < len(out) - 1:
        s_start, s_end, seg_type = out[i]
        if (s_end - s_start) < min_length:
            next_start, next_end, next_type = out[i + 1]
            out[i + 1] = (s_start, next_end, next_type)
            out.pop(i)
        else:
            i += 1
    if len(out) > 1 and (out[0][1] - out[0][0]) < min_length:
        next_start, next_end, next_type = out[1]
        out[1] = (out[0][0], next_end, next_type)
        out.pop(0)
    return out


def _extend_curves_over_short_straights(
    segments: List[Tuple[float, float, str]],
    max_straight_length: float,
) -> List[Tuple[float, float, str]]:
    """Merge short straight segments into adjacent curve segments so that each
    curve segment covers the full bend (including entry/exit transitions that
    fall below the curvature threshold).
    """
    if max_straight_length <= 0 or len(segments) <= 1:
        return segments
    # Pass 1: merge short straight into previous when previous is curve
    out: List[Tuple[float, float, str]] = []
    for s_start, s_end, seg_type in segments:
        length = s_end - s_start
        if (
            seg_type == "straight"
            and length <= max_straight_length
            and out
            and out[-1][2] == "curve"
        ):
            prev_start, prev_end, prev_type = out[-1]
            out[-1] = (prev_start, s_end, prev_type)
        else:
            out.append((s_start, s_end, seg_type))
    # Pass 2: merge short straight into next when next is curve
    i = 0
    while i < len(out):
        s_start, s_end, seg_type = out[i]
        length = s_end - s_start
        if (
            seg_type == "straight"
            and length <= max_straight_length
            and i + 1 < len(out)
            and out[i + 1][2] == "curve"
        ):
            next_start, next_end, next_type = out[i + 1]
            out[i + 1] = (s_start, next_end, next_type)
            out.pop(i)
        else:
            i += 1
    return out


def _merge_consecutive_same_type(
    segments: List[Tuple[float, float, str]],
) -> List[Tuple[float, float, str]]:
    """Merge consecutive segments that have the same type (curve/straight).
    Produces a list where segment types strictly alternate, so one continuous
    curve or straight is a single segment instead of alternating short chunks.
    """
    if not segments:
        return segments
    out: List[Tuple[float, float, str]] = [segments[0]]
    for s_start, s_end, seg_type in segments[1:]:
        prev_start, prev_end, prev_type = out[-1]
        if seg_type == prev_type:
            out[-1] = (prev_start, s_end, prev_type)
        else:
            out.append((s_start, s_end, seg_type))
    return out


def _build_curve_straight_segments(
    centerline: Any,
    curvature_threshold: float = CURVATURE_THRESHOLD,
) -> List[Tuple[float, float, str]]:
    """Build (s_start, s_end, type) segments for one road from centerline curvature.

    type is 'curve' or 'straight'. Consecutive vertices with same classification
    are merged. Returns list ordered by s.
    """
    ls = getattr(centerline, "lineString", None)
    if ls is None:
        return []
    try:
        coords = list(getattr(ls, "coords", []))
    except Exception:
        return []
    if len(coords) < 3:
        return [(0.0, getattr(centerline, "length", 0.0) or 0.0, "straight")]

    n = len(coords)
    s_cum: List[float] = [0.0]
    for i in range(1, n):
        s_cum.append(s_cum[-1] + _dist2(coords[i - 1], coords[i]))
    total_s = s_cum[-1]
    if total_s < 1e-9:
        return [(0.0, total_s, "straight")]

    # Curvature at each vertex; endpoints inherit neighbor
    curvature: List[float] = [0.0] * n
    for i in range(1, n - 1):
        curvature[i] = _curvature_at_vertex(coords[i - 1], coords[i], coords[i + 1])
    if n > 1:
        curvature[0] = curvature[1]
        curvature[n - 1] = curvature[n - 2]

    # Label each vertex: curve or straight
    labels: List[str] = [
        "curve" if c > curvature_threshold else "straight" for c in curvature
    ]

    # Merge consecutive same label into segments (s_start, s_end, type)
    segments: List[Tuple[float, float, str]] = []
    seg_start_s = s_cum[0]
    seg_type = labels[0]
    for i in range(1, n):
        if labels[i] != seg_type:
            segments.append((seg_start_s, s_cum[i], seg_type))
            seg_start_s = s_cum[i]
            seg_type = labels[i]
    segments.append((seg_start_s, s_cum[n - 1], seg_type))

    # Merge segments shorter than MIN_SEGMENT_LENGTH into an adjacent segment
    segments = _merge_short_segments(segments, MIN_SEGMENT_LENGTH)
    # Extend curves over short adjacent straights so each curve covers the full bend
    segments = _extend_curves_over_short_straights(
        segments, MAX_STRAIGHT_ABSORB_INTO_CURVE
    )
    # Merge consecutive same-type segments so curve/straight strictly alternate
    segments = _merge_consecutive_same_type(segments)
    return segments


def get_curvature_and_segments_for_centerline(
    centerline: Any,
    curvature_threshold: float = CURVATURE_THRESHOLD,
) -> Tuple[List[float], List[float], List[Tuple[float, float, str]]]:
    """Return (s_cumulative, curvature_per_vertex, curve_straight_segments) for diagnostics.

    s_cumulative[i] = arc length to vertex i; curvature_per_vertex[i] = curvature (1/m) at vertex i;
    curve_straight_segments = list of (s_start, s_end, "curve"|"straight") after merges.
    """
    ls = getattr(centerline, "lineString", None)
    if ls is None:
        return [], [], []
    try:
        coords = list(getattr(ls, "coords", []))
    except Exception:
        return [], [], []
    n = len(coords)
    if n < 3:
        total_s = 0.0
        if n >= 2:
            total_s = _dist2(coords[0], coords[1])
        return [0.0] * n, [0.0] * n, [(0.0, total_s, "straight")]
    s_cum: List[float] = [0.0]
    for i in range(1, n):
        s_cum.append(s_cum[-1] + _dist2(coords[i - 1], coords[i]))
    curvature: List[float] = [0.0] * n
    for i in range(1, n - 1):
        curvature[i] = _curvature_at_vertex(coords[i - 1], coords[i], coords[i + 1])
    curvature[0] = curvature[1]
    curvature[n - 1] = curvature[n - 2]
    labels: List[str] = [
        "curve" if c > curvature_threshold else "straight" for c in curvature
    ]
    segments: List[Tuple[float, float, str]] = []
    seg_start_s = s_cum[0]
    seg_type = labels[0]
    for i in range(1, n):
        if labels[i] != seg_type:
            segments.append((seg_start_s, s_cum[i], seg_type))
            seg_start_s = s_cum[i]
            seg_type = labels[i]
    segments.append((seg_start_s, s_cum[n - 1], seg_type))
    segments = _merge_short_segments(segments, MIN_SEGMENT_LENGTH)
    segments = _extend_curves_over_short_straights(
        segments, MAX_STRAIGHT_ABSORB_INTO_CURVE
    )
    segments = _merge_consecutive_same_type(segments)
    return s_cum, curvature, segments


def _segment_for_curve_straight(
    road_idx: int,
    s: float,
    road_segments: List[List[Tuple[float, float, str]]],
    segment_id_offset: List[int],
) -> Tuple[int, str]:
    """Look up (segment_id, type) for (road_idx, s) from curve/straight segments."""
    if road_idx < 0 or road_idx >= len(road_segments):
        return (1, "straight")
    segs = road_segments[road_idx]
    offset = segment_id_offset[road_idx]
    for i, (s_start, s_end, seg_type) in enumerate(segs):
        if s_start <= s < s_end:
            return (offset + i + 1, seg_type)
        if i == len(segs) - 1 and s >= s_end - 1e-6:
            return (offset + i + 1, seg_type)
    return (offset + 1, "straight")


def _road_s_at_point(
    x: float, y: float, centerline: Any
) -> Optional[float]:
    """Return arc length s (meters) along the centerline to the projected point, or None."""
    try:
        ls = getattr(centerline, "lineString", None)
        if ls is None:
            return None
        pt = makeShapelyPoint((x, y))
        s = float(ls.project(pt))
        return s
    except Exception:
        return None


def _segment_for_road_s(
    road_idx: int,
    s: float,
    road_length: float,
    conventional: List[Tuple[int, float, float, int, str]],
) -> Tuple[int, str]:
    """Look up (segment_id, segment_name) for (road_idx, s). s and road_length in meters."""
    s_frac = s / road_length if road_length > 0 else 0.0
    s_frac = max(0.0, min(1.0, s_frac))
    for r_idx, s_start, s_end, seg_id, name in conventional:
        if r_idx == road_idx and s_start <= s_frac < s_end:
            return (seg_id, name)
    # Fallback: segment id from road index (1-based)
    return (road_idx + 1, "")


def build_waypoint_segment_map(
    waypoints: List[Tuple[float, ...]],
    track: Any,
    use_curvature_segments: bool = True,
    use_conventional_laguna: bool = False,
) -> List[Tuple[int, str]]:
    """Build (segment_id, segment_name) for each waypoint from the OpenDRIVE track.

    Each waypoint (x, y) is projected onto the nearest road centerline (main racing
    or pit) to get arc length s; then the segment is determined from the map.
    Including both main and pit roads keeps logging consistent for lap and pitlane TTLs.

    Segment modes (in order of precedence):
    - use_curvature_segments True (default): derive curve/straight segments from
      centerline curvature. Segment names are "main straight", "main curve", "pit straight", "pit curve".
    - use_conventional_laguna True and 2 roads: fixed Laguna Seca named sections.
    - Else: one segment per road (id only, name "").

    Args:
        waypoints: List of waypoint tuples (at least x, y); can be (x,y) or (x,y,z).
        track: RacingTrack with _mainRacingRoads and _pitRoads (OpenDRIVE).
        use_curvature_segments: If True, use curvature-derived curve/straight segments (default).
        use_conventional_laguna: If True and not curvature, use Laguna Seca named sections when 2 roads.

    Returns:
        List of (segment_id, segment_name) per waypoint. segment_name is "main straight", "main curve",
        "pit straight", "pit curve", or (for conventional/coarse modes) section name or "".
    """
    n_wp = len(waypoints)
    if n_wp == 0:
        return []

    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])
    roads = main_roads + pit_roads
    n_main_roads = len(main_roads)
    if not roads:
        return [(1, "")] * n_wp

    centerlines: List[Any] = []
    road_lengths: List[float] = []
    for road in roads:
        cl = _get_road_centerline(road)
        if cl is not None:
            centerlines.append(cl)
            road_lengths.append(getattr(cl, "length", 0.0) or 0.0)

    if not centerlines:
        return [(1, "")] * n_wp

    # Build curve/straight segments per road when requested
    road_segments: List[List[Tuple[float, float, str]]] = []
    segment_id_offset: List[int] = []
    if use_curvature_segments:
        off = 1
        for cl in centerlines:
            segs = _build_curve_straight_segments(cl)
            road_segments.append(segs)
            segment_id_offset.append(off)
            off += len(segs)
    else:
        road_segments = []
        segment_id_offset = []

    # CC-2: LAGUNA_SECA_SEGMENTS deleted (was OLD-LagunaSeca.xodr fallback).
    # The conventional path now always falls through to coarse-one-segment-per-road.
    conventional = []  # type: List[Tuple[int, float, float, int, str]]

    segment_map: List[Tuple[int, str]] = []
    for i in range(n_wp):
        wp = waypoints[i]
        x, y = float(wp[0]), float(wp[1])
        point = Vector(x, y, 0.0) if len(wp) >= 3 else Vector(x, y)

        best_road_idx = 0
        best_dist = float("inf")
        for idx, centerline in enumerate(centerlines):
            try:
                d = centerline.distanceTo(point)
            except Exception:
                d = float("inf")
            if d < best_dist:
                best_dist = d
                best_road_idx = idx

        if use_curvature_segments and road_segments:
            s = _road_s_at_point(x, y, centerlines[best_road_idx])
            prefix = "main " if best_road_idx < n_main_roads else "pit "
            if s is not None:
                seg_id, seg_type = _segment_for_curve_straight(
                    best_road_idx, s, road_segments, segment_id_offset
                )
                segment_map.append((seg_id, prefix + seg_type))
            else:
                segment_map.append((segment_id_offset[best_road_idx] + 1, prefix + "straight"))
        elif conventional:
            prefix = "main " if best_road_idx < n_main_roads else "pit "
            s = _road_s_at_point(x, y, centerlines[best_road_idx])
            if s is not None:
                L = road_lengths[best_road_idx]
                seg_id, seg_name = _segment_for_road_s(
                    best_road_idx, s, L, conventional
                )
                segment_map.append((seg_id, prefix + seg_name if seg_name else prefix.rstrip()))
            else:
                segment_map.append((best_road_idx + 1, prefix + "road"))
        else:
            prefix = "main " if best_road_idx < n_main_roads else "pit "
            segment_map.append((best_road_idx + 1, prefix + "road"))

    # Extract ring topology from OpenDRIVE so path-respecting logic can validate advancement
    main_ring_segment_ids: List[int] = []
    pit_ring_segment_ids: List[int] = []
    pit_exit_transitions: List[Tuple[int, int]] = []
    pit_enter_transitions: List[Tuple[int, int]] = []
    if road_segments and segment_id_offset and len(roads) == len(road_segments):
        main_ring_roads, pit_ring_roads = build_ring_topology(track)
        main_ring_segment_ids = _ring_roads_to_segment_ids(
            main_ring_roads, roads, road_segments, segment_id_offset
        )
        pit_ring_segment_ids = _ring_roads_to_segment_ids(
            pit_ring_roads, roads, road_segments, segment_id_offset
        )
        # Pit exit: (last pit ring segment -> first main ring segment)
        if main_ring_segment_ids and pit_ring_segment_ids:
            pit_exit_transitions = [(pit_ring_segment_ids[-1], main_ring_segment_ids[0])]
        # Pit enter: (last segment of main road before pit junction -> first pit ring segment)
        main_road_before_pit = get_pit_entry_main_road(track)
        if main_road_before_pit is not None and pit_ring_segment_ids:
            try:
                idx = next(i for i, r in enumerate(roads) if r is main_road_before_pit)
            except StopIteration:
                idx = -1
            if 0 <= idx < len(road_segments) and idx < len(segment_id_offset):
                last_seg_id = segment_id_offset[idx] + len(road_segments[idx])
                pit_enter_transitions = [(last_seg_id, pit_ring_segment_ids[0])]

    # Apply path-respecting logic: only "advance" segment when it matches stretch and
    # (when rings are available) is the same or next segment in the current path's ring.
    path_respecting, main_sequence, pit_sequence = _build_path_respecting_and_sequences(
        segment_map,
        min_stretch_waypoints=MIN_STRETCH_WAYPOINTS,
        main_ring_segment_ids=main_ring_segment_ids if main_ring_segment_ids else None,
        pit_ring_segment_ids=pit_ring_segment_ids if pit_ring_segment_ids else None,
    )

    return _SegmentMapWithSequences(
        path_respecting,
        main_sequence,
        pit_sequence,
        main_ring_segment_ids=main_ring_segment_ids,
        pit_ring_segment_ids=pit_ring_segment_ids,
        pit_exit_transitions=pit_exit_transitions,
        pit_enter_transitions=pit_enter_transitions,
    )


def _ttl_polyline_from_waypoints(waypoints: List[Tuple[float, ...]]) -> Optional[PolylineRegion]:
    """Build a PolylineRegion from a list of waypoints (x,y) or (x,y,z) for use as a centerline."""
    if not waypoints or len(waypoints) < 2:
        return None
    points = [(float(wp[0]), float(wp[1])) for wp in waypoints]
    return PolylineRegion(points=points)


def build_waypoint_segment_map_from_ttl(
    main_ttl_waypoints: List[Tuple[float, ...]],
    pit_ttl_waypoints: Optional[List[Tuple[float, ...]]] = None,
    waypoints: Optional[List[Tuple[float, ...]]] = None,
    use_curvature_segments: bool = True,
    curvature_threshold: float = CURVATURE_THRESHOLD,
):
    """Build segment map from TTL centerlines (same pipeline as visualization).

    Uses the same logic as visualize_racing_segments --ttl-folder:
    - Build polyline(s) from main_ttl_waypoints and optionally pit_ttl_waypoints.
    - Segment each with _build_curve_straight_segments (same CURVATURE_THRESHOLD,
      MIN_SEGMENT_LENGTH, MAX_STRAIGHT_ABSORB_INTO_CURVE as OpenDRIVE).
    - Assign each waypoint to main or pit: in overlap (distance to main <
      TTL_OVERLAP_MAIN_WINS_M or closer to main than pit) assign to main so
      Andretti/Corkscrew are main; otherwise pit. Then project to that centerline
      and look up segment (curve/straight).

    Returns the same _SegmentMapWithSequences as build_waypoint_segment_map so
    get_segment_at_waypoint, get_ring_segment_ids, and behaviors work unchanged.
    """
    wp_list = waypoints if waypoints is not None else main_ttl_waypoints
    n_wp = len(wp_list)
    if n_wp == 0:
        return _SegmentMapWithSequences([], [], [], [], [], [], [])

    main_poly = _ttl_polyline_from_waypoints(main_ttl_waypoints)
    if main_poly is None:
        return _SegmentMapWithSequences(
            [(1, "main straight")] * n_wp, [(1, "main straight")], [],
            main_ring_segment_ids=[1], pit_ring_segment_ids=[],
            pit_exit_transitions=[], pit_enter_transitions=[],
        )

    centerlines: List[Any] = [main_poly]
    if pit_ttl_waypoints and len(pit_ttl_waypoints) >= 2:
        pit_poly = _ttl_polyline_from_waypoints(pit_ttl_waypoints)
        if pit_poly is not None:
            centerlines.append(pit_poly)
    n_main_roads = 1
    n_roads = len(centerlines)

    road_segments: List[List[Tuple[float, float, str]]] = []
    segment_id_offset: List[int] = []
    if use_curvature_segments:
        off = 1
        for cl in centerlines:
            segs = _build_curve_straight_segments(cl, curvature_threshold=curvature_threshold)
            road_segments.append(segs)
            segment_id_offset.append(off)
            off += len(segs)
    else:
        base = 1
        for cl in centerlines:
            length = getattr(cl, "length", None) or (
                getattr(cl.lineString, "length", 0.0) if getattr(cl, "lineString", None) else 0.0
            )
            road_segments.append([(0.0, length, "straight")])
            segment_id_offset.append(base)
            base += 1

    raw_map: List[Tuple[int, str]] = []
    for i in range(n_wp):
        wp = wp_list[i]
        x, y = float(wp[0]), float(wp[1])
        point = Vector(x, y, 0.0) if len(wp) >= 3 else Vector(x, y)

        if n_roads == 1:
            best_road_idx = 0
        else:
            try:
                d_main = centerlines[0].distanceTo(point)
            except Exception:
                d_main = float("inf")
            try:
                d_pit = centerlines[1].distanceTo(point)
            except Exception:
                d_pit = float("inf")
            if d_main <= d_pit or d_main < TTL_OVERLAP_MAIN_WINS_M:
                best_road_idx = 0
            else:
                best_road_idx = 1

        prefix = "main " if best_road_idx < n_main_roads else "pit "
        if use_curvature_segments and road_segments:
            s = _road_s_at_point(x, y, centerlines[best_road_idx])
            if s is not None:
                seg_id, seg_type = _segment_for_curve_straight(
                    best_road_idx, s, road_segments, segment_id_offset
                )
                raw_map.append((seg_id, prefix + seg_type))
            else:
                raw_map.append((segment_id_offset[best_road_idx], prefix + "straight"))
        else:
            raw_map.append((segment_id_offset[best_road_idx], prefix + "road"))

    main_ring_segment_ids = [segment_id_offset[0] + k for k in range(len(road_segments[0]))]
    pit_ring_segment_ids = (
        [segment_id_offset[1] + k for k in range(len(road_segments[1]))] if n_roads > 1 else []
    )

    pit_exit_transitions: List[Tuple[int, int]] = []
    pit_enter_transitions: List[Tuple[int, int]] = []
    if main_ring_segment_ids and pit_ring_segment_ids:
        stretch = _stretches_from_raw(raw_map, min_stretch_waypoints=MIN_STRETCH_WAYPOINTS)
        for i in range(1, n_wp):
            if stretch[i] == "pit" and stretch[i - 1] == "main":
                pit_enter_transitions.append((raw_map[i - 1][0], pit_ring_segment_ids[0]))
                break
        for i in range(1, n_wp):
            if stretch[i] == "main" and stretch[i - 1] == "pit":
                pit_exit_transitions.append((raw_map[i - 1][0], main_ring_segment_ids[0]))
                break

    path_respecting, main_sequence, pit_sequence = _build_path_respecting_and_sequences(
        raw_map,
        min_stretch_waypoints=MIN_STRETCH_WAYPOINTS,
        main_ring_segment_ids=main_ring_segment_ids if main_ring_segment_ids else None,
        pit_ring_segment_ids=pit_ring_segment_ids if pit_ring_segment_ids else None,
    )

    return _SegmentMapWithSequences(
        path_respecting,
        main_sequence,
        pit_sequence,
        main_ring_segment_ids=main_ring_segment_ids,
        pit_ring_segment_ids=pit_ring_segment_ids,
        pit_exit_transitions=pit_exit_transitions,
        pit_enter_transitions=pit_enter_transitions,
    )


def _ring_roads_to_segment_ids(
    ring_roads: List[Any],
    roads: List[Any],
    road_segments: List[List[Tuple[str, Optional[float], Optional[float]]]],
    segment_id_offset: List[int],
) -> List[int]:
    """Convert ordered list of roads (main or pit ring) to ordered segment IDs (1-based)."""
    out: List[int] = []
    for road in ring_roads:
        try:
            idx = next(i for i, r in enumerate(roads) if r is road)
        except StopIteration:
            continue
        if idx >= len(road_segments) or idx >= len(segment_id_offset):
            continue
        base = segment_id_offset[idx]
        n = len(road_segments[idx])
        for k in range(n):
            out.append(base + k + 1)
    return out


def _path_type(segment_name: str) -> str:
    """Return 'main' or 'pit' from segment name prefix."""
    if not segment_name:
        return "main"
    return "pit" if segment_name.startswith("pit ") else "main"


def _stretches_from_raw(
    segment_map: List[Tuple[int, str]],
    min_stretch_waypoints: int,
) -> List[str]:
    """Compute stretch ('main' or 'pit') per waypoint from raw segment map.

    Uses run-length encoding of path type; merges runs shorter than
    min_stretch_waypoints so we get a single path (e.g. main -> pit -> main).
    """
    n = len(segment_map)
    if n == 0:
        return []
    types = [_path_type(seg[1]) for seg in segment_map]
    # Run-length encode: (start_idx, length, type)
    runs: List[Tuple[int, int, str]] = []
    i = 0
    while i < n:
        t = types[i]
        j = i
        while j < n and types[j] == t:
            j += 1
        runs.append((i, j - i, t))
        i = j
    # Merge short runs into adjacent (prefer merging into next run to preserve junction order)
    merged: List[Tuple[int, int, str]] = []
    for start, length, t in runs:
        if merged and length < min_stretch_waypoints:
            # Merge this short run into previous
            prev_start, prev_len, prev_t = merged[-1]
            merged[-1] = (prev_start, prev_len + length, prev_t)
        elif merged and merged[-1][1] < min_stretch_waypoints:
            # Previous run was short; merge previous into this
            prev_start, prev_len, prev_t = merged.pop()
            merged.append((prev_start, prev_len + length, t))
        else:
            merged.append((start, length, t))
    # If we still have a short run at the end, merge into previous
    if len(merged) >= 2 and merged[-1][1] < min_stretch_waypoints:
        prev_start, prev_len, prev_t = merged[-2]
        _, short_len, _ = merged[-1]
        merged[-2] = (prev_start, prev_len + short_len, prev_t)
        merged.pop()
    # Build stretch per waypoint
    stretch: List[str] = ["main"] * n
    for start, length, t in merged:
        for k in range(start, min(start + length, n)):
            stretch[k] = t
    return stretch


def _next_in_ring(seg_id: int, ring_ids: List[int]) -> Optional[int]:
    """Return the segment ID that comes after seg_id in ring_ids, or None."""
    try:
        idx = ring_ids.index(seg_id)
        if idx + 1 < len(ring_ids):
            return ring_ids[idx + 1]
    except ValueError:
        pass
    return None


def _segment_valid_advancement(
    seg_id: int,
    path_type: str,
    last_seg_id: Optional[int],
    main_ring_ids: Optional[List[int]],
    pit_ring_ids: Optional[List[int]],
) -> bool:
    """True if seg_id is a valid advancement on path_type (same, or next in ring)."""
    ring = (pit_ring_ids if path_type == "pit" else main_ring_ids) or []
    if not ring:
        return True
    if seg_id not in ring:
        return False
    if last_seg_id is None:
        return True
    if seg_id == last_seg_id:
        return True
    return _next_in_ring(last_seg_id, ring) == seg_id


def _build_path_respecting_and_sequences(
    raw_map: List[Tuple[int, str]],
    min_stretch_waypoints: int = MIN_STRETCH_WAYPOINTS,
    main_ring_segment_ids: Optional[List[int]] = None,
    pit_ring_segment_ids: Optional[List[int]] = None,
) -> Tuple[List[Tuple[int, str]], List[Tuple[int, str]], List[Tuple[int, str]]]:
    """Build path-respecting segment map and main/pit segment sequences.

    Path-respecting map: at each waypoint we only accept the raw segment if it
    matches the stretch (main vs pit) and, when ring topology is provided, is
    the same or next segment in that path's ring; otherwise we keep the previous
    segment (ignore invalid advancement).

    Returns:
        (path_respecting_map, main_sequence, pit_sequence)
        - path_respecting_map: list of (segment_id, segment_name) per waypoint
        - main_sequence: ordered list of (segment_id, segment_name) on main path
        - pit_sequence: ordered list of (segment_id, segment_name) on pit path
    """
    n = len(raw_map)
    if n == 0:
        return [], [], []
    stretch = _stretches_from_raw(raw_map, min_stretch_waypoints)
    path_respecting: List[Tuple[int, str]] = []
    main_seen: List[Tuple[int, str]] = []
    pit_seen: List[Tuple[int, str]] = []
    last_main: Optional[Tuple[int, str]] = None
    last_pit: Optional[Tuple[int, str]] = None
    for i in range(n):
        raw_seg = raw_map[i]
        seg_id, seg_name = raw_seg
        t = stretch[i]
        raw_type = _path_type(seg_name)
        if raw_type == t:
            last_id = (last_pit[0] if last_pit else None) if t == "pit" else (last_main[0] if last_main else None)
            if _segment_valid_advancement(
                seg_id, t, last_id, main_ring_segment_ids, pit_ring_segment_ids
            ):
                path_respecting.append(raw_seg)
                if t == "main":
                    last_main = raw_seg
                    if not main_seen or main_seen[-1] != raw_seg:
                        main_seen.append(raw_seg)
                else:
                    last_pit = raw_seg
                    if not pit_seen or pit_seen[-1] != raw_seg:
                        pit_seen.append(raw_seg)
            else:
                # Same path but invalid advancement (e.g. jump to parallel segment): keep previous
                prev = last_pit if t == "pit" else last_main
                if prev is not None:
                    path_respecting.append(prev)
                else:
                    path_respecting.append(raw_seg)
        else:
            # Wrong path: keep previous segment (ignore this advancement)
            prev = last_pit if t == "pit" else last_main
            if prev is not None:
                path_respecting.append(prev)
            else:
                path_respecting.append(raw_seg)
                if t == "main":
                    last_main = raw_seg
                else:
                    last_pit = raw_seg
    return path_respecting, main_seen, pit_seen


class _SegmentMapWithSequences(list):
    """List of (segment_id, segment_name) per waypoint with main/pit sequences and ring topology.

    Subclasses list so indexing and len() work; get_segment_at_waypoint uses it as-is.
    """

    def __init__(
        self,
        path_respecting_map: List[Tuple[int, str]],
        main_sequence: List[Tuple[int, str]],
        pit_sequence: List[Tuple[int, str]],
        main_ring_segment_ids: Optional[List[int]] = None,
        pit_ring_segment_ids: Optional[List[int]] = None,
        pit_exit_transitions: Optional[List[Tuple[int, int]]] = None,
        pit_enter_transitions: Optional[List[Tuple[int, int]]] = None,
    ):
        super().__init__(path_respecting_map)
        self._main_sequence = main_sequence
        self._pit_sequence = pit_sequence
        self._main_ring_segment_ids = main_ring_segment_ids or []
        self._pit_ring_segment_ids = pit_ring_segment_ids or []
        self._pit_exit_transitions = pit_exit_transitions or []
        self._pit_enter_transitions = pit_enter_transitions or []

    @property
    def main_sequence(self) -> List[Tuple[int, str]]:
        return self._main_sequence

    @property
    def pit_sequence(self) -> List[Tuple[int, str]]:
        return self._pit_sequence

    @property
    def main_ring_segment_ids(self) -> List[int]:
        """Ordered segment IDs along the main ring (from OpenDRIVE topology)."""
        return self._main_ring_segment_ids

    @property
    def pit_ring_segment_ids(self) -> List[int]:
        """Ordered segment IDs along the pit ring (from OpenDRIVE topology)."""
        return self._pit_ring_segment_ids

    @property
    def pit_exit_transitions(self) -> List[Tuple[int, int]]:
        """(from_seg_id, to_seg_id) pairs that denote pit exit (e.g. 27 -> 1)."""
        return self._pit_exit_transitions

    @property
    def pit_enter_transitions(self) -> List[Tuple[int, int]]:
        """(from_seg_id, to_seg_id) pairs that denote pit enter (e.g. 15 -> 26)."""
        return self._pit_enter_transitions


def get_segment_sequences(
    segment_map: Optional[Sequence[Tuple[int, str]]],
) -> Tuple[Optional[List[Tuple[int, str]]], Optional[List[Tuple[int, str]]]]:
    """Return (main_sequence, pit_sequence) if this map was built with path-respecting logic.

    The racing library builds two sequences: the ordered list of segments on the main
    path and on the pit path. When progressing along the TTL, we only accept a segment
    advancement if the new segment belongs to the current path (main or pit). If the
    segment map was built by build_waypoint_segment_map, it is already path-respecting
    and the sequences are attached. This getter returns them when available.

    Returns:
        (main_sequence, pit_sequence); each is None or a list of (segment_id, segment_name).
    """
    if segment_map is None:
        return (None, None)
    main_seq = getattr(segment_map, "main_sequence", None) or getattr(
        segment_map, "_main_sequence", None
    )
    pit_seq = getattr(segment_map, "pit_sequence", None) or getattr(
        segment_map, "_pit_sequence", None
    )
    return (main_seq, pit_seq)


def get_ring_segment_ids(
    segment_map: Optional[Sequence[Tuple[int, str]]],
) -> Tuple[List[int], List[int]]:
    """Return (main_ring_segment_ids, pit_ring_segment_ids) from OpenDRIVE-derived ring topology.

    When the segment map was built by build_waypoint_segment_map with a track that has
    ring topology, these lists give the ordered segment IDs along the main ring and pit
    ring. Use them to validate advancement (e.g. only accept a new segment if it is the
    next in the current path's ring).

    Returns:
        (main_ring_segment_ids, pit_ring_segment_ids); each is a list of 1-based segment IDs.
    """
    if segment_map is None:
        return ([], [])
    main_ids = getattr(segment_map, "main_ring_segment_ids", None) or getattr(
        segment_map, "_main_ring_segment_ids", []
    )
    pit_ids = getattr(segment_map, "pit_ring_segment_ids", None) or getattr(
        segment_map, "_pit_ring_segment_ids", []
    )
    return (main_ids or [], pit_ids or [])


def get_pit_transitions(
    segment_map: Optional[Sequence[Tuple[int, str]]],
) -> Tuple[List[Tuple[int, int]], List[Tuple[int, int]]]:
    """Return (pit_exit_transitions, pit_enter_transitions) from OpenDRIVE-derived topology.

    Each transition is a list of (from_seg_id, to_seg_id) pairs. Pit exit e.g. (27, 1);
    pit enter e.g. (15, 26). Not hardcoded — derived from ring topology for any map.
    """
    if segment_map is None:
        return ([], [])
    exit_t = getattr(segment_map, "pit_exit_transitions", None) or getattr(
        segment_map, "_pit_exit_transitions", []
    )
    enter_t = getattr(segment_map, "pit_enter_transitions", None) or getattr(
        segment_map, "_pit_enter_transitions", []
    )
    return (exit_t or [], enter_t or [])


def get_segment_label(segment_id: int, segment_name: Optional[str] = None) -> str:
    """Return a log-friendly segment label (e.g. 'segment 1' or 'segment 6 Corkscrew')."""
    if segment_name:
        return f"segment {segment_id} {segment_name}"
    return f"segment {segment_id}"


def get_segment_at_waypoint(
    wp_idx: int,
    segment_map: Optional[List[Tuple[int, str]]],
) -> Optional[Tuple[int, str]]:
    """Return (segment_id, segment_name) at the given waypoint index, or None if no map."""
    if not segment_map or wp_idx < 0 or wp_idx >= len(segment_map):
        return None
    return segment_map[wp_idx]


def get_segment_at_waypoint_ring_strict(
    wp_idx: int,
    segment_map: Optional[Sequence[Tuple[int, str]]],
    current_path: str,
    last_valid_segment_id: Optional[int],
    last_valid_segment_name: Optional[str],
) -> Tuple[Optional[int], str, Optional[str]]:
    """Return (effective_segment_id, effective_segment_name, transition_kind) respecting ring topology.

    If the segment at wp_idx is an illegal switch (not same, not next in current path's
    ring), it is ignored and the last valid segment is returned — unless the transition
    (last_valid_segment_id, raw_seg_id) is a defined pit exit or pit enter; then the new
    segment is accepted and transition_kind is 'pit_exit' or 'pit_enter'. Caller should
    store the returned (id, name) as last valid and, when transition_kind is set, switch
    route (e.g. pit_exit -> Lap/R2, pit_enter -> Pit/R1).

    current_path: 'pit' or 'main' (e.g. from ego's route).
    last_valid_segment_id / last_valid_segment_name: from previous step (None / '' on first).
    transition_kind: None, 'pit_exit', or 'pit_enter' (from OpenDRIVE-derived transition lists).
    """
    raw = get_segment_at_waypoint(wp_idx, segment_map)
    main_ring, pit_ring = get_ring_segment_ids(segment_map)
    ring = pit_ring if current_path == "pit" else main_ring
    pit_exit_transitions, pit_enter_transitions = get_pit_transitions(segment_map)

    def with_transition(seg_id: Optional[int], seg_name: str, kind: Optional[str]):
        return (seg_id, seg_name or "", kind)

    if not ring:
        # No ring topology: use raw segment; fallback to last valid if no raw
        if raw:
            return with_transition(raw[0], raw[1], None)
        return with_transition(last_valid_segment_id, last_valid_segment_name or "", None)

    seg_id, seg_name = raw if raw else (last_valid_segment_id, last_valid_segment_name or "")
    if seg_id is None:
        return with_transition(last_valid_segment_id, last_valid_segment_name or "", None)

    # Allowed pit exit: (last_seg, raw_seg) in pit_exit_transitions → accept and report pit_exit
    if last_valid_segment_id is not None and (last_valid_segment_id, seg_id) in pit_exit_transitions:
        return with_transition(seg_id, seg_name, "pit_exit")
    # Allowed pit enter: (last_seg, raw_seg) in pit_enter_transitions → accept and report pit_enter
    if last_valid_segment_id is not None and (last_valid_segment_id, seg_id) in pit_enter_transitions:
        return with_transition(seg_id, seg_name, "pit_enter")

    valid = (
        seg_id in ring
        and (
            last_valid_segment_id is None
            or seg_id == last_valid_segment_id
            or _next_in_ring(last_valid_segment_id, ring) == seg_id
        )
    )
    if valid:
        return with_transition(seg_id, seg_name, None)
    return with_transition(last_valid_segment_id, last_valid_segment_name or "", None)


def position_nearest_road_is_pit(x: float, y: float, track: Any) -> bool:
    """Return True if (x, y) is nearest to a pit road centerline (for pit speed limit by position).

    Projects the point onto all main and pit road centerlines; if the closest road is a pit road,
    returns True. Call at moderate frequency (e.g. every 10 behavior steps) and cache if needed.
    """
    main_roads = list(getattr(track, "_mainRacingRoads", None) or [])
    pit_roads = list(getattr(track, "_pitRoads", None) or [])
    roads = main_roads + pit_roads
    n_main = len(main_roads)
    if not roads:
        return False
    point = Vector(x, y, 0.0)
    best_idx = 0
    best_d = float("inf")
    for idx, road in enumerate(roads):
        cl = _get_road_centerline(road)
        if cl is None:
            continue
        try:
            d = cl.distanceTo(point)
        except Exception:
            continue
        if d < best_d:
            best_d = d
            best_idx = idx
    return best_idx >= n_main
