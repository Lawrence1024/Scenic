"""OpenDRIVE-based segment mapping for racing evaluation and logging.

Segments can be:
- Curve/straight (default): derived from centerline curvature. Where curvature
  exceeds a threshold the track is a "curve" segment; where it is below, "straight".
  Alternating curve/straight segments give many segments for fine-grained analysis.
- Conventional (Laguna Seca): fixed named sections (Front Straight, Andretti Hairpin,
  etc.) when use_conventional_laguna is True and track has 2 roads.
- Coarse: one segment per main racing road when neither of the above applies.

Waypoints (x, y) are projected onto the nearest road centerline to get s; only
(x, y) from the waypoint file is used.

Segments are deterministic for the same OpenDRIVE map: same centerline geometry
and CURVATURE_THRESHOLD produce the same segment boundaries every run. Use
scenic.domains.racing.segments.visualize_racing_segments to visualize the segments.
"""

import math
from typing import List, Optional, Tuple, Any

from scenic.core.geometry import makeShapelyPoint
from scenic.core.vectors import Vector

# Curvature threshold (1/m): above this = curve, below = straight. ~0.015 ≈ 67 m radius.
CURVATURE_THRESHOLD = 0.015

# -----------------------------------------------------------------------------
# Laguna Seca conventional segments (fallback: named sections by s-fraction).
# -----------------------------------------------------------------------------
LAGUNA_SECA_SEGMENTS: List[Tuple[int, float, float, int, str]] = [
    (0, 0.00, 0.10, 1, "Front Straight+T1"),
    (0, 0.10, 0.25, 3, "T3-T4"),
    (0, 0.25, 0.40, 4, "T5-T6"),
    (0, 0.40, 0.58, 5, "Rahal Straight"),
    (0, 0.58, 0.68, 6, "Corkscrew"),
    (0, 0.68, 0.78, 7, "Rainey Curve"),
    (0, 0.78, 1.00, 8, "T10-T11"),
    (1, 0.00, 1.00, 2, "Andretti Hairpin"),
]


def _get_road_centerline(road: Any) -> Optional[Any]:
    """Get the centerline polyline of the first lane of a road (OpenDRIVE geometry)."""
    if not getattr(road, "lanes", None) or len(road.lanes) == 0:
        return None
    lane = road.lanes[0]
    return getattr(lane, "centerline", None)


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
    return segments


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
    exclude_pit: bool = True,
    use_curvature_segments: bool = True,
    use_conventional_laguna: bool = False,
) -> List[Tuple[int, str]]:
    """Build (segment_id, segment_name) for each waypoint from the OpenDRIVE track.

    Each waypoint (x, y) is projected onto the nearest main racing road centerline
    to get arc length s; then the segment is determined from the map.

    Segment modes (in order of precedence):
    - use_curvature_segments True (default): derive curve/straight segments from
      centerline curvature. Many segments for fine-grained analysis; name is "curve" or "straight".
    - use_conventional_laguna True and 2 roads: fixed Laguna Seca named sections.
    - Else: one segment per road (id only, name "").

    Args:
        waypoints: List of waypoint tuples (at least x, y); can be (x,y) or (x,y,z).
        track: RacingTrack with _mainRacingRoads (list of Road objects from OpenDRIVE).
        exclude_pit: If True, only main racing roads are used (default).
        use_curvature_segments: If True, use curvature-derived curve/straight segments (default).
        use_conventional_laguna: If True and not curvature, use Laguna Seca named sections when 2 roads.

    Returns:
        List of (segment_id, segment_name) per waypoint. segment_name is "curve"/"straight" or section name or "".
    """
    n_wp = len(waypoints)
    if n_wp == 0:
        return []

    roads = getattr(track, "_mainRacingRoads", None)
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

    # Conventional Laguna only if not using curvature and 2 roads
    conventional = (
        LAGUNA_SECA_SEGMENTS
        if (not use_curvature_segments and use_conventional_laguna and len(centerlines) == 2)
        else []
    )

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
            if s is not None:
                seg_id, seg_name = _segment_for_curve_straight(
                    best_road_idx, s, road_segments, segment_id_offset
                )
                segment_map.append((seg_id, seg_name))
            else:
                segment_map.append((segment_id_offset[best_road_idx] + 1, "straight"))
        elif conventional:
            s = _road_s_at_point(x, y, centerlines[best_road_idx])
            if s is not None:
                L = road_lengths[best_road_idx]
                seg_id, seg_name = _segment_for_road_s(
                    best_road_idx, s, L, conventional
                )
                segment_map.append((seg_id, seg_name))
            else:
                segment_map.append((best_road_idx + 1, ""))
        else:
            segment_map.append((best_road_idx + 1, ""))

    return segment_map


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
