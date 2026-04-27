"""Phase 2: race-relative situation features for one opponent (planner inputs).

Computes semantics (ahead/behind, Δs, lateral relation, overlap, short-horizon risk,
segment context) from ego/opponent geometry. Intended for logging now and tactical
planning in Phase 3+.

Δs uses arc length along the ego TTL polyline when ``waypoints`` and ``lap_length_m``
are provided; otherwise falls back to longitudinal distance along ego heading (same
idea as Phase 0 ``nearest_opp_ds``).
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import List, Optional, Sequence, Tuple

# ---------------------------------------------------------------------------
# Geometry helpers
# ---------------------------------------------------------------------------


def polyline_lap_length_m(waypoints: Sequence[Sequence[float]]) -> float:
    """Closed-loop length: sum of segments i -> i+1 with last -> first."""
    n = len(waypoints)
    if n < 2:
        return 0.0
    total = 0.0
    for i in range(n):
        x0, y0 = float(waypoints[i][0]), float(waypoints[i][1])
        x1, y1 = float(waypoints[(i + 1) % n][0]), float(waypoints[(i + 1) % n][1])
        dx, dy = x1 - x0, y1 - y0
        total += (dx * dx + dy * dy) ** 0.5
    return total


def _arc_length_project_xy(
    x: float, y: float, waypoints: Sequence[Sequence[float]]
) -> Optional[float]:
    try:
        from shapely.geometry import LineString, Point
    except ImportError:
        return None
    if not waypoints or len(waypoints) < 2:
        return None
    coords = [(float(p[0]), float(p[1])) for p in waypoints]
    ls = LineString(coords)
    if ls.is_empty or ls.length <= 0:
        return None
    return float(ls.project(Point(float(x), float(y))))


def wrap_delta_s(delta: float, lap_length: float) -> float:
    """Map delta to (-L/2, L/2] (opponent relative to ego along forward lap)."""
    if lap_length <= 1e-6:
        return delta
    d = delta
    half = lap_length * 0.5
    # Shift to [0, L) then to (-L/2, L/2]
    d = (d + half) % lap_length - half
    return d


def waypoint_segment_run_progress(
    segment_map: Optional[Sequence[Tuple[int, str]]],
    wp_idx: int,
    segment_id: Optional[int],
) -> Optional[float]:
    """Fraction [0,1] along the current segment id run containing wp_idx."""
    if (
        not segment_map
        or segment_id is None
        or wp_idx < 0
        or wp_idx >= len(segment_map)
    ):
        return None
    lo = wp_idx
    while lo > 0 and segment_map[lo - 1][0] == segment_id:
        lo -= 1
    hi = wp_idx
    while hi + 1 < len(segment_map) and segment_map[hi + 1][0] == segment_id:
        hi += 1
    if hi <= lo:
        return 0.5
    return (wp_idx - lo) / float(hi - lo)


# ---------------------------------------------------------------------------
# Segment context (planner-facing)
# ---------------------------------------------------------------------------


def planner_segment_context(
    segment_name: str,
    segment_progress: Optional[float],
    curvature_ahead_max: float,
) -> str:
    """Coarse context: straight | corner_entry | corner_body | corner_exit."""
    sn = (segment_name or "").lower()
    is_straight_word = "straight" in sn
    is_curve_word = (
        "curve" in sn or "hairpin" in sn or "corkscrew" in sn or "andretti" in sn
    )
    if is_straight_word and not is_curve_word:
        return "straight"
    if is_curve_word:
        if segment_progress is not None:
            if segment_progress < 0.25:
                return "corner_entry"
            if segment_progress > 0.75:
                return "corner_exit"
            return "corner_body"
        # No progress: use curvature lookahead as a weak proxy
        if curvature_ahead_max >= 0.025:
            return "corner_body"
        if curvature_ahead_max >= 0.014:
            return "corner_entry"
        return "corner_exit"
    if curvature_ahead_max < 0.008:
        return "straight"
    if curvature_ahead_max >= 0.02:
        return "corner_body"
    return "corner_entry"


# ---------------------------------------------------------------------------
# Overlap + hysteresis
# ---------------------------------------------------------------------------

def _classify_overlap_raw(
    ahead: bool,
    longitudinal_m: float,
    lateral_m: float,
    ego_speed_mps: float,
    opp_speed_mps: float,
    long_side: float = 5.5,
    lat_side: float = 3.8,
    long_partial_lon: float = 14.0,
    lat_partial_lat: float = 0.8,
    closing_behind_speed: float = 0.8,
) -> str:
    """Rule-based overlap bucket (before hysteresis).

    lat_partial_lat reduced from 2.0 → 0.8 m: a fellow on a parallel TTL (~1 m
    lateral) must NOT trigger partial_overlap, which would lock _is_release_hazard()
    and prevent protected_follow from clearing after a successful pass.
    """
    al = abs(longitudinal_m)
    atl = abs(lateral_m)
    behind = not ahead

    if al <= long_side and atl <= lat_side:
        return "side_by_side"
    if al <= long_partial_lon and atl <= lat_partial_lat:
        # Pure in-line tailgating (small |lateral|, beyond side-by-side long range)
        # stays in ahead/behind buckets — partial overlap implies meaningful lateral offset.
        if atl < 1.2 and al > long_side:
            pass
        else:
            return "partial_overlap"

    if behind:
        if opp_speed_mps > ego_speed_mps + closing_behind_speed and al < 80.0:
            return "closing_behind"
        return "clear_behind"

    return "clear_ahead"


def stabilize_overlap_state(
    previous: str,
    proposed: str,
    longitudinal_m: float,
    lateral_m: float,
    long_enter_side: float = 4.5,
    lat_enter_side: float = 3.2,
    long_exit_side: float = 7.5,
    lat_exit_side: float = 5.2,
) -> str:
    """Schmitt-style holding for side_by_side / partial_overlap to reduce flicker."""
    al = abs(longitudinal_m)
    atl = abs(lateral_m)
    if previous == "side_by_side":
        if proposed == "side_by_side":
            return "side_by_side"
        if al < long_exit_side and atl < lat_exit_side:
            return "side_by_side"
    if previous == "partial_overlap":
        if proposed in ("side_by_side", "partial_overlap"):
            if al < long_exit_side or atl < lat_exit_side:
                return proposed if proposed == "side_by_side" else "partial_overlap"
        if proposed == "partial_overlap":
            return "partial_overlap"
        if al < long_exit_side and atl < lat_exit_side:
            return "partial_overlap"
    if proposed == "side_by_side":
        if al <= long_enter_side and atl <= lat_enter_side:
            return "side_by_side"
        return "partial_overlap"
    return proposed


# ---------------------------------------------------------------------------
# Main API
# ---------------------------------------------------------------------------


@dataclass(frozen=True)
class OpponentSituation:
    ahead: bool
    delta_s_m: float
    delta_s_source: str
    lateral_relation: str
    closing_speed_mps: float
    overlap_state: str
    collision_risk_01: float
    segment_context: str
    distance_m: float
    longitudinal_m: float
    lateral_m: float
    opponent_speed_mps: float = 0.0


def _lateral_relation_from_lateral_m(lateral_m: float, eps: float = 0.35) -> str:
    """Opponent vs ego using ego-frame lateral offset (left normal · (opp - ego))."""
    if lateral_m > eps:
        return "left"
    if lateral_m < -eps:
        return "right"
    return "on_line"


def closing_speed_mps(ahead: bool, ego_speed_mps: float, opp_speed_mps: float) -> float:
    """Positive => gap along race direction is shrinking (proxy, parallel-track)."""
    if ahead:
        return ego_speed_mps - opp_speed_mps
    return opp_speed_mps - ego_speed_mps


def collision_risk_short_horizon(
    distance_m: float,
    closing_speed_mps: float,
    lateral_sep_m: float,
    ttc_horizon_s: float = 4.0,
    lat_scale_m: float = 4.0,
    *,
    longitudinal_m: Optional[float] = None,
) -> float:
    """Heuristic [0,1]: higher when close, closing fast, and aligned laterally.

    When ``longitudinal_m`` is set, along-track separation drives closing / TTC; lateral
    separation modulates severity (narrow lateral + speed matters more than Euclidean blend).
    """
    along_m = abs(float(longitudinal_m)) if longitudinal_m is not None else float(distance_m)
    if distance_m <= 0:
        return 1.0
    if closing_speed_mps <= 0.05:
        base = max(0.0, 1.0 - along_m / 35.0)
    else:
        ttc = along_m / closing_speed_mps
        base = max(0.0, 1.0 - min(ttc, ttc_horizon_s) / ttc_horizon_s)
    lat_factor = max(0.2, 1.0 - min(abs(lateral_sep_m), lat_scale_m) / lat_scale_m)
    return min(1.0, base * lat_factor)


def assess_nearest_opponent(
    ego_xy: Tuple[float, float],
    ego_heading_rad: float,
    ego_speed_mps: float,
    opp_xy: Tuple[float, float],
    opp_speed_mps: float,
    *,
    ego_progress_s_m: Optional[float] = None,
    waypoints: Optional[Sequence[Sequence[float]]] = None,
    lap_length_m: Optional[float] = None,
    segment_map: Optional[Sequence[Tuple[int, str]]] = None,
    ego_wp_idx: int = 0,
    segment_id: Optional[int] = None,
    segment_name: str = "",
    curvature_ahead_max: float = 0.0,
    previous_overlap_state: str = "clear_ahead",
) -> Tuple[OpponentSituation, str]:
    """Compute :class:`OpponentSituation` and return updated overlap state string."""

    import math

    px, py = float(ego_xy[0]), float(ego_xy[1])
    ox, oy = float(opp_xy[0]), float(opp_xy[1])
    dx, dy = ox - px, oy - py
    dist = (dx * dx + dy * dy) ** 0.5

    ch, sh = math.cos(ego_heading_rad), math.sin(ego_heading_rad)
    longitudinal_m = dx * ch + dy * sh
    lateral_m = dx * (-sh) + dy * ch

    ahead = longitudinal_m >= 0.0

    lat_rel = _lateral_relation_from_lateral_m(lateral_m)
    close_spd = closing_speed_mps(ahead, ego_speed_mps, opp_speed_mps)

    delta_source = "heading_proxy"
    delta_s = longitudinal_m
    if (
        waypoints is not None
        and len(waypoints) >= 2
        and ego_progress_s_m is not None
    ):
        opp_s = _arc_length_project_xy(ox, oy, waypoints)
        if opp_s is not None:
            L = lap_length_m if lap_length_m is not None else polyline_lap_length_m(
                waypoints
            )
            if L > 1e-3:
                delta_s = wrap_delta_s(opp_s - float(ego_progress_s_m), L)
                delta_source = "polyline"

    seg_prog = waypoint_segment_run_progress(segment_map, ego_wp_idx, segment_id)
    seg_ctx = planner_segment_context(segment_name, seg_prog, curvature_ahead_max)

    raw_overlap = _classify_overlap_raw(
        ahead,
        longitudinal_m,
        lateral_m,
        ego_speed_mps,
        opp_speed_mps,
    )
    overlap = stabilize_overlap_state(previous_overlap_state, raw_overlap, longitudinal_m, lateral_m)

    risk = collision_risk_short_horizon(
        dist, close_spd, lateral_m, longitudinal_m=longitudinal_m
    )

    sit = OpponentSituation(
        ahead=ahead,
        delta_s_m=float(delta_s),
        delta_s_source=delta_source,
        lateral_relation=lat_rel,
        closing_speed_mps=float(close_spd),
        overlap_state=overlap,
        collision_risk_01=float(risk),
        segment_context=seg_ctx,
        distance_m=float(dist),
        longitudinal_m=float(longitudinal_m),
        lateral_m=float(lateral_m),
        opponent_speed_mps=float(opp_speed_mps),
    )
    return sit, overlap


def format_opponent_log_line(t_sim_s: float, sit: OpponentSituation) -> str:
    return (
        f"[Phase2] t={t_sim_s:.2f}s ahead={1 if sit.ahead else 0} "
        f"delta_s_m={sit.delta_s_m:.2f}({sit.delta_s_source}) "
        f"lat_rel={sit.lateral_relation} closing_mps={sit.closing_speed_mps:.2f} "
        f"overlap={sit.overlap_state} risk_01={sit.collision_risk_01:.3f} "
        f"seg_ctx={sit.segment_context} dist_m={sit.distance_m:.2f} "
        f"lon_m={sit.longitudinal_m:.2f} lat_m={sit.lateral_m:.2f}"
    )


__all__ = [
    "OpponentSituation",
    "assess_nearest_opponent",
    "planner_segment_context",
    "polyline_lap_length_m",
    "format_opponent_log_line",
    "waypoint_segment_run_progress",
    "stabilize_overlap_state",
    "collision_risk_short_horizon",
]
