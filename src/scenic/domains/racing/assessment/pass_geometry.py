"""Pass-window look-ahead — geometric viability check for an overtake.

Walks ego on a candidate pass-side TTL and the opponent on the optimal TTL
forward in arc-length, sampling at fixed dt. At each sample, computes the
2D euclidean distance between projected ego and opp positions. If the
minimum distance over the window is below ``min_lat_clearance_m``, the pass
is rejected — the pass-side TTL converges with the opponent's path within
the predicted pass duration (e.g. right TTL re-merging into optimal at a
corner entry, the F2_tactical failure mode).

Three modes:
- ``side="left"``  ego walks on side_waypoints (left TTL),  opp on optimal_waypoints
- ``side="right"`` ego walks on side_waypoints (right TTL), opp on optimal_waypoints
- ``side="merge_back"`` ego walks on optimal_waypoints (the destination during
  the post-pass merge), opp on optimal_waypoints — checks that fellow won't
  intersect the merge path within the lookahead window.
"""

from __future__ import annotations

from typing import Optional, Sequence, Tuple

# Default lookahead — observed F2 successful pass side-by-side window is 2-3 s.
DEFAULT_PASS_DURATION_S = 2.5
DEFAULT_SAMPLE_DT_S = 0.25
# SD-9: raised from 1.6 m to 2.5 m. The 1.6 m threshold was derived as
# "IAC half-width 0.96 m × 2 = 1.92 m for body-touching; +ε = 1.6 m" — wrong
# arithmetic: 0.96+0.96 = 1.92, so 1.6 m is BELOW even body-touching minimum.
# Realistic racing safety: 1.92 m bodies-touching + 0.5-1.0 m steering/oscillation
# buffer = ~2.5 m centerline-to-centerline minimum.
#
# Empirical confirmation from F-bank (full_stack_20260427_082305):
#   F2 right TTL @ start section: 2.08 m centerline-to-centerline → 0.15 m body
#       clearance → "barely clears" → with 2.5 m threshold this section correctly
#       rejects the pass; ego stays in FOLLOW until fellow moves or geometry widens.
#   F3L right TTL (fellow on left): 5+ m centerline-to-centerline → 3+ m body
#       buffer → SAFE pass; threshold change preserves F3L behavior.
DEFAULT_MIN_LAT_CLEARANCE_M = 2.5


# SD-10h: cache Shapely LineString objects per polyline. Polylines (TTL
# waypoints) are static for the duration of a run, but pre-SD-10h
# `_xy_at_arclength` rebuilt the LineString from 3500 (x,y,z) points on
# EVERY call — and path_collision_predicted calls this 32 times per
# control tick (16 samples × 2 polylines). Measured F2_tactical:
#   mean tick_ms = 139.5, p99 = 258, max = 757 (50ms budget)
#   wall_t / sim_t = 4.7× (30s sim took 140s wall)
# This cache keyed on Python id(waypoints) drops the per-call cost from
# ~3ms (list comp + Shapely C-struct build of 3500 pts) to ~50µs
# (dict lookup + LineString.interpolate). Expected effect: tick_ms down
# from 140ms to ~40ms (under budget).
#
# Caveats:
#   - Cache key is id(waypoints). If a caller passes a NEW list with the
#     same content, that's a cache miss (correct: the cache is for stable
#     refs, not value equality).
#   - Mem cost: each LineString holds ~3500 (x,y) tuples; ~3 polylines per
#     run = ~10k tuples cached = ~150 KB. Acceptable.
#   - Cache lives for process lifetime. If a long-running process loads
#     many distinct polylines, this grows unbounded. For test/sim workloads
#     (≤10 distinct polylines per run) this is fine.
_LINESTRING_CACHE: dict = {}


def _xy_at_arclength(
    waypoints: Sequence[Sequence[float]], s_m: float, lap_length_m: float
) -> Optional[Tuple[float, float]]:
    """Interpolate (x, y) at arc-length s on a lap-loop polyline.

    Returns None if shapely unavailable or polyline degenerate.
    SD-10h: caches the LineString on id(waypoints) — see module note above.
    SD-10j: hybrid key (id + n + first_xy + last_xy) prevents stale-hit when
    Python recycles a GC'd polyline's id (manifested under random test ordering).
    """
    if not waypoints or len(waypoints) < 2 or lap_length_m <= 0.0:
        return None
    n = len(waypoints)
    fx, fy = float(waypoints[0][0]), float(waypoints[0][1])
    lx, ly = float(waypoints[-1][0]), float(waypoints[-1][1])
    cache_key = (id(waypoints), n, fx, fy, lx, ly)
    cached = _LINESTRING_CACHE.get(cache_key)
    if cached is None:
        try:
            from shapely.geometry import LineString
        except ImportError:
            return None
        coords = [(float(p[0]), float(p[1])) for p in waypoints]
        # Close the loop so interpolate beyond the last segment wraps to first.
        if coords[0] != coords[-1]:
            coords = coords + [coords[0]]
        ls = LineString(coords)
        if ls.is_empty or ls.length <= 0:
            return None
        _LINESTRING_CACHE[cache_key] = ls
    else:
        ls = cached
    s_wrapped = float(s_m) % float(lap_length_m)
    s_clamped = max(0.0, min(float(ls.length), s_wrapped))
    pt = ls.interpolate(s_clamped)
    return (float(pt.x), float(pt.y))


def _clear_linestring_cache() -> None:
    """Test helper: clear the cache. Used by tests that pass throwaway
    polyline lists to avoid id() collision artifacts."""
    _LINESTRING_CACHE.clear()


def pass_window_check(
    side: str,
    *,
    ego_s_m: float,
    ego_speed_mps: float,
    opp_s_m: float,
    opp_speed_mps: float,
    optimal_waypoints: Sequence[Sequence[float]],
    side_waypoints: Sequence[Sequence[float]],
    lap_length_m: float,
    pass_duration_s: float = DEFAULT_PASS_DURATION_S,
    sample_dt_s: float = DEFAULT_SAMPLE_DT_S,
    min_lat_clearance_m: float = DEFAULT_MIN_LAT_CLEARANCE_M,
) -> Tuple[bool, dict]:
    """Walk ego and opp polylines forward in time; report geometric viability.

    Returns ``(ok, diag)`` where:
      - ``ok`` is True if min(clearance) over the window >= min_lat_clearance_m.
      - ``diag`` carries ``{side, min_clear_m, closest_t_s, samples}``.

    For side in {"left", "right"}: ego walks ``side_waypoints``; opp walks
    ``optimal_waypoints``.
    For side == "merge_back": ego walks ``optimal_waypoints`` (destination of
    the merge); opp also walks ``optimal_waypoints`` (since fellow is now
    behind ego on the same line, this checks fellow doesn't catch up into
    ego's merge path).
    """

    if side not in ("left", "right", "merge_back"):
        return False, {"side": side, "reason": "invalid_side"}

    if side == "merge_back":
        ego_track = optimal_waypoints
    else:
        ego_track = side_waypoints

    if not ego_track or not optimal_waypoints or lap_length_m <= 0.0:
        # Insufficient data — fail open (don't block the pass on a missing input).
        return True, {"side": side, "reason": "insufficient_data"}

    n_steps = max(2, int(float(pass_duration_s) / float(sample_dt_s)))
    min_clear = float("inf")
    closest_t = 0.0
    samples: list = []

    for i in range(n_steps + 1):
        t = i * float(sample_dt_s)
        e_s = float(ego_s_m) + float(ego_speed_mps) * t
        o_s = float(opp_s_m) + float(opp_speed_mps) * t
        ego_xy = _xy_at_arclength(ego_track, e_s, lap_length_m)
        opp_xy = _xy_at_arclength(optimal_waypoints, o_s, lap_length_m)
        if ego_xy is None or opp_xy is None:
            # Shapely unavailable on this tick — skip this sample (fail open).
            continue
        dx = ego_xy[0] - opp_xy[0]
        dy = ego_xy[1] - opp_xy[1]
        d = (dx * dx + dy * dy) ** 0.5
        samples.append((t, d))
        if d < min_clear:
            min_clear = d
            closest_t = t

    if min_clear == float("inf"):
        # No valid samples (likely no shapely). Fail open so absence of shapely
        # doesn't block all overtakes.
        return True, {"side": side, "reason": "no_samples"}

    ok = bool(min_clear >= float(min_lat_clearance_m))
    return ok, {
        "side": side,
        "min_clear_m": float(min_clear),
        "closest_t_s": float(closest_t),
        "samples": samples,
    }


# SD-4: predicted-path-collision check for brake-trigger gating.
# Tighter horizon (1.5s) and finer sampling (0.1s) than pass_window_check —
# brake triggers care about IMMINENT collision, not pass-completion-window.
DEFAULT_COLLISION_HORIZON_S = 1.5
DEFAULT_COLLISION_SAMPLE_DT_S = 0.1
DEFAULT_COLLISION_MIN_CLEAR_M = 1.6
DEFAULT_COLLISION_BREACH_DEBOUNCE = 2


def path_collision_predicted(
    *,
    ego_track: Sequence[Sequence[float]],
    opp_track: Sequence[Sequence[float]],
    ego_s_m: float,
    ego_speed_mps: float,
    opp_s_m: float,
    opp_speed_mps: float,
    lap_length_m: float,
    horizon_s: float = DEFAULT_COLLISION_HORIZON_S,
    sample_dt_s: float = DEFAULT_COLLISION_SAMPLE_DT_S,
    min_clear_m: float = DEFAULT_COLLISION_MIN_CLEAR_M,
    require_consecutive_breach: int = DEFAULT_COLLISION_BREACH_DEBOUNCE,
    opp_trajectory: Optional[Sequence[Tuple[float, float, float, Optional[float]]]] = None,
) -> Tuple[bool, dict]:
    """Predict whether ego and opp trajectories will come within min_clear_m.

    SEMANTIC NOTE — mirror of pass_window_check:
      - Returns ``True`` ⇔ COLLISION is predicted.
      - Returns ``False`` ⇔ paths stay safely apart (or insufficient data).

    For each sample t in [0, horizon_s], walks ego forward on ``ego_track`` from
    ``ego_s_m + ego_speed_mps · t``. Opponent xy at each sample comes from one of
    two sources:
      - ``opp_trajectory`` (PREFERRED, SD-12c): list of ``(t, x, y, s)`` from
        FellowPredictor.trajectory(). Uses the fellow's true observed xy
        propagated by CV velocity. Handles stationary fellow off-line correctly
        (the F9 case) without any speed=0 special case.
      - polyline projection (FALLBACK): ``opp_s_m + opp_speed_mps · t`` mapped
        through ``_xy_at_arclength(opp_track, ...)``. Used when opp_trajectory
        is None (backward compat for tests / callers that don't thread it).
        Known limitation: misclassifies stationary off-line fellow as a
        polyline-projected ON-line obstacle.

    A "breach" is a sample where distance < min_clear_m; collision is declared
    only after ``require_consecutive_breach`` consecutive breaches (debounces
    single-sample numerical noise at polyline sutures).

    FAIL-CLOSED on missing data (shapely unavailable, empty polylines, lap=0):
    returns ``(False, {"reason": "insufficient_data"})``. This is the OPPOSITE
    of pass_window_check's fail-open — for brake gating we don't want missing
    data to trigger a brake.

    Used by SD-4c/4d to gate every brake-trigger in the racing planner. Snapshot
    triggers (overlap_flag, gap_ok, risk, etc.) become fast-fail filters; this
    function is the AUTHORITY on whether to actually brake.
    """
    if not ego_track or not opp_track or lap_length_m <= 0.0:
        return False, {"reason": "insufficient_data"}

    n_steps = max(2, int(float(horizon_s) / float(sample_dt_s)))
    min_clear = float("inf")
    closest_t = 0.0
    breach_run = 0
    max_breach_run = 0
    samples: list = []
    # A sample at less than half min_clear means the cars actually OVERLAP
    # (not merely close). This is automatic collision regardless of debounce —
    # protects against the high-closing-speed case where ego shoots past opp
    # between consecutive samples and only one sample registers the contact.
    hard_overlap_threshold = float(min_clear_m) * 0.5

    # SD-12c: build a fast index for opp_trajectory lookup if provided.
    _use_opp_traj = bool(opp_trajectory) and len(opp_trajectory) > 0

    def _opp_xy_at(t_off: float):
        if _use_opp_traj:
            # Find the trajectory sample closest in time. Linear scan bounded
            # by trajectory length (~21 for 10s @ 0.5dt — cheap).
            best_i = 0
            best_dt = abs(opp_trajectory[0][0] - t_off)
            for j in range(1, len(opp_trajectory)):
                d_t = abs(opp_trajectory[j][0] - t_off)
                if d_t < best_dt:
                    best_dt = d_t
                    best_i = j
            return (float(opp_trajectory[best_i][1]),
                    float(opp_trajectory[best_i][2]))
        # Fallback: polyline projection
        o_s = float(opp_s_m) + float(opp_speed_mps) * t_off
        return _xy_at_arclength(opp_track, o_s, lap_length_m)

    for i in range(n_steps + 1):
        t = i * float(sample_dt_s)
        e_s = float(ego_s_m) + float(ego_speed_mps) * t
        ego_xy = _xy_at_arclength(ego_track, e_s, lap_length_m)
        opp_xy = _opp_xy_at(t)
        if ego_xy is None or opp_xy is None:
            # Insufficient data on this sample (likely no shapely). Bail out
            # fail-closed for the whole call.
            return False, {"reason": "no_samples"}
        dx = ego_xy[0] - opp_xy[0]
        dy = ego_xy[1] - opp_xy[1]
        d = (dx * dx + dy * dy) ** 0.5
        samples.append((t, d))
        if d < min_clear:
            min_clear = d
            closest_t = t
        if d < float(min_clear_m):
            breach_run += 1
            if breach_run > max_breach_run:
                max_breach_run = breach_run
        else:
            breach_run = 0

    # Collision if either:
    #   (a) at any sample the cars actually overlap (d < min_clear/2), OR
    #   (b) at least N consecutive samples breach min_clear (debounces noise).
    hard_overlap = bool(min_clear < hard_overlap_threshold)
    collision = bool(hard_overlap or max_breach_run >= int(require_consecutive_breach))
    return collision, {
        "min_clear_m": float(min_clear),
        "closest_t_s": float(closest_t),
        "breach_count": int(max_breach_run),
        "hard_overlap": hard_overlap,
        "samples": samples,
    }


def select_tracks_for_state(planner_state: str, ego_active_ttl: str) -> Tuple[str, str]:
    """Decide which polyline ego and opp are each currently on, given planner state.

    Returns ``(ego_track_name, opp_track_name)`` where each name is in
    {"optimal", "left", "right"}. Centralises the "which TTL is each car on"
    decision so callers don't duplicate the conditional.

    Assumptions:
      - opp is always assumed to be on "optimal" (the F-bank fellow scripts
        all drive a fixed TTL, and even when they're on left/right we treat
        their projection onto optimal as the rear-end check basis).
      - ego is on whatever TTL the planner currently has it on, derived from
        ``ego_active_ttl`` and the planner_state. During SETUP/COMMIT/HOLD
        ego is on the side TTL; during FOLLOW/FREE_RUN/ABORT ego is on
        whatever active_ttl says.
    """
    state = str(planner_state or "")
    if state in ("SETUP_PASS_LEFT", "COMMIT_PASS_LEFT", "HOLD_PASS_LEFT"):
        return "left", "optimal"
    if state in ("SETUP_PASS_RIGHT", "COMMIT_PASS_RIGHT", "HOLD_PASS_RIGHT"):
        return "right", "optimal"
    # FREE_RUN / FOLLOW / ABORT_PASS / SETUP_LEFT / SETUP_RIGHT (stale) etc.
    ego = str(ego_active_ttl or "optimal") if str(ego_active_ttl or "") in ("optimal", "left", "right") else "optimal"
    return ego, "optimal"


__all__ = [
    "pass_window_check",
    "path_collision_predicted",
    "select_tracks_for_state",
    "DEFAULT_PASS_DURATION_S",
    "DEFAULT_SAMPLE_DT_S",
    "DEFAULT_MIN_LAT_CLEARANCE_M",
    "DEFAULT_COLLISION_HORIZON_S",
    "DEFAULT_COLLISION_SAMPLE_DT_S",
    "DEFAULT_COLLISION_MIN_CLEAR_M",
    "DEFAULT_COLLISION_BREACH_DEBOUNCE",
]
