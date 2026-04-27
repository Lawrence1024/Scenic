"""Unit tests for SD-3a pass_window_check geometric look-ahead +
SD-4a path_collision_predicted brake-trigger gate."""

import math

from scenic.domains.racing.assessment import (
    pass_window_check,
    path_collision_predicted,
    select_tracks_for_state,
)


def _make_straight_polyline(x0: float, y0: float, dx: float, dy: float, n: int):
    """Build an n-point polyline starting at (x0, y0), step (dx, dy) per point."""
    return [(x0 + i * dx, y0 + i * dy, 0.0) for i in range(n)]


def test_pass_window_clear_when_polylines_parallel():
    """Two perfectly parallel straight polylines, 3 m apart laterally.

    Ego on side TTL at y=3, opp on optimal at y=0. Distance = 3 m at every
    sample, well above min_lat_clearance_m=1.6. Pass should be cleared.
    """
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)  # y=0
    side = _make_straight_polyline(0.0, 3.0, 1.0, 0.0, 200)     # y=3
    lap_length_m = 199.0
    ok, diag = pass_window_check(
        "right",
        ego_s_m=10.0,
        ego_speed_mps=20.0,
        opp_s_m=10.0,
        opp_speed_mps=10.0,
        optimal_waypoints=optimal,
        side_waypoints=side,
        lap_length_m=lap_length_m,
        pass_duration_s=2.0,
        sample_dt_s=0.25,
        min_lat_clearance_m=1.6,
    )
    assert ok is True, f"parallel polylines must clear, got diag={diag}"
    assert diag["min_clear_m"] >= 2.5  # ~3 m minus tiny longitudinal mismatch


def test_pass_window_rejected_when_side_ttl_converges_into_opp_path():
    """Side TTL starts 4 m offset but converges to optimal at s=20 m.

    Ego closes longitudinally on opp during the lookahead window. As ego
    advances along the converging side TTL, lateral clearance collapses.
    Pass must be rejected — this is the F2_tactical right-TTL-into-corner
    failure mode in synthetic form.
    """
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)  # y=0
    # Side polyline: starts at y=4 at x=0, linearly converges to y=0 at x=20.
    side = []
    for i in range(200):
        x = float(i)
        if x <= 20.0:
            y = 4.0 - (4.0 / 20.0) * x
        else:
            y = 0.0
        side.append((x, y, 0.0))
    lap_length_m = 199.0
    ok, diag = pass_window_check(
        "right",
        ego_s_m=10.0,    # ego starts 10 m in (lat ~2 m on side TTL)
        ego_speed_mps=15.0,
        opp_s_m=12.0,    # opp slightly ahead
        opp_speed_mps=8.0,
        optimal_waypoints=optimal,
        side_waypoints=side,
        lap_length_m=lap_length_m,
        pass_duration_s=2.0,
        sample_dt_s=0.25,
        min_lat_clearance_m=1.6,
    )
    assert ok is False, f"converging side TTL must reject, got diag={diag}"
    assert diag["min_clear_m"] < 1.6
    assert diag["closest_t_s"] >= 0.0


def test_pass_window_merge_back_clear_when_opp_safely_behind():
    """Merge_back mode: ego on optimal path, opp far behind on optimal.

    With opp 15 m behind and same forward speed, merge path stays clear
    over the 1 s lookahead. Should return ok=True.
    """
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)  # y=0
    side = _make_straight_polyline(0.0, 3.0, 1.0, 0.0, 200)     # not used in merge_back logic
    lap_length_m = 199.0
    ok, diag = pass_window_check(
        "merge_back",
        ego_s_m=50.0,
        ego_speed_mps=20.0,
        opp_s_m=35.0,    # 15 m behind
        opp_speed_mps=18.0,  # 2 m/s slower → falling further behind
        optimal_waypoints=optimal,
        side_waypoints=side,
        lap_length_m=lap_length_m,
        pass_duration_s=1.0,
        sample_dt_s=0.25,
        min_lat_clearance_m=1.6,
    )
    assert ok is True, f"opp 15 m behind on same line must clear, got diag={diag}"
    # ego pulling away → clearance grows over the window.
    assert diag["min_clear_m"] >= 15.0


def test_pass_window_invalid_side_returns_false():
    """Sanity: an invalid side label returns (False, ...) with reason."""
    poly = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 10)
    ok, diag = pass_window_check(
        "diagonal",
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=0.0, opp_speed_mps=10.0,
        optimal_waypoints=poly, side_waypoints=poly, lap_length_m=9.0,
    )
    assert ok is False
    assert diag.get("reason") == "invalid_side"


def test_pass_window_insufficient_data_fails_open():
    """Empty waypoints → fail open (don't block all overtakes when data missing)."""
    ok, diag = pass_window_check(
        "left",
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=0.0, opp_speed_mps=10.0,
        optimal_waypoints=[], side_waypoints=[], lap_length_m=0.0,
    )
    assert ok is True
    assert diag.get("reason") == "insufficient_data"


# ============================================================
# SD-4a: path_collision_predicted unit tests (mirror semantics)
# ============================================================


def test_collision_predicted_head_on_rear_end():
    """Both cars on same line; ego closing fast → collision predicted within 1.5s."""
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)
    # ego at s=0 going 25 m/s; opp at s=10 going 5 m/s.
    # Closing 20 m/s; gap 10m → collision in ~0.5s.
    collision, diag = path_collision_predicted(
        ego_track=optimal, opp_track=optimal,
        ego_s_m=0.0, ego_speed_mps=25.0,
        opp_s_m=10.0, opp_speed_mps=5.0,
        lap_length_m=199.0,
        horizon_s=1.5, sample_dt_s=0.1, min_clear_m=1.6,
    )
    assert collision is True, f"Head-on rear-end MUST predict collision; diag={diag}"
    assert diag["min_clear_m"] < 1.6
    assert diag["closest_t_s"] >= 0.0
    # Hard-overlap path triggers when ego shoots through opp between samples
    # (single sample at d=0). The debounce-bypass via hard_overlap saves us.
    assert diag["hard_overlap"] is True


def test_collision_predicted_parallel_non_converging_clear():
    """ego on left TTL (y=4), opp on optimal (y=0), both going 20 m/s.
    Lateral gap stays at 4m forever — NO collision predicted."""
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)
    left = _make_straight_polyline(0.0, 4.0, 1.0, 0.0, 200)
    collision, diag = path_collision_predicted(
        ego_track=left, opp_track=optimal,
        ego_s_m=10.0, ego_speed_mps=20.0,
        opp_s_m=12.0, opp_speed_mps=20.0,
        lap_length_m=199.0,
    )
    assert collision is False, f"Parallel non-converging MUST NOT predict collision; diag={diag}"
    assert diag["min_clear_m"] >= 3.5  # ~4m lateral with tiny longitudinal jitter
    assert diag["breach_count"] == 0


def test_collision_predicted_fail_closed_on_missing_data():
    """Missing polylines → fail-CLOSED (no collision). Opposite of pass_window_check."""
    collision, diag = path_collision_predicted(
        ego_track=[], opp_track=[],
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=0.0, opp_speed_mps=10.0,
        lap_length_m=0.0,
    )
    # Critical: this returns FALSE (no brake) when data is missing, so brake
    # triggers fall through to their snapshot fallback rather than triggering.
    assert collision is False
    assert diag.get("reason") == "insufficient_data"


def test_collision_predicted_breach_debounce_filters_single_sample():
    """A single-sample sub-min_clear breach must NOT latch a collision.

    Construct ego and opp on parallel lines but with one segment of opp's
    polyline jagged enough to drop within min_clear at exactly one sample.
    """
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)
    # Side polyline: parallel at y=2.0 except one suture-glitch dip to y=1.0
    # at index ~12. With ego starting at s=10, sampling at 0.1s × 20 m/s
    # → reaches s=12 at t=0.1, returns to y=2.0 by t=0.2.
    side = []
    for i in range(200):
        x = float(i)
        y = 2.0 if i != 12 else 1.0  # one-sample lateral dip
        side.append((x, y, 0.0))
    collision, diag = path_collision_predicted(
        ego_track=side, opp_track=optimal,
        ego_s_m=10.0, ego_speed_mps=20.0,
        opp_s_m=10.0, opp_speed_mps=20.0,
        lap_length_m=199.0,
        sample_dt_s=0.1, min_clear_m=1.6,
        require_consecutive_breach=2,
    )
    # Even though min_clear momentarily dips below 1.6m at the suture, only
    # ONE sample breaches. Debounce requires 2 consecutive → no latch.
    assert collision is False, f"Single-sample dip should not latch; diag={diag}"
    # min_clear may or may not be below 1.6m here (Shapely interpolation may
    # smooth between segments). The KEY assertion is breach_count.
    assert diag["breach_count"] < 2


def test_collision_predicted_f3_style_parallel_ttl_no_collision():
    """F3L-style: ego on right TTL (y=-3), opp on optimal (y=0). Constant.
    Like F3L scenario where fellow holds left lane and ego passes right —
    must NOT predict collision."""
    optimal = _make_straight_polyline(0.0, 0.0, 1.0, 0.0, 200)
    right = _make_straight_polyline(0.0, -3.0, 1.0, 0.0, 200)
    collision, diag = path_collision_predicted(
        ego_track=right, opp_track=optimal,
        ego_s_m=20.0, ego_speed_mps=22.0,  # ego closing
        opp_s_m=22.0, opp_speed_mps=9.0,
        lap_length_m=199.0,
    )
    assert collision is False, f"F3-style parallel pass MUST be safe; diag={diag}"


def test_select_tracks_for_state_dispatch():
    """select_tracks_for_state returns (left, optimal) during left-side passes,
    (right, optimal) during right-side, (active_ttl, optimal) otherwise."""
    assert select_tracks_for_state("COMMIT_PASS_LEFT", "optimal") == ("left", "optimal")
    assert select_tracks_for_state("SETUP_PASS_RIGHT", "optimal") == ("right", "optimal")
    assert select_tracks_for_state("HOLD_PASS_LEFT", "left") == ("left", "optimal")
    assert select_tracks_for_state("FOLLOW", "optimal") == ("optimal", "optimal")
    assert select_tracks_for_state("FREE_RUN", "right") == ("right", "optimal")
    # Unknown / garbage active_ttl → defaults to "optimal".
    assert select_tracks_for_state("ABORT_PASS", "garbage") == ("optimal", "optimal")


def test_select_tracks_for_state_abort_pass_uses_active_ttl():
    """SD-7 regression: during ABORT_PASS, ego may still be physically on
    the side TTL (SD-2d keeps the commit-side TTL while side-by-side, for
    up to ~1s). PathPredict must walk the SIDE polyline, not optimal.
    Pre-SD-7, ABORT_PASS was treated as "ego on optimal" → wrong polyline
    → spurious collision predictions → user-visible parallel braking on F2.
    """
    # Right-side commit just aborted; ego still on right TTL (SD-2d hold).
    assert select_tracks_for_state("ABORT_PASS", "right") == ("right", "optimal"), (
        "ABORT_PASS with active_ttl='right' must use the right polyline"
    )
    # Left-side commit just aborted; ego still on left TTL (SD-2d hold).
    assert select_tracks_for_state("ABORT_PASS", "left") == ("left", "optimal"), (
        "ABORT_PASS with active_ttl='left' must use the left polyline"
    )
    # ABORT_PASS after lateral cleared (ttl_switch back to optimal already happened).
    assert select_tracks_for_state("ABORT_PASS", "optimal") == ("optimal", "optimal")
