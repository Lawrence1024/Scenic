"""Unit tests for SD-3a pass_window_check geometric look-ahead."""

import math

from scenic.domains.racing.assessment import pass_window_check


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
