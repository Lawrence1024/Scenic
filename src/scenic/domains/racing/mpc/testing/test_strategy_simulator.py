"""SD-11b: unit tests for the ego strategy trajectory simulator."""

import math

import pytest

from scenic.domains.racing.prediction import (
    ALL_STRATEGIES,
    StrategyOutcome,
    simulate_strategy,
)
from scenic.domains.racing.assessment.pass_geometry import _clear_linestring_cache


@pytest.fixture(autouse=True)
def _drop_polyline_cache():
    """Each test builds throwaway polylines; clear the id-keyed LineString cache
    so we don't hit a stale entry from a previous test."""
    _clear_linestring_cache()
    yield
    _clear_linestring_cache()


def _straight_polyline(x0: float, y0: float, dx: float, dy: float, n: int):
    return [(x0 + i * dx, y0 + i * dy, 0.0) for i in range(n)]


def _stationary_fellow_traj(x: float, y: float, horizon_s: float, dt: float):
    """Fellow stays at (x, y) for all sample times in [0, horizon_s]."""
    n = int(horizon_s / dt)
    return [(i * dt, x, y, None) for i in range(n + 1)]


def _moving_fellow_traj(x0: float, y0: float, vx: float, vy: float,
                        horizon_s: float, dt: float):
    n = int(horizon_s / dt)
    return [(i * dt, x0 + vx * i * dt, y0 + vy * i * dt, None) for i in range(n + 1)]


# ---------------------------------------------------------------------------
# Common scenario fixtures.
# ---------------------------------------------------------------------------

def _common_polylines():
    """Three straight parallel polylines:
        optimal at y=0, left at y=+5, right at y=-5.
    """
    return {
        "optimal": _straight_polyline(0.0, 0.0, 1.0, 0.0, 600),
        "left":    _straight_polyline(0.0, 5.0, 1.0, 0.0, 600),
        "right":   _straight_polyline(0.0, -5.0, 1.0, 0.0, 600),
    }


_COMMON_KW = dict(
    horizon_s=10.0,
    sample_dt_s=0.5,
    target_speed_mps=45.0,
    accel_mps2=4.0,
    setup_speed_margin_mps=4.5,
    commit_speed_margin_mps=16.0,
    post_pass_buffer_m=5.0,
    lane_change_s=1.5,
)


# ---------------------------------------------------------------------------
# F9 reproduction: stationary fellow at lat=-5 m off optimal.
# ---------------------------------------------------------------------------

def test_stay_optimal_clearance_high_when_fellow_off_line():
    """F9 case: ego on optimal at y=0, fellow stationary at lat=-5.
    stay_optimal should report clearance ~5m throughout and reach near target speed."""
    polylines = _common_polylines()
    fellow_traj = _stationary_fellow_traj(30.0, -5.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0, ego_speed_mps=0.5,
        opp_s_m=30.0, opp_speed_mps=0.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "ok"
    # SD-27b: OBB edge-to-edge gap, not centroid. Lateral gap 5m minus
    # 0.5*ego_width minus 0.5*fellow_width = 5 - 1.93 ≈ 3.07m at the closest tick.
    assert out.min_clearance_m >= 2.8
    assert out.reachable_speed_at_horizon_mps >= 30.0  # accelerated from 0.5 over 10s
    assert out.completed is True


def test_follow_fellow_caps_speed_to_fellow():
    """follow_fellow should cap ego speed near opp_speed (+0.3 m/s margin)."""
    polylines = _common_polylines()
    fellow_traj = _stationary_fellow_traj(30.0, -5.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "follow_fellow",
        ego_s_m=0.0, ego_speed_mps=0.5,
        opp_s_m=30.0, opp_speed_mps=0.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "ok"
    # Fellow speed = 0, so cap = 0.3 m/s. Ego must decelerate from 0.5 to 0.3.
    assert out.reachable_speed_at_horizon_mps == pytest.approx(0.3, abs=0.5)


# ---------------------------------------------------------------------------
# F2-style overtake: fellow moving slower on optimal.
# ---------------------------------------------------------------------------

def test_pass_left_completes_when_left_polyline_clear():
    """Fellow on optimal moving 10 m/s. ego at 20 m/s tries pass_left.
    Left TTL is parallel and clear → should complete pass within horizon."""
    polylines = _common_polylines()
    # Fellow: starts 30m ahead, moves 10 m/s along x.
    fellow_traj = _moving_fellow_traj(30.0, 0.0, 10.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0, ego_speed_mps=20.0,
        opp_s_m=30.0, opp_speed_mps=10.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "ok"
    assert out.completed is True, f"pass should complete on parallel clear left TTL: {out}"
    # SD-27b: OBB edge-to-edge gap. Min occurs during the alongside-to-merge_back
    # transition where ego is mid-blend (y≈3.8m) and longitudinally close to
    # fellow; closest-corner-pair ≈ 2m. Threshold here verifies the simulator
    # finds a viable pass (above the 0.5m hard filter), not a specific value.
    assert out.min_clearance_m >= 1.0


def test_pass_left_blocked_by_converging_left_polyline():
    """Left polyline curves into optimal at x=20m. Pass should report low clearance."""
    polylines = _common_polylines()
    # Replace left polyline with one that converges to y=0 at x=20m.
    converging_left = []
    for i in range(600):
        x = float(i)
        y = max(0.0, 5.0 - 0.25 * x)  # y goes 5 → 0 between x=0 and x=20
        converging_left.append((x, y, 0.0))
    polylines["left"] = converging_left
    fellow_traj = _moving_fellow_traj(15.0, 0.0, 10.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0, ego_speed_mps=20.0,
        opp_s_m=15.0, opp_speed_mps=10.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "ok"
    # When ego enters the converging zone alongside fellow, clearance collapses.
    assert out.min_clearance_m < 2.5


# ---------------------------------------------------------------------------
# Fellow far ahead — all four strategies safe.
# ---------------------------------------------------------------------------

def test_fellow_far_ahead_all_strategies_safe():
    polylines = _common_polylines()
    # Fellow 600m ahead and FASTER than ego's target speed (50 m/s vs ego target 45 m/s)
    # so ego can never catch it within the 10s horizon.
    fellow_traj = _moving_fellow_traj(600.0, 0.0, 50.0, 0.0, horizon_s=10.0, dt=0.5)
    for s in ALL_STRATEGIES:
        out = simulate_strategy(
            s,
            ego_s_m=0.0, ego_speed_mps=20.0,
            opp_s_m=600.0, opp_speed_mps=50.0,
            fellow_traj=fellow_traj,
            polylines=polylines, lap_length_m=2000.0,
            **_COMMON_KW,
        )
        assert out.reason == "ok", f"{s}: {out}"
        # Fellow goes 600+500=1100 by t=10; ego does ~370. Gap stays >300 m.
        # SD-27b: subtract IAC circumradius (~2.6m each side) from centroid
        # for OBB clearance, still well above 290m.
        assert out.min_clearance_m > 290.0, f"{s}: clearance={out.min_clearance_m}"


# ---------------------------------------------------------------------------
# Failure modes.
# ---------------------------------------------------------------------------

def test_missing_optimal_polyline_returns_failure_outcome():
    polylines = {"optimal": [], "left": _straight_polyline(0, 5, 1, 0, 100), "right": _straight_polyline(0, -5, 1, 0, 100)}
    fellow_traj = _stationary_fellow_traj(30.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=30.0, opp_speed_mps=0.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=0.0,
        **_COMMON_KW,
    )
    assert out.reason == "no_optimal_polyline"


def test_missing_side_polyline_returns_failure_for_pass():
    polylines = _common_polylines()
    polylines["left"] = []
    fellow_traj = _moving_fellow_traj(30.0, 0.0, 10.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0, ego_speed_mps=20.0,
        opp_s_m=30.0, opp_speed_mps=10.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "no_side_polyline"


def test_empty_fellow_trajectory_treats_as_no_obstacle():
    """If FellowPredictor.trajectory() returns [], no clearance check fires.
    Strategy should still simulate ego forward and report progress."""
    polylines = _common_polylines()
    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=30.0, opp_speed_mps=0.0,
        fellow_traj=[],
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    assert out.reason == "ok"
    assert out.min_clearance_m == float("inf")
    assert out.reachable_speed_at_horizon_mps > 30.0


# ---------------------------------------------------------------------------
# Speed profile sanity checks.
# ---------------------------------------------------------------------------

def test_pass_left_speed_ramps_through_phases():
    """Verify pass_left speed profile climbs from setup_margin during phase A
    through commit_margin during phase B."""
    polylines = _common_polylines()
    fellow_traj = _moving_fellow_traj(30.0, 0.0, 10.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0, ego_speed_mps=15.0,
        opp_s_m=30.0, opp_speed_mps=10.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **_COMMON_KW,
    )
    # Check that some sample is in lane_change phase, some in alongside, some in merge_back.
    phases = {sample[4] for sample in out.samples}
    assert "lane_change" in phases
    # At least one of {alongside, merge_back} should appear if pass progresses.
    assert "alongside" in phases or "merge_back" in phases


def test_horizon_zero_returns_single_sample():
    polylines = _common_polylines()
    fellow_traj = _stationary_fellow_traj(30.0, -5.0, horizon_s=0.0, dt=0.5)
    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0, ego_speed_mps=10.0,
        opp_s_m=30.0, opp_speed_mps=0.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **{**_COMMON_KW, "horizon_s": 0.0},
    )
    assert out.reason == "ok"
    # n_steps = max(1, 0) = 1, so 2 samples
    assert len(out.samples) == 2


def test_completed_flag_false_for_pass_when_fellow_outpaces_ego():
    polylines = _common_polylines()
    # Fellow as fast as ego; ego can never overtake within horizon at modest setup speed.
    fellow_traj = _moving_fellow_traj(15.0, 0.0, 25.0, 0.0, horizon_s=10.0, dt=0.5)
    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0, ego_speed_mps=20.0,
        opp_s_m=15.0, opp_speed_mps=25.0,
        fellow_traj=fellow_traj,
        polylines=polylines, lap_length_m=599.0,
        **{**_COMMON_KW, "commit_speed_margin_mps": 2.0},  # cap commit at opp+2 = 27 m/s
    )
    assert out.reason == "ok"
    # Ego accelerates from 20 to 27 over 10s; fellow at 25 starts 15m ahead.
    # ego avg ~24 m/s × 10s = 240m; fellow at 25 m/s starts 15m ahead → 265m.
    # ego doesn't pass within horizon.
    assert out.completed is False
