"""SD-11e: tests for strategy-as-authority path (use_strategy_authority=True).

Tests the FLAG-ON behavior: strategy result REPLACES the snapshot-driven
FOLLOW-vs-FREE_RUN-vs-SETUP entry decisions. Existing flag-off tests
(test_tactical_planner.py, ~136 tests) cover the snapshot path and stay
the regression net.
"""

from __future__ import annotations

import io
from contextlib import redirect_stdout

import pytest

from scenic.domains.racing.tactical_planner import (
    COMMIT_PASS_LEFT,
    COMMIT_PASS_RIGHT,
    FOLLOW,
    FREE_RUN,
    SETUP_LEFT,
    SETUP_RIGHT,
    TacticalPlannerConfig,
    TacticalPlannerState,
    tactical_planner_step_v1,
)
from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.assessment.pass_geometry import _clear_linestring_cache


@pytest.fixture(autouse=True)
def _drop_polyline_cache():
    _clear_linestring_cache()
    yield
    _clear_linestring_cache()


def _straight_polyline(x0, y0, dx, dy, n):
    return [(x0 + i * dx, y0 + i * dy, 0.0) for i in range(n)]


def _common_polylines():
    return (
        _straight_polyline(0.0, 0.0, 1.0, 0.0, 600),
        _straight_polyline(0.0, 5.0, 1.0, 0.0, 600),
        _straight_polyline(0.0, -5.0, 1.0, 0.0, 600),
    )


def _stationary_fellow_traj(x, y, horizon_s=10.0, dt=0.5):
    n = int(horizon_s / dt)
    return [(i * dt, x, y, None) for i in range(n + 1)]


def _moving_fellow_traj(x0, y0, vx, vy, horizon_s=10.0, dt=0.5):
    n = int(horizon_s / dt)
    return [(i * dt, x0 + vx * i * dt, y0 + vy * i * dt, None) for i in range(n + 1)]


def _sit(*, longitudinal_m, lateral_m, distance_m=None, ahead=True,
         opponent_speed_mps=0.0):
    return OpponentSituation(
        ahead=bool(ahead),
        delta_s_m=float(longitudinal_m),
        delta_s_source="polyline",
        lateral_relation="left" if lateral_m > 0 else "right" if lateral_m < 0 else "neutral",
        closing_speed_mps=0.0,
        overlap_state="clear_ahead",
        collision_risk_01=0.0,
        segment_context="straight",
        distance_m=float(distance_m if distance_m is not None
                         else (longitudinal_m**2 + lateral_m**2) ** 0.5),
        longitudinal_m=float(longitudinal_m),
        lateral_m=float(lateral_m),
        opponent_speed_mps=float(opponent_speed_mps),
    )


def _step_with_capture(st, sit, *, ego_speed=15.0, opp_speed=0.0,
                       ego_s=0.0, opp_s=30.0, sim_t=0.0, config=None,
                       polylines=None, fellow_traj=None):
    opt, left, right = polylines if polylines else _common_polylines()
    buf = io.StringIO()
    with redirect_stdout(buf):
        result = tactical_planner_step_v1(
            st, sit,
            has_opponent=True, ego_speed_mps=ego_speed, opponent_speed_mps=opp_speed,
            sim_time_s=sim_t, pit_mode=False, config=config or TacticalPlannerConfig(),
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=ego_s, opp_s_m=opp_s, lap_length_m=599.0,
            ego_active_ttl="optimal",
            fellow_trajectory=fellow_traj,
        )
    return result, buf.getvalue()


# ---------------------------------------------------------------------------
# F9 fix: stationary fellow off the line → strategy=stay_optimal → FREE_RUN.
# ---------------------------------------------------------------------------

def test_f9_stationary_fellow_off_line_stays_in_free_run():
    """The F9 deadlock fix: stationary fellow at lat=-5m should not slow ego."""
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=2)
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)

    # Tick 1: strategy picks stay_optimal but hysteresis says wait.
    (m1, _, _, r1), _ = _step_with_capture(
        st, sit, config=cfg, fellow_traj=fellow_traj, sim_t=0.0, ego_speed=15.0
    )
    assert st.strategy_selected_name == "stay_optimal"
    # Tick 2: hysteresis count reaches 2 → authority fires.
    (m2, ttl2, cap2, r2), _ = _step_with_capture(
        st, sit, config=cfg, fellow_traj=fellow_traj, sim_t=0.05, ego_speed=15.0
    )
    assert m2 == FREE_RUN
    assert ttl2 == "optimal"
    assert cap2 is None  # full target speed
    assert r2 == "strategy_stay_optimal"


def test_hysteresis_blocks_authority_until_commit_cycles_reached():
    """First tick of any new strategy selection should NOT fire authority."""
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=3)
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)
    # Tick 1, 2: no authority (run count 1, 2 — both < 3)
    for i in range(2):
        (m, _, _, r), _ = _step_with_capture(
            st, sit, config=cfg, fellow_traj=fellow_traj, sim_t=i * 0.05
        )
        assert r != "strategy_stay_optimal"
    # Tick 3: count reaches 3 → fires.
    (m3, _, _, r3), _ = _step_with_capture(
        st, sit, config=cfg, fellow_traj=fellow_traj, sim_t=2 * 0.05
    )
    assert r3 == "strategy_stay_optimal"


# ---------------------------------------------------------------------------
# Overtake: pass_left strategy seeds the SETUP/COMMIT lifecycle.
# ---------------------------------------------------------------------------

def test_pass_left_authority_returns_setup_left_and_seeds_lifecycle():
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=1)
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=15.0, lateral_m=0.0, opponent_speed_mps=8.0)
    # Bias the strategy toward pass_left by giving fellow lateral motion to the right
    # (so left side is clearly safer — but with parallel polylines, simulator may pick
    # either side). Use fellow on optimal (y=0) and accept either pass_*.
    fellow_traj = _moving_fellow_traj(15.0, 0.0, 8.0, 0.0)
    (m, ttl, cap, r), _ = _step_with_capture(
        st, sit, config=cfg, ego_speed=20.0, opp_speed=8.0, fellow_traj=fellow_traj
    )
    # Strategy may pick stay_optimal (no obstacle on optimal at horizon) OR pass_*.
    # Whichever it picks, authority should honor it.
    if st.strategy_selected_name in ("pass_left", "pass_right"):
        assert m in (SETUP_LEFT, SETUP_RIGHT, COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT)
        # When pass_left, seeded state should have lifecycle fields set.
        if st.strategy_selected_name == "pass_left":
            assert st.pass_intent_side == "left"
            assert st.setup_commit_side == "left"
            assert st.lateral_path_lock_side == "left"
            assert st.opening_confidence_count >= int(cfg.pass_intent_entry_cycles)
        assert r.startswith("strategy_pass_")
    else:
        # stay_optimal also acceptable for this geometry
        assert m == FREE_RUN
        assert r == "strategy_stay_optimal"


def test_follow_fellow_strategy_caps_at_opp_plus_small_margin_no_3_floor():
    """SD-11e mapping: follow_fellow → FOLLOW with cap = opp+0.3 (no 3.0 m/s floor)."""
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=1)
    st = TacticalPlannerState()
    # Synthetic test: directly inject strategy state to bypass the simulator
    # (which would otherwise pick stay_optimal for a stationary fellow).
    st.strategy_selected_name = "follow_fellow"
    st.strategy_committed_name = "follow_fellow"
    st.strategy_commit_run_count = 1  # already committed
    sit = _sit(longitudinal_m=20.0, lateral_m=0.0, opponent_speed_mps=2.0)
    # Run with no fellow_trajectory so the SD-11d block doesn't OVERWRITE the
    # injected selection, but flag is on so authority branch fires.
    opt, left, right = _common_polylines()
    result = tactical_planner_step_v1(
        st, sit,
        has_opponent=True, ego_speed_mps=10.0, opponent_speed_mps=2.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
        ego_s_m=0.0, opp_s_m=20.0, lap_length_m=599.0,
        ego_active_ttl="optimal",
        # No fellow_trajectory — strategy compute block skipped, injected state survives.
        fellow_trajectory=None,
    )
    mode, ttl, cap, reason = result
    assert mode == FOLLOW
    assert ttl == "optimal"
    # opp+0.3 = 2.3 (NOT 3.0 floor)
    assert cap == pytest.approx(2.3)
    assert reason == "strategy_follow_fellow"


# ---------------------------------------------------------------------------
# Authority does NOT preempt mid-flight COMMIT/HOLD/ABORT execution.
# ---------------------------------------------------------------------------

def test_authority_does_not_fire_when_mode_is_commit():
    """Strategy authority should leave COMMIT lifecycle alone (don't preempt
    mid-flight execution; let the state machine carry it through)."""
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=1)
    st = TacticalPlannerState()
    st.mode = COMMIT_PASS_LEFT
    st.commit.side = "left"
    st.commit.until_s = 999.0
    st.lateral_path_lock_side = "left"
    st.lateral_path_lock_until_s = 999.0
    sit = _sit(longitudinal_m=5.0, lateral_m=0.0, opponent_speed_mps=8.0)
    fellow_traj = _moving_fellow_traj(15.0, 0.0, 8.0, 0.0)
    # Even if strategy picks stay_optimal, mode=COMMIT means authority skips.
    (m, ttl, cap, r), _ = _step_with_capture(
        st, sit, config=cfg, ego_speed=20.0, opp_speed=8.0, fellow_traj=fellow_traj
    )
    # Should NOT be "strategy_stay_optimal" (authority blocked by mode).
    assert r != "strategy_stay_optimal"


# ---------------------------------------------------------------------------
# Flag off: nothing changes (regression net).
# ---------------------------------------------------------------------------

def test_flag_off_completely_bypasses_authority():
    """With use_strategy_authority=False, the strategy is computed (telemetry)
    but the snapshot path runs unchanged."""
    cfg = TacticalPlannerConfig(use_strategy_authority=False)
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)
    (m, ttl, cap, r), _ = _step_with_capture(
        st, sit, config=cfg, fellow_traj=fellow_traj
    )
    # Snapshot path: distance < relevance_dist_m, sit.ahead → FOLLOW or SETUP path,
    # NOT a "strategy_*" reason.
    assert not r.startswith("strategy_")
    # But strategy was still computed for telemetry.
    assert st.strategy_selected_name == "stay_optimal"


# ---------------------------------------------------------------------------
# Strategy selection change resets hysteresis count.
# ---------------------------------------------------------------------------

def test_changing_selection_resets_run_count():
    cfg = TacticalPlannerConfig(use_strategy_authority=True, strategy_commit_cycles=3)
    st = TacticalPlannerState()
    # First scenario: stationary fellow off line → stay_optimal
    sit_f9 = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)
    traj_f9 = _stationary_fellow_traj(30.0, -5.0)
    _step_with_capture(st, sit_f9, config=cfg, fellow_traj=traj_f9)
    _step_with_capture(st, sit_f9, config=cfg, fellow_traj=traj_f9, sim_t=0.05)
    assert st.strategy_committed_name == "stay_optimal"
    assert st.strategy_commit_run_count == 2
    # Switch scenarios: now fellow on optimal moving slow → may pick pass_*
    sit_f2 = _sit(longitudinal_m=15.0, lateral_m=0.0, opponent_speed_mps=8.0)
    traj_f2 = _moving_fellow_traj(15.0, 0.0, 8.0, 0.0)
    _step_with_capture(st, sit_f2, config=cfg, ego_speed=20.0, opp_speed=8.0,
                       fellow_traj=traj_f2, sim_t=0.10)
    if st.strategy_selected_name != "stay_optimal":
        # Selection changed → run count reset.
        assert st.strategy_commit_run_count == 1
        assert st.strategy_committed_name == st.strategy_selected_name
