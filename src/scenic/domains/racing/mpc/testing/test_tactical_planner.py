"""Tests for Phase 3 tactical planner."""

from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import (
    FOLLOW,
    FREE_RUN,
    SETUP_LEFT,
    SETUP_RIGHT,
    TacticalPlannerConfig,
    TacticalPlannerState,
    tactical_planner_step,
)


def _sit(**kwargs):
    defaults = dict(
        ahead=True,
        delta_s_m=15.0,
        delta_s_source="heading_proxy",
        lateral_relation="on_line",
        closing_speed_mps=2.0,
        overlap_state="clear_ahead",
        collision_risk_01=0.2,
        segment_context="straight",
        distance_m=18.0,
        longitudinal_m=15.0,
        lateral_m=1.0,
    )
    defaults.update(kwargs)
    return OpponentSituation(**defaults)


def test_free_run_no_opponent():
    st = TacticalPlannerState()
    m, ttl, cap = tactical_planner_step(
        st, None, has_opponent=False, ego_speed_mps=30.0, opponent_speed_mps=0.0,
        sim_time_s=0.0, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m == FREE_RUN and ttl == "optimal" and cap is None


def test_free_run_opponent_far():
    st = TacticalPlannerState()
    s = _sit(distance_m=120.0, longitudinal_m=80.0)
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=30.0, opponent_speed_mps=25.0,
        sim_time_s=0.0, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m == FREE_RUN


def test_follow_when_blocked_not_safe():
    st = TacticalPlannerState()
    s = _sit(
        segment_context="corner_body",
        collision_risk_01=0.9,
        distance_m=30.0,
        longitudinal_m=20.0,
    )
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=40.0, opponent_speed_mps=28.0,
        sim_time_s=0.0, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m == FOLLOW
    assert ttl == "optimal"
    assert cap is not None and cap < 40.0


def test_setup_left_when_opponent_on_right():
    st = TacticalPlannerState()
    s = _sit(
        lateral_relation="right",
        collision_risk_01=0.1,
        segment_context="straight",
        overlap_state="clear_ahead",
        distance_m=35.0,
        longitudinal_m=25.0,
    )
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=35.0, opponent_speed_mps=30.0,
        sim_time_s=0.0, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m == SETUP_LEFT
    assert ttl == "left"
    assert cap is None


def test_pit_forces_free_run():
    st = TacticalPlannerState()
    st.mode = FOLLOW
    s = _sit()
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=10.0, opponent_speed_mps=8.0,
        sim_time_s=0.0, pit_mode=True, config=TacticalPlannerConfig(),
    )
    assert m == FREE_RUN and ttl == "optimal"


def test_setup_flip_cooldown():
    st = TacticalPlannerState()
    st.mode = SETUP_LEFT
    st.last_flip_sim_time_s = 0.0
    cfg = TacticalPlannerConfig(setup_flip_cooldown_s=100.0)
    s = _sit(lateral_relation="right", collision_risk_01=0.1, longitudinal_m=25.0, distance_m=35.0)
    m, ttl, _ = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=35.0, opponent_speed_mps=30.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
    )
    assert m == SETUP_LEFT
    assert ttl == "left"


def test_partial_overlap_forces_follow_even_when_risk_low():
    st = TacticalPlannerState()
    s = _sit(
        overlap_state="partial_overlap",
        collision_risk_01=0.05,
        segment_context="straight",
        distance_m=20.0,
        longitudinal_m=18.0,
    )
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=28.0, opponent_speed_mps=24.0,
        sim_time_s=1.0, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m == FOLLOW and ttl == "optimal" and cap is not None


def test_setup_reentry_cooldown_after_setup_exit():
    st = TacticalPlannerState(mode=SETUP_RIGHT)
    cfg = TacticalPlannerConfig(setup_reentry_cooldown_s=2.0)
    # First call: unsafe overlap kicks us out of setup and records setup exit.
    s_bad = _sit(
        overlap_state="partial_overlap",
        collision_risk_01=0.2,
        segment_context="straight",
        distance_m=10.0,
        longitudinal_m=9.0,
    )
    m0, ttl0, cap0 = tactical_planner_step(
        st, s_bad, has_opponent=True, ego_speed_mps=26.0, opponent_speed_mps=22.0,
        sim_time_s=5.0, pit_mode=False, config=cfg,
    )
    assert m0 == FOLLOW and ttl0 == "optimal" and cap0 is not None
    # Second call: conditions are now pass-safe, but cooldown should still hold FOLLOW.
    s_good = _sit(
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        segment_context="straight",
        distance_m=24.0,
        longitudinal_m=22.0,
        lateral_relation="right",
    )
    m1, ttl1, cap1 = tactical_planner_step(
        st, s_good, has_opponent=True, ego_speed_mps=26.0, opponent_speed_mps=22.0,
        sim_time_s=6.0, pit_mode=False, config=cfg,
    )
    assert m1 == FOLLOW and ttl1 == "optimal" and cap1 is not None
