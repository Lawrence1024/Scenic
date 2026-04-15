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
    tactical_planner_step_v1,
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
    assert m == FOLLOW and ttl == "optimal" and cap is not None
    m2, ttl2, cap2 = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=35.0, opponent_speed_mps=30.0,
        sim_time_s=0.05, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m2 == FOLLOW and ttl2 == "optimal" and cap2 is not None
    m3, ttl3, cap3 = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=35.0, opponent_speed_mps=30.0,
        sim_time_s=0.10, pit_mode=False, config=TacticalPlannerConfig(),
    )
    assert m3 == SETUP_LEFT
    assert ttl3 == "left"
    assert cap3 is None


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


def test_relation_behind_but_close_proximity_stays_follow():
    st = TacticalPlannerState()
    s = _sit(
        lateral_relation="aligned",
        collision_risk_01=0.2,
        segment_context="straight",
        overlap_state="side_by_side",
        distance_m=12.0,
        longitudinal_m=-1.0,
        ahead=False,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=28.0,
        opponent_speed_mps=24.0,
        sim_time_s=3.0,
        pit_mode=False,
        config=TacticalPlannerConfig(),
        assessment_relation="behind",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=False,
    )
    assert m == FOLLOW and ttl == "optimal" and cap is not None
    assert reason in ("proximity_hazard_follow", "contact_recovery_hold")


def test_closing_flag_blocks_setup_entry():
    st = TacticalPlannerState()
    s = _sit(
        lateral_relation="right",
        collision_risk_01=0.05,
        segment_context="straight",
        overlap_state="clear_ahead",
        distance_m=30.0,
        longitudinal_m=20.0,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=25.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=TacticalPlannerConfig(),
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.25,
    )
    assert m == FOLLOW and ttl == "optimal" and cap is not None
    assert reason == "protected_follow_envelope"


def test_follow_pressure_hold_blocks_setup_even_when_side_open():
    st = TacticalPlannerState()
    cfg = TacticalPlannerConfig()
    s = _sit(
        lateral_relation="right",
        collision_risk_01=0.1,
        segment_context="straight",
        overlap_state="clear_ahead",
        distance_m=10.0,
        longitudinal_m=9.0,
    )
    # Simulate sustained pressure window (tight/unsafe gap) and ensure setup is blocked.
    for i in range(3):
        m, ttl, cap, reason = tactical_planner_step_v1(
            st,
            s,
            has_opponent=True,
            ego_speed_mps=30.0,
            opponent_speed_mps=24.0,
            sim_time_s=0.05 * i,
            pit_mode=False,
            config=cfg,
            assessment_relation="ahead",
            assessment_gap_ok=False,
            assessment_optimal_open=False,
            assessment_left_open=True,
            assessment_right_open=True,
        )
        assert m == FOLLOW and ttl == "optimal" and cap is not None
    # Even when gap briefly appears OK, planner should still hold FOLLOW due to pressure latch.
    m2, ttl2, cap2, reason2 = tactical_planner_step_v1(
        st,
        _sit(
            lateral_relation="right",
            collision_risk_01=0.05,
            segment_context="straight",
            overlap_state="clear_ahead",
            distance_m=24.0,
            longitudinal_m=22.0,
        ),
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.25,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=True,
    )
    assert m2 == FOLLOW and ttl2 == "optimal" and cap2 is not None
    assert reason2 in ("follow_pressure_hold", "setup_candidate_collect", "protected_follow_envelope")


def test_protected_follow_envelope_blocks_free_run_when_not_blocked():
    st = TacticalPlannerState()
    s = _sit(
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        segment_context="straight",
        distance_m=65.0,
        longitudinal_m=50.0,  # Not blocked by geometric threshold alone.
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=29.0,
        opponent_speed_mps=23.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=TacticalPlannerConfig(),
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.3,
    )
    assert m == FOLLOW and ttl == "optimal" and cap is not None
    assert reason == "protected_follow_envelope"


def test_contact_recovery_hold_persists_after_overlap_clears():
    st = TacticalPlannerState()
    cfg = TacticalPlannerConfig(contact_recovery_hold_s=0.5, protected_follow_release_cycles=1)
    s_overlap = _sit(
        overlap_state="side_by_side",
        collision_risk_01=0.2,
        segment_context="straight",
        distance_m=12.0,
        longitudinal_m=3.0,
    )
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st,
        s_overlap,
        has_opponent=True,
        ego_speed_mps=24.0,
        opponent_speed_mps=20.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=False,
    )
    assert m0 == FOLLOW and ttl0 == "optimal" and cap0 is not None
    assert reason0 == "contact_recovery_hold"

    s_clear = _sit(
        ahead=False,
        overlap_state="clear_ahead",
        distance_m=35.0,
        longitudinal_m=-5.0,
    )
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st,
        s_clear,
        has_opponent=True,
        ego_speed_mps=24.0,
        opponent_speed_mps=20.0,
        sim_time_s=0.2,
        pit_mode=False,
        config=cfg,
        assessment_relation="behind",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
    )
    assert m1 == FOLLOW and ttl1 == "optimal" and cap1 is not None
    assert reason1 == "contact_recovery_hold"


def test_protected_follow_releases_into_setup_when_opening_stably_clear():
    st = TacticalPlannerState()
    cfg = TacticalPlannerConfig(
        protected_follow_release_cycles=2,
        setup_reentry_cooldown_s=0.0,
    )
    s_hazard = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.2,
        distance_m=26.0,
        longitudinal_m=22.0,
        ahead=True,
    )
    # First enter protected FOLLOW due to unsafe/closing pressure.
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st,
        s_hazard,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.35,
    )
    assert m0 == FOLLOW and ttl0 == "optimal" and cap0 is not None
    assert reason0 == "protected_follow_envelope"

    s_open = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=30.0,
        longitudinal_m=24.0,
        ahead=True,
    )
    # Stable open window should release the protected-follow latch.
    m1, ttl1, cap1, _reason1 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.05,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m1 == FOLLOW and ttl1 == "optimal" and cap1 is not None
    m2, ttl2, cap2, _reason2 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.10,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m2 == FOLLOW and ttl2 == "optimal" and cap2 is not None
    # After latch release, setup entry persistence still applies.
    m3, ttl3, cap3, reason3 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.15,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m3 == FOLLOW and ttl3 == "optimal" and cap3 is not None
    assert reason3 in ("setup_candidate_collect", "setup_candidate_reset")
    m4, ttl4, cap4, reason4 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.20,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    if m4 == FOLLOW:
        assert ttl4 == "optimal" and cap4 is not None
        assert reason4 in ("setup_candidate_collect", "setup_candidate_reset")
        m5, ttl5, cap5, reason5 = tactical_planner_step_v1(
            st,
            s_open,
            has_opponent=True,
            ego_speed_mps=30.0,
            opponent_speed_mps=24.0,
            sim_time_s=0.25,
            pit_mode=False,
            config=cfg,
            assessment_relation="ahead",
            assessment_gap_ok=True,
            assessment_optimal_open=False,
            assessment_left_open=True,
            assessment_right_open=False,
            assessment_closing_flag=False,
            assessment_emergency_risk_01=0.05,
        )
        assert m5 == SETUP_LEFT
        assert ttl5 == "left"
        assert cap5 is None
        assert reason5 in ("setup_left_open", "setup_flip_cooldown_hold")
    else:
        assert m4 == SETUP_LEFT
        assert ttl4 == "left"
        assert cap4 is None
        assert reason4 in ("setup_left_open", "setup_flip_cooldown_hold", "setup_commit_left_hold")


def test_setup_commit_hold_keeps_setup_during_moderate_pressure():
    st = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(
        pass_intent_entry_cycles=1,
        setup_commit_entry_cycles=1,
        setup_commit_hold_s=0.5,
        follow_tight_headway_s=0.5,
    )
    s_open = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=1.5,
        ahead=True,
    )
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=25.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m0 == SETUP_LEFT and ttl0 == "left" and cap0 is None
    assert reason0 in ("setup_commit_left_hold", "setup_left_open")

    # Moderate pressure (gap temporarily not ok), but opening remains asymmetric and clear.
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=25.0,
        sim_time_s=0.2,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m1 == SETUP_LEFT and ttl1 == "left" and cap1 is None
    assert reason1 == "setup_commit_left_hold"


def test_setup_commit_cancels_on_hard_hazard():
    st = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(
        pass_intent_entry_cycles=1,
        setup_commit_entry_cycles=1,
        setup_commit_hold_s=1.0,
        follow_tight_headway_s=0.5,
    )
    s_open = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=1.2,
        ahead=True,
    )
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=25.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m0 == SETUP_LEFT and ttl0 == "left" and cap0 is None
    assert reason0 in ("setup_commit_left_hold", "setup_left_open")

    # Side-by-side overlap is a hard hazard; commit hold must be dropped.
    s_hazard = _sit(
        lateral_relation="right",
        overlap_state="side_by_side",
        collision_risk_01=0.2,
        distance_m=12.0,
        longitudinal_m=3.0,
        closing_speed_mps=0.6,
        ahead=True,
    )
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st,
        s_hazard,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=25.0,
        sim_time_s=0.1,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.35,
    )
    assert m1 == FOLLOW and ttl1 == "optimal" and cap1 is not None
    assert reason1 in ("contact_recovery_hold", "protected_follow_envelope")


def test_pass_intent_commit_arms_from_follow_and_enters_setup():
    st = TacticalPlannerState(mode=FOLLOW, last_setup_side="left")
    cfg = TacticalPlannerConfig(
        pass_intent_entry_cycles=2,
        pass_intent_hold_s=0.8,
        setup_commit_entry_cycles=1,
        setup_commit_hold_s=0.8,
        setup_reentry_cooldown_s=0.0,
    )
    s_open = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.08,
        distance_m=28.0,
        longitudinal_m=22.0,
        closing_speed_mps=1.2,
        ahead=True,
    )
    # First cycle: collect intent, still in FOLLOW.
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m0 == FOLLOW and ttl0 == "optimal" and cap0 is not None
    assert reason0 in ("setup_candidate_collect", "setup_candidate_reset")

    # Second cycle: intent reaches threshold, commit hold should force setup-left.
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st,
        s_open,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=0.05,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m1 == SETUP_LEFT and ttl1 == "left" and cap1 is None
    assert reason1 == "setup_commit_left_hold"


def test_lateral_path_lock_holds_setup_during_protected_follow():
    st = TacticalPlannerState(
        mode=FOLLOW,
        protected_follow_active=True,
        lateral_path_lock_side="left",
        lateral_path_lock_until_s=5.0,
    )
    cfg = TacticalPlannerConfig(follow_tight_headway_s=0.5)
    s = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.08,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=1.0,
        ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m == SETUP_LEFT and ttl == "left" and cap is None
    assert reason == "lateral_path_lock_left_hold"
