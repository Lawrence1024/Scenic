"""Tests for Phase 3 tactical planner."""

from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import (
    ABORT_PASS,
    COMMIT_PASS_LEFT,
    COMMIT_PASS_RIGHT,
    FOLLOW,
    FREE_RUN,
    HOLD_PASS_LEFT,
    HOLD_PASS_RIGHT,
    SETUP_LEFT,
    SETUP_RIGHT,
    CommitPlannerState,
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
        opponent_speed_mps=20.0,
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
        longitudinal_m=20.0,
        closing_speed_mps=5.0,  # SD-2e: realistic overtake closing speed
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
    # SETUP now caps speed to opponent + margin when opponent is ahead
    assert cap3 is not None
    assert cap3 <= 35.0  # SD-2f: opponent_speed (30) + setup_speed_margin (4.5) = 34.5


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
    s = _sit(lateral_relation="right", collision_risk_01=0.1, longitudinal_m=20.0, distance_m=35.0,
             closing_speed_mps=5.0)  # SD-2e: realistic overtake closing speed
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
    # Use both corridors closed (symmetric blockage) so safety_pressure fires.
    # An asymmetric opening (left_open ^ right_open) would suppress safety_pressure
    # because the fellow is on a parallel TTL and is not a collision threat.
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
        assessment_left_open=False,
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
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=5.0,  # SD-2e: realistic overtake closing speed
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
        assert cap5 is not None  # SETUP caps speed when opponent ahead
        assert reason5 in ("setup_left_open", "setup_flip_cooldown_hold")
    else:
        assert m4 == SETUP_LEFT
        assert ttl4 == "left"
        assert cap4 is not None  # SETUP caps speed when opponent ahead
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
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
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
    assert m0 == SETUP_LEFT and ttl0 == "left" and cap0 is not None
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
    assert m1 == SETUP_LEFT and ttl1 == "left" and cap1 is not None
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
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
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
    assert m0 == SETUP_LEFT and ttl0 == "left" and cap0 is not None
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
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
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
    assert m1 == SETUP_LEFT and ttl1 == "left" and cap1 is not None
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
    assert m == SETUP_LEFT and ttl == "left" and cap is not None
    assert reason == "lateral_path_lock_left_hold"


def test_commit_from_setup_chain_left():
    st = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,  # bypass speed cap — this test covers chain logic
        # SD-3b: bypass setup+commit gap gates — this test covers chain logic
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        setup_commit_entry_cycles=1,
        pass_intent_entry_cycles=1,
        setup_reentry_cooldown_s=0.0,
        follow_tight_headway_s=0.5,
    )
    s = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.08,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
        ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
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
        assessment_emergency_risk_01=0.08,
    )
    # SD-3f: COMMIT cap = opp_speed + commit_speed_margin_mps (24 + 16 = 40).
    # Raised from SD-2e's 8 m/s to 16 so the pass actually completes within
    # commit_hold_s (the SD-2e value left ego running parallel forever).
    assert m == COMMIT_PASS_LEFT and ttl == "left"
    assert cap is not None
    assert abs(cap - 40.0) < 0.01
    assert reason == "commit_pass_left"
    assert st.commit.trigger == "setup_chain_commit_left"
    assert st.commit.post_event_state == COMMIT_PASS_LEFT


def test_commit_abort_on_commit_hazard():
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        lateral_relation="right",
        overlap_state="side_by_side",
        collision_risk_01=0.6,
        distance_m=11.0,
        longitudinal_m=2.5,
        closing_speed_mps=2.5,
        ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.7,
    )
    # SD-2d: while ego is still side-by-side (relation_ahead, lateral_m=1.0 < 3.0),
    # ABORT keeps the commit-side TTL ("left") instead of reverting to optimal.
    # Reverting at this point would steer ego LATERALLY across the fellow, which
    # is the bug observed in F2_tactical at t=7.35s (right→optimal contact).
    assert m == ABORT_PASS and ttl == "left" and cap is None
    assert reason == "abort_commit_invalidated"
    assert st.commit.abort_trigger == "commit_invalidated_hazard"
    assert st.commit.post_event_state == ABORT_PASS


def test_commit_abort_reverts_to_optimal_when_laterally_clear():
    """SD-2d boundary: abort SHOULD return to optimal once laterally clear.

    The keep-commit-side rule only applies inside abort_keep_ttl_lat_m (=3.0).
    A fellow that has drifted ≥ 3 m off-axis is not in the swerve-into-it zone,
    so reverting to optimal is safe and lets ego rejoin the racing line.
    """
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    st.commit.side = "left"
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        lateral_relation="right",
        overlap_state="side_by_side",
        collision_risk_01=0.6,
        distance_m=11.0,
        longitudinal_m=2.5,
        lateral_m=3.4,  # ≥ abort_keep_ttl_lat_m=3.0 → no swerve-into-it risk
        closing_speed_mps=2.5,
        ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=31.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.7,
    )
    assert m == ABORT_PASS and ttl == "optimal" and cap is None
    assert reason == "abort_commit_invalidated"


def test_commit_does_not_abort_on_stationary_offaxis_overlap():
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        stationary_overlap_relief_enabled=True,
    )
    s = _sit(
        ahead=True,
        overlap_state="partial_overlap",
        collision_risk_01=0.10,
        distance_m=20.0,
        longitudinal_m=6.0,
        lateral_m=3.6,
        closing_speed_mps=0.0,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=0.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    # SD-3f: COMMIT cap = opp_speed + commit_speed_margin_mps (0 + 16 = 16 m/s).
    assert m == COMMIT_PASS_LEFT and ttl == "left"
    assert cap is not None and abs(cap - 16.0) < 0.01
    assert reason == "commit_pass_left_hold"
    assert st.commit.abort_trigger == "none"


def test_commit_pass_success_returns_free_run():
    """SD-3d: when fellow is laterally clear (|lateral_m| ≥ merge_safe_lat_m),
    COMMIT success goes directly to FREE_RUN — no HOLD needed.

    Covers F6/F7-style parallel-TTL passes where ego cleared laterally during
    the COMMIT itself.
    """
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        ahead=False,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=35.0,
        longitudinal_m=-6.0,
        # SD-3d: laterally clear → skip HOLD, straight to FREE_RUN.
        lateral_m=3.0,
        closing_speed_mps=0.0,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=2.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="behind",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m == FREE_RUN and ttl == "optimal" and cap is None
    assert reason == "pass_success_free_run"
    assert st.commit.pass_success is True
    assert st.commit.post_event_state == FREE_RUN


def test_commit_abort_success_recovers_to_follow_when_pressure_clears():
    st = TacticalPlannerState(mode=ABORT_PASS)
    st.commit.abort_until_s = 0.0
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        ahead=True,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=30.0,
        longitudinal_m=20.0,
        closing_speed_mps=0.2,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=28.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m == FOLLOW and ttl == "optimal" and cap is not None
    assert reason == "abort_success_follow"
    assert st.commit.abort_success is True
    assert st.commit.post_event_state == FOLLOW


def test_commit_does_not_free_run_on_large_gap_while_still_ahead_if_closing():
    st = TacticalPlannerState()
    cfg_on = TacticalPlannerConfig(commit_abort_enabled=True, follow_tight_headway_s=0.5)
    s = _sit(
        ahead=True,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=95.0,
        longitudinal_m=25.0,
        closing_speed_mps=0.5,
        segment_context="straight",
    )
    m_on, _ttl_on, _cap_on, reason_on = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg_on,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.1,
    )
    assert reason_on != "opponent_not_blocking"

    st2 = TacticalPlannerState()
    cfg_off = TacticalPlannerConfig(commit_abort_enabled=False)
    m_off, _ttl_off, _cap_off, reason_off = tactical_planner_step_v1(
        st2,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg_off,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.1,
    )
    assert m_off == FREE_RUN and reason_off == "opponent_not_blocking"


def test_commit_ttl_clear_does_not_fire_while_fellow_still_ahead():
    """TTL-clear path must NOT fire while the fellow is still ahead, even after
    commit_success_time_s elapses.  Ego must hold the commit until the fellow is
    physically behind (relation_ahead=False), at which point free_run success fires."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_success_time_s=1.0,
        commit_hold_s=0.5,
    )
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    s_ahead = _sit(
        ahead=True,
        lateral_relation="right",  # fellow is to the right; ego took left
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=30.0,
        longitudinal_m=20.0,
        closing_speed_mps=0.2,
    )

    # Simulate several cycles with fellow always ahead and left side open.
    # Before commit_success_time_s elapses we should stay in COMMIT.
    m0, ttl0, cap0, reason0 = tactical_planner_step_v1(
        st, s_ahead,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m0 == COMMIT_PASS_LEFT and ttl0 == "left"

    # Still before threshold — hold.
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st, s_ahead,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=0.5, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m1 == COMMIT_PASS_LEFT and ttl1 == "left"

    # Past commit_success_time_s=1.0 but fellow STILL ahead — commit must hold.
    m2, ttl2, cap2, reason2 = tactical_planner_step_v1(
        st, s_ahead,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=1.1, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m2 == COMMIT_PASS_LEFT and ttl2 == "left"
    assert st.commit.pass_success is False

    # Fellow drops behind — free_run success fires.
    # SD-3d: lateral_m=3.0 > merge_safe_lat_m=2.5 → already laterally clear → skip HOLD.
    s_behind = _sit(
        ahead=False,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=30.0,
        longitudinal_m=-5.0,
        lateral_m=3.0,
        closing_speed_mps=0.0,
    )
    m3, ttl3, cap3, reason3 = tactical_planner_step_v1(
        st, s_behind,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=1.2, pit_mode=False, config=cfg,
        assessment_relation="behind",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m3 == FREE_RUN and ttl3 == "optimal"
    assert reason3 == "pass_success_free_run"
    assert st.commit.pass_success is True


def test_commit_ttl_clear_success_does_not_fire_when_passing_side_blocked():
    """TTL-clear success must NOT fire if the passing side is closed — that would
    be a bogus success while the route is actually occupied."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_success_time_s=1.0,
        commit_hold_s=0.5,
    )
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    s_ahead = _sit(
        ahead=True,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=30.0,
        longitudinal_m=20.0,
        closing_speed_mps=0.2,
    )
    # Seed commit_start_s by running one cycle.
    tactical_planner_step_v1(
        st, s_ahead,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead", assessment_gap_ok=True,
        assessment_optimal_open=False, assessment_left_open=True,
        assessment_right_open=False, assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    # Past threshold but LEFT side is NOW closed — should remain in commit, not declare success.
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s_ahead,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=20.0,
        sim_time_s=1.1, pit_mode=False, config=cfg,
        assessment_relation="ahead", assessment_gap_ok=True,
        assessment_optimal_open=False, assessment_left_open=False,  # left blocked
        assessment_right_open=False, assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m == COMMIT_PASS_LEFT
    assert st.commit.pass_success is False


def test_commit_protected_follow_not_released_when_emergency_risk_nonzero():
    """Protected-follow must NOT release into SETUP when emergency_risk_01 > 0.25.

    Reproduces the F4 root cause: asymmetric opening (right side available) combined
    with closing_flag=True previously allowed release of protected_follow even while
    the fellow was rapidly decelerating (emergency_risk_01 = 0.370 at t=8.85s in F4).
    The guard uses a threshold of > 0.25 so that moderate-risk scenarios (F2 risk ~0.206)
    can still release protected_follow while genuine emergency deceleration (risk > 0.25)
    keeps the latch engaged.
    """
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="straight",
    )
    # Prime state: protected_follow is active, one more clear cycle would normally release it.
    st = TacticalPlannerState()
    st.protected_follow_active = True
    st.protected_follow_clear_count = int(cfg.protected_follow_release_cycles) - 1
    # Asymmetric opening (right side only) + closing_flag=True would satisfy the old release
    # condition, but emergency_risk_01=0.37 > 0 must prevent the release.
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=8.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.37,
    )
    # Must stay in FOLLOW — protected_follow should not have released.
    assert m == FOLLOW, f"Expected FOLLOW but got {m} (reason={reason})"
    assert cap is not None and cap < 24.0
    assert st.protected_follow_active, "protected_follow_active must still be True"


def test_commit_fires_at_moderate_risk_when_pass_safe_passes():
    """SD-6: removed the commit_approach_risk_max=0.10 hesitancy gate.

    Pre-SD-6, commit was blocked whenever (closing_flag AND risk > 0.10).
    Risk grew above 0.10 the moment ego started closing on fellow, so the
    gate effectively NEVER permitted commit during normal F2-style overtakes.
    Now: as long as pass_safe (which checks risk vs the larger 0.48 ceiling)
    permits, COMMIT fires. Geometric look-ahead (SD-3c _commit_geom_ok) and
    SD-4 predicted_collision are the actual collision-defense layers.
    """
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,  # Allow commit in one cycle for test speed
        # SD-3b: bypass setup+commit gap gates — this test covers risk gate
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.1,
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="straight",
    )
    # Inject commit-active state directly (setup_commit fired, now waiting for phase11 commit).
    st = TacticalPlannerState()
    st.mode = "SETUP_PASS_RIGHT"
    st.setup_commit_side = "right"
    st.setup_commit_candidate_count = int(cfg.setup_commit_entry_cycles)
    st.setup_commit_until_s = 999.0
    st.pass_intent_side = "right"
    st.pass_intent_candidate_count = int(cfg.pass_intent_entry_cycles)
    st.pass_intent_until_s = 999.0
    st.lateral_path_lock_side = "right"
    st.lateral_path_lock_until_s = 999.0
    # Run with emergency_risk_01 = 0.42 (above old 0.10 gate, below 0.48 pass_safe ceiling).
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=7.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.42,
    )
    # SD-6: commit MUST fire at moderate risk — the closing+risk hesitancy
    # gate is removed. Brakes will still apply if predicted_collision fires
    # via SD-4 EMERGENCY_STABLE.
    assert m == COMMIT_PASS_RIGHT, f"SD-6: commit should fire at risk=0.42, got {m} ({reason})"


def test_commit_still_blocked_when_pass_safe_risk_above_ceiling():
    """SD-6 boundary: even after dropping commit_approach_risk_max, the
    pass_safe_risk_max=0.48 gate inside pass_safe still blocks commit when
    risk is genuinely high. This is the remaining risk-based brake on commit.
    """
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.6,  # raw above pass_safe_risk_max
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="straight",
    )
    st = TacticalPlannerState()
    st.mode = "SETUP_PASS_RIGHT"
    st.setup_commit_side = "right"
    st.setup_commit_candidate_count = int(cfg.setup_commit_entry_cycles)
    st.setup_commit_until_s = 999.0
    st.pass_intent_side = "right"
    st.pass_intent_candidate_count = int(cfg.pass_intent_entry_cycles)
    st.pass_intent_until_s = 999.0
    st.lateral_path_lock_side = "right"
    st.lateral_path_lock_until_s = 999.0
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=7.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=True,
        assessment_emergency_risk_01=0.60,  # > pass_safe_risk_max=0.48
    )
    assert m != COMMIT_PASS_RIGHT, f"Commit should be blocked at risk=0.60, got {m} ({reason})"


def test_commit_relaxes_to_free_run_when_far_nonclosing_low_risk():
    st = TacticalPlannerState()
    cfg_on = TacticalPlannerConfig(
        commit_abort_enabled=True,
        ahead_relax_free_run_enabled=True,
        ahead_relax_min_gap_m=30.0,
        ahead_relax_max_risk_01=0.15,
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=95.0,
        longitudinal_m=25.0,
        closing_speed_mps=0.2,
        segment_context="straight",
    )
    m_on, _ttl_on, _cap_on, reason_on = tactical_planner_step_v1(
        st,
        s,
        has_opponent=True,
        ego_speed_mps=30.0,
        opponent_speed_mps=24.0,
        sim_time_s=1.0,
        pit_mode=False,
        config=cfg_on,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.06,
    )
    assert m_on == FREE_RUN
    assert reason_on == "opponent_not_blocking"


# ---------------------------------------------------------------------------
# Phase 12: segment-conditioned tactical intelligence
# ---------------------------------------------------------------------------

def _make_seg_commit_setup_state():
    """Helper: state with an active setup-chain (commit_until active, pass_intent active)."""
    st = TacticalPlannerState()
    st.mode = "SETUP_PASS_RIGHT"
    st.setup_commit_side = "right"
    st.setup_commit_candidate_count = 5
    st.setup_commit_until_s = 999.0
    st.pass_intent_side = "right"
    st.pass_intent_candidate_count = 5
    st.pass_intent_until_s = 999.0
    st.lateral_path_lock_side = "right"
    st.lateral_path_lock_until_s = 999.0
    return st


def test_seg_corner_body_blocks_commit():
    """Phase 12: corner_body segment must block commit entry even when all other gates pass."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        # SD-3b: bypass setup+commit gap gates — this test covers segment gating
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        segment_aware_enabled=True,
        corner_body_blocks_commit=True,
        pass_requires_straight=False,  # Phase 12 owns segment gating
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.05,  # Well below pass_safe_risk_max — would normally commit
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="corner_body",
    )
    st = _make_seg_commit_setup_state()
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=9.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.0,
    )
    assert m != COMMIT_PASS_RIGHT, f"corner_body must block commit, got {m} ({reason})"
    assert st.commit.trigger == "none"
    assert st.segment_modifier == "blocked"


def test_seg_corner_entry_blocks_commit_when_risk_elevated():
    """Phase 12: corner_entry with collision_risk_01 > corner_entry_commit_risk_max blocks commit."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        # SD-3b: bypass setup+commit gap gates — this test covers segment gating
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        segment_aware_enabled=True,
        corner_entry_commit_risk_max=0.30,
        pass_requires_straight=False,  # Phase 12 owns segment gating
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.40,  # Above 0.30 corner_entry threshold, below 0.48 global max
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="corner_entry",
    )
    st = _make_seg_commit_setup_state()
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=9.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.0,
    )
    assert m != COMMIT_PASS_RIGHT, f"corner_entry with risk=0.40 > 0.30 should block commit, got {m}"
    assert st.commit.trigger == "none"
    assert st.segment_modifier == "conservative"


def test_seg_corner_entry_allows_commit_when_risk_low():
    """Phase 12: corner_entry with collision_risk_01 <= corner_entry_commit_risk_max allows commit."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,  # bypass speed cap — this test covers segment gating
        # SD-3b: bypass setup+commit gap gates — this test covers segment gating
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        segment_aware_enabled=True,
        corner_entry_commit_risk_max=0.30,
        pass_requires_straight=False,  # Phase 12 owns segment gating
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.10,  # Below 0.30 threshold — commit allowed
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="corner_entry",
    )
    st = _make_seg_commit_setup_state()
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=9.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.0,
    )
    assert m == COMMIT_PASS_RIGHT, f"corner_entry with low risk should allow commit, got {m}"
    assert st.segment_modifier == "conservative"


def test_seg_straight_unchanged_from_phase11():
    """Phase 12: on straight segment, behavior is identical to Phase 11 (no additional blocks)."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,  # bypass speed cap — this test covers segment gating
        # SD-3b: bypass setup+commit gap gates — this test covers segment gating
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        segment_aware_enabled=True,
        pass_requires_straight=False,  # Phase 12 owns segment gating
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.10,
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="straight",
    )
    st = _make_seg_commit_setup_state()
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=9.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.0,
    )
    assert m == COMMIT_PASS_RIGHT, f"straight segment must not block commit, got {m}"
    assert st.segment_modifier == "relaxed"


def test_seg_disabled_corner_body_does_not_block():
    """Phase 12 disabled: corner_body does NOT block commit (Phase 11 behavior preserved)."""
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,  # bypass speed cap — this test covers segment gating
        # SD-3b: bypass setup+commit gap gates — this test covers segment gating
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        segment_aware_enabled=False,  # Phase 12 off
        pass_requires_straight=False,  # Remove straight gate for apples-to-apples comparison
    )
    s = _sit(
        ahead=True,
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=40.0,
        longitudinal_m=38.0,
        closing_speed_mps=15.0,
        segment_context="corner_body",
    )
    st = _make_seg_commit_setup_state()
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=24.0, opponent_speed_mps=9.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False,
        assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.0,
    )
    assert m == COMMIT_PASS_RIGHT, f"Phase 12 disabled: corner_body must not block, got {m}"
    assert st.segment_modifier == "normal"


def test_commit_blocked_when_above_speed_cap():
    """Commit must not fire when ego speed exceeds commit_max_speed_mps.

    Root cause for Phase 11 spin-out (F2/F4): COMMIT_PASS fires at racing speed
    (~12.7 m/s). MPC applies large steer ± brake simultaneously, causing
    oscillation and 13–16m CTE spin-outs. The speed cap gates commit entry so
    the planner holds in SETUP until the ego has slowed to a speed where MPC
    has authority to execute the lateral TTL move without oscillating.
    """
    s = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.08,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
        ahead=True,
    )
    base_cfg = dict(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        setup_commit_entry_cycles=1,
        pass_intent_entry_cycles=1,
        setup_reentry_cooldown_s=0.0,
        follow_tight_headway_s=0.5,
        commit_max_speed_mps=9.0,
        # SD-3b: bypass setup+commit gap gates — this test covers speed cap
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
    )
    common_call = dict(
        has_opponent=True,
        opponent_speed_mps=7.0,
        sim_time_s=0.0,
        pit_mode=False,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.08,
    )

    # Above cap — commit must be blocked, planner holds in SETUP.
    st_fast = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    cfg_fast = TacticalPlannerConfig(**base_cfg)
    m_fast, _, _, _ = tactical_planner_step_v1(
        st_fast, s, ego_speed_mps=12.7, config=cfg_fast, **common_call
    )
    assert m_fast == SETUP_LEFT, (
        f"Commit must be blocked above speed cap, got {m_fast}"
    )
    assert st_fast.commit.candidate_count == 0

    # At or below cap — commit must fire.
    st_slow = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    cfg_slow = TacticalPlannerConfig(**base_cfg)
    m_slow, ttl_slow, _, reason_slow = tactical_planner_step_v1(
        st_slow, s, ego_speed_mps=8.5, config=cfg_slow, **common_call
    )
    assert m_slow == COMMIT_PASS_LEFT, (
        f"Commit must fire at or below speed cap, got {m_slow}"
    )
    assert ttl_slow == "left"
    assert reason_slow == "commit_pass_left"


def test_commit_opposing_commit_cooldown_blocks_then_releases():
    """After a commit exits on side X, opposing-side commits must be blocked until
    commit_opposing_commit_cooldown_s has elapsed. Same-side re-commit is allowed
    immediately. This prevents the right→left oscillation that caused spin-outs.
    """
    s = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.08,
        distance_m=24.0,
        longitudinal_m=20.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
        ahead=True,
    )
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,
        # SD-3b: bypass setup+commit gap gates — this test covers cooldown logic
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
        opposing_commit_cooldown_s=4.0,
        setup_commit_entry_cycles=1,
        pass_intent_entry_cycles=1,
        setup_reentry_cooldown_s=0.0,
        follow_tight_headway_s=0.5,
    )
    common = dict(
        has_opponent=True,
        opponent_speed_mps=7.0,
        pit_mode=False,
        config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=True,
        assessment_right_open=False,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.08,
    )

    # Simulate a left commit that has already exited at t=0.0s.
    st = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    st.commit.last_side = "left"
    st.commit.last_exit_s = 0.0

    # At t=1.0s — same-side (left) re-commit must be ALLOWED (no cooldown on same side).
    m_same, _, _, _ = tactical_planner_step_v1(
        st, s, ego_speed_mps=12.0, sim_time_s=1.0, **common
    )
    assert m_same == COMMIT_PASS_LEFT, f"Same-side re-commit must fire, got {m_same}"

    # Simulate a right-commit exit at t=0.0s; try to commit left at t=1.0s (< 4s cooldown).
    st2 = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    st2.commit.last_side = "right"
    st2.commit.last_exit_s = 0.0
    m_blocked, _, _, _ = tactical_planner_step_v1(
        st2, s, ego_speed_mps=12.0, sim_time_s=1.0, **common
    )
    assert m_blocked == SETUP_LEFT, (
        f"Opposing-side commit must be blocked during cooldown, got {m_blocked}"
    )

    # At t=5.0s — cooldown expired, opposing commit must now fire.
    st3 = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    st3.commit.last_side = "right"
    st3.commit.last_exit_s = 0.0
    m_released, _, _, _ = tactical_planner_step_v1(
        st3, s, ego_speed_mps=12.0, sim_time_s=5.0, **common
    )
    assert m_released == COMMIT_PASS_LEFT, (
        f"Opposing commit must fire after cooldown, got {m_released}"
    )


def test_commit_blocked_when_gap_too_large():
    """Commit must not fire when longitudinal gap exceeds commit gap gate.

    SD-3b: gate is now Δv-derived. Override slope=0/intercept=40/ceiling=40 so
    the gate is effectively "gap <= 40m" regardless of Δv — this isolates the
    test to verify the gap-gate semantics (>40m blocked, ≤40m fires).
    """
    cfg = TacticalPlannerConfig(
        commit_abort_enabled=True,
        commit_entry_cycles=1,
        commit_max_speed_mps=40.0,
        commit_gap_dv_slope=0.0,
        commit_gap_dv_intercept_m=40.0,
        commit_gap_dv_ceiling_m=40.0,
        setup_gap_dv_slope=0.0,
        setup_gap_dv_intercept_m=40.0,
        setup_gap_dv_ceiling_m=40.0,
        setup_commit_entry_cycles=1,
        pass_intent_entry_cycles=1,
        setup_reentry_cooldown_s=0.0,
        follow_tight_headway_s=0.5,
    )
    common = dict(
        has_opponent=True,
        ego_speed_mps=12.0,
        opponent_speed_mps=7.0,
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

    # Fellow is 45m ahead — beyond the 40m gate — commit must be blocked.
    st_far = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    s_far = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=49.0,
        longitudinal_m=45.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
        ahead=True,
    )
    m_far, _, _, _ = tactical_planner_step_v1(st_far, s_far, sim_time_s=0.0, **common)
    assert m_far != COMMIT_PASS_LEFT, f"Commit must be blocked at 45m gap, got {m_far}"
    assert st_far.commit.candidate_count == 0

    # Fellow is 35m ahead — within the 40m gate — commit must fire.
    st_close = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left")
    s_close = _sit(
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=39.0,
        longitudinal_m=35.0,
        closing_speed_mps=4.0,  # SD-2e: realistic overtake closing speed
        ahead=True,
    )
    m_close, ttl_close, _, reason_close = tactical_planner_step_v1(
        st_close, s_close, sim_time_s=0.0, **common
    )
    assert m_close == COMMIT_PASS_LEFT, f"Commit must fire at 35m gap, got {m_close}"
    assert ttl_close == "left"


def test_pass_safe_feasibility_passes_at_matched_speed():
    """SD-2g: pass_safe must be a feasibility check, not actual-state check.

    F2_tactical was stuck for 13 sec in matched-speed FOLLOW (closing≈0) because
    pass_safe required closing >= 3.0 m/s — unsatisfiable from matched-speed
    steady state (MPC brakes-for-distance, closing decays to 0). The new check
    asks "under SETUP cap, COULD ego close?" — which is true at matched speed
    (ego_speed + setup_margin > opp_speed + pass_min).
    """
    st = TacticalPlannerState(mode=FOLLOW, last_setup_side="left",
                              setup_candidate_side="right",
                              setup_candidate_count=10)  # already past candidate persistence
    cfg = TacticalPlannerConfig(
        # SD-3b: pin Δv-derived gates so the test is independent of slope:
        # cap = 0*Δv + 28 = 28 m for SETUP entry.
        setup_gap_dv_slope=0.0,
        setup_gap_dv_intercept_m=28.0,
        setup_gap_dv_ceiling_m=28.0,
        setup_speed_margin_mps=4.5,
        pass_min_relative_speed_mps=0.3,
        ahead_relax_free_run_enabled=False,
    )
    s = _sit(
        lateral_relation="left",   # fellow on left → preferred = right
        collision_risk_01=0.05,
        segment_context="straight",
        overlap_state="clear_ahead",
        distance_m=18.0,
        longitudinal_m=18.0,        # ≤ 28 → SETUP gate clears
        closing_speed_mps=0.0,      # matched speed — old gate would block here
        ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True,
        ego_speed_mps=11.5,         # opp + follow_margin
        opponent_speed_mps=9.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=False,    # gap_ok=False per F2_tactical state
        assessment_optimal_open=False,
        assessment_left_open=False, assessment_right_open=True,
        assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    # SETUP must fire even though current closing=0; SETUP cap will then accelerate
    # ego and the actual closing will materialize.
    assert m == SETUP_RIGHT, f"SETUP must fire at matched speed when feasible, got {m}"
    assert ttl == "right"


def test_commit_pass_success_enters_hold_then_releases_when_clear():
    """SD-3d: when COMMIT succeeds with ego still side-by-side (|lateral_m| < 2.5),
    enter HOLD on the same side TTL, then release to FREE_RUN once
    delta_s_behind exceeds hold_release_long_m AND geometry is safe.

    This is the structural fix for the F2_tactical "cut back into fellow on
    revert-to-optimal" failure mode.
    """
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    st.commit.side = "left"
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)

    # Tick 1: COMMIT success but still side-by-side → enter HOLD on left.
    s_just_passed = _sit(
        ahead=False,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=5.0,
        longitudinal_m=-2.0,    # ego just barely ahead
        lateral_m=1.5,          # < merge_safe_lat_m=2.5 → still in danger
        closing_speed_mps=0.0,
        delta_s_m=-2.0,
    )
    m1, ttl1, cap1, reason1 = tactical_planner_step_v1(
        st, s_just_passed,
        has_opponent=True, ego_speed_mps=18.0, opponent_speed_mps=10.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.05,
    )
    assert m1 == HOLD_PASS_LEFT, f"Expected HOLD_PASS_LEFT entry, got {m1}"
    assert ttl1 == "left", "HOLD must keep the COMMIT-side TTL"
    assert reason1 == "hold_pass_entry"
    assert cap1 is not None and cap1 >= 11.5  # max(ego_at_entry=18, opp+1.5=11.5) = 18

    # Tick 2: ego has pulled further ahead → release.
    # hold_release_long_m(Δv=8) = 6.4 + 0.3·8 = 8.8m → delta_s_behind=10 satisfies.
    s_clear = _sit(
        ahead=False,
        lateral_relation="right",
        overlap_state="clear_ahead",
        collision_risk_01=0.05,
        distance_m=10.0,
        longitudinal_m=-10.0,
        lateral_m=1.5,           # still side_by_side per lateral check, but
                                 # longitudinal clearance has built up
        closing_speed_mps=0.0,
        delta_s_m=-10.0,         # ego 10m ahead → > 8.8m hold_release threshold
    )
    m2, ttl2, cap2, reason2 = tactical_planner_step_v1(
        st, s_clear,
        has_opponent=True, ego_speed_mps=18.0, opponent_speed_mps=10.0,
        sim_time_s=0.5, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.05,
    )
    assert m2 == FREE_RUN, f"HOLD must release to FREE_RUN, got {m2}"
    assert ttl2 == "optimal", "HOLD release returns to optimal TTL"
    assert reason2 == "hold_release_merge_safe"


def test_commit_pass_success_hold_aborts_on_hazard_reappearance():
    """SD-3d: HOLD must transition to ABORT when a hard hazard reappears
    (e.g., fellow drafts back alongside or relation flips ahead again)."""
    st = TacticalPlannerState(mode=HOLD_PASS_LEFT, last_setup_side="left")
    st.commit.side = "left"
    st.commit.hold_pass_side = "left"
    st.commit.hold_entry_s = 0.0
    st.commit.hold_speed_at_entry_mps = 18.0
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    # Fellow drafts back alongside — overlap=side_by_side triggers overlap_hazard_now.
    s_hazard = _sit(
        ahead=False,
        lateral_relation="right",
        overlap_state="side_by_side",
        collision_risk_01=0.4,
        distance_m=2.0,
        longitudinal_m=-1.0,
        lateral_m=0.5,
        closing_speed_mps=0.0,
        delta_s_m=-1.0,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s_hazard,
        has_opponent=True, ego_speed_mps=18.0, opponent_speed_mps=10.0,
        sim_time_s=0.5, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=False,
        assessment_optimal_open=False,
        assessment_left_open=False, assessment_right_open=False,
        assessment_closing_flag=True, assessment_emergency_risk_01=0.5,
    )
    assert m == ABORT_PASS, f"HOLD should ABORT on hazard, got {m}"
    assert reason == "abort_hold_hazard"


def _make_polyline(x0, y0, dx, dy, n):
    return [(x0 + i * dx, y0 + i * dy, 0.0) for i in range(n)]


def test_setup_rejects_side_when_pass_window_geometry_unsafe():
    """SD-3c: when pass_window_check returns False, SETUP must skip that side.

    Right TTL converges with optimal at s=20m → pass_window_check rejects right.
    Left TTL is parallel and clear → planner switches target to left.

    Avoids triggering safety_pressure (which would short-circuit to
    protected_follow_envelope) by using asymmetric corridor + closing_flag=False.
    """
    st = TacticalPlannerState(
        # Already past candidate persistence so SETUP can fire this tick.
        setup_candidate_side="right", setup_candidate_count=10,
    )
    cfg = TacticalPlannerConfig(
        ahead_relax_free_run_enabled=False,
        # Permissive Δv gates so look-ahead is the only blocker.
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
    )
    # Optimal: y=0 straight. Right TTL: starts y=4 then converges to y=0 at x=20.
    optimal = _make_polyline(0.0, 0.0, 1.0, 0.0, 200)
    right = []
    for i in range(200):
        x = float(i)
        y = 4.0 - (4.0 / 20.0) * x if x <= 20.0 else 0.0
        right.append((x, y, 0.0))
    left = _make_polyline(0.0, 4.0, 1.0, 0.0, 200)  # parallel, clear
    s = _sit(
        lateral_relation="left",   # fellow drifting left → preferred = right
        collision_risk_01=0.05,
        overlap_state="clear_ahead",
        distance_m=20.0, longitudinal_m=20.0,
        closing_speed_mps=2.0, ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=14.0, opponent_speed_mps=9.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead", assessment_gap_ok=True,
        assessment_optimal_open=False,
        # Asymmetric (R only) → asymmetric_opening=True → safety_pressure suppressed.
        assessment_left_open=False, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.05,
        optimal_waypoints=optimal,
        side_waypoints_left=left, side_waypoints_right=right,
        ego_s_m=10.0, opp_s_m=12.0, lap_length_m=199.0,
    )
    # Right was preferred but its geometry converges → planner switches to left.
    assert ttl == "left", f"Geometry should redirect SETUP to left, got ttl={ttl}, reason={reason}"
    assert m == SETUP_LEFT


def test_setup_blocked_when_both_pass_windows_geometry_unsafe():
    """SD-3c: when both sides converge with fellow's path, stay in FOLLOW.

    Both side TTLs converge to optimal at s=20m → pass_window_check rejects both.
    Planner returns FOLLOW with reason="pass_window_unsafe_both_sides".
    """
    st = TacticalPlannerState(
        setup_candidate_side="right", setup_candidate_count=10,
    )
    cfg = TacticalPlannerConfig(
        ahead_relax_free_run_enabled=False,
        setup_gap_dv_intercept_m=999.0, setup_gap_dv_ceiling_m=999.0,
        commit_gap_dv_intercept_m=999.0, commit_gap_dv_ceiling_m=999.0,
    )
    optimal = _make_polyline(0.0, 0.0, 1.0, 0.0, 200)
    converging_right = []
    converging_left = []
    for i in range(200):
        x = float(i)
        if x <= 20.0:
            y_r = 4.0 - (4.0 / 20.0) * x
            y_l = -4.0 + (4.0 / 20.0) * x
        else:
            y_r = 0.0
            y_l = 0.0
        converging_right.append((x, y_r, 0.0))
        converging_left.append((x, y_l, 0.0))
    s = _sit(
        lateral_relation="left",
        collision_risk_01=0.05,
        overlap_state="clear_ahead",
        distance_m=20.0, longitudinal_m=20.0,
        closing_speed_mps=2.0, ahead=True,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=14.0, opponent_speed_mps=9.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead", assessment_gap_ok=True,
        assessment_optimal_open=False,
        assessment_left_open=False, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.05,
        optimal_waypoints=optimal,
        side_waypoints_left=converging_left, side_waypoints_right=converging_right,
        ego_s_m=10.0, opp_s_m=12.0, lap_length_m=199.0,
    )
    assert m == FOLLOW, f"Both sides converging → must stay in FOLLOW, got {m}"
    assert reason == "pass_window_unsafe_both_sides"


def test_setup_blocked_when_fellow_too_far():
    """SD-2f / SD-3b: SETUP entry must be gated by Δv-derived gap formula.

    F2_tactical first attempt entered SETUP at gap=42m and physically converged
    on the right TTL toward the fellow over 4 sec → contact. Stay in FOLLOW until
    close enough that SETUP→COMMIT can complete in 1-2 cycles.
    """
    st = TacticalPlannerState()
    # Disable ahead_relax so we don't take the FREE_RUN escape; we want to verify
    # the SD-3b gap-gate path specifically. Pin gate to 28m via slope=0/intercept=28.
    cfg = TacticalPlannerConfig(
        setup_gap_dv_slope=0.0,
        setup_gap_dv_intercept_m=28.0,
        setup_gap_dv_ceiling_m=28.0,
        ahead_relax_free_run_enabled=False,
    )
    s = _sit(
        lateral_relation="right",
        collision_risk_01=0.05,
        segment_context="straight",
        overlap_state="clear_ahead",
        distance_m=32.0,
        longitudinal_m=30.0,  # > 28 → SETUP blocked by SD-2f
        closing_speed_mps=5.0,
        ahead=True,
    )
    # Run several cycles to let candidate counter accumulate; SETUP still must not fire.
    for t in (0.0, 0.05, 0.10, 0.15, 0.20):
        m, ttl, _, reason = tactical_planner_step_v1(
            st, s,
            has_opponent=True, ego_speed_mps=14.0, opponent_speed_mps=9.0,
            sim_time_s=t, pit_mode=False, config=cfg,
            assessment_relation="ahead",
            assessment_gap_ok=True, assessment_optimal_open=False,
            assessment_left_open=False, assessment_right_open=True,
            assessment_closing_flag=True, assessment_emergency_risk_01=0.05,
        )
    assert m == FOLLOW, f"SETUP must be blocked when fellow >28m away, got {m}"
    assert reason == "setup_too_far_follow"


def test_setup_timeout_bails_back_to_follow():
    """SD-2f: SETUP must time out if it can't reach COMMIT within setup_max_hold_s.

    Safety net for the F2_tactical-style failure where SETUP holds on a side TTL
    while ego physically converges with the fellow → contact. After the timeout,
    bail back to FOLLOW on optimal so the lateral motion stops.
    """
    st = TacticalPlannerState(mode=SETUP_LEFT, last_setup_side="left",
                              setup_entry_s=0.0)
    cfg = TacticalPlannerConfig(
        setup_max_hold_s=2.0,
        # SD-3b: bypass setup gap gate so the test isolates the timeout path.
        setup_gap_dv_ceiling_m=999.0,
    )
    s = _sit(
        lateral_relation="right",  # fellow on right → preferred pass side = left
        collision_risk_01=0.05,
        overlap_state="clear_ahead",
        distance_m=20.0,
        longitudinal_m=20.0,
        closing_speed_mps=5.0,
        ahead=True,
    )
    # Inside the hold window — SETUP should still be active.
    m_inside, ttl_inside, _, _ = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=14.0, opponent_speed_mps=9.0,
        sim_time_s=1.5, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True, assessment_optimal_open=False,
        assessment_left_open=True, assessment_right_open=False,
        assessment_closing_flag=True, assessment_emergency_risk_01=0.30,
    )
    assert m_inside == SETUP_LEFT, f"SETUP should hold inside timeout, got {m_inside}"

    # Past the timeout — must bail to FOLLOW on optimal.
    m_after, ttl_after, _, reason_after = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=14.0, opponent_speed_mps=9.0,
        sim_time_s=2.6, pit_mode=False, config=cfg,
        assessment_relation="ahead",
        assessment_gap_ok=True, assessment_optimal_open=False,
        assessment_left_open=True, assessment_right_open=False,
        assessment_closing_flag=True, assessment_emergency_risk_01=0.30,
    )
    assert m_after == FOLLOW, f"SETUP must bail after timeout, got {m_after}"
    assert ttl_after == "optimal"
    assert reason_after == "setup_timeout_follow"


def test_predicted_collision_bypassed_for_stationary_lateral_clear():
    """SD-10d regression: F9 stationary roadside fellow must NOT trigger
    predicted_collision via PathPredict's polyline-projection bug.

    Pre-SD-10d: PathPredict walked opp at fixed s on optimal_track for
    stationary opp, but if the side TTL ego walked happened to curve near
    the projected-opp point, min_clear dropped below threshold → false
    collision → 292x EMERGENCY_STABLE → ego parked at v=0.

    Post-SD-10d: when opp_speed <= stationary_opp_speed_mps (1.5) AND
    |sit.lateral_m| > stationary_overlap_relief_lateral_m (2.0), bypass
    PathPredict and report no collision regardless of polyline geometry.
    """
    st = TacticalPlannerState()
    cfg = TacticalPlannerConfig()
    # Construct a side TTL that DOES curve back into optimal at s=20m, just
    # like F9's left TTL. Without the bypass, this would trigger predicted_collision.
    optimal = _make_polyline(0.0, 0.0, 1.0, 0.0, 200)
    converging_left = []
    for i in range(200):
        x = float(i)
        y = -4.0 + (4.0 / 20.0) * x if x <= 20.0 else 0.0
        converging_left.append((x, y, 0.0))
    converging_right = []
    for i in range(200):
        x = float(i)
        y = 4.0 - (4.0 / 20.0) * x if x <= 20.0 else 0.0
        converging_right.append((x, y, 0.0))
    # Fellow STATIONARY (opp_speed=0) and laterally OFF the racing line (lat=-5.5).
    s = _sit(
        ahead=True, lateral_relation="right",
        overlap_state="clear_ahead", collision_risk_01=0.0,
        distance_m=12.0, longitudinal_m=10.0, lateral_m=-5.5,
        closing_speed_mps=0.0, opponent_speed_mps=0.0,
    )
    m, ttl, cap, reason = tactical_planner_step_v1(
        st, s,
        has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=0.0,
        sim_time_s=0.0, pit_mode=False, config=cfg,
        assessment_relation="ahead", assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.0,
        optimal_waypoints=optimal,
        side_waypoints_left=converging_left, side_waypoints_right=converging_right,
        ego_s_m=10.0, opp_s_m=10.0, lap_length_m=199.0,
        ego_active_ttl="optimal",
    )
    # The bypass means predicted_collision MUST be False for this stationary
    # laterally-clear case, regardless of how the side TTL polylines curve.
    assert st.predicted_collision is False, (
        f"Stationary lateral-clear fellow should not trigger predicted_collision; "
        f"got predicted_collision={st.predicted_collision}, ego_track={st.predicted_collision_ego_track}"
    )
    assert st.predicted_collision_ego_track == "bypass", (
        f"Bypass marker should be set; got ego_track={st.predicted_collision_ego_track}"
    )
