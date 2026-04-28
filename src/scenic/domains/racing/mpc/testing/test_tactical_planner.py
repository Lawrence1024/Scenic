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


def test_pit_forces_free_run():
    st = TacticalPlannerState()
    st.mode = FOLLOW
    s = _sit()
    m, ttl, cap = tactical_planner_step(
        st, s, has_opponent=True, ego_speed_mps=10.0, opponent_speed_mps=8.0,
        sim_time_s=0.0, pit_mode=True, config=TacticalPlannerConfig(),
    )
    assert m == FREE_RUN and ttl == "optimal"


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
    """SD-12c: in COMMIT_PASS_LEFT with a stationary fellow laterally clear,
    COMMIT must hold (not abort). Pre-SD-12c this was achieved via the
    stationary_overlap_relief snapshot hack. Post-SD-12c the same outcome
    comes from the proper fix: opp_trajectory threading lets PathPredict
    compute real geometric clearance, predicted_collision=False, and
    _apply_predicted_collision_gate suppresses the snapshot hazard.
    """
    st = TacticalPlannerState(mode=COMMIT_PASS_LEFT, last_setup_side="left")
    cfg = TacticalPlannerConfig(commit_abort_enabled=True)
    s = _sit(
        ahead=True,
        overlap_state="partial_overlap",
        collision_risk_01=0.10,
        distance_m=20.0,
        longitudinal_m=6.0,
        lateral_m=3.6,
        closing_speed_mps=0.0,
    )
    # SD-12c: supply polylines + stationary fellow trajectory so
    # path_collision_predicted correctly says no collision.
    optimal = _make_polyline(0.0, 0.0, 1.0, 0.0, 200)
    left_wp = _make_polyline(0.0, 5.0, 1.0, 0.0, 200)
    right_wp = _make_polyline(0.0, -5.0, 1.0, 0.0, 200)
    fellow_traj = [(i * 0.1, 6.0, 3.6, None) for i in range(16)]
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
        optimal_waypoints=optimal,
        side_waypoints_left=left_wp,
        side_waypoints_right=right_wp,
        ego_s_m=0.0,
        opp_s_m=6.0,
        lap_length_m=199.0,
        ego_active_ttl="left",
        fellow_trajectory=fellow_traj,
    )
    # SD-3f: COMMIT cap = opp_speed + commit_speed_margin_mps (0 + 16 = 16 m/s).
    assert m == COMMIT_PASS_LEFT and ttl == "left"
    assert cap is not None and abs(cap - 16.0) < 0.01
    assert reason == "commit_pass_left_hold"
    assert st.commit.abort_trigger == "none"


def test_commit_pass_success_enters_hold_for_merge_back_ramp():
    """SD-12b: COMMIT success ALWAYS enters HOLD_PASS_{side} (even when
    laterally clear) so MPC has merge_back_ramp_s of smoothing for the
    side-TTL → optimal lateral transition. Pre-SD-12b laterally-clear
    pass_success returned FREE_RUN immediately, causing the F3L/F3R
    "massive swerl back" the user observed (one-tick ttl snap).
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
        # Laterally clear (used to skip HOLD pre-SD-12b; now always enters HOLD).
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
    assert m == HOLD_PASS_LEFT and ttl == "left"
    assert reason == "hold_pass_entry"
    assert st.commit.pass_success is True
    assert st.commit.hold_pass_side == "left"


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

    # Fellow drops behind — pass_success fires.
    # SD-12b: ALL pass_success now routes through HOLD_PASS_{side} for
    # merge_back_ramp_s before releasing to FREE_RUN (not just side-by-side
    # cases). Even with lateral_m=3.0 > merge_safe_lat_m=2.5, ego enters
    # HOLD first so MPC has a smoothing window for the side→optimal flip.
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
    assert m3 == HOLD_PASS_LEFT and ttl3 == "left"
    assert reason3 == "hold_pass_entry"
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


def _make_seg_commit_setup_state():
    """Helper: state with an active setup-chain (commit_until active, pass_intent active).

    SD-10b: opening_confidence_count replaces the pass_intent_candidate_count
    + setup_commit_candidate_count chain. Setting it well above the threshold
    keeps the test independent of cycle-count config tuning.
    """
    st = TacticalPlannerState()
    st.mode = "SETUP_PASS_RIGHT"
    st.setup_commit_side = "right"
    st.setup_commit_until_s = 999.0
    st.pass_intent_side = "right"
    st.pass_intent_until_s = 999.0
    st.opening_confidence_count = 5
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
    # SD-12b: also wait merge_back_ramp_s=0.8s after hold_entry to satisfy
    # the new ramp gate. sim_time=1.0s gives elapsed=1.0 > 0.8.
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
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=True,
        assessment_optimal_open=True,
        assessment_left_open=True, assessment_right_open=True,
        assessment_closing_flag=False, assessment_emergency_risk_01=0.05,
    )
    assert m2 == FREE_RUN, f"HOLD must release to FREE_RUN, got {m2}"
    assert ttl2 == "optimal", "HOLD release returns to optimal TTL"
    assert reason2 == "hold_release_merge_safe"


def test_sd12b_hold_release_blocked_until_merge_back_ramp_s():
    """SD-12b regression: HOLD must NOT release to FREE_RUN within
    merge_back_ramp_s of hold_entry, even when long_ok and merge_geom_ok
    are satisfied. Gives MPC a smoothing window for the side→optimal
    lateral transition (the F3L/F3R "massive swerl" fix).
    """
    st = TacticalPlannerState(mode=HOLD_PASS_LEFT, last_setup_side="left")
    st.commit.side = "left"
    st.commit.hold_pass_side = "left"
    st.commit.hold_entry_s = 0.0
    st.commit.hold_speed_at_entry_mps = 18.0
    cfg = TacticalPlannerConfig(commit_abort_enabled=True, merge_back_ramp_s=0.8)
    s_clear = _sit(
        ahead=False, lateral_relation="right",
        overlap_state="clear_ahead", collision_risk_01=0.05,
        distance_m=10.0, longitudinal_m=-10.0,
        lateral_m=3.0, closing_speed_mps=0.0,
        delta_s_m=-10.0,
    )
    # Tick at 0.5s (within 0.8s ramp) — release blocked even though geometry OK.
    m_within, ttl_within, _, reason_within = tactical_planner_step_v1(
        st, s_clear,
        has_opponent=True, ego_speed_mps=18.0, opponent_speed_mps=10.0,
        sim_time_s=0.5, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=True,
        assessment_optimal_open=True, assessment_left_open=True,
        assessment_right_open=True, assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m_within == HOLD_PASS_LEFT and ttl_within == "left"
    assert reason_within == "hold_pass_hold"
    # Tick at 1.0s (past 0.8s ramp) — release fires.
    m_past, ttl_past, _, reason_past = tactical_planner_step_v1(
        st, s_clear,
        has_opponent=True, ego_speed_mps=18.0, opponent_speed_mps=10.0,
        sim_time_s=1.0, pit_mode=False, config=cfg,
        assessment_relation="behind", assessment_gap_ok=True,
        assessment_optimal_open=True, assessment_left_open=True,
        assessment_right_open=True, assessment_closing_flag=False,
        assessment_emergency_risk_01=0.05,
    )
    assert m_past == FREE_RUN and ttl_past == "optimal"
    assert reason_past == "hold_release_merge_safe"


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


def test_predicted_collision_correctly_handles_stationary_lateral_clear_via_opp_trajectory():
    """SD-12c regression: F9 stationary roadside fellow must NOT trigger
    predicted_collision when opp_trajectory is supplied (the FellowPredictor
    output). Replaces the SD-10d speed=0 special-case bypass.

    The fix is now at the source: path_collision_predicted uses opp_trajectory
    (CV-extrapolated true xy from FellowPredictor) instead of a polyline
    projection. Stationary fellow → CV velocity ≈ 0 → trajectory samples all
    sit at the observed off-line xy → real geometric clearance computed
    correctly, no false positive regardless of how the side TTL polyline
    curves.
    """
    st = TacticalPlannerState()
    cfg = TacticalPlannerConfig()
    optimal = _make_polyline(0.0, 0.0, 1.0, 0.0, 200)
    # Side TTL that curves back into optimal at s=20m (the F9 geometry).
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
    # Fellow STATIONARY (opp_speed=0) at lat=-5.5m off the racing line.
    s = _sit(
        ahead=True, lateral_relation="right",
        overlap_state="clear_ahead", collision_risk_01=0.0,
        distance_m=12.0, longitudinal_m=10.0, lateral_m=-5.5,
        closing_speed_mps=0.0, opponent_speed_mps=0.0,
    )
    # SD-12c: provide the fellow trajectory as a CV-extrapolated zero-motion
    # series at the fellow's actual roadside xy. Fellow is at (10, -5.5)
    # in the world frame (matches sit.longitudinal_m=10, sit.lateral_m=-5.5
    # with ego at origin heading +x).
    fellow_traj = [(i * 0.1, 10.0, -5.5, None) for i in range(16)]  # 1.5s @ 0.1dt
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
        fellow_trajectory=fellow_traj,
    )
    # Per SD-12c the prediction layer correctly computes geometric clearance
    # vs the fellow's true xy (5.5m away laterally) — no collision predicted.
    assert st.predicted_collision is False, (
        f"Stationary lateral-clear fellow should not trigger predicted_collision; "
        f"got predicted_collision={st.predicted_collision}, "
        f"min_clear={st.predicted_collision_min_clear_m}, "
        f"ego_track={st.predicted_collision_ego_track}"
    )
    # Min clear should be roughly the lateral offset (5.5m).
    assert st.predicted_collision_min_clear_m >= 4.0, (
        f"Min clearance should reflect 5.5m lateral offset, "
        f"got min_clear={st.predicted_collision_min_clear_m}"
    )
