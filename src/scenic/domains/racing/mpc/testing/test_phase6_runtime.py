"""Tests for Phase 6 orchestration shells."""

from scenic.domains.racing.phase6_runtime import (
    FOLLOW,
    FREE_RUN,
    build_phase6_state_snapshot,
    phase6_guard_step,
    phase6_planner_step,
)


def test_phase6_planner_free_run_without_opponent():
    state = build_phase6_state_snapshot(
        has_opponent=False,
        pit_mode=False,
        current_ttl="optimal",
        ego_speed_mps=25.0,
        opponent_speed_mps=None,
        opponent_distance_m=None,
        overlap_state="none",
        segment_context="straight",
        ahead_flag=False,
    )
    decision = phase6_planner_step(state, target_speed_mps=30.0, speed_cap_mps=None)
    assert decision.planner_state == FREE_RUN
    assert decision.active_ttl == "optimal"
    assert decision.decision_reason == "no_opponent"


def test_phase6_planner_follow_when_opponent_ahead_and_close():
    state = build_phase6_state_snapshot(
        has_opponent=True,
        pit_mode=False,
        current_ttl="optimal",
        ego_speed_mps=30.0,
        opponent_speed_mps=20.0,
        opponent_distance_m=18.0,
        overlap_state="clear_ahead",
        segment_context="straight",
        ahead_flag=True,
    )
    decision = phase6_planner_step(state, target_speed_mps=35.0, speed_cap_mps=None)
    assert decision.planner_state == FOLLOW
    assert decision.active_ttl == "optimal"
    assert decision.target_speed_cap_mps is not None
    assert decision.target_speed_cap_mps <= 23.0


def test_phase6_guard_is_pass_through_default():
    state = build_phase6_state_snapshot(
        has_opponent=True,
        pit_mode=False,
        current_ttl="optimal",
        ego_speed_mps=20.0,
        opponent_speed_mps=18.0,
        opponent_distance_m=25.0,
        overlap_state="clear_ahead",
        segment_context="straight",
        ahead_flag=True,
    )
    decision = phase6_planner_step(state, target_speed_mps=30.0, speed_cap_mps=28.0)
    guarded = phase6_guard_step(decision)
    assert guarded.planner_state == decision.planner_state
    assert guarded.active_ttl == decision.active_ttl
    assert guarded.guard_active is False
    assert guarded.guard_reason == "none"
