"""Unit tests for Phase 10 stability guard."""

from scenic.domains.racing.safety.stability_guard import (
    Phase10StabilityGuardConfig,
    Phase10StabilityGuardState,
    phase10_guard_step,
    phase10_handle_ttl_switch,
)


def test_phase10_ttl_switch_rate_limit_blocks_fast_flip():
    state = Phase10StabilityGuardState(last_ttl="optimal")
    cfg = Phase10StabilityGuardConfig(ttl_switch_min_interval_s=1.0)
    ttl1, blocked1 = phase10_handle_ttl_switch(
        state,
        config=cfg,
        sim_time_s=1.0,
        current_ttl="optimal",
        requested_ttl="left",
    )
    assert ttl1 == "left"
    assert blocked1 is False
    ttl2, blocked2 = phase10_handle_ttl_switch(
        state,
        config=cfg,
        sim_time_s=1.2,
        current_ttl="left",
        requested_ttl="optimal",
    )
    assert ttl2 == "left"
    assert blocked2 is True


def test_phase10_guard_limits_steer_slew():
    state = Phase10StabilityGuardState(last_steer_cmd_rad=0.0, last_ttl="optimal")
    cfg = Phase10StabilityGuardConfig(max_steer_rate_rad_per_s=1.0)
    d = phase10_guard_step(
        state,
        config=cfg,
        sim_time_s=2.0,
        control_dt_s=0.1,
        planner_state="SETUP_PASS_LEFT",
        active_ttl="left",
        decision_reason="setup_left_open",
        steer_cmd_rad=0.5,
        throttle_cmd=0.8,
        brake_cmd=0.0,
        pit_mode=False,
        phase8_gap_ok=True,
        phase8_overlap_flag=False,
        phase8_closing_flag=False,
        phase8_emergency_risk_01=0.1,
        ttl_switch_blocked=False,
    )
    assert abs(d.steer_cmd_rad) <= 0.1001
    assert d.steer_limited is True
    assert d.guard_active is True


def test_phase10_guard_enters_emergency_stable():
    state = Phase10StabilityGuardState(last_ttl="optimal")
    cfg = Phase10StabilityGuardConfig(
        emergency_risk_enter_01=0.8,
        emergency_overlap_brake_floor=0.5,
        emergency_max_steer_abs_rad=0.1,
    )
    d = phase10_guard_step(
        state,
        config=cfg,
        sim_time_s=3.0,
        control_dt_s=0.05,
        planner_state="FOLLOW",
        active_ttl="optimal",
        decision_reason="protected_follow_envelope",
        steer_cmd_rad=0.3,
        throttle_cmd=0.9,
        brake_cmd=0.0,
        pit_mode=False,
        phase8_gap_ok=False,
        phase8_overlap_flag=True,
        phase8_closing_flag=True,
        phase8_emergency_risk_01=0.95,
        ttl_switch_blocked=False,
    )
    assert d.emergency_stable_mode is True
    assert d.planner_state == "EMERGENCY_STABLE"
    assert d.throttle_cmd == 0.0
    assert d.brake_cmd >= 0.5
    assert abs(d.steer_cmd_rad) <= 0.1001


def test_phase10_guard_reapproach_hold_suppresses_throttle_after_emergency():
    state = Phase10StabilityGuardState(last_ttl="optimal")
    cfg = Phase10StabilityGuardConfig(
        emergency_risk_enter_01=0.8,
        emergency_hold_s=0.5,
        reapproach_recovery_hold_s=1.0,
        reapproach_max_throttle=0.15,
    )
    # Trigger emergency first.
    _ = phase10_guard_step(
        state,
        config=cfg,
        sim_time_s=0.0,
        control_dt_s=0.05,
        planner_state="FOLLOW",
        active_ttl="optimal",
        decision_reason="protected_follow_envelope",
        steer_cmd_rad=0.0,
        throttle_cmd=0.9,
        brake_cmd=0.0,
        pit_mode=False,
        phase8_gap_ok=False,
        phase8_overlap_flag=True,
        phase8_closing_flag=True,
        phase8_emergency_risk_01=0.95,
        ttl_switch_blocked=False,
    )
    # After emergency latch window, recovery hold should still cap throttle.
    d = phase10_guard_step(
        state,
        config=cfg,
        sim_time_s=0.7,
        control_dt_s=0.05,
        planner_state="SETUP_PASS_LEFT",
        active_ttl="left",
        decision_reason="setup_commit_left_hold",
        steer_cmd_rad=0.02,
        throttle_cmd=0.9,
        brake_cmd=0.0,
        pit_mode=False,
        phase8_gap_ok=False,
        phase8_overlap_flag=False,
        phase8_closing_flag=True,
        phase8_emergency_risk_01=0.40,
        ttl_switch_blocked=False,
    )
    assert d.emergency_stable_mode is False
    assert d.guard_active is True
    assert d.guard_reason in ("reapproach_hold", "steer_rate_limited", "brake_steer_coupled")
    assert d.throttle_cmd <= 0.1501
