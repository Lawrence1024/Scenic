"""Unit tests for stability guard."""

from scenic.domains.racing.safety.stability_guard import (
    StabilityGuardConfig,
    StabilityGuardState,
    stability_guard_step,
    stability_guard_handle_ttl_switch,
)


def test_guard_ttl_switch_rate_limit_blocks_fast_flip():
    state = StabilityGuardState(last_ttl="optimal")
    cfg = StabilityGuardConfig(ttl_switch_min_interval_s=1.0)
    ttl1, blocked1 = stability_guard_handle_ttl_switch(
        state,
        config=cfg,
        sim_time_s=1.0,
        current_ttl="optimal",
        requested_ttl="left",
    )
    assert ttl1 == "left"
    assert blocked1 is False
    ttl2, blocked2 = stability_guard_handle_ttl_switch(
        state,
        config=cfg,
        sim_time_s=1.2,
        current_ttl="left",
        requested_ttl="optimal",
    )
    assert ttl2 == "left"
    assert blocked2 is True


def test_guard_guard_limits_steer_slew():
    state = StabilityGuardState(last_steer_cmd_rad=0.0, last_ttl="optimal")
    cfg = StabilityGuardConfig(max_steer_rate_rad_per_s=1.0)
    d = stability_guard_step(
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
        gap_ok=True,
        overlap_flag=False,
        closing_flag=False,
        emergency_risk_01=0.1,
        ttl_switch_blocked=False,
    )
    assert abs(d.steer_cmd_rad) <= 0.1001
    assert d.steer_limited is True
    assert d.guard_active is True


def test_guard_guard_enters_emergency_stable():
    state = StabilityGuardState(last_ttl="optimal")
    cfg = StabilityGuardConfig(
        emergency_risk_enter_01=0.8,
        emergency_overlap_brake_floor=0.5,
        emergency_max_steer_abs_rad=0.1,
    )
    d = stability_guard_step(
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
        gap_ok=False,
        overlap_flag=True,
        closing_flag=True,
        emergency_risk_01=0.95,
        ttl_switch_blocked=False,
    )
    assert d.emergency_stable_mode is True
    assert d.planner_state == "EMERGENCY_STABLE"
    assert d.throttle_cmd == 0.0
    assert d.brake_cmd >= 0.5
    assert abs(d.steer_cmd_rad) <= 0.1001


def test_guard_commit_pass_bypasses_reapproach_hold():
    """During COMMIT_PASS_RIGHT, reapproach_hold must NOT fire even when gap_ok=False."""
    state = StabilityGuardState(last_ttl="right")
    cfg = StabilityGuardConfig(
        reapproach_retrigger_risk_01=0.45,
        reapproach_max_throttle=0.20,
        reapproach_brake_floor=0.12,
    )
    d = stability_guard_step(
        state,
        config=cfg,
        sim_time_s=4.0,
        control_dt_s=0.05,
        planner_state="COMMIT_PASS_RIGHT",
        active_ttl="right",
        decision_reason="commit_pass_right_hold",
        steer_cmd_rad=0.02,
        throttle_cmd=0.95,
        brake_cmd=0.0,
        pit_mode=False,
        gap_ok=False,
        overlap_flag=False,
        closing_flag=True,
        emergency_risk_01=0.40,
        ttl_switch_blocked=False,
    )
    # Guard should NOT activate reapproach during committed pass.
    assert d.guard_reason != "reapproach_hold"
    assert d.throttle_cmd > 0.20  # Not capped to reapproach_max_throttle
    assert d.brake_cmd < 0.12     # No reapproach brake floor


def test_guard_commit_pass_still_triggers_emergency_on_overlap():
    """Even during COMMIT_PASS, true overlap should still trigger emergency-stable."""
    state = StabilityGuardState(last_ttl="left")
    cfg = StabilityGuardConfig(
        emergency_risk_enter_01=0.85,
        emergency_overlap_brake_floor=0.60,
    )
    d = stability_guard_step(
        state,
        config=cfg,
        sim_time_s=5.0,
        control_dt_s=0.05,
        planner_state="COMMIT_PASS_LEFT",
        active_ttl="left",
        decision_reason="commit_pass_left_hold",
        steer_cmd_rad=0.1,
        throttle_cmd=0.9,
        brake_cmd=0.0,
        pit_mode=False,
        gap_ok=False,
        overlap_flag=True,
        closing_flag=True,
        emergency_risk_01=0.90,
        ttl_switch_blocked=False,
    )
    # Emergency should still fire — overlap is a real collision signal.
    assert d.emergency_stable_mode is True
    assert d.throttle_cmd == 0.0
    assert d.brake_cmd >= 0.60


def test_guard_guard_reapproach_hold_suppresses_throttle_after_emergency():
    state = StabilityGuardState(last_ttl="optimal")
    cfg = StabilityGuardConfig(
        emergency_risk_enter_01=0.8,
        emergency_hold_s=0.5,
        reapproach_recovery_hold_s=1.0,
        reapproach_max_throttle=0.15,
    )
    # Trigger emergency first.
    _ = stability_guard_step(
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
        gap_ok=False,
        overlap_flag=True,
        closing_flag=True,
        emergency_risk_01=0.95,
        ttl_switch_blocked=False,
    )
    # After emergency latch window, recovery hold should still cap throttle.
    d = stability_guard_step(
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
        gap_ok=False,
        overlap_flag=False,
        closing_flag=True,
        emergency_risk_01=0.40,
        ttl_switch_blocked=False,
    )
    assert d.emergency_stable_mode is False
    assert d.guard_active is True
    assert d.guard_reason in ("reapproach_hold", "steer_rate_limited", "brake_steer_coupled")
    assert d.throttle_cmd <= 0.1501
