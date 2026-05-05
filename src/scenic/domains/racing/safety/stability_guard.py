"""Stability guard — anti-swerve / anti-chatter / emergency-stable command filter.

Authority model (post-SD-4d):
  - When ``predicted_collision_available=True`` (production caller threads
    polylines through the planner): EMERGENCY_STABLE entry is triggered
    EXCLUSIVELY by ``predicted_collision``. Snapshot heuristics (overlap_flag,
    risk thresholds, gap_ok+closing_flag) are bypassed — they were the source
    of spurious brakes during clear side-by-side passes (F2/F3 contact).
  - When ``predicted_collision_available=False`` (legacy callers, unit tests
    not threading polylines): falls back to today's snapshot logic for backward
    compat.

The legacy fallback is intentional — tests that don't construct full TTL
polylines still work. Production callers always set _available=True.

SD-41E: safety supervisor — pre-MPC reference-swap channel.
``should_swap_for_emergency`` and ``swap_reference_for_emergency`` give the
guard a third channel that runs BETWEEN the planner and the MPC. When an
emergency is predicted, the planner's reference is replaced with a
structured safe-stop trajectory (vx ramps linearly to 0 over ~1.5 s)
before the MPC consumes it. The MPC then tracks the ramp naturally and
produces strong brake commands without any post-MPC command clipping.
This eliminates the SD-36 panic-brake bypass band-aid (Stage F).
"""

from __future__ import annotations

from dataclasses import dataclass

import numpy as np

from scenic.domains.racing.planner.planner_reference import PlannerReference


@dataclass
class StabilityGuardConfig:
    max_steer_rate_rad_per_s: float = 2.8
    high_steer_abs_rad: float = 0.20
    max_brake_when_high_steer: float = 0.35
    ttl_switch_min_interval_s: float = 0.75
    # SD-35: lowered from 0.85 to 0.75. Under the new exponential TTC ramp
    # in race_situation._compute_emergency_risk, 0.75 corresponds to ttc ~1.5 s
    # — last-chance window to floor brakes (1.5g decel ≈ 22 m/s reduction in
    # 1.5 s). Was 0.85 (≈ttc 0.7 s under old metric), too late for the F14
    # active-blocker case where contact occurred before the threshold was crossed.
    emergency_risk_enter_01: float = 0.75
    # SD-35: exit lowered from 0.55 to 0.50. Maintains the original 0.25
    # hysteresis spread between enter and exit thresholds.
    emergency_risk_exit_01: float = 0.50
    emergency_hold_s: float = 0.8
    emergency_brake_floor: float = 0.30
    emergency_overlap_brake_floor: float = 0.60
    emergency_closing_brake_floor: float = 0.45
    emergency_max_steer_abs_rad: float = 0.15
    reapproach_recovery_hold_s: float = 1.2
    reapproach_retrigger_risk_01: float = 0.55
    reapproach_max_throttle: float = 0.20
    reapproach_brake_floor: float = 0.12
    reapproach_release_risk_01: float = 0.35


@dataclass
class StabilityGuardState:
    last_steer_cmd_rad: float = 0.0
    last_ttl: str = "optimal"
    last_ttl_switch_sim_time_s: float = -1.0e9
    emergency_latch_until_s: float = -1.0e9
    recovery_hold_until_s: float = -1.0e9


@dataclass
class StabilityGuardDecision:
    planner_state: str
    active_ttl: str
    decision_reason: str
    steer_cmd_rad: float
    throttle_cmd: float
    brake_cmd: float
    guard_active: bool
    guard_reason: str
    steer_limited: bool
    brake_limited: bool
    ttl_switch_blocked: bool
    emergency_stable_mode: bool



def stability_guard_handle_ttl_switch(
    state: StabilityGuardState,
    *,
    config: StabilityGuardConfig,
    sim_time_s: float,
    current_ttl: str,
    requested_ttl: str,
    planner_state: str = "FREE_RUN",
) -> tuple[str, bool]:
    """Rate-limit TTL switches to reduce path-switch chatter.

    RC-6: when the tactical planner is mid-pass (COMMIT_PASS_LEFT/RIGHT or ABORT_PASS),
    the rate-limit is bypassed. The rate-limit exists to suppress chatter during normal
    driving; planner intent during a committed maneuver wins over chatter-protection.
    Otherwise the planner can choose 'switch to left for overtake', the guard says 'no,
    last switch was 0.5 s ago', and the MPC keeps a stale racing-line reference at the
    worst possible time.
    """
    cur = str(current_ttl or "optimal")
    req = str(requested_ttl or cur)
    if not state.last_ttl:
        state.last_ttl = cur
    if req == cur:
        state.last_ttl = cur
        return cur, False
    if str(planner_state) in ("COMMIT_PASS_LEFT", "COMMIT_PASS_RIGHT", "ABORT_PASS"):
        state.last_ttl = req
        state.last_ttl_switch_sim_time_s = float(sim_time_s)
        return req, False  # planner intent wins; no rate-limit during commit/abort
    dt = float(sim_time_s) - float(state.last_ttl_switch_sim_time_s)
    if dt < float(config.ttl_switch_min_interval_s):
        state.last_ttl = cur
        return cur, True
    state.last_ttl = req
    state.last_ttl_switch_sim_time_s = float(sim_time_s)
    return req, False


def stability_guard_step(
    state: StabilityGuardState,
    *,
    config: StabilityGuardConfig,
    sim_time_s: float,
    control_dt_s: float,
    planner_state: str,
    active_ttl: str,
    decision_reason: str,
    steer_cmd_rad: float,
    throttle_cmd: float,
    brake_cmd: float,
    pit_mode: bool,
    gap_ok: bool,
    overlap_flag: bool,
    closing_flag: bool,
    emergency_risk_01: float,
    ttl_switch_blocked: bool,
    # SD-4b: predicted-path-collision result from tactical_planner_step_v1.
    # When predicted_collision_available is False (legacy callers without
    # polylines), fall back to today's snapshot logic. SD-4d will rewrite
    # the emergency_trigger to use predicted_collision as the sole authority
    # when it IS available — this commit only adds the kwargs (no behavior change).
    predicted_collision: bool = False,
    predicted_collision_available: bool = False,
) -> StabilityGuardDecision:
    """Apply command-level stability constraints and emergency containment."""
    steer = float(steer_cmd_rad)
    throttle = max(0.0, min(1.0, float(throttle_cmd)))
    brake = max(0.0, min(1.0, float(brake_cmd)))
    mode = str(planner_state or "FREE_RUN")
    ttl = str(active_ttl or "optimal")
    reason = str(decision_reason or "none")

    steer_limited = False
    brake_limited = False
    emergency_stable_mode = False
    guard_active = False
    guard_reason = "none"

    dt = max(1.0e-3, float(control_dt_s))
    max_delta = float(config.max_steer_rate_rad_per_s) * dt
    delta = steer - float(state.last_steer_cmd_rad)
    if abs(delta) > max_delta:
        steer = float(state.last_steer_cmd_rad) + (max_delta if delta > 0.0 else -max_delta)
        steer_limited = True
        guard_active = True
        guard_reason = "steer_rate_limited"

    if abs(steer) >= float(config.high_steer_abs_rad) and brake > float(config.max_brake_when_high_steer):
        brake = float(config.max_brake_when_high_steer)
        brake_limited = True
        guard_active = True
        guard_reason = "brake_steer_coupled"

    risk = max(0.0, min(1.0, float(emergency_risk_01)))
    in_pass_maneuver = str(planner_state or "") in (
        "COMMIT_PASS_LEFT", "COMMIT_PASS_RIGHT",
    )
    # SD-4d: emergency_trigger now uses predicted_collision as authority when
    # available. Snapshot heuristics (overlap_flag, risk, gap_ok+closing) only
    # apply as fallback for legacy callers without polyline data. The user's
    # explicit ask: "we only emergency break if we predict the path will collide".
    if bool(predicted_collision_available):
        emergency_trigger = bool((not pit_mode) and predicted_collision)
    else:
        emergency_trigger = bool(
            (not pit_mode)
            and (
                bool(overlap_flag)
                or risk >= float(config.emergency_risk_enter_01)
                or (
                    (not in_pass_maneuver)
                    and (not bool(gap_ok))
                    and bool(closing_flag)
                    and risk >= 0.50
                )
            )
        )
    if emergency_trigger:
        state.emergency_latch_until_s = max(
            float(state.emergency_latch_until_s),
            float(sim_time_s) + float(config.emergency_hold_s),
        )
        state.recovery_hold_until_s = max(
            float(state.recovery_hold_until_s),
            float(sim_time_s) + float(config.reapproach_recovery_hold_s),
        )
    emergency_latched = float(sim_time_s) < float(state.emergency_latch_until_s)
    # SD-4d: when predicted_collision is available, exit_ok is simply
    # "predicted_collision==False". Otherwise fall back to the snapshot exit gate.
    if bool(predicted_collision_available):
        emergency_exit_ok = bool(not predicted_collision)
    else:
        emergency_exit_ok = bool(
            bool(gap_ok)
            and (not bool(overlap_flag))
            and risk <= float(config.emergency_risk_exit_01)
            and (not bool(closing_flag))
        )
    # During ABORT_PASS with safe conditions, force-release both latches immediately.
    # The ego has passed the opponent; holding emergency/recovery braking is unnecessary.
    in_abort_pass = str(planner_state or "") == "ABORT_PASS"
    if in_abort_pass and emergency_exit_ok:
        state.emergency_latch_until_s = min(float(state.emergency_latch_until_s), float(sim_time_s))
        state.recovery_hold_until_s = min(float(state.recovery_hold_until_s), float(sim_time_s))
        emergency_latched = False
    elif emergency_latched and emergency_exit_ok:
        state.emergency_latch_until_s = min(float(state.emergency_latch_until_s), float(sim_time_s))
        emergency_latched = False

    if emergency_latched:
        emergency_stable_mode = True
        guard_active = True
        guard_reason = "emergency_stable"
        mode = "EMERGENCY_STABLE"
        reason = "stability_guard_emergency_stable"
        throttle = 0.0
        if bool(overlap_flag):
            brake = max(brake, float(config.emergency_overlap_brake_floor))
        elif (not bool(gap_ok)) and bool(closing_flag):
            brake = max(brake, float(config.emergency_closing_brake_floor))
        else:
            brake = max(brake, float(config.emergency_brake_floor))
        if abs(steer) > float(config.emergency_max_steer_abs_rad):
            steer = float(config.emergency_max_steer_abs_rad) if steer > 0.0 else -float(config.emergency_max_steer_abs_rad)
            steer_limited = True
    elif (not pit_mode):
        retrigger = bool(
            (not in_pass_maneuver)
            and (
                bool(overlap_flag)
                or ((not bool(gap_ok)) and bool(closing_flag))
                or (risk >= float(config.reapproach_retrigger_risk_01))
            )
        )
        if retrigger:
            state.recovery_hold_until_s = max(
                float(state.recovery_hold_until_s),
                float(sim_time_s) + 0.5,
            )
        recovery_latched = bool(float(sim_time_s) < float(state.recovery_hold_until_s))
        recovery_release_ok = bool(
            bool(gap_ok)
            and (not bool(overlap_flag))
            and (not bool(closing_flag))
            and (risk <= float(config.reapproach_release_risk_01))
        )
        if recovery_latched and recovery_release_ok:
            state.recovery_hold_until_s = min(float(state.recovery_hold_until_s), float(sim_time_s))
            recovery_latched = False
        if recovery_latched:
            guard_active = True
            if guard_reason == "none":
                guard_reason = "reapproach_hold"
            throttle = min(throttle, float(config.reapproach_max_throttle))
            if (not bool(gap_ok)) or bool(closing_flag):
                brake = max(brake, float(config.reapproach_brake_floor))
                # SD-38: enforce throttle/brake mutual exclusion when the
                # reapproach_hold brake floor fires. The guard previously
                # capped throttle (above) AND floored brake independently,
                # producing simultaneous outputs (e.g. throttle=0.20 and
                # brake=0.12 in F14 t=21.8s onward). A coordinated controller
                # never commands both. Brake wins -- if we believe the
                # situation is bad enough to floor brake, it's bad enough
                # to release throttle entirely.
                if brake > 0.05:
                    throttle = 0.0

    if ttl_switch_blocked:
        guard_active = True
        if guard_reason == "none":
            guard_reason = "ttl_switch_blocked"

    state.last_steer_cmd_rad = float(steer)
    state.last_ttl = str(ttl or state.last_ttl or "optimal")
    return StabilityGuardDecision(
        planner_state=mode,
        active_ttl=ttl,
        decision_reason=reason,
        steer_cmd_rad=float(steer),
        throttle_cmd=float(throttle),
        brake_cmd=float(brake),
        guard_active=bool(guard_active),
        guard_reason=str(guard_reason),
        steer_limited=bool(steer_limited),
        brake_limited=bool(brake_limited),
        ttl_switch_blocked=bool(ttl_switch_blocked),
        emergency_stable_mode=bool(emergency_stable_mode),
    )


def should_swap_for_emergency(
    *,
    config: StabilityGuardConfig,
    pit_mode: bool,
    predicted_collision: bool,
    predicted_collision_available: bool,
    overlap_flag: bool = False,
    closing_flag: bool = False,
    gap_ok: bool = True,
    emergency_risk_01: float = 0.0,
    in_pass_maneuver: bool = False,
) -> bool:
    """SD-41E: pre-MPC trigger that mirrors stability_guard_step's emergency_trigger.

    Returns True iff the safety supervisor should swap the planner's
    reference for a safe-stop reference this tick. Same authority model:
    predicted_collision is sole authority when available; snapshot
    heuristics (overlap / risk / closing+gap) are the legacy fallback.
    """
    if pit_mode:
        return False
    if predicted_collision_available:
        return bool(predicted_collision)
    risk = max(0.0, min(1.0, float(emergency_risk_01)))
    return bool(
        bool(overlap_flag)
        or risk >= float(config.emergency_risk_enter_01)
        or (
            (not in_pass_maneuver)
            and (not bool(gap_ok))
            and bool(closing_flag)
            and risk >= 0.50
        )
    )


def swap_reference_for_emergency(
    ref: PlannerReference,
    *,
    current_speed_mps: float,
    control_dt_s: float,
    ramp_duration_s: float = 1.5,
) -> PlannerReference:
    """SD-41E: return a safe-stop variant of the planner reference.

    The lateral profile (x_m / y_m / psi_rad / kappa_radpm / s_m) is
    preserved so the MPC continues tracking the same TTL while braking.
    Only the longitudinal columns change:

      ramp_to_zero[i] = max(0, v0 * (1 - t_i / T))
      vx_mps[i] = min(planner_vx[i], ramp_to_zero[i])

    The element-wise min is load-bearing: when the planner is already
    commanding a deep brake (e.g. ABORT_PASS with vx=3 m/s, or FOLLOW
    behind an active blocker with vx=12 m/s), we MUST NOT raise the
    reference above the planner's intent. Initial Stage E used
    `vx_mps = ramp_to_zero` directly, anchoring the ramp at
    current_speed_mps; this produced an *acceleration* command at the
    moment of predicted collision when the planner was already braking.
    Observed F14 t=17.0s: planner vx0=12.75, ramp vx0=34.19 — MPC saw
    v_error=0 and produced no brake → contact.

    With the min(): emergency means "at least as much brake as the
    planner already asked for, plus more along the horizon if the
    ramp demands it". The ramp-to-zero adds force at the *tail* of
    the horizon (where the planner's ABORT might level off), giving
    the MPC a cleaner profile to track to a stop.

    The swapped reference is marked ``is_safe_stop=True`` and its mode
    is overwritten to ``EMERGENCY_STABLE`` so downstream telemetry can
    distinguish "planner intent" from "supervisor override".

    Why ~1.5 s ramp: at 25 m/s ego this asks for ~1.7 g longitudinal —
    within the IAC Dallara's brake envelope while leaving margin.
    Shorter ramps over-saturate the brake; longer ramps may not stop
    in time.
    """
    if ref is None or ref.vx_mps is None or ref.vx_mps.size == 0:
        return ref
    n = int(ref.vx_mps.shape[0])
    dt = max(1.0e-3, float(control_dt_s))
    T = max(dt, float(ramp_duration_s))
    v0 = max(0.0, float(current_speed_mps))

    t_grid = np.arange(n, dtype=np.float64) * dt
    ramp_to_zero = np.maximum(0.0, v0 * (1.0 - t_grid / T))
    planner_vx = np.asarray(ref.vx_mps, dtype=np.float64)
    vx = np.minimum(planner_vx, ramp_to_zero)

    # ax recomputed from the actual vx to stay consistent with the profile
    # the MPC will track. Forward difference; tail copies the previous step.
    ax = np.zeros(n, dtype=np.float64)
    if n >= 2 and dt > 0.0:
        ax[:-1] = (vx[1:] - vx[:-1]) / dt
        ax[-1] = ax[-2]

    return PlannerReference(
        s_m=ref.s_m,
        x_m=ref.x_m,
        y_m=ref.y_m,
        psi_rad=ref.psi_rad,
        kappa_radpm=ref.kappa_radpm,
        vx_mps=vx,
        ax_mps2=ax,
        traj_id=int(ref.traj_id),
        t_planner_stamp_s=float(ref.t_planner_stamp_s),
        mode="EMERGENCY_STABLE",
        decision_reason="safety_supervisor_safe_stop",
        is_safe_stop=True,
        binding_cap_source="safe_stop",
        ttl_key=str(ref.ttl_key or "optimal"),
    )


def format_stability_guard_log_line(sim_time_s: float, decision: StabilityGuardDecision) -> str:
    """Structured telemetry for guard log parse."""
    return (
        f"[Guard] t={sim_time_s:.2f}s guard_active={1 if decision.guard_active else 0} "
        f"guard_reason={decision.guard_reason} steer_limited={1 if decision.steer_limited else 0} "
        f"brake_limited={1 if decision.brake_limited else 0} ttl_switch_blocked={1 if decision.ttl_switch_blocked else 0} "
        f"emergency_stable_mode={1 if decision.emergency_stable_mode else 0} "
        f"planner_state={decision.planner_state} active_ttl={decision.active_ttl} "
        f"decision_reason={decision.decision_reason} steer={decision.steer_cmd_rad:.3f} "
        f"throttle={decision.throttle_cmd:.3f} brake={decision.brake_cmd:.3f}"
    )
