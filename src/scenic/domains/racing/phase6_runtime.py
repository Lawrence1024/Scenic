"""Phase 6 orchestration shells and observability helpers.

Phase 6 intentionally keeps behavior conservative while introducing explicit layer
boundaries which can be evolved in later phases.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional


def _headway_distance_m(speed_mps: float, time_headway_s: float, floor_m: float) -> float:
    """Minimum distance implied by time headway at ``speed_mps`` (Phase 8-style dynamic gap).

    ``floor_m`` is a low-speed physical floor so near-stationary ego still keeps a real gap.
    Aligns with ``phase-8-situation-assessment-and-dynamic-gap.md`` (time-headway baseline).
    """

    v = max(0.0, float(speed_mps))
    return max(float(floor_m), v * float(time_headway_s))


FREE_RUN = "FREE_RUN"
FOLLOW = "FOLLOW"
PIT_YIELD = "PIT_YIELD"
NO_OPPONENT = "no_opponent"


@dataclass
class Phase6StateSnapshot:
    """Normalized state snapshot from the Phase 6 state-extraction shell."""

    has_opponent: bool
    pit_mode: bool
    current_ttl: str
    ego_speed_mps: float
    opponent_speed_mps: Optional[float]
    opponent_distance_m: Optional[float]
    overlap_state: str
    segment_context: str
    ahead_flag: bool


@dataclass
class Phase6PlannerDecision:
    """Planner shell output (still conservative in Phase 6)."""

    planner_state: str
    active_ttl: str
    target_speed_cap_mps: Optional[float]
    decision_reason: str


@dataclass
class Phase6GuardDecision:
    """Guard shell output (pass-through in Phase 6)."""

    planner_state: str
    active_ttl: str
    target_speed_cap_mps: Optional[float]
    decision_reason: str
    guard_active: bool
    guard_reason: str
    steer_limited: bool
    brake_limited: bool
    ttl_switch_blocked: bool
    emergency_stable_mode: bool


def build_phase6_state_snapshot(
    *,
    has_opponent: bool,
    pit_mode: bool,
    current_ttl: str,
    ego_speed_mps: float,
    opponent_speed_mps: Optional[float],
    opponent_distance_m: Optional[float],
    overlap_state: Optional[str],
    segment_context: Optional[str],
    ahead_flag: Optional[bool],
) -> Phase6StateSnapshot:
    """State extraction shell used each control cycle."""

    return Phase6StateSnapshot(
        has_opponent=bool(has_opponent),
        pit_mode=bool(pit_mode),
        current_ttl=str(current_ttl or "optimal"),
        ego_speed_mps=float(ego_speed_mps or 0.0),
        opponent_speed_mps=float(opponent_speed_mps) if opponent_speed_mps is not None else None,
        opponent_distance_m=float(opponent_distance_m) if opponent_distance_m is not None else None,
        overlap_state=str(overlap_state or "none"),
        segment_context=str(segment_context or "none"),
        ahead_flag=bool(ahead_flag) if ahead_flag is not None else False,
    )


def phase6_planner_step(
    state: Phase6StateSnapshot,
    *,
    target_speed_mps: float,
    speed_cap_mps: Optional[float],
) -> Phase6PlannerDecision:
    """Conservative planner shell for Phase 6 (no advanced tactical behavior yet)."""

    base_cap = float(target_speed_mps or 0.0)
    if speed_cap_mps is not None:
        base_cap = min(base_cap, float(speed_cap_mps))

    if state.pit_mode:
        return Phase6PlannerDecision(
            planner_state=PIT_YIELD,
            active_ttl=state.current_ttl,
            target_speed_cap_mps=base_cap,
            decision_reason="pit_mode_segment_guard",
        )

    if not state.has_opponent:
        return Phase6PlannerDecision(
            planner_state=FREE_RUN,
            active_ttl=state.current_ttl,
            target_speed_cap_mps=base_cap,
            decision_reason=NO_OPPONENT,
        )

    # Keep Phase 6 planner simple: single speed-scaled follow band (headway + floor).
    # Detailed overlap/close-gap shaping is deferred to Phase 8+ assessment/planner layers.
    follow_trigger_m = _headway_distance_m(
        float(state.ego_speed_mps), time_headway_s=1.35, floor_m=32.0
    )
    if state.ahead_flag and state.opponent_distance_m is not None and state.opponent_distance_m <= follow_trigger_m:
        capped = base_cap
        if state.opponent_speed_mps is not None:
            capped = min(capped, max(6.0, float(state.opponent_speed_mps) + 1.5))
        return Phase6PlannerDecision(
            planner_state=FOLLOW,
            active_ttl=state.current_ttl,
            target_speed_cap_mps=capped,
            decision_reason="opponent_ahead_follow_band",
        )

    return Phase6PlannerDecision(
        planner_state=FREE_RUN,
        active_ttl=state.current_ttl,
        target_speed_cap_mps=base_cap,
        decision_reason="opponent_not_blocking",
    )


def phase6_guard_step(decision: Phase6PlannerDecision) -> Phase6GuardDecision:
    """Safety guard shell (pass-through for Phase 6 baseline wiring)."""

    return Phase6GuardDecision(
        planner_state=decision.planner_state,
        active_ttl=decision.active_ttl,
        target_speed_cap_mps=decision.target_speed_cap_mps,
        decision_reason=decision.decision_reason,
        guard_active=False,
        guard_reason="none",
        steer_limited=False,
        brake_limited=False,
        ttl_switch_blocked=False,
        emergency_stable_mode=False,
    )


def format_phase6_state_log_line(sim_time_s: float, state: Phase6StateSnapshot) -> str:
    dist_s = f"{state.opponent_distance_m:.2f}" if state.opponent_distance_m is not None else "na"
    opp_v_s = f"{state.opponent_speed_mps:.2f}" if state.opponent_speed_mps is not None else "na"
    return (
        f"[Phase6State] t={sim_time_s:.2f}s has_opponent={1 if state.has_opponent else 0} "
        f"pit_mode={1 if state.pit_mode else 0} ttl={state.current_ttl} ego_speed={state.ego_speed_mps:.2f} "
        f"opp_speed={opp_v_s} opp_dist={dist_s} overlap={state.overlap_state} seg={state.segment_context} "
        f"ahead={1 if state.ahead_flag else 0}"
    )


def format_phase6_planner_log_line(sim_time_s: float, decision: Phase6PlannerDecision) -> str:
    cap_s = f"{decision.target_speed_cap_mps:.2f}" if decision.target_speed_cap_mps is not None else "na"
    return (
        f"[Phase6Planner] t={sim_time_s:.2f}s planner_state={decision.planner_state} "
        f"active_ttl={decision.active_ttl} target_speed_cap={cap_s} decision_reason={decision.decision_reason}"
    )


def format_phase6_guard_log_line(sim_time_s: float, guard: Phase6GuardDecision) -> str:
    return (
        f"[Phase6Guard] t={sim_time_s:.2f}s guard_active={1 if guard.guard_active else 0} "
        f"guard_reason={guard.guard_reason} steer_limited={1 if guard.steer_limited else 0} "
        f"brake_limited={1 if guard.brake_limited else 0} ttl_switch_blocked={1 if guard.ttl_switch_blocked else 0} "
        f"emergency_stable_mode={1 if guard.emergency_stable_mode else 0}"
    )


def format_phase6_executor_log_line(
    sim_time_s: float,
    *,
    active_ttl: str,
    planner_state: str,
    decision_reason: str,
    final_steer: float,
    final_throttle: float,
    final_brake: float,
) -> str:
    return (
        f"[Phase6Executor] t={sim_time_s:.2f}s executor_call=1 planner_state={planner_state} "
        f"active_ttl={active_ttl} decision_reason={decision_reason} "
        f"steer={float(final_steer):.3f} throttle={float(final_throttle):.3f} brake={float(final_brake):.3f}"
    )
