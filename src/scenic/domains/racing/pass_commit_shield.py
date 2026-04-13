"""Phase 4: pass commit, abort, and safety shield layered on Phase 3 tactical output.

Runs **after** ``tactical_planner_step`` when enabled. Can promote SETUP_* to
``COMMIT_PASS_*``, emit ``ABORT_PASS`` (tuck to optimal + follow cap), or
``EMERGENCY_AVOID`` (strong speed reduction) when risk / corridor checks fail.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import (
    FOLLOW,
    FREE_RUN,
    SETUP_LEFT,
    SETUP_RIGHT,
    TacticalPlannerConfig,
)

COMMIT_PASS_LEFT = "COMMIT_PASS_LEFT"
COMMIT_PASS_RIGHT = "COMMIT_PASS_RIGHT"
ABORT_PASS = "ABORT_PASS"
EMERGENCY_AVOID = "EMERGENCY_AVOID"


@dataclass
class PassShieldConfig:
    """Thresholds for shield and commit timing."""

    emergency_risk_01: float = 0.82
    abort_commit_risk_01: float = 0.52
    abort_setup_risk_01: float = 0.68
    overlap_abort_risk_01: float = 0.58
    side_by_side_abort_distance_m: float = 9.0
    setup_overlap_abort_distance_m: float = 11.0
    setup_partial_overlap_abort_distance_m: float = 7.5
    overlap_guard_lateral_m: float = 2.2
    commit_dwell_s: float = 0.45
    commit_max_risk_01: float = 0.38
    commit_min_distance_m: float = 10.0
    emergency_speed_scale: float = 0.48
    emergency_overlap_distance_m: float = 4.0
    emergency_overlap_closing_mps: float = 0.25
    emergency_overlap_lateral_m: float = 2.2
    emergency_release_risk_01: float = 0.52
    emergency_release_distance_m: float = 8.0


@dataclass
class PassShieldState:
    commit_active: bool = False
    commit_side: Optional[str] = None
    setup_enter_sim_time_s: float = -1.0e9
    emergency_active: bool = False
    last_mode3: str = ""
    last_ttl3: str = "optimal"


def _follow_cap(opponent_speed_mps: float, config: TacticalPlannerConfig) -> float:
    cap = float(opponent_speed_mps) + config.follow_speed_margin_mps
    return max(3.0, cap)


def _emergency_cap(ego_speed_mps: float, opponent_speed_mps: float, cfg: PassShieldConfig) -> float:
    c = min(
        float(ego_speed_mps) * cfg.emergency_speed_scale,
        float(opponent_speed_mps) + 2.0,
        22.0,
    )
    return max(2.5, c)


def pass_shield_step(
    state: PassShieldState,
    sit: Optional[OpponentSituation],
    mode3: str,
    ttl3: str,
    cap3: Optional[float],
    *,
    has_opponent: bool,
    ego_speed_mps: float,
    opponent_speed_mps: float,
    sim_time_s: float,
    pit_mode: bool,
    tactical_config: TacticalPlannerConfig,
    shield_config: PassShieldConfig,
) -> Tuple[str, str, Optional[float], Optional[str]]:
    """Layer Phase 4 on Phase 3. Returns ``(mode, ttl, cap, reason_or_none)``.

    ``reason`` is a short tag when shield overrides (for logs / metrics).
    """
    if pit_mode or not has_opponent or sit is None:
        state.commit_active = False
        state.commit_side = None
        state.setup_enter_sim_time_s = -1.0e9
        state.emergency_active = False
        state.last_mode3 = mode3
        state.last_ttl3 = ttl3
        return mode3, ttl3, cap3, None

    # --- Emergency (highest priority) ---
    overlap_state = str(getattr(sit, "overlap_state", "") or "")
    lateral_abs = abs(float(getattr(sit, "lateral_m", 0.0) or 0.0))
    overlap_near = overlap_state in ("partial_overlap", "side_by_side")
    emergency_overlap = (
        overlap_near
        and sit.distance_m <= shield_config.emergency_overlap_distance_m
        and sit.closing_speed_mps >= shield_config.emergency_overlap_closing_mps
        and lateral_abs <= shield_config.emergency_overlap_lateral_m
    )
    if sit.collision_risk_01 >= shield_config.emergency_risk_01 or emergency_overlap:
        state.commit_active = False
        state.commit_side = None
        state.setup_enter_sim_time_s = -1.0e9
        state.emergency_active = True
        cap_e = _emergency_cap(ego_speed_mps, opponent_speed_mps, shield_config)
        state.last_mode3 = mode3
        state.last_ttl3 = "optimal"
        if emergency_overlap and sit.collision_risk_01 < shield_config.emergency_risk_01:
            return EMERGENCY_AVOID, "optimal", cap_e, "emergency_overlap"
        return EMERGENCY_AVOID, "optimal", cap_e, "emergency_risk"

    # If emergency mode was previously engaged, hold it until geometry/risk truly clears.
    if state.emergency_active:
        emergency_overlap_hold = overlap_near and lateral_abs <= shield_config.emergency_overlap_lateral_m
        emergency_hold = (
            emergency_overlap_hold
            or sit.collision_risk_01 >= shield_config.emergency_release_risk_01
            or sit.distance_m <= shield_config.emergency_release_distance_m
        )
        if emergency_hold:
            cap_e = _emergency_cap(ego_speed_mps, opponent_speed_mps, shield_config)
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return EMERGENCY_AVOID, "optimal", cap_e, "emergency_hold"
        state.emergency_active = False

    # Phase 3 returned FOLLOW / FREE_RUN — normally end commit, but guard release
    # when vehicles are still in close overlap to avoid abrupt lane-transition churn.
    if mode3 in (FREE_RUN, FOLLOW):
        if state.commit_active and (
            (
                overlap_state in ("side_by_side", "partial_overlap")
                and lateral_abs <= shield_config.overlap_guard_lateral_m
            )
            or (
                sit.distance_m <= shield_config.side_by_side_abort_distance_m
                and lateral_abs <= shield_config.overlap_guard_lateral_m
            )
        ):
            state.commit_active = False
            state.commit_side = None
            state.emergency_active = False
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "release_overlap_guard"
        state.commit_active = False
        state.commit_side = None

    # Track entering SETUP (for dwell-based commit); refresh anchor on TTL change
    if mode3 in (SETUP_LEFT, SETUP_RIGHT):
        if state.last_mode3 not in (SETUP_LEFT, SETUP_RIGHT) or ttl3 != state.last_ttl3:
            state.setup_enter_sim_time_s = float(sim_time_s)
    elif not state.commit_active:
        state.setup_enter_sim_time_s = -1.0e9

    # --- Abort while committed: corridor / overlap ---
    if state.commit_active and state.commit_side is not None:
        if sit.segment_context in ("corner_body", "corner_entry") and sit.collision_risk_01 >= shield_config.abort_commit_risk_01:
            state.commit_active = False
            state.commit_side = None
            state.emergency_active = False
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "corridor"
        if (
            sit.overlap_state == "side_by_side"
            and sit.distance_m <= shield_config.side_by_side_abort_distance_m
            and lateral_abs <= shield_config.overlap_guard_lateral_m
        ):
            state.commit_active = False
            state.commit_side = None
            state.emergency_active = False
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "overlap_side_by_side"
        if (
            sit.overlap_state == "partial_overlap"
            and sit.collision_risk_01 >= shield_config.overlap_abort_risk_01
            and lateral_abs <= shield_config.overlap_guard_lateral_m
        ):
            state.commit_active = False
            state.commit_side = None
            state.emergency_active = False
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "overlap"

    # --- Promote SETUP -> COMMIT on straight with sustained safe dwell ---
    if (
        not state.commit_active
        and mode3 == SETUP_RIGHT
        and ttl3 == "right"
        and sit.segment_context == "straight"
        and sit.collision_risk_01 <= shield_config.commit_max_risk_01
        and sit.distance_m >= shield_config.commit_min_distance_m
        and sit.overlap_state == "clear_ahead"
    ):
        dwell = float(sim_time_s) - float(state.setup_enter_sim_time_s)
        if dwell >= shield_config.commit_dwell_s and state.setup_enter_sim_time_s > -1.0e8:
            state.commit_active = True
            state.commit_side = "right"
            state.last_mode3 = mode3
            state.last_ttl3 = ttl3
            return COMMIT_PASS_RIGHT, ttl3, cap3, "commit_dwell_right"

    if (
        not state.commit_active
        and mode3 == SETUP_LEFT
        and ttl3 == "left"
        and sit.segment_context == "straight"
        and sit.collision_risk_01 <= shield_config.commit_max_risk_01
        and sit.distance_m >= shield_config.commit_min_distance_m
        and sit.overlap_state == "clear_ahead"
    ):
        dwell = float(sim_time_s) - float(state.setup_enter_sim_time_s)
        if dwell >= shield_config.commit_dwell_s and state.setup_enter_sim_time_s > -1.0e8:
            state.commit_active = True
            state.commit_side = "left"
            state.last_mode3 = mode3
            state.last_ttl3 = ttl3
            return COMMIT_PASS_LEFT, ttl3, cap3, "commit_dwell_left"

    # --- Hold commit (same TTL as Phase 3 chose) ---
    if state.commit_active and state.commit_side == "right" and ttl3 == "right" and mode3 == SETUP_RIGHT:
        state.last_mode3 = mode3
        state.last_ttl3 = ttl3
        return COMMIT_PASS_RIGHT, ttl3, cap3, None
    if state.commit_active and state.commit_side == "left" and ttl3 == "left" and mode3 == SETUP_LEFT:
        state.last_mode3 = mode3
        state.last_ttl3 = ttl3
        return COMMIT_PASS_LEFT, ttl3, cap3, None

    # --- Abort risky SETUP before commit ---
    if not state.commit_active and mode3 in (SETUP_LEFT, SETUP_RIGHT):
        if (
            sit.overlap_state == "side_by_side"
            and sit.distance_m <= shield_config.setup_overlap_abort_distance_m
            and lateral_abs <= shield_config.overlap_guard_lateral_m
        ):
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "setup_overlap"
        if (
            sit.overlap_state == "partial_overlap"
            and sit.distance_m <= shield_config.setup_partial_overlap_abort_distance_m
            and lateral_abs <= shield_config.overlap_guard_lateral_m
        ):
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "setup_overlap"
        if sit.collision_risk_01 >= shield_config.abort_setup_risk_01:
            state.last_mode3 = mode3
            state.last_ttl3 = "optimal"
            return ABORT_PASS, "optimal", _follow_cap(opponent_speed_mps, tactical_config), "setup_risk"

    state.last_mode3 = mode3
    state.last_ttl3 = ttl3
    return mode3, ttl3, cap3, None


__all__ = [
    "ABORT_PASS",
    "COMMIT_PASS_LEFT",
    "COMMIT_PASS_RIGHT",
    "EMERGENCY_AVOID",
    "PassShieldConfig",
    "PassShieldState",
    "pass_shield_step",
]
