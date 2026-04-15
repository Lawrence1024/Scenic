"""Phase 3: conservative tactical planner (FREE_RUN / FOLLOW / SETUP_LEFT / SETUP_RIGHT).

Maps Phase 2-style opponent situation into a TTL choice and optional follow speed cap.
Designed to be called each control cycle from ``FollowRacingLineMPCBehavior``.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Dict, Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation

FREE_RUN = "FREE_RUN"
FOLLOW = "FOLLOW"
SETUP_LEFT = "SETUP_LEFT"
SETUP_RIGHT = "SETUP_RIGHT"
SETUP_PASS_LEFT = "SETUP_PASS_LEFT"
SETUP_PASS_RIGHT = "SETUP_PASS_RIGHT"


@dataclass
class TacticalPlannerConfig:
    """Tunable thresholds (meters, seconds, m/s)."""

    relevance_dist_m: float = 95.0
    blocked_longitudinal_m: float = 42.0
    blocked_distance_m: float = 62.0
    pass_safe_risk_max: float = 0.48
    pass_requires_straight: bool = True
    setup_flip_cooldown_s: float = 4.0
    follow_speed_margin_mps: float = 2.5
    follow_tight_gap_m: float = 14.0
    follow_tight_headway_s: float = 0.9
    blocked_headway_s: float = 1.8
    setup_min_headway_s: float = 0.6
    hard_ttc_s: float = 1.4
    pass_min_relative_speed_mps: float = 0.8
    follow_tight_cap_scale: float = 0.92
    setup_min_distance_m: float = 9.0
    setup_partial_overlap_lateral_m: float = 1.8
    setup_reentry_cooldown_s: float = 0.9
    contact_recovery_hold_s: float = 1.0
    protected_follow_release_cycles: int = 3
    setup_commit_entry_cycles: int = 2
    setup_commit_hold_s: float = 1.2
    setup_commit_min_closing_mps: float = 0.3
    pass_intent_entry_cycles: int = 2
    pass_intent_hold_s: float = 1.6
    lateral_path_lock_hold_s: float = 1.6


@dataclass
class TacticalPlannerState:
    mode: str = FREE_RUN
    last_setup_side: str = "left"
    last_flip_sim_time_s: float = -1.0e9
    last_setup_exit_sim_time_s: float = -1.0e9
    setup_candidate_side: str = ""
    setup_candidate_count: int = 0
    follow_pressure_count: int = 0
    protected_follow_active: bool = False
    protected_follow_clear_count: int = 0
    recovery_hold_until_s: float = -1.0e9
    setup_commit_side: str = ""
    setup_commit_candidate_count: int = 0
    setup_commit_until_s: float = -1.0e9
    pass_intent_side: str = ""
    pass_intent_candidate_count: int = 0
    pass_intent_until_s: float = -1.0e9
    lateral_path_lock_side: str = ""
    lateral_path_lock_until_s: float = -1.0e9


def apply_ttl_key_to_agent(agent, ttl_key: str, ttl_cache: dict, file_by_sel: Dict[str, str]) -> bool:
    """Switch ego TTL polyline and invalidate segment/progress caches (Scenic agent)."""
    if ttl_key not in ttl_cache:
        return False
    region_new, pts_new = ttl_cache[ttl_key]
    agent.ttl = region_new
    agent.waypoints = list(pts_new)
    agent.ttl_selection = ttl_key
    agent.ttlFileName = file_by_sel.get(ttl_key, getattr(agent, "ttlFileName", None))
    agent._waypoint_segment_map = None
    agent._last_valid_segment_id = None
    agent._last_valid_segment_name = ""
    agent._cached_cumulative_dist_wp_idx = 0
    agent._cached_cumulative_dist_to_wp = 0.0
    agent._waypoint_progress = 0.0
    agent._waypoint_progress_idx = 0
    return True


def tactical_planner_step(
    state: TacticalPlannerState,
    sit: Optional[OpponentSituation],
    *,
    has_opponent: bool,
    ego_speed_mps: float,
    opponent_speed_mps: float,
    sim_time_s: float,
    pit_mode: bool,
    config: TacticalPlannerConfig,
) -> Tuple[str, str, Optional[float]]:
    """Backward-compatible Phase 3 API.

    Return ``(mode, ttl_key, speed_cap_mps_or_None)``.

    ``ttl_key`` is one of ``optimal``, ``left``, ``right``.
    ``speed_cap_mps`` is applied in FOLLOW when not None.
    """
    mode, ttl_key, cap, _reason = tactical_planner_step_v1(
        state,
        sit,
        has_opponent=has_opponent,
        ego_speed_mps=ego_speed_mps,
        opponent_speed_mps=opponent_speed_mps,
        sim_time_s=sim_time_s,
        pit_mode=pit_mode,
        config=config,
    )
    return mode, ttl_key, cap


def _canonical_mode(mode: str) -> str:
    if mode == SETUP_LEFT:
        return SETUP_PASS_LEFT
    if mode == SETUP_RIGHT:
        return SETUP_PASS_RIGHT
    return mode


def tactical_planner_step_v1(
    state: TacticalPlannerState,
    sit: Optional[OpponentSituation],
    *,
    has_opponent: bool,
    ego_speed_mps: float,
    opponent_speed_mps: float,
    sim_time_s: float,
    pit_mode: bool,
    config: TacticalPlannerConfig,
    assessment_relation: Optional[str] = None,
    assessment_gap_ok: Optional[bool] = None,
    assessment_optimal_open: Optional[bool] = None,
    assessment_left_open: Optional[bool] = None,
    assessment_right_open: Optional[bool] = None,
    assessment_closing_flag: Optional[bool] = None,
    assessment_emergency_risk_01: Optional[float] = None,
) -> Tuple[str, str, Optional[float], str]:
    """Phase 9 API: return ``(mode, ttl_key, speed_cap, decision_reason)``.

    This API adds explainable decision reasons and accepts Phase 8 assessment
    inputs for relation, gap safety, and corridor openness while preserving the
    conservative tactical planner behavior envelope.
    """
    def _follow_result(reason: str) -> Tuple[str, str, Optional[float], str]:
        state.mode = FOLLOW
        cap = float(opponent_speed_mps) + config.follow_speed_margin_mps
        if sit is not None:
            ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
            dynamic_tight_gap = max(
                float(config.follow_tight_gap_m),
                ref_speed * float(config.follow_tight_headway_s),
            )
        else:
            dynamic_tight_gap = float(config.follow_tight_gap_m)
        if sit is not None and sit.distance_m < dynamic_tight_gap:
            cap = min(
                cap,
                float(opponent_speed_mps) * config.follow_tight_cap_scale + 0.5,
            )
        cap = max(3.0, cap)
        return FOLLOW, "optimal", cap, reason

    def _setup_result(side: str, reason: str) -> Tuple[str, str, Optional[float], str]:
        state.last_setup_side = side
        if side == "left":
            state.mode = SETUP_LEFT
            return SETUP_LEFT, "left", None, reason
        state.mode = SETUP_RIGHT
        return SETUP_RIGHT, "right", None, reason

    def _clear_safety_latches() -> None:
        state.protected_follow_active = False
        state.protected_follow_clear_count = 0
        state.recovery_hold_until_s = -1.0e9

    def _clear_setup_commit() -> None:
        state.setup_commit_side = ""
        state.setup_commit_candidate_count = 0
        state.setup_commit_until_s = -1.0e9

    def _clear_pass_intent() -> None:
        state.pass_intent_side = ""
        state.pass_intent_candidate_count = 0
        state.pass_intent_until_s = -1.0e9

    def _clear_lateral_lock() -> None:
        state.lateral_path_lock_side = ""
        state.lateral_path_lock_until_s = -1.0e9

    def _is_proximity_hazard() -> bool:
        if sit is None:
            return False
        overlap_hazard = sit.overlap_state in ("partial_overlap", "side_by_side")
        ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
        dynamic_blocked_gap = max(
            float(config.follow_tight_gap_m),
            float(config.blocked_distance_m),
            ref_speed * float(config.blocked_headway_s),
        )
        close_hazard = sit.distance_m < dynamic_blocked_gap
        return bool(overlap_hazard or close_hazard)

    def _is_release_hazard() -> bool:
        if sit is None:
            return False
        overlap_hazard = sit.overlap_state in ("partial_overlap", "side_by_side")
        ref_speed = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
        dynamic_tight_gap = max(
            float(config.follow_tight_gap_m),
            ref_speed * float(config.follow_tight_headway_s),
        )
        tight_gap_hazard = sit.distance_m < dynamic_tight_gap
        return bool(overlap_hazard or tight_gap_hazard)

    if pit_mode:
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "pit_mode_guard"

    if not has_opponent or sit is None:
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "no_opponent"

    if sit.distance_m > config.relevance_dist_m:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_safety_latches()
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "opponent_far_free_run"

    relation_ahead = bool(sit.ahead)
    if isinstance(assessment_relation, str):
        if assessment_relation == "ahead":
            relation_ahead = True
        elif assessment_relation == "behind":
            relation_ahead = False
    overlap_hazard_now = sit.overlap_state in ("partial_overlap", "side_by_side")
    if overlap_hazard_now:
        state.recovery_hold_until_s = max(
            float(state.recovery_hold_until_s),
            float(sim_time_s) + float(config.contact_recovery_hold_s),
        )
        state.protected_follow_active = True
        state.protected_follow_clear_count = 0
    in_recovery_hold = float(sim_time_s) < float(state.recovery_hold_until_s)
    emergency_risk_high = bool(
        assessment_emergency_risk_01 is not None
        and float(assessment_emergency_risk_01) >= float(config.pass_safe_risk_max)
    )
    left_open_now = True if assessment_left_open is None else bool(assessment_left_open)
    right_open_now = True if assessment_right_open is None else bool(assessment_right_open)
    asymmetric_opening = bool(left_open_now) ^ bool(right_open_now)
    ref_speed_mps = max(0.0, float(ego_speed_mps), float(opponent_speed_mps))
    dynamic_tight_gap_m = max(
        float(config.follow_tight_gap_m),
        ref_speed_mps * float(config.follow_tight_headway_s),
    )
    dynamic_blocked_gap_m = max(
        float(config.blocked_distance_m),
        ref_speed_mps * float(config.blocked_headway_s),
    )
    dynamic_setup_min_gap_m = max(
        float(config.setup_min_distance_m),
        ref_speed_mps * float(config.setup_min_headway_s),
    )
    closing_speed_pos_mps = max(0.0, float(sit.closing_speed_mps))
    ttc_s = (
        (float(sit.distance_m) / closing_speed_pos_mps)
        if closing_speed_pos_mps > 1.0e-6
        else 1.0e9
    )
    opening_window_available = bool(
        (left_open_now or right_open_now)
        and asymmetric_opening
        and (not emergency_risk_high)
        and (not ((assessment_gap_ok is False) and bool(assessment_closing_flag)))
        and (not _is_release_hazard())
        and (not in_recovery_hold)
    )
    hard_hazard_now = bool(
        in_recovery_hold
        or overlap_hazard_now
        or (sit.distance_m < dynamic_tight_gap_m)
        or (ttc_s < float(config.hard_ttc_s))
        or emergency_risk_high
        or ((assessment_gap_ok is False) and (not opening_window_available))
    )
    lateral_lock_side = str(state.lateral_path_lock_side or "")
    lateral_lock_active = bool(
        lateral_lock_side in ("left", "right")
        and float(sim_time_s) < float(state.lateral_path_lock_until_s)
        and (not hard_hazard_now)
    )

    safety_pressure = bool(
        ((assessment_gap_ok is False) and (not opening_window_available))
        or (bool(assessment_closing_flag) and (not opening_window_available))
        or emergency_risk_high
        or (sit.distance_m < dynamic_tight_gap_m)
        or (ttc_s < float(config.hard_ttc_s))
    )

    if relation_ahead and safety_pressure:
        state.protected_follow_active = True
        state.protected_follow_clear_count = 0
    elif state.protected_follow_active:
        # Structural opening-release rule:
        # keep FOLLOW while hazards are present, but allow release into setup
        # when a pass opening is stably clear even if opponent remains ahead.
        opening_stably_clear = bool(
            relation_ahead
            and opening_window_available
            and ((assessment_gap_ok is True) or asymmetric_opening)
            and ((not bool(assessment_closing_flag)) or asymmetric_opening)
        )
        clear_now = opening_stably_clear or (
            (not relation_ahead) and (not _is_release_hazard()) and (not in_recovery_hold)
        )
        if clear_now:
            state.protected_follow_clear_count += 1
        else:
            state.protected_follow_clear_count = 0
        if state.protected_follow_clear_count >= int(config.protected_follow_release_cycles):
            state.protected_follow_active = False
            state.protected_follow_clear_count = 0

    if in_recovery_hold:
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count += 1
        return _follow_result("contact_recovery_hold")

    if state.protected_follow_active:
        _commit_side_pre = str(state.setup_commit_side or "")
        _intent_side_pre = str(state.pass_intent_side or "")
        _lock_side_pre = str(state.lateral_path_lock_side or "")
        _hold_side_pre = _lock_side_pre if _lock_side_pre in ("left", "right") else (
            _commit_side_pre if _commit_side_pre in ("left", "right") else _intent_side_pre
        )
        _hold_active_pre = bool(
            _hold_side_pre in ("left", "right")
            and (
                lateral_lock_active
                or float(sim_time_s) < float(state.setup_commit_until_s)
                or float(sim_time_s) < float(state.pass_intent_until_s)
            )
            and (not in_recovery_hold)
            and (not emergency_risk_high)
            and (not _is_release_hazard())
        )
        if _hold_active_pre:
            return _setup_result(_hold_side_pre, f"lateral_path_lock_{_hold_side_pre}_hold")
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count += 1
        return _follow_result("protected_follow_envelope")

    if not relation_ahead:
        if _is_proximity_hazard():
            state.setup_candidate_side = ""
            state.setup_candidate_count = 0
            state.follow_pressure_count += 1
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
            return _follow_result("proximity_hazard_follow")
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "opponent_behind_free_run"

    blocked = (
        sit.longitudinal_m > 0.0
        and sit.longitudinal_m < config.blocked_longitudinal_m
        and sit.distance_m < dynamic_blocked_gap_m
    )
    if assessment_gap_ok is False:
        blocked = True
    if not blocked:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
        state.mode = FREE_RUN
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        state.follow_pressure_count = 0
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        return FREE_RUN, "optimal", None, "opponent_not_blocking"

    straight_ok = (not config.pass_requires_straight) or (sit.segment_context == "straight")
    overlap_side = sit.overlap_state == "side_by_side"
    overlap_partial_unsafe = (
        sit.overlap_state == "partial_overlap"
        and (
            sit.distance_m < config.setup_min_distance_m
            or abs(float(sit.lateral_m)) < config.setup_partial_overlap_lateral_m
        )
    )
    overlap_unsafe_for_setup = overlap_side or overlap_partial_unsafe
    close_for_setup = sit.distance_m < dynamic_setup_min_gap_m
    pass_safe = (
        straight_ok
        and sit.collision_risk_01 <= config.pass_safe_risk_max
        and not overlap_unsafe_for_setup
        and not (close_for_setup and sit.overlap_state == "side_by_side")
        and (closing_speed_pos_mps >= float(config.pass_min_relative_speed_mps))
    )
    if bool(assessment_closing_flag):
        pass_safe = pass_safe and asymmetric_opening
    if assessment_emergency_risk_01 is not None:
        pass_safe = pass_safe and (not emergency_risk_high)
    left_open = True if assessment_left_open is None else bool(assessment_left_open)
    right_open = True if assessment_right_open is None else bool(assessment_right_open)
    if sit.lateral_relation == "left":
        preferred_side = "right"
    elif sit.lateral_relation == "right":
        preferred_side = "left"
    else:
        preferred_side = state.last_setup_side
    if preferred_side == "left" and not left_open and right_open:
        preferred_side = "right"
    elif preferred_side == "right" and not right_open and left_open:
        preferred_side = "left"
    if assessment_optimal_open is False and assessment_gap_ok is False:
        pass_safe = pass_safe and (left_open or right_open)
    if (not left_open) and (not right_open):
        pass_safe = False

    # Structural safety hold: sustained close-gap pressure blocks setup attempts.
    in_follow_pressure = bool(
        ((assessment_gap_ok is False) and (not opening_window_available))
        or (bool(assessment_closing_flag) and (not opening_window_available))
        or (sit.distance_m < dynamic_tight_gap_m)
        or (ttc_s < float(config.hard_ttc_s))
    )
    if in_follow_pressure:
        state.follow_pressure_count += 1
    else:
        state.follow_pressure_count = max(0, int(state.follow_pressure_count) - 1)

    pass_intent_active = bool(
        state.pass_intent_side in ("left", "right")
        and float(sim_time_s) < float(state.pass_intent_until_s)
    )
    commit_active = bool(
        state.setup_commit_side in ("left", "right")
        and float(sim_time_s) < float(state.setup_commit_until_s)
    )

    # Arm pass intent from FOLLOW/SETUP when opening is consistently good.
    intent_candidate_ok = bool(
        pass_safe
        and opening_window_available
        and preferred_side in ("left", "right")
        and (closing_speed_pos_mps >= float(config.setup_commit_min_closing_mps))
        and (not in_recovery_hold)
    )
    if intent_candidate_ok:
        if state.pass_intent_side != preferred_side:
            state.pass_intent_side = preferred_side
            state.pass_intent_candidate_count = 1
        else:
            state.pass_intent_candidate_count += 1
        if state.pass_intent_candidate_count >= int(config.pass_intent_entry_cycles):
            state.pass_intent_until_s = max(
                float(state.pass_intent_until_s),
                float(sim_time_s) + float(config.pass_intent_hold_s),
            )
            pass_intent_active = True
    else:
        state.pass_intent_candidate_count = max(0, int(state.pass_intent_candidate_count) - 1)

    # Promote pass intent into setup commit hold as soon as side-consistent setup intent exists.
    if pass_intent_active:
        intent_side = str(state.pass_intent_side or preferred_side)
        if intent_side not in ("left", "right"):
            intent_side = preferred_side
        if intent_side in ("left", "right"):
            if state.setup_commit_side != intent_side:
                state.setup_commit_side = intent_side
                state.setup_commit_candidate_count = 1
            else:
                state.setup_commit_candidate_count += 1
            if state.setup_commit_candidate_count >= int(config.setup_commit_entry_cycles):
                state.setup_commit_until_s = max(
                    float(state.setup_commit_until_s),
                    float(sim_time_s) + float(config.setup_commit_hold_s),
                )
                commit_active = True

    hard_commit_hazard = bool(
        in_recovery_hold
        or overlap_unsafe_for_setup
        or (sit.distance_m < dynamic_tight_gap_m)
        or (ttc_s < float(config.hard_ttc_s))
        or emergency_risk_high
        or ((assessment_gap_ok is False) and (not opening_window_available))
    )
    if (pass_intent_active or commit_active) and hard_commit_hazard:
        _clear_setup_commit()
        _clear_pass_intent()
        _clear_lateral_lock()
        pass_intent_active = False
        commit_active = False
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)

    if commit_active:
        side = str(state.setup_commit_side or preferred_side)
        if side not in ("left", "right"):
            side = preferred_side
        if side in ("left", "right"):
            state.lateral_path_lock_side = side
            state.lateral_path_lock_until_s = max(
                float(state.lateral_path_lock_until_s),
                float(sim_time_s) + float(config.lateral_path_lock_hold_s),
            )
            return _setup_result(side, f"setup_commit_{side}_hold")

    if not pass_safe:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        if assessment_gap_ok is False:
            return _follow_result("gap_not_ok_follow")
        return _follow_result("ahead_blocking_follow")

    # Keep FOLLOW while pressure is sustained even if one frame appears pass-safe.
    if state.follow_pressure_count >= 3:
        if state.mode in (SETUP_LEFT, SETUP_RIGHT):
            state.last_setup_exit_sim_time_s = float(sim_time_s)
            _clear_setup_commit()
            _clear_pass_intent()
            _clear_lateral_lock()
        state.setup_candidate_side = ""
        state.setup_candidate_count = 0
        return _follow_result("follow_pressure_hold")

    # Avoid setup chatter: once we exit SETUP due to unsafe/blocked geometry,
    # hold FOLLOW briefly before allowing a fresh setup entry.
    if state.mode not in (SETUP_LEFT, SETUP_RIGHT):
        if float(sim_time_s) - float(state.last_setup_exit_sim_time_s) < config.setup_reentry_cooldown_s:
            state.setup_candidate_side = ""
            state.setup_candidate_count = 0
            return _follow_result("setup_reentry_cooldown_hold")

    target_side = preferred_side

    if state.mode == SETUP_LEFT and target_side == "right":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "left"
            flip_held = True
        else:
            state.last_flip_sim_time_s = sim_time_s
            flip_held = False
    elif state.mode == SETUP_RIGHT and target_side == "left":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "right"
            flip_held = True
        else:
            state.last_flip_sim_time_s = sim_time_s
            flip_held = False
    else:
        flip_held = False

    # Setup entry persistence: require side-consistent setup signal before entering setup.
    if state.mode not in (SETUP_LEFT, SETUP_RIGHT):
        if state.setup_candidate_side != target_side:
            state.setup_candidate_side = target_side
            state.setup_candidate_count = 1
            return _follow_result("setup_candidate_collect")
        state.setup_candidate_count += 1
        if state.setup_candidate_count < 3:
            return _follow_result("setup_candidate_collect")
    else:
        state.setup_candidate_side = target_side
        state.setup_candidate_count = 0

    if state.setup_commit_side and state.setup_commit_side != target_side:
        _clear_setup_commit()
    if state.pass_intent_side and state.pass_intent_side != target_side:
        _clear_pass_intent()
    if state.lateral_path_lock_side and state.lateral_path_lock_side != target_side:
        _clear_lateral_lock()
    if target_side == "left":
        reason = "setup_flip_cooldown_hold" if flip_held else "setup_left_open"
        state.lateral_path_lock_side = "left"
        state.lateral_path_lock_until_s = max(
            float(state.lateral_path_lock_until_s),
            float(sim_time_s) + float(config.lateral_path_lock_hold_s),
        )
        return _setup_result("left", reason)
    reason = "setup_flip_cooldown_hold" if flip_held else "setup_right_open"
    state.lateral_path_lock_side = "right"
    state.lateral_path_lock_until_s = max(
        float(state.lateral_path_lock_until_s),
        float(sim_time_s) + float(config.lateral_path_lock_hold_s),
    )
    return _setup_result("right", reason)


__all__ = [
    "FREE_RUN",
    "FOLLOW",
    "SETUP_LEFT",
    "SETUP_RIGHT",
    "SETUP_PASS_LEFT",
    "SETUP_PASS_RIGHT",
    "TacticalPlannerConfig",
    "TacticalPlannerState",
    "_canonical_mode",
    "apply_ttl_key_to_agent",
    "tactical_planner_step",
    "tactical_planner_step_v1",
]
