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
    follow_tight_cap_scale: float = 0.92


@dataclass
class TacticalPlannerState:
    mode: str = FREE_RUN
    last_setup_side: str = "left"
    last_flip_sim_time_s: float = -1.0e9


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
    """Return ``(mode, ttl_key, speed_cap_mps_or_None)``.

    ``ttl_key`` is one of ``optimal``, ``left``, ``right``.
    ``speed_cap_mps`` is applied in FOLLOW when not None.
    """
    if pit_mode:
        state.mode = FREE_RUN
        return FREE_RUN, "optimal", None

    if not has_opponent or sit is None:
        state.mode = FREE_RUN
        return FREE_RUN, "optimal", None

    if sit.distance_m > config.relevance_dist_m:
        state.mode = FREE_RUN
        return FREE_RUN, "optimal", None

    if not sit.ahead:
        state.mode = FREE_RUN
        return FREE_RUN, "optimal", None

    blocked = (
        sit.longitudinal_m > 0.0
        and sit.longitudinal_m < config.blocked_longitudinal_m
        and sit.distance_m < config.blocked_distance_m
    )
    if not blocked:
        state.mode = FREE_RUN
        return FREE_RUN, "optimal", None

    straight_ok = (not config.pass_requires_straight) or (sit.segment_context == "straight")
    pass_safe = (
        straight_ok
        and sit.collision_risk_01 <= config.pass_safe_risk_max
        and sit.overlap_state != "side_by_side"
    )

    if not pass_safe:
        state.mode = FOLLOW
        cap = float(opponent_speed_mps) + config.follow_speed_margin_mps
        if sit.distance_m < config.follow_tight_gap_m:
            cap = min(
                cap,
                float(opponent_speed_mps) * config.follow_tight_cap_scale + 0.5,
            )
        cap = max(3.0, cap)
        return FOLLOW, "optimal", cap

    if sit.lateral_relation == "left":
        target_side = "right"
    elif sit.lateral_relation == "right":
        target_side = "left"
    else:
        target_side = state.last_setup_side

    if state.mode == SETUP_LEFT and target_side == "right":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "left"
        else:
            state.last_flip_sim_time_s = sim_time_s
    elif state.mode == SETUP_RIGHT and target_side == "left":
        if sim_time_s - state.last_flip_sim_time_s < config.setup_flip_cooldown_s:
            target_side = "right"
        else:
            state.last_flip_sim_time_s = sim_time_s

    state.last_setup_side = target_side
    if target_side == "left":
        state.mode = SETUP_LEFT
        return SETUP_LEFT, "left", None
    state.mode = SETUP_RIGHT
    return SETUP_RIGHT, "right", None


__all__ = [
    "FREE_RUN",
    "FOLLOW",
    "SETUP_LEFT",
    "SETUP_RIGHT",
    "TacticalPlannerConfig",
    "TacticalPlannerState",
    "apply_ttl_key_to_agent",
    "tactical_planner_step",
]
