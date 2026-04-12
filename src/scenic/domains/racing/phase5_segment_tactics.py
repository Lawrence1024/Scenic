"""Phase 5: segment-aware tactical shaping layered on Phase 3 output.

This layer adjusts tactical intent based on segment context before Phase 4 shielding:
- discourage fresh setup attempts at corner entry unless overlap is already established
- block setup in corner body (favor stable follow)
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import (
    FOLLOW,
    SETUP_LEFT,
    SETUP_RIGHT,
    TacticalPlannerConfig,
)


@dataclass
class Phase5SegmentTacticsConfig:
    """Thresholds for segment-aware tactical shaping."""

    corner_entry_requires_overlap: bool = True
    corner_entry_overlap_states_ok: tuple[str, ...] = ("partial_overlap", "side_by_side")
    corner_body_block_setup: bool = True
    use_follow_cap_on_override: bool = True


@dataclass
class Phase5SegmentTacticsState:
    """Runtime state for override/release logging."""

    override_active: bool = False
    last_reason: Optional[str] = None


def _follow_cap(opponent_speed_mps: float, tactical_cfg: TacticalPlannerConfig) -> float:
    cap = float(opponent_speed_mps) + tactical_cfg.follow_speed_margin_mps
    return max(3.0, cap)


def phase5_segment_tactics_step(
    state: Phase5SegmentTacticsState,
    sit: Optional[OpponentSituation],
    mode_in: str,
    ttl_in: str,
    cap_in: Optional[float],
    *,
    has_opponent: bool,
    pit_mode: bool,
    opponent_speed_mps: float,
    tactical_config: TacticalPlannerConfig,
    phase5_config: Phase5SegmentTacticsConfig,
) -> Tuple[str, str, Optional[float], Optional[str]]:
    """Return ``(mode_out, ttl_out, cap_out, reason_or_none)``."""
    if pit_mode or not has_opponent or sit is None:
        state.override_active = False
        state.last_reason = None
        return mode_in, ttl_in, cap_in, None

    seg = str(getattr(sit, "segment_context", "none") or "none")
    overlap = str(getattr(sit, "overlap_state", "none") or "none")
    setup_mode = mode_in in (SETUP_LEFT, SETUP_RIGHT)

    # Corner body: no fresh setup commitments; stabilize into follow.
    if setup_mode and phase5_config.corner_body_block_setup and seg == "corner_body":
        cap = cap_in
        if phase5_config.use_follow_cap_on_override or cap is None:
            cap = _follow_cap(opponent_speed_mps, tactical_config)
        state.override_active = True
        state.last_reason = "body_no_new_setup"
        return FOLLOW, "optimal", cap, "body_no_new_setup"

    # Corner entry: require established overlap before setup.
    if setup_mode and phase5_config.corner_entry_requires_overlap and seg == "corner_entry":
        if overlap not in phase5_config.corner_entry_overlap_states_ok:
            cap = cap_in
            if phase5_config.use_follow_cap_on_override or cap is None:
                cap = _follow_cap(opponent_speed_mps, tactical_config)
            state.override_active = True
            state.last_reason = "entry_conservative"
            return FOLLOW, "optimal", cap, "entry_conservative"

    state.override_active = False
    state.last_reason = None
    return mode_in, ttl_in, cap_in, None


__all__ = [
    "Phase5SegmentTacticsConfig",
    "Phase5SegmentTacticsState",
    "phase5_segment_tactics_step",
]

