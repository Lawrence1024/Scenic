"""Stability guard package."""

from scenic.domains.racing.safety.stability_guard import (
    StabilityGuardConfig,
    StabilityGuardDecision,
    StabilityGuardState,
    format_stability_guard_log_line,
    stability_guard_step,
    stability_guard_handle_ttl_switch,
    # Backward-compatibility aliases
    Phase10StabilityGuardConfig,
    Phase10StabilityGuardDecision,
    Phase10StabilityGuardState,
    format_phase10_guard_log_line,
    phase10_guard_step,
    phase10_handle_ttl_switch,
)

__all__ = [
    "StabilityGuardConfig",
    "StabilityGuardDecision",
    "StabilityGuardState",
    "format_stability_guard_log_line",
    "stability_guard_step",
    "stability_guard_handle_ttl_switch",
    # Backward-compatibility aliases
    "Phase10StabilityGuardConfig",
    "Phase10StabilityGuardDecision",
    "Phase10StabilityGuardState",
    "format_phase10_guard_log_line",
    "phase10_guard_step",
    "phase10_handle_ttl_switch",
]
