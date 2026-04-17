"""Race situation assessment helpers (safe-gap, corridor occupancy, emergency risk)."""

from .race_situation import (
    RaceSituationAssessment,
    RaceSituationState,
    assess_race_situation,
    compute_dynamic_safe_gap_m,
    format_assessment_log_line,
    # Backward-compatibility aliases
    Phase8Assessment,
    Phase8AssessmentState,
    assess_phase8_situation_stateful,
    format_phase8_assessment_log_line,
)

__all__ = [
    "RaceSituationAssessment",
    "RaceSituationState",
    "assess_race_situation",
    "compute_dynamic_safe_gap_m",
    "format_assessment_log_line",
    # Backward-compatibility aliases
    "Phase8Assessment",
    "Phase8AssessmentState",
    "assess_phase8_situation_stateful",
    "format_phase8_assessment_log_line",
]
