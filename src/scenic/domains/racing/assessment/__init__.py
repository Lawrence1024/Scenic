"""Race situation assessment helpers (safe-gap, corridor occupancy, emergency risk,
pass-window geometric look-ahead)."""

from .race_situation import (
    RaceSituationAssessment,
    RaceSituationState,
    assess_race_situation,
    compute_dynamic_safe_gap_m,
    format_assessment_log_line,
)
from .pass_geometry import (
    DEFAULT_MIN_LAT_CLEARANCE_M,
    DEFAULT_PASS_DURATION_S,
    DEFAULT_SAMPLE_DT_S,
    pass_window_check,
)

__all__ = [
    "RaceSituationAssessment",
    "RaceSituationState",
    "assess_race_situation",
    "compute_dynamic_safe_gap_m",
    "format_assessment_log_line",
    "pass_window_check",
    "DEFAULT_MIN_LAT_CLEARANCE_M",
    "DEFAULT_PASS_DURATION_S",
    "DEFAULT_SAMPLE_DT_S",
]
