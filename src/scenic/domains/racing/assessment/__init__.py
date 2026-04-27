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
    DEFAULT_COLLISION_BREACH_DEBOUNCE,
    DEFAULT_COLLISION_HORIZON_S,
    DEFAULT_COLLISION_MIN_CLEAR_M,
    DEFAULT_COLLISION_SAMPLE_DT_S,
    DEFAULT_MIN_LAT_CLEARANCE_M,
    DEFAULT_PASS_DURATION_S,
    DEFAULT_SAMPLE_DT_S,
    pass_window_check,
    path_collision_predicted,
    select_tracks_for_state,
)

__all__ = [
    "RaceSituationAssessment",
    "RaceSituationState",
    "assess_race_situation",
    "compute_dynamic_safe_gap_m",
    "format_assessment_log_line",
    "pass_window_check",
    "path_collision_predicted",
    "select_tracks_for_state",
    "DEFAULT_MIN_LAT_CLEARANCE_M",
    "DEFAULT_PASS_DURATION_S",
    "DEFAULT_SAMPLE_DT_S",
    "DEFAULT_COLLISION_HORIZON_S",
    "DEFAULT_COLLISION_SAMPLE_DT_S",
    "DEFAULT_COLLISION_MIN_CLEAR_M",
    "DEFAULT_COLLISION_BREACH_DEBOUNCE",
]
