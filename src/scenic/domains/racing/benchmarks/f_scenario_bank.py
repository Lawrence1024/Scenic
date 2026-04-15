"""Shared F-scenario bank definitions used across post-Phase-5 runners."""

from __future__ import annotations

from typing import Dict, Tuple


F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F0_ego_alone.scenic",
    "F1_fellow_behind_optimal_cruise.scenic",
    "F2_fellow_ahead_optimal_slower.scenic",
    "F3L_fellow_ahead_left_cruise.scenic",
    "F3R_fellow_ahead_right_cruise.scenic",
    "F4_fellow_ahead_sudden_stop.scenic",
    "F5_fellow_ahead_swerve_out_of_control.scenic",
    "F6_fellow_left_occupied_deterministic.scenic",
    "F7_fellow_right_occupied_deterministic.scenic",
    "F8_corner_entry_fellow_ahead_optimal.scenic",
)

PHASE6_F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F0_ego_alone.scenic",
    "F1_fellow_behind_optimal_cruise.scenic",
    "F2_fellow_ahead_optimal_slower.scenic",
)

PHASE7_F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F2_fellow_ahead_optimal_slower.scenic",
    "F4_fellow_ahead_sudden_stop.scenic",
    "F5_fellow_ahead_swerve_out_of_control.scenic",
    "F6_fellow_left_occupied_deterministic.scenic",
    "F7_fellow_right_occupied_deterministic.scenic",
)

PHASE8_F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F1_fellow_behind_optimal_cruise.scenic",
    "F2_fellow_ahead_optimal_slower.scenic",
    "F4_fellow_ahead_sudden_stop.scenic",
    "F6_fellow_left_occupied_deterministic.scenic",
    "F7_fellow_right_occupied_deterministic.scenic",
)

PHASE_TO_DEFAULT_F_SCENARIO_NAMES: Dict[int, Tuple[str, ...]] = {
    6: PHASE6_F_SCENARIO_NAMES,
    7: PHASE7_F_SCENARIO_NAMES,
    8: PHASE8_F_SCENARIO_NAMES,
}
