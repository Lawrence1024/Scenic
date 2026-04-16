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
    "F9_fellow_stationary_roadside_obstacle.scenic",
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

PHASE9_F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F0_ego_alone.scenic",
    "F1_fellow_behind_optimal_cruise.scenic",
    "F2_fellow_ahead_optimal_slower.scenic",
    "F6_fellow_left_occupied_deterministic.scenic",
    "F7_fellow_right_occupied_deterministic.scenic",
)

PHASE10_F_SCENARIO_NAMES: Tuple[str, ...] = (
    "F2_fellow_ahead_optimal_slower.scenic",
    "F4_fellow_ahead_sudden_stop.scenic",
    "F5_fellow_ahead_swerve_out_of_control.scenic",
    "F6_fellow_left_occupied_deterministic.scenic",
    "F7_fellow_right_occupied_deterministic.scenic",
)

PHASE11_F_SCENARIO_NAMES: Tuple[str, ...] = (
    # Focused Phase 11 debug set: left-TTL pass, right-TTL pass, roadside obstacle.
    # F2/F4/F5 commented out until F3L/F3R/F9 overtake is proven working.
    "F3L_fellow_ahead_left_cruise.scenic",          # fellow on left TTL, slow (20 mph) — right-side pass
    "F3R_fellow_ahead_right_cruise.scenic",         # fellow on right TTL, slow (20 mph) — left-side pass
    "F9_fellow_stationary_roadside_obstacle.scenic", # parked on shoulder — trivial pass
    # "F2_fellow_ahead_optimal_slower.scenic",        # optimal TTL, slow fellow (20 mph) — primary overtake case
    # "F4_fellow_ahead_sudden_stop.scenic",           # sudden stop — emergency response
    # "F5_fellow_ahead_swerve_out_of_control.scenic", # swerve — abort path
)

PHASE12_F_SCENARIO_NAMES: Tuple[str, ...] = (
    # Corner-entry scenarios (Phase 12 segment-aware validation)
    "F8_corner_entry_fellow_ahead_optimal.scenic",
    "F10_corner_entry_fellow_left_occupied.scenic",
    "F11_corner_entry_fellow_right_occupied.scenic",
    "F12_corner_entry_fellow_sudden_stop.scenic",
)

PHASE_TO_DEFAULT_F_SCENARIO_NAMES: Dict[int, Tuple[str, ...]] = {
    6: PHASE6_F_SCENARIO_NAMES,
    7: PHASE7_F_SCENARIO_NAMES,
    8: PHASE8_F_SCENARIO_NAMES,
    9: PHASE9_F_SCENARIO_NAMES,
    10: PHASE10_F_SCENARIO_NAMES,
    11: PHASE11_F_SCENARIO_NAMES,
    12: PHASE12_F_SCENARIO_NAMES,
}
