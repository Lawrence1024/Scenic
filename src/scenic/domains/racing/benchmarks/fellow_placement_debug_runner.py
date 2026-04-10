"""Benchmark runner for fellow placement debugging scenarios.

This runner targets placement correctness/repro:
- commanded _racing_st_offset vs observed [Fellow s,t],
- road consistency at spawn (ego vs fellow projected road),
- optional [FellowHarness] runtime continuity metrics.

Usage:
    python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner
    python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner --repeats 5
"""

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    FELLOW_PLACEMENT_DEBUG_DIGEST_KEYS,
    FELLOW_PLACEMENT_DEBUG_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
)


if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="FellowPlacementDebugRunner",
                run_id_prefix="fellow_placement_debug",
                default_scenario_dir="examples/racing/fellow_placement_debug",
                default_sim_steps=2000,
                phase1_switches=False,
                phase2_lines=True,
                phase3_tactical=False,
                fellow_harness=True,
                fellow_placement_debug=True,
                default_repeats=1,
                digest_keys=FELLOW_PLACEMENT_DEBUG_DIGEST_KEYS,
                csv_fields=(
                    "scenario",
                    "scenario_instance",
                    "repeat_index",
                    "repeat_count",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
                    "collision",
                    "off_track",
                    "near_miss_count",
                    "waypoint_hits",
                    "min_opponent_distance_m",
                    "phase0_samples",
                    "phase2_line_count",
                    "phase2_assess_errors",
                )
                + FELLOW_HARNESS_SUMMARY_KEYS
                + FELLOW_PLACEMENT_DEBUG_SUMMARY_KEYS,
                extra_summary_keys=(
                    "fellow_placement_from_ego_offset_observed",
                    "placement_s_error_m",
                    "placement_t_error_m",
                    "road_id_mismatch",
                    "unexpected_pit_projection",
                ),
            )
        )
    )

