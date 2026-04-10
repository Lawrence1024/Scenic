"""Benchmark runner for the fellow / traffic harness (not an ego phase).

Validates opponent placement logs and optional ``[FellowHarness]`` readback lines.
Default: ``examples/racing/fellow_smoke``, **2000** steps (~20 s at 0.01 s/step).

Usage (repo root)::

    python -m scenic.domains.racing.benchmarks.fellow_runner

Override horizon: ``--time 3000`` (~30 s). Outputs match other phase runners
(``summary.json``, ``summary.csv``, ``logs/*.log``, ``BENCHMARK_AI_DIGEST_*``).
"""

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_DIGEST_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
)

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="FellowHarnessRunner",
                run_id_prefix="fellow_smoke",
                default_scenario_dir="examples/racing/fellow_smoke",
                default_sim_steps=2000,
                phase1_switches=False,
                phase2_lines=True,
                phase3_tactical=False,
                fellow_harness=True,
                digest_keys=FELLOW_HARNESS_DIGEST_KEYS,
                csv_fields=(
                    "scenario",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
                    "collision",
                    "off_track",
                    "near_miss_count",
                    "waypoint_hits",
                    "phase0_samples",
                    "phase2_line_count",
                    "phase2_assess_errors",
                    "min_opponent_distance_m",
                    "fellow_placement_from_ego_offset_observed",
                    "fellow_st_log_present",
                    "fellow_s0",
                    "fellow_t0",
                    "fellow_t_out_of_band",
                    "fellow_harness_line_count",
                    "fellow_speed_min_mps",
                    "fellow_speed_max_mps",
                    "fellow_max_speed_step_jump_mps",
                    "fellow_speed_stuck_near_zero",
                    "fellow_position_range_m",
                ),
                extra_summary_keys=(
                    "fellow_harness_line_count",
                    "fellow_placement_from_ego_offset_observed",
                ),
            )
        )
    )
