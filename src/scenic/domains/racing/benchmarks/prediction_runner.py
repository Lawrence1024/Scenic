"""Benchmark runner for Phase 7 fellow next-step prediction.

Uses ``examples/racing/f_shared/`` and the Phase 7 subset from `PHASE7_F_SCENARIO_NAMES`.
Passes ``-p prediction_enabled True`` so logs include ``[Phase7Prediction]`` lines.
"""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE7_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase7Runner",
            run_id_prefix="phase7",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE7_F_SCENARIO_NAMES,
            scenic_extra_args=("-p", "prediction_enabled", "True"),
            csv_fields=(
                "scenario",
                "scenario_instance",
                "repeat_index",
                "repeat_count",
                "return_code",
                "lap_completion_status",
                "lap_time_s",
                "prediction_line_count",
                "prediction_error_next_step_mean",
                "prediction_error_next_step_max",
                "prediction_error_zero_motion_mean",
                "prediction_error_hold_last_mean",
                "prediction_gain_vs_zero_mean",
                "prediction_regret_vs_hold_mean",
                "prediction_ratio_vs_hold_mean",
                "phase6_state_line_count",
                "phase6_planner_line_count",
                "phase6_guard_line_count",
                "phase6_executor_line_count",
                "phase6_guard_active_count",
                "eval_contact_overlap_count",
                "eval_contact_near_count",
                "eval_contact_overlap_dspace_invalid_count",
                "eval_contact_near_dspace_invalid_count",
                "phase0_samples",
                "phase2_line_count",
                "phase2_assess_errors",
                "collision",
                "off_track",
                "near_miss_count",
                "waypoint_hits",
                "min_opponent_distance_m",
            )
            + FELLOW_HARNESS_SUMMARY_KEYS,
            phase1_switches=False,
            phase2_lines=True,
            phase3_tactical=False,
            fellow_harness=True,
            digest_keys=tuple(standard_benchmark_digest_keys_with_fellow()),
            extra_summary_keys=(
                "prediction_line_count",
                "prediction_error_next_step_mean",
                "prediction_error_next_step_max",
                "prediction_gain_vs_zero_mean",
                "prediction_regret_vs_hold_mean",
                "prediction_ratio_vs_hold_mean",
                "min_opponent_distance_m",
                "eval_contact_overlap_dspace_invalid_count",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
