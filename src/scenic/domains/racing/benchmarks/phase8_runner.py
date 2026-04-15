"""Benchmark runner for Phase 8 assessment + dynamic safe-gap telemetry."""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE8_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase8Runner",
            run_id_prefix="phase8",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE8_F_SCENARIO_NAMES,
            scenic_extra_args=(
                "-p",
                "phase7_prediction_enabled",
                "True",
                "-p",
                "phase8_assessment_enabled",
                "True",
            ),
            csv_fields=(
                "scenario",
                "scenario_instance",
                "repeat_index",
                "repeat_count",
                "return_code",
                "lap_completion_status",
                "lap_time_s",
                "phase8_assessment_line_count",
                "phase8_fellow_relation_ahead_count",
                "phase8_fellow_relation_behind_count",
                "phase8_gap_ok_rate",
                "phase8_safe_gap_mean",
                "phase8_actual_gap_mean",
                "phase8_optimal_open_rate",
                "phase8_left_open_rate",
                "phase8_right_open_rate",
                "phase8_closing_flag_rate",
                "phase8_emergency_risk_mean",
                "phase7_prediction_error_next_step_mean",
                "phase7_prediction_ratio_vs_hold_mean",
                "phase6_state_line_count",
                "phase6_planner_line_count",
                "phase6_guard_line_count",
                "phase6_executor_line_count",
                "eval_contact_overlap_count",
                "eval_contact_near_count",
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
                "phase8_assessment_line_count",
                "phase8_gap_ok_rate",
                "phase8_safe_gap_mean",
                "phase8_optimal_open_rate",
                "phase8_left_open_rate",
                "phase8_right_open_rate",
                "phase8_emergency_risk_mean",
                "phase7_prediction_error_next_step_mean",
                "phase7_prediction_ratio_vs_hold_mean",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
