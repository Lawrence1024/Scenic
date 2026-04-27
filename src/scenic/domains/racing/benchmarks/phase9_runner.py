"""Benchmark runner for Phase 9 tactical planner v1."""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE9_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase9Runner",
            run_id_prefix="phase9",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE9_F_SCENARIO_NAMES,
            scenic_extra_args=(
                "-p",
                "tactical_planner_enabled",
                "True",
                "-p",
                "assessment_enabled",
                "True",
                "-p",
                "prediction_enabled",
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
                "phase9_planner_line_count",
                "phase9_free_run_count",
                "phase9_follow_count",
                "phase9_setup_pass_left_count",
                "phase9_setup_pass_right_count",
                "phase9_state_change_count",
                "phase9_gap_ok_rate",
                "assessment_assessment_line_count",
                "assessment_fellow_relation_ahead_count",
                "assessment_fellow_relation_behind_count",
                "assessment_left_open_rate",
                "assessment_right_open_rate",
                "assessment_emergency_risk_mean",
                "phase6_state_line_count",
                "phase6_planner_line_count",
                "phase6_guard_line_count",
                "phase6_executor_line_count",
                "collision",
                "off_track",
                "near_miss_count",
                "waypoint_hits",
                "min_opponent_distance_m",
            )
            + FELLOW_HARNESS_SUMMARY_KEYS,
            phase1_switches=False,
            phase2_lines=True,
            phase3_tactical=True,
            fellow_harness=True,
            digest_keys=tuple(standard_benchmark_digest_keys_with_fellow()),
            extra_summary_keys=(
                "phase9_planner_line_count",
                "phase9_follow_count",
                "phase9_setup_pass_left_count",
                "phase9_setup_pass_right_count",
                "phase9_state_change_count",
                "phase9_gap_ok_rate",
                "assessment_left_open_rate",
                "assessment_right_open_rate",
                "assessment_emergency_risk_mean",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
