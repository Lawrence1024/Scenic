"""Benchmark runner for Phase 11 pass commit/abort lifecycle."""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE11_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase11Runner",
            run_id_prefix="phase11",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE11_F_SCENARIO_NAMES,
            scenic_extra_args=(
                "-p",
                "tactical_planner_enabled",
                "True",
                "-p",
                "prediction_enabled",
                "True",
                "-p",
                "assessment_enabled",
                "True",
                "-p",
                "stability_guard_enabled",
                "True",
                "-p",
                "commit_abort_enabled",
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
                "commit_planner_line_count",
                "commit_commit_trigger_count",
                "commit_abort_trigger_count",
                "commit_pass_success_count",
                "commit_abort_success_count",
                "commit_commit_pass_left_count",
                "commit_commit_pass_right_count",
                "commit_abort_pass_count",
                "guard_guard_active_count",
                "guard_emergency_stable_count",
                "phase9_state_change_count",
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
                "commit_planner_line_count",
                "commit_commit_trigger_count",
                "commit_abort_trigger_count",
                "commit_pass_success_count",
                "commit_abort_success_count",
                "commit_commit_pass_left_count",
                "commit_commit_pass_right_count",
                "commit_abort_pass_count",
                "guard_guard_active_count",
                "guard_emergency_stable_count",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())

