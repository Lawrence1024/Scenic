"""Full-stack benchmark runner — runs all F-scenarios with the complete intelligence pipeline.

Enables all layers: prediction, assessment, tactical planner, stability guard,
commit/abort lifecycle, and segment-aware gating.  This is the "smartest" stack
configuration and the primary integration test.

Usage:
    python src/scenic/domains/racing/benchmarks/full_stack_runner.py
"""

from scenic.domains.racing.benchmarks.f_scenario_bank import F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


# All F-scenarios except F0 (ego alone — no opponent to test against).
FULL_STACK_SCENARIO_NAMES = tuple(
    name for name in F_SCENARIO_NAMES if name != "F0_ego_alone.scenic"
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="FullStackRunner",
            run_id_prefix="full_stack",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=FULL_STACK_SCENARIO_NAMES,
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
                "-p",
                "segment_aware_enabled",
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
                "hazard_state_change_count",
                "collision",
                "off_track",
                "near_miss_count",
                "waypoint_hits",
                "min_opponent_distance_m",
            )
            + FELLOW_HARNESS_SUMMARY_KEYS,
            scripted_switches=False,
            opponent_lines=True,
            tactical_tactical=True,
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
