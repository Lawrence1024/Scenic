"""Benchmark runner for Phase 10 stability guard and emergency policy."""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE10_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase10Runner",
            run_id_prefix="phase10",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE10_F_SCENARIO_NAMES,
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
                "-p",
                "stability_guard_enabled",
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
                "guard_guard_line_count",
                "guard_guard_active_count",
                "guard_steer_limited_count",
                "guard_brake_limited_count",
                "guard_ttl_switch_blocked_count",
                "guard_emergency_stable_count",
                "phase9_state_change_count",
                "phase9_setup_pass_left_count",
                "phase9_setup_pass_right_count",
                "phase6_executor_line_count",
                "phase6_guard_active_count",
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
                "guard_guard_line_count",
                "guard_guard_active_count",
                "guard_steer_limited_count",
                "guard_brake_limited_count",
                "guard_ttl_switch_blocked_count",
                "guard_emergency_stable_count",
                "phase9_state_change_count",
                "phase9_setup_pass_left_count",
                "phase9_setup_pass_right_count",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())

