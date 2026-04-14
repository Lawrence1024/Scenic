"""Benchmark runner for Phase 6 architecture-skeleton scenarios.

Uses the shared `examples/racing/f_shared/` scenario bank and defaults to the
Phase 6 subset (`F0`, `F1`, `F2`) via `default_scenario_names`.
"""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE6_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase6Runner",
            run_id_prefix="phase6",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE6_F_SCENARIO_NAMES,
            csv_fields=(
                "scenario",
                "scenario_instance",
                "repeat_index",
                "repeat_count",
                "return_code",
                "lap_completion_status",
                "lap_time_s",
                "phase6_state_line_count",
                "phase6_planner_line_count",
                "phase6_guard_line_count",
                "phase6_executor_line_count",
                "phase6_guard_active_count",
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
                "phase6_planner_line_count",
                "phase6_guard_active_count",
                "min_opponent_distance_m",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
