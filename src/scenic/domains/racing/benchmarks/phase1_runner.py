#!/usr/bin/env python3
"""Benchmark runner for Phase 1 planner-MPC scenarios."""

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase1Runner",
            run_id_prefix="phase1",
            default_scenario_dir="examples/racing/scripted_planner",
            csv_fields=(
                "scenario",
                "scenario_instance",
                "repeat_index",
                "repeat_count",
                "return_code",
                "lap_completion_status",
                "lap_time_s",
                "scripted_switch_observed",
                "scripted_switch_count",
                "ttl_switch_count",
                "min_opponent_distance_m",
                "collision",
                "collision_count",
                "collision_eval_hull_overlap",
                "off_track",
                "near_miss_count",
                "eval_contact_overlap_count",
                "eval_contact_near_count",
                "waypoint_hits",
                "baseline_samples",
            )
            + FELLOW_HARNESS_SUMMARY_KEYS,
            scripted_switches=True,
            opponent_lines=False,
            tactical_tactical=False,
            fellow_harness=True,
            digest_keys=tuple(standard_benchmark_digest_keys_with_fellow()),
            extra_summary_keys=(
                "scripted_switch_observed",
                "ttl_switch_count",
                "collision_count",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
