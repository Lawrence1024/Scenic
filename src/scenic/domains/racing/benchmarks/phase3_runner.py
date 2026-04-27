"""Benchmark runner for Phase 3 (tactical) racing scenarios.

Default bank: ``examples/racing/tactical_tactical/`` — same seven layouts as
``baseline_benchmark`` (``00``–``06``), with ``tactical_planner_enabled=True`` on ego.
Default ``--time`` matches other phase runners (**2000** steps ≈ **20 s**).

``PHASE3_RUNNER_SPEC`` is shared with ``tactical_on_baseline_runner`` (backward-compatible
module alias; same metrics, optional ``run_id_prefix`` for old scripts).
"""

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)

PHASE3_RUNNER_SPEC = PhaseRunnerSpec(
    runner_label="Phase3Runner",
    run_id_prefix="phase3",
    default_scenario_dir="examples/racing/tactical_tactical",
    csv_fields=(
        "scenario",
        "return_code",
        "lap_completion_status",
        "lap_time_s",
        "tactical_ttl_switch_count",
        "tactical_tactical_status_count",
        "opponent_line_count",
        "opponent_overlap_count",
        "opponent_seg_ctx_count",
        "opponent_opponent_none_lines",
        "opponent_assess_errors",
        "min_opponent_distance_m",
        "collision",
        "off_track",
        "near_miss_count",
        "waypoint_hits",
        "baseline_samples",
        "ttl_switch_count",
    )
    + FELLOW_HARNESS_SUMMARY_KEYS,
    scripted_switches=False,
    opponent_lines=True,
    tactical_tactical=True,
    fellow_harness=True,
    digest_keys=tuple(standard_benchmark_digest_keys_with_fellow()),
)


def main() -> int:
    return run_phase_main(PHASE3_RUNNER_SPEC)


if __name__ == "__main__":
    raise SystemExit(main())
