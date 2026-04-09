"""Run Phase 0 benchmark scenarios with tactical planner enabled (Phase 3 cross-check).

Same metrics collection as ``phase3_runner`` (Phase 2 + Phase 3 log parsers).
Default scenario bank: ``examples/racing/phase3_on_phase0_bank/`` (Phase 0 layouts
with ``tactical_planner_enabled=True`` on ego).

Usage (repo root):

    python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0

After the run, copy the ``BENCHMARK_AI_DIGEST_BEGIN`` … ``END`` block from the
terminal or attach ``summary.json`` from the printed ``run_dir``.
"""

from scenic.domains.racing.benchmarks.phase_run_common import PhaseRunnerSpec, run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="Phase3OnPhase0Runner",
                run_id_prefix="phase3_on_phase0",
                default_scenario_dir="examples/racing/phase3_on_phase0_bank",
                default_sim_steps=3000,
                csv_fields=(
                    "scenario",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
                    "phase3_ttl_switch_count",
                    "phase3_tactical_status_count",
                    "phase2_line_count",
                    "phase2_overlap_count",
                    "phase2_seg_ctx_count",
                    "phase2_opponent_none_lines",
                    "phase2_assess_errors",
                    "min_opponent_distance_m",
                    "collision",
                    "off_track",
                    "near_miss_count",
                    "waypoint_hits",
                    "phase0_samples",
                    "ttl_switch_count",
                ),
                phase1_switches=False,
                phase2_lines=True,
                phase3_tactical=True,
            )
        )
    )
