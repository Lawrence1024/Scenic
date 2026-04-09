"""Benchmark runner for Phase 3 (tactical) racing scenarios.

For the Phase 0 layout bank with tactical enabled on ego, use
``phase3_on_phase0_runner`` (see ``examples/racing/phase3_on_phase0_bank/README.md``).
"""

from scenic.domains.racing.benchmarks.phase_run_common import PhaseRunnerSpec, run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="Phase3Runner",
                run_id_prefix="phase3",
                default_scenario_dir="examples/racing/phase3_tactical",
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
                    "collision",
                    "off_track",
                    "waypoint_hits",
                    "phase0_samples",
                ),
                phase1_switches=False,
                phase2_lines=True,
                phase3_tactical=True,
            )
        )
    )
