"""Benchmark runner for Phase 2 (situation assessment) racing scenarios."""

from scenic.domains.racing.benchmarks.phase_run_common import PhaseRunnerSpec, run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="Phase2Runner",
                run_id_prefix="phase2",
                default_scenario_dir="examples/racing/phase2_assessment",
                csv_fields=(
                    "scenario",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
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
                phase3_tactical=False,
            )
        )
    )
