"""Benchmark runner for Phase 5 (segment-aware tactical shaping) scenarios."""

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="Phase5Runner",
                run_id_prefix="phase5",
                default_scenario_dir="examples/racing/phase5_segments",
                csv_fields=(
                    "scenario",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
                    "phase1_switch_count",
                    "phase1_switch_observed",
                    "phase3_ttl_switch_count",
                    "phase3_tactical_status_count",
                    "phase4_tactical_line_count",
                    "phase4_abort_pass_count",
                    "phase4_emergency_avoid_count",
                    "phase4_commit_pass_count",
                    "phase4_event_commit_pass_left",
                    "phase4_event_commit_pass_right",
                    "phase4_event_shield_release",
                    "phase5_tactical_line_count",
                    "phase5_ttl_switch_count",
                    "phase5_event_segment_override",
                    "phase5_event_segment_release",
                    "phase5_override_count",
                    "phase2_line_count",
                    "collision",
                    "off_track",
                    "waypoint_hits",
                    "phase0_samples",
                )
                + FELLOW_HARNESS_SUMMARY_KEYS,
                phase1_switches=True,
                phase2_lines=True,
                phase3_tactical=True,
                fellow_harness=True,
                digest_keys=tuple(standard_benchmark_digest_keys_with_fellow()),
            )
        )
    )
