"""Benchmark runner for Phase 4 (pass commit, abort, shield) racing scenarios.

Default directory: ``examples/racing/phase4_pass_shield`` (ego typically uses
``tactical_planner_enabled=True`` and ``pass_commit_shield_enabled=True``).

``collect_metrics_from_log`` prefers ``[Phase4Event]`` lines (one per mode entry
or shield release); legacy logs fall back to ``[Phase4Tactical]`` substrings.

**Contract:** Pass/fail summaries use **on-board Scenic telemetry** only (e.g.
``[Phase0]`` center-to-center gap, ``[Phase0Event]`` collision/near-miss,
``[Phase2]``/``[Phase3]``/``[Phase4]`` lines). Optional ``[EvalGT]`` dSPACE sensor
lines are for **offline comparison** and are **not** ingested into these metrics.
"""

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
                runner_label="Phase4Runner",
                run_id_prefix="phase4",
                default_scenario_dir="examples/racing/phase4_pass_shield",
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
                    "eval_contact_overlap_count",
                    "eval_contact_near_count",
                    "collision_eval_hull_overlap",
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
