"""Benchmark runner for Phase 6 (multi-car) racing scenarios.

Scenarios are not listed here. ``run_phase_main`` runs every ``*.scenic`` in the
default directory (``examples/racing/phase6_multi``), sorted by name. New
files in that folder are included automatically.

When Phase 6 is implemented and logs new KPI tags, update ``PhaseRunnerSpec``
below and ``phase_run_common.collect_metrics_from_log``. See
``examples/racing/README.md`` (Phases 4–6).
"""

from scenic.domains.racing.benchmarks.phase_run_common import PhaseRunnerSpec, run_phase_main

if __name__ == "__main__":
    raise SystemExit(
        run_phase_main(
            PhaseRunnerSpec(
                runner_label="Phase6Runner",
                run_id_prefix="phase6",
                default_scenario_dir="examples/racing/phase6_multi",
                csv_fields=(
                    "scenario",
                    "return_code",
                    "lap_completion_status",
                    "lap_time_s",
                    "phase1_switch_count",
                    "phase1_switch_observed",
                    "phase3_ttl_switch_count",
                    "phase3_tactical_status_count",
                    "phase2_line_count",
                    "collision",
                    "off_track",
                    "waypoint_hits",
                    "phase0_samples",
                ),
                phase1_switches=True,
                phase2_lines=True,
                phase3_tactical=True,
            )
        )
    )
