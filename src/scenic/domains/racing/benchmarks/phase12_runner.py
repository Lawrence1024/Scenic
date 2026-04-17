"""Benchmark runner for Phase 12 segment-aware tactical intelligence.

Runs the four corner-entry scenarios that validate segment-conditioned gating:
  F8:  corner-entry, fellow ahead on optimal (open corridor baseline)
  F10: corner-entry, fellow on left TTL at 45 mph  (paired with F6)
  F11: corner-entry, fellow on right TTL at 45 mph (paired with F7)
  F12: corner-entry, fellow ahead with sudden stop  (paired with F4; corner commits must be blocked)

Phase 12 gating rules:
  - corner_body:  no new protected-follow release; commit entry blocked
  - corner_entry: tighter collision-risk gate on commit entry (max 0.30)
  - straight:     Phase 11 behavior (no additional restriction)
"""

from scenic.domains.racing.benchmarks.f_scenario_bank import PHASE12_F_SCENARIO_NAMES
from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    PhaseRunnerSpec,
    run_phase_main,
    standard_benchmark_digest_keys_with_fellow,
)


def main() -> int:
    return run_phase_main(
        PhaseRunnerSpec(
            runner_label="Phase12Runner",
            run_id_prefix="phase12",
            default_scenario_dir="examples/racing/f_shared",
            default_scenario_names=PHASE12_F_SCENARIO_NAMES,
            scenic_extra_args=(
                "-p",
                "tactical_planner_enabled",
                "True",
                "-p",
                "prediction_enabled",
                "True",
                "-p",
                "assessment_enabled",
                "True",
                "-p",
                "stability_guard_enabled",
                "True",
                "-p",
                "commit_abort_enabled",
                "True",
                "-p",
                "segment_aware_enabled",
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
                "phase11_planner_line_count",
                "phase11_commit_trigger_count",
                "phase11_abort_trigger_count",
                "phase11_pass_success_count",
                "phase11_abort_success_count",
                "phase11_commit_pass_left_count",
                "phase11_commit_pass_right_count",
                "phase11_abort_pass_count",
                "phase12_seg_straight_count",
                "phase12_seg_corner_entry_count",
                "phase12_seg_corner_body_count",
                "phase12_seg_corner_exit_count",
                "phase12_seg_modifier_blocked_count",
                "phase12_seg_modifier_conservative_count",
                "phase10_guard_active_count",
                "phase10_emergency_stable_count",
                "phase9_state_change_count",
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
                "phase11_planner_line_count",
                "phase11_commit_trigger_count",
                "phase11_abort_trigger_count",
                "phase11_pass_success_count",
                "phase11_abort_success_count",
                "phase11_commit_pass_left_count",
                "phase11_commit_pass_right_count",
                "phase11_abort_pass_count",
                "phase12_seg_straight_count",
                "phase12_seg_corner_entry_count",
                "phase12_seg_corner_body_count",
                "phase12_seg_modifier_blocked_count",
                "phase12_seg_modifier_conservative_count",
                "phase10_guard_active_count",
                "phase10_emergency_stable_count",
            ),
        )
    )


if __name__ == "__main__":
    raise SystemExit(main())
