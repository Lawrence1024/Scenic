"""Tests for benchmark log parsing with canonical event signals.

**Full dSPACE Phase 4 runs** should show ego shield / tactical evidence via
``[Phase4Event]`` plus ego situation
assessment ``[Phase2]`` and baseline ``[Phase0]`` when those log lines are enabled.
With ``param fellowHarnessLog = True``, logs also include ``[FellowHarness]`` samples;
**fellow pose/speed** supports whether the traffic car is present in readback — not
whether a pass was "correct" by itself. Validate ego behavior against Phase4 counts
and ego-side metrics in ``summary.json``.

**Controller vs benchmark:** The stack only sees ego/fellow state from readback
(positions, etc.). ``[EvalGT]`` / dSPACE ``Dist_Object_1`` is **not** used for
control. ``collect_metrics_from_log`` ingests ``[EvalEvent] type=eval_contact`` for
hull/sensor-based contact counts, not raw ``[EvalGT]`` numbers into ``min_opponent_distance_m``.
"""

from pathlib import Path

import pytest

from scenic.domains.racing.benchmarks.phase_run_common import collect_metrics_from_log, finalize_row


def test_collect_metrics_phase4_events_preferred(tmp_path: Path) -> None:
    log = tmp_path / "run.log"
    log.write_text(
        "[Phase4Event] t=1.00s event=commit_pass_right mode3=SETUP_RIGHT eff=COMMIT_PASS_RIGHT reason=commit_dwell_right risk_01=0.10 seg=straight overlap=clear_ahead\n"
        "[Phase4Event] t=2.00s event=abort_pass mode3=SETUP_RIGHT eff=ABORT_PASS reason=setup_risk risk_01=0.70 seg=straight overlap=clear_ahead\n"
        "[Phase4Event] t=3.00s event=shield_release from=ABORT_PASS to=FOLLOW mode3=FOLLOW risk_01=0.20 seg=straight overlap=clear_ahead\n"
        ,
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log)
    assert m["phase4_commit_pass_count"] == 1
    assert m["phase4_event_commit_pass_left"] == 0
    assert m["phase4_event_commit_pass_right"] == 1
    assert m["phase4_abort_pass_count"] == 1
    assert m["phase4_emergency_avoid_count"] == 0
    assert m["phase4_event_shield_release"] == 1


def test_collect_metrics_ignores_eval_gt_and_sensor_strings(tmp_path: Path) -> None:
    """``[EvalGT]`` must not affect min opponent distance or phase4 counts."""
    log = tmp_path / "with_evalgt.log"
    log.write_text(
        "[Phase0] t=1.00s ttl=optimal planner_mode=follow_mpc ego_s=1.00 ego_speed=10.00 "
        "nearest_opp_ds=1.00 nearest_opp_rel_speed=0.00 nearest_opp_dist=12.345\n"
        "[EvalGT] t=1.00s dspace_obj1_raw_m=99.999 dspace_valid=1 bbox_gap_m=7.000 "
        "nearest_opp_center_dist_m=12.345 center_minus_bbox_m=5.345 center_minus_gt_m=-87.654 "
        "bbox_minus_gt_m=-92.000\n"
        "[Phase4Event] t=1.00s event=commit_pass_right\n",
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log)
    assert m["min_opponent_distance_m"] == pytest.approx(12.345)
    assert m["phase4_commit_pass_count"] == 1
    assert not any("eval_gt" in str(k).lower() for k in m)
    assert m.get("eval_contact_overlap_count") == 0


def test_collect_metrics_eval_contact_events(tmp_path: Path) -> None:
    log = tmp_path / "eval_contact.log"
    log.write_text(
        "[EvalEvent] t=1.00s type=eval_contact severity=near bbox_gap_m=0.80 dspace_obj1_m=1.20 dspace_valid=1\n"
        "[EvalEvent] t=2.00s type=eval_contact severity=overlap bbox_gap_m=0.000 dspace_obj1_m=na dspace_valid=0\n",
        encoding="utf-8",
    )
    m = finalize_row(collect_metrics_from_log(log))
    assert m["eval_contact_near_count"] == 1
    assert m["eval_contact_overlap_count"] == 1
    assert m["collision_eval_hull_overlap"] is True
    assert m["collision_count"] == 1
    assert m["collision"] is True
    assert m["near_miss_count"] == 1


def test_phase0_collision_and_near_miss_events_finalize_row(tmp_path: Path) -> None:
    """Eval contact events drive canonical collision/near-miss safety flags."""
    log = tmp_path / "events.log"
    log.write_text(
        "[EvalEvent] t=1.00s type=eval_contact severity=near bbox_gap_m=0.80 dspace_obj1_m=1.20 dspace_valid=1\n"
        "[EvalEvent] t=2.00s type=eval_contact severity=overlap bbox_gap_m=0.000 dspace_obj1_m=na dspace_valid=0\n",
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log)
    m = finalize_row(m)
    assert m["near_miss_count"] == 1
    assert m["collision_count"] == 1
    assert m["collision"] is True


def test_phase4_pass_shield_activity_coherent_synthetic_log(tmp_path: Path) -> None:
    """Typical successful Phase4 run: Phase3 setup, Phase4 commit + release, no collision."""
    log = tmp_path / "coherent.log"
    log.write_text(
        "[Phase0] t=1.00s ttl=optimal planner_mode=follow_mpc ego_s=10.00 ego_speed=15.00 "
        "nearest_opp_ds=20.00 nearest_opp_rel_speed=-1.00 nearest_opp_dist=25.00\n"
        "[Phase3Tactical] t=1.00s ttl_switch optimal->right mode=SETUP_RIGHT\n"
        "[Phase4Event] t=1.10s event=commit_pass_right\n"
        "[Phase4Event] t=3.00s event=shield_release from=COMMIT_PASS_RIGHT to=FOLLOW\n"
        "[Phase0] t=4.00s ttl=right planner_mode=follow_mpc ego_s=50.00 ego_speed=18.00 "
        "nearest_opp_ds=40.00 nearest_opp_rel_speed=0.00 nearest_opp_dist=45.00\n",
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log, phase3_tactical=True)
    assert m["phase3_ttl_switch_count"] == 1
    assert m["phase4_commit_pass_count"] == 1
    assert m["phase4_event_shield_release"] == 1
    assert m["collision_count"] == 0
    assert finalize_row(m)["collision"] is False
