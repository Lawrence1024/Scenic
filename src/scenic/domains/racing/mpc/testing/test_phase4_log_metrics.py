"""Tests for Phase 4 benchmark log parsing ([Phase4Event] vs legacy [Phase4Tactical]).

**Full dSPACE Phase 4 runs** should show ego shield / tactical evidence via
``[Phase4Event]`` (preferred) or legacy ``[Phase4Tactical]``, plus ego situation
assessment ``[Phase2]`` and baseline ``[Phase0]`` when those log lines are enabled.
With ``param fellowHarnessLog = True``, logs also include ``[FellowHarness]`` samples;
**fellow pose/speed** supports whether the traffic car is present in readback — not
whether a pass was "correct" by itself. Validate ego behavior against Phase4 counts
and ego-side metrics in ``summary.json``.
"""

from pathlib import Path

from scenic.domains.racing.benchmarks.phase_run_common import collect_metrics_from_log


def test_collect_metrics_phase4_events_preferred(tmp_path: Path) -> None:
    log = tmp_path / "run.log"
    log.write_text(
        "[Phase4Event] t=1.00s event=commit_pass_right mode3=SETUP_RIGHT eff=COMMIT_PASS_RIGHT reason=commit_dwell_right risk_01=0.10 seg=straight overlap=clear_ahead\n"
        "[Phase4Event] t=2.00s event=abort_pass mode3=SETUP_RIGHT eff=ABORT_PASS reason=setup_risk risk_01=0.70 seg=straight overlap=clear_ahead\n"
        "[Phase4Event] t=3.00s event=shield_release from=ABORT_PASS to=FOLLOW mode3=FOLLOW risk_01=0.20 seg=straight overlap=clear_ahead\n"
        "[Phase4Tactical] t=1.00s eff_mode=COMMIT_PASS_RIGHT ttl=right cap=30.0 reason=none\n",
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log)
    assert m["phase4_commit_pass_count"] == 1
    assert m["phase4_event_commit_pass_left"] == 0
    assert m["phase4_event_commit_pass_right"] == 1
    assert m["phase4_abort_pass_count"] == 1
    assert m["phase4_emergency_avoid_count"] == 0
    assert m["phase4_event_shield_release"] == 1
    assert m["phase4_tactical_line_count"] == 1


def test_collect_metrics_phase4_legacy_tac_fallback(tmp_path: Path) -> None:
    log = tmp_path / "legacy.log"
    log.write_text(
        "[Phase4Tactical] t=1.00s eff_mode=COMMIT_PASS_LEFT ttl=left cap=30.0 reason=none\n"
        "[Phase4Tactical] t=1.00s eff_mode=ABORT_PASS ttl=optimal cap=25.0 reason=setup_risk\n",
        encoding="utf-8",
    )
    m = collect_metrics_from_log(log)
    assert m["phase4_commit_pass_count"] == 1
    assert m["phase4_abort_pass_count"] == 1
    assert m["phase4_event_commit_pass_left"] == 0
    assert m["phase4_event_shield_release"] == 0
    assert m["phase4_tactical_line_count"] == 2
