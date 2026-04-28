"""SD-11c: unit tests for the strategy selector pure function."""

import pytest

from scenic.domains.racing.planner import select_strategy
from scenic.domains.racing.prediction.strategy_simulator import StrategyOutcome


def _outcome(strategy, *, clearance, progress, speed=30.0, completed=True, reason="ok"):
    return StrategyOutcome(
        strategy=strategy,
        reachable_progress_at_horizon_m=float(progress),
        reachable_speed_at_horizon_mps=float(speed),
        min_clearance_m=float(clearance),
        closest_t_s=0.0,
        completed=bool(completed),
        samples=[],
        reason=reason,
    )


def test_pass_left_wins_when_highest_progress():
    outcomes = [
        _outcome("stay_optimal",  clearance=4.0, progress=200.0),
        _outcome("follow_fellow", clearance=4.0, progress=120.0),
        _outcome("pass_left",     clearance=3.0, progress=300.0),
        _outcome("pass_right",    clearance=3.0, progress=280.0),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5)
    assert sel.name == "pass_left"
    assert sel.reason == "primary"
    assert sel.survivors == ["pass_left", "pass_right", "stay_optimal", "follow_fellow"]


def test_stay_optimal_wins_tiebreak_when_progress_close():
    """Within progress_tiebreak_m, stay_optimal beats pass_*."""
    outcomes = [
        _outcome("stay_optimal",  clearance=4.0, progress=300.0),
        _outcome("follow_fellow", clearance=4.0, progress=120.0),
        _outcome("pass_left",     clearance=3.0, progress=300.4),  # within 0.5 m
        _outcome("pass_right",    clearance=3.0, progress=300.4),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5, progress_tiebreak_m=0.5)
    assert sel.name == "stay_optimal"
    assert sel.reason == "primary"


def test_pass_beats_follow_when_only_those_survive():
    outcomes = [
        _outcome("stay_optimal",  clearance=1.0, progress=300.0),  # filtered out
        _outcome("follow_fellow", clearance=4.0, progress=120.0),
        _outcome("pass_left",     clearance=3.0, progress=280.0),
        _outcome("pass_right",    clearance=1.0, progress=270.0),  # filtered out
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5)
    assert sel.name == "pass_left"
    assert sel.reason == "primary"


def test_only_follow_fellow_survives_filter():
    outcomes = [
        _outcome("stay_optimal",  clearance=1.0, progress=300.0),
        _outcome("follow_fellow", clearance=3.0, progress=120.0),
        _outcome("pass_left",     clearance=1.0, progress=280.0),
        _outcome("pass_right",    clearance=1.0, progress=280.0),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5)
    assert sel.name == "follow_fellow"
    assert sel.reason == "primary"


def test_soft_fallback_to_follow_when_all_below_hard_threshold():
    """All strategies below min_clearance_m=2.5, but follow_fellow above
    soft_clearance_m=1.5 → soft fallback returns follow."""
    outcomes = [
        _outcome("stay_optimal",  clearance=1.0, progress=300.0),
        _outcome("follow_fellow", clearance=2.0, progress=120.0),
        _outcome("pass_left",     clearance=1.0, progress=280.0),
        _outcome("pass_right",    clearance=1.0, progress=280.0),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5, soft_clearance_m=1.5)
    assert sel.name == "follow_fellow"
    assert sel.reason == "soft_fallback_follow"
    assert sel.survivors == []


def test_last_resort_stay_optimal_when_everything_blocked():
    """All strategies catastrophically below both thresholds → stay_optimal as
    last resort (SD-4 emergency brake will catch it at runtime)."""
    outcomes = [
        _outcome("stay_optimal",  clearance=0.5, progress=300.0),
        _outcome("follow_fellow", clearance=0.5, progress=120.0),
        _outcome("pass_left",     clearance=0.5, progress=280.0),
        _outcome("pass_right",    clearance=0.5, progress=280.0),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5, soft_clearance_m=1.5)
    assert sel.name == "stay_optimal"
    assert sel.reason == "last_resort_stay"


def test_empty_outcomes_raises():
    with pytest.raises(ValueError):
        select_strategy([])


def test_chosen_outcome_is_the_actual_outcome_object():
    out_pass = _outcome("pass_left", clearance=3.0, progress=300.0)
    outcomes = [
        _outcome("stay_optimal", clearance=4.0, progress=200.0),
        out_pass,
    ]
    sel = select_strategy(outcomes)
    assert sel.chosen_outcome is out_pass


def test_pass_right_can_win_when_pass_left_filtered_out():
    outcomes = [
        _outcome("stay_optimal",  clearance=4.0, progress=200.0),
        _outcome("follow_fellow", clearance=4.0, progress=120.0),
        _outcome("pass_left",     clearance=1.0, progress=300.0),  # filtered
        _outcome("pass_right",    clearance=3.0, progress=290.0),
    ]
    sel = select_strategy(outcomes, min_clearance_m=2.5)
    assert sel.name == "pass_right"


def test_all_outcomes_preserved_in_diagnostics():
    outcomes = [
        _outcome("stay_optimal",  clearance=4.0, progress=200.0),
        _outcome("follow_fellow", clearance=4.0, progress=120.0),
        _outcome("pass_left",     clearance=3.0, progress=300.0),
        _outcome("pass_right",    clearance=3.0, progress=280.0),
    ]
    sel = select_strategy(outcomes)
    assert len(sel.all_outcomes) == 4
    assert {o.strategy for o in sel.all_outcomes} == {
        "stay_optimal", "follow_fellow", "pass_left", "pass_right"
    }
