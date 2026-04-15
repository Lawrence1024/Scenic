"""Unit tests for Phase 8 race-situation assessment helpers."""

from scenic.domains.racing.assessment import (
    Phase8AssessmentState,
    assess_phase8_situation_stateful,
    compute_dynamic_safe_gap_m,
)
from scenic.domains.racing.situation_assessment import OpponentSituation


def _sit(
    *,
    ahead: bool,
    delta_s_m: float,
    lateral_m: float,
    long_m: float,
    closing_mps: float,
    overlap: str = "clear_ahead",
    risk: float = 0.1,
) -> OpponentSituation:
    return OpponentSituation(
        ahead=ahead,
        delta_s_m=delta_s_m,
        delta_s_source="polyline",
        lateral_relation="left" if lateral_m > 0.35 else ("right" if lateral_m < -0.35 else "on_line"),
        closing_speed_mps=closing_mps,
        overlap_state=overlap,
        collision_risk_01=risk,
        segment_context="straight",
        distance_m=abs(delta_s_m),
        longitudinal_m=long_m,
        lateral_m=lateral_m,
    )


def test_dynamic_safe_gap_grows_with_speed():
    g0 = compute_dynamic_safe_gap_m(0.0)
    g1 = compute_dynamic_safe_gap_m(10.0)
    g2 = compute_dynamic_safe_gap_m(20.0)
    assert g0 < g1 < g2


def test_no_opponent_defaults_open_and_gap_ok():
    a, _st = assess_phase8_situation_stateful(
        sit=None,
        ego_speed_mps=20.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=None,
        state=Phase8AssessmentState(),
    )
    assert a.fellow_relation == "none"
    assert a.gap_ok is True
    assert a.optimal_open is True and a.left_open is True and a.right_open is True


def test_ahead_left_occupancy_blocks_left_corridor():
    a, _st = assess_phase8_situation_stateful(
        sit=_sit(ahead=True, delta_s_m=30.0, lateral_m=2.0, long_m=30.0, closing_mps=2.0),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(30.0, 2.0),
        state=Phase8AssessmentState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.left_open is False
    assert a.right_open is True


def test_gap_ok_false_when_actual_gap_below_safe_gap():
    a, _st = assess_phase8_situation_stateful(
        sit=_sit(ahead=True, delta_s_m=8.0, lateral_m=0.0, long_m=8.0, closing_mps=5.0),
        ego_speed_mps=20.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(8.0, 0.0),
        state=Phase8AssessmentState(),
    )
    assert a.actual_gap_m is not None
    assert a.safe_gap_m > a.actual_gap_m
    assert a.gap_ok is False


def test_stateful_relation_hysteresis_holds_near_zero_delta_s():
    st = Phase8AssessmentState(previous_relation="ahead")
    a, st2 = assess_phase8_situation_stateful(
        sit=_sit(ahead=False, delta_s_m=0.5, lateral_m=0.0, long_m=0.5, closing_mps=1.0),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(0.5, 0.0),
        state=st,
    )
    assert a.fellow_relation == "ahead"
    assert st2.previous_relation == "ahead"


def test_stateful_emergency_risk_emphasizes_overlap_and_closing_gap():
    a, _st = assess_phase8_situation_stateful(
        sit=_sit(
            ahead=True,
            delta_s_m=4.0,
            lateral_m=0.1,
            long_m=4.0,
            closing_mps=8.0,
            overlap="side_by_side",
            risk=0.15,
        ),
        ego_speed_mps=25.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(4.0, 0.1),
        state=Phase8AssessmentState(),
    )
    assert a.gap_ok is False
    assert a.emergency_risk_01 >= 0.85
