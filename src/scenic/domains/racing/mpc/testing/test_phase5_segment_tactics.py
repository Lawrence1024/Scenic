"""Tests for Phase 5 segment-aware tactical shaping."""

from scenic.domains.racing.phase5_segment_tactics import (
    Phase5SegmentTacticsConfig,
    Phase5SegmentTacticsState,
    phase5_segment_tactics_step,
)
from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.tactical_planner import FOLLOW, SETUP_RIGHT, TacticalPlannerConfig


def _sit(**kwargs):
    defaults = dict(
        ahead=True,
        delta_s_m=15.0,
        delta_s_source="heading_proxy",
        lateral_relation="on_line",
        closing_speed_mps=2.0,
        overlap_state="clear_ahead",
        collision_risk_01=0.2,
        segment_context="straight",
        distance_m=18.0,
        longitudinal_m=15.0,
        lateral_m=1.0,
    )
    defaults.update(kwargs)
    return OpponentSituation(**defaults)


def test_corner_entry_blocks_setup_without_overlap():
    st = Phase5SegmentTacticsState()
    sit = _sit(segment_context="corner_entry", overlap_state="clear_ahead")
    m, ttl, cap, reason = phase5_segment_tactics_step(
        st,
        sit,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        pit_mode=False,
        opponent_speed_mps=20.0,
        tactical_config=TacticalPlannerConfig(),
        phase5_config=Phase5SegmentTacticsConfig(),
    )
    assert m == FOLLOW
    assert ttl == "optimal"
    assert cap is not None
    assert reason == "entry_conservative"


def test_corner_entry_allows_setup_with_established_overlap():
    st = Phase5SegmentTacticsState()
    sit = _sit(segment_context="corner_entry", overlap_state="side_by_side")
    m, ttl, cap, reason = phase5_segment_tactics_step(
        st,
        sit,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        pit_mode=False,
        opponent_speed_mps=20.0,
        tactical_config=TacticalPlannerConfig(),
        phase5_config=Phase5SegmentTacticsConfig(),
    )
    assert m == SETUP_RIGHT
    assert ttl == "right"
    assert reason is None


def test_corner_body_blocks_setup():
    st = Phase5SegmentTacticsState()
    sit = _sit(segment_context="corner_body", overlap_state="side_by_side")
    m, ttl, cap, reason = phase5_segment_tactics_step(
        st,
        sit,
        SETUP_RIGHT,
        "right",
        None,
        has_opponent=True,
        pit_mode=False,
        opponent_speed_mps=20.0,
        tactical_config=TacticalPlannerConfig(),
        phase5_config=Phase5SegmentTacticsConfig(),
    )
    assert m == FOLLOW
    assert ttl == "optimal"
    assert cap is not None
    assert reason == "body_no_new_setup"

