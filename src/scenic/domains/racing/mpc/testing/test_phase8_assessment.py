"""Unit tests for Phase 8 race-situation assessment helpers."""

from scenic.domains.racing.assessment import (
    RaceSituationState,
    assess_race_situation,
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
    opponent_speed_mps: float = 20.0,
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
        opponent_speed_mps=opponent_speed_mps,
    )


def test_dynamic_safe_gap_grows_with_speed():
    g0 = compute_dynamic_safe_gap_m(0.0)
    g1 = compute_dynamic_safe_gap_m(10.0)
    g2 = compute_dynamic_safe_gap_m(20.0)
    assert g0 < g1 < g2


def test_dynamic_safe_gap_uses_shorter_headway_for_parallel_ttl():
    """Laterally separated opponent (parallel TTL) should use shorter headway."""
    same_line = compute_dynamic_safe_gap_m(18.0, lateral_offset_m=0.5)
    parallel = compute_dynamic_safe_gap_m(18.0, lateral_offset_m=2.5)
    # At 18 m/s: same_line = 6+18*0.80 = 20.4, parallel = 6+18*0.35 = 12.3
    assert same_line > 19.0
    assert parallel < 13.0
    assert parallel < same_line


def test_closing_flag_suppressed_for_laterally_separated_opponent():
    """Closing at moderate speed with large lateral offset should NOT set closing_flag."""
    a, _st = assess_race_situation(
        sit=_sit(ahead=True, delta_s_m=25.0, lateral_m=2.5, long_m=25.0, closing_mps=1.5),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(25.0, 2.5),
        state=RaceSituationState(),
    )
    # 1.5 m/s closing is below the 2.0 m/s parallel threshold.
    assert a.closing_flag is False


def test_closing_flag_still_fires_for_same_line_slow_closing():
    """Same line (small lateral), slow closing should still set closing_flag."""
    a, _st = assess_race_situation(
        sit=_sit(ahead=True, delta_s_m=25.0, lateral_m=0.3, long_m=25.0, closing_mps=0.5),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(25.0, 0.3),
        state=RaceSituationState(),
    )
    assert a.closing_flag is True


def test_no_opponent_defaults_open_and_gap_ok():
    a, _st = assess_race_situation(
        sit=None,
        ego_speed_mps=20.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=None,
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "none"
    assert a.gap_ok is True
    assert a.optimal_open is True and a.left_open is True and a.right_open is True


def test_ahead_left_occupancy_blocks_left_corridor():
    a, _st = assess_race_situation(
        sit=_sit(ahead=True, delta_s_m=30.0, lateral_m=2.0, long_m=30.0, closing_mps=2.0),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(30.0, 2.0),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.left_open is False
    assert a.right_open is True


def test_gap_ok_false_when_actual_gap_below_safe_gap():
    a, _st = assess_race_situation(
        sit=_sit(ahead=True, delta_s_m=8.0, lateral_m=0.0, long_m=8.0, closing_mps=5.0),
        ego_speed_mps=20.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(8.0, 0.0),
        state=RaceSituationState(),
    )
    assert a.actual_gap_m is not None
    assert a.safe_gap_m > a.actual_gap_m
    assert a.gap_ok is False


def test_stateful_relation_hysteresis_holds_near_zero_delta_s():
    st = RaceSituationState(previous_relation="ahead")
    a, st2 = assess_race_situation(
        sit=_sit(ahead=False, delta_s_m=0.5, lateral_m=0.0, long_m=0.5, closing_mps=1.0),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(0.5, 0.0),
        state=st,
    )
    assert a.fellow_relation == "ahead"
    assert st2.previous_relation == "ahead"


def test_flyby_lateral_offset_damps_longitudinal_gap_pressure_at_speed():
    """Side-by-side with adequate along-track slot: do not max rear-end pressure just because ego is fast.

    Requires ``actual_gap >= safe_gap`` so fly-by damping applies; inside safe envelope it is disabled.
    """
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=42.0,
            lateral_m=2.4,
            long_m=42.0,
            closing_mps=12.0,
            overlap="partial_overlap",
            risk=0.08,
            opponent_speed_mps=22.0,
        ),
        ego_speed_mps=28.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(42.0, 2.4),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.gap_ok is True
    assert a.emergency_risk_01 < 0.72


def test_flyby_damp_disabled_when_gap_below_safe_gap_same_line():
    """Short longitudinal headway on same line: no fly-by discount — rear-end risk stays visible."""
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=12.0,
            lateral_m=0.5,
            long_m=12.0,
            closing_mps=10.0,
            overlap="partial_overlap",
            risk=0.08,
            opponent_speed_mps=18.0,
        ),
        ego_speed_mps=24.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(12.0, 0.5),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.gap_ok is False
    assert a.emergency_risk_01 >= 0.35


def test_flyby_damp_stays_active_for_parallel_ttl_inside_safe_gap():
    """Parallel TTL (large lateral): fly-by damping stays active even inside safe_gap."""
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=12.0,
            lateral_m=2.6,
            long_m=12.0,
            closing_mps=10.0,
            overlap="partial_overlap",
            risk=0.08,
            opponent_speed_mps=18.0,
        ),
        ego_speed_mps=24.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(12.0, 2.6),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.gap_ok is False
    # Fly-by damping is active for lateral > 1.5m, so risk is lower than same-line
    assert a.emergency_risk_01 < 0.35


def test_co_linear_close_range_preserves_nonzero_risk_from_gap_and_closing():
    """Near co-linear and closing still yields meaningful risk without extra speed-only term."""
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=18.0,
            lateral_m=0.35,
            long_m=18.0,
            closing_mps=8.0,
            overlap="clear_ahead",
            risk=0.05,
            opponent_speed_mps=20.0,
        ),
        ego_speed_mps=32.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(18.0, 0.35),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.emergency_risk_01 >= 0.20


def test_roadside_stationary_partial_overlap_does_not_max_out_emergency_risk():
    """Shoulder-parked obstacle: large |lateral| must not trigger overlap=0.9 panic."""
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=22.0,
            lateral_m=-3.8,
            long_m=22.0,
            closing_mps=18.0,
            overlap="partial_overlap",
            risk=0.12,
            opponent_speed_mps=0.0,
        ),
        ego_speed_mps=22.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(22.0, -3.8),
        state=RaceSituationState(),
    )
    assert a.fellow_relation == "ahead"
    assert a.overlap_flag is False
    assert a.emergency_risk_01 < 0.55


def test_centered_slow_opponent_at_safe_range_yields_asymmetric_opening():
    """SD-2a-bias: centered slow fellow at meaningful range opens exactly one side.

    Pre-bias behavior: |lat|<1.5 unconditionally blocked both corridors, locking
    the planner into protected_follow_active forever (XOR gate at
    tactical_planner.py:381 needs exactly one side open). Post-bias: bucket by
    danger and open right by default for the safe centered case.
    """
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=44.0,
            lateral_m=0.0,
            long_m=44.0,
            closing_mps=2.0,
            overlap="clear_ahead",
            risk=0.05,
            opponent_speed_mps=9.0,
        ),
        ego_speed_mps=11.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(44.0, 0.0),
        state=RaceSituationState(),
    )
    # gap=44m (well clear of close_quarters<12), closing=2 m/s (not hot_closing>8),
    # opp_speed/ego_speed=0.82 (>0.5 but closing fails the hot_closing AND-clause).
    # Right-bias: left blocked, right open. Asymmetric opening unlocks the planner.
    assert a.left_open is False
    assert a.right_open is True
    assert (a.left_open ^ a.right_open) is True  # asymmetric_opening for tactical_planner


def test_centered_close_quarters_keeps_strict_both_blocked():
    """SD-2a-bias safety: a centered fellow inside 12 m must NOT be opened.

    Close-quarters geometry is genuine rear-end danger; the bucket falls back
    to the original both-blocked behavior so the planner cannot try a pass.
    """
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=8.0,
            lateral_m=0.0,
            long_m=8.0,
            closing_mps=3.0,
            overlap="clear_ahead",
            risk=0.20,
            opponent_speed_mps=15.0,
        ),
        ego_speed_mps=18.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(8.0, 0.0),
        state=RaceSituationState(),
    )
    assert a.left_open is False
    assert a.right_open is False


def test_centered_hot_closing_keeps_strict_both_blocked():
    """SD-2a-bias safety: centered fellow with high closing + comparable speed must stay blocked.

    Hot closing (closing > 8 m/s AND opponent still ≥ 50% ego speed) means
    we're rear-ending a moving target — a pass attempt here is dangerous, not
    an overtake opportunity.
    """
    a, _st = assess_race_situation(
        sit=_sit(
            ahead=True,
            delta_s_m=20.0,
            lateral_m=0.0,
            long_m=20.0,
            closing_mps=12.0,
            overlap="clear_ahead",
            risk=0.30,
            opponent_speed_mps=15.0,
        ),
        ego_speed_mps=27.0,
        ego_xy=(0.0, 0.0),
        ego_heading_rad=0.0,
        predicted_opp_xy=(20.0, 0.0),
        state=RaceSituationState(),
    )
    assert a.left_open is False
    assert a.right_open is False


def test_stateful_emergency_risk_emphasizes_overlap_and_closing_gap():
    a, _st = assess_race_situation(
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
        state=RaceSituationState(),
    )
    assert a.gap_ok is False
    assert a.emergency_risk_01 >= 0.85
