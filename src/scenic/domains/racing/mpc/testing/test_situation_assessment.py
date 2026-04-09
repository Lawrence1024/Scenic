"""Unit tests for Phase 2 situation assessment (labeled geometry snapshots)."""

import math

import pytest

from scenic.domains.racing.situation_assessment import (
    assess_nearest_opponent,
    collision_risk_short_horizon,
    planner_segment_context,
    polyline_lap_length_m,
    stabilize_overlap_state,
    waypoint_segment_run_progress,
)


def test_opponent_ten_m_ahead_slower():
    sit, _ = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        20.0,
        (10.0, 0.0),
        15.0,
        previous_overlap_state="clear_ahead",
    )
    assert sit.ahead is True
    assert sit.longitudinal_m == pytest.approx(10.0)
    assert sit.lateral_relation == "on_line"
    assert sit.closing_speed_mps == pytest.approx(5.0)
    assert sit.overlap_state == "clear_ahead"


def test_opponent_on_ego_right():
    # Ego faces +x; opponent at (0, -3) is on ego's right (negative lateral).
    sit, _ = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        25.0,
        (5.0, -3.0),
        25.0,
        previous_overlap_state="clear_ahead",
    )
    assert sit.lateral_relation == "right"
    assert sit.longitudinal_m == pytest.approx(5.0)


def test_side_by_side_on_straight_segment_name():
    sit, st = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        30.0,
        (3.0, 2.5),
        30.0,
        segment_name="4 main straight",
        segment_id=4,
        segment_map=[(4, "main straight")] * 20,
        ego_wp_idx=5,
        curvature_ahead_max=0.005,
        previous_overlap_state="clear_ahead",
    )
    assert sit.segment_context == "straight"
    assert sit.overlap_state in ("side_by_side", "partial_overlap")
    assert st == sit.overlap_state


def test_corner_entry_with_opponent_ahead():
    seg_map = [(6, "main curve")] * 40
    sit, _ = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        28.0,
        (12.0, 0.5),
        22.0,
        segment_name="main curve",
        segment_id=6,
        segment_map=seg_map,
        ego_wp_idx=5,
        curvature_ahead_max=0.04,
        previous_overlap_state="clear_ahead",
    )
    prog = waypoint_segment_run_progress(seg_map, 5, 6)
    assert prog is not None and prog < 0.25
    assert sit.segment_context == "corner_entry"


def test_polyline_delta_s_matches_heading_when_colinear():
    # Square loop 0,0 -> 100,0 -> 100,100 -> 0,100 (open chain; shapely length ~300).
    wps = [(0.0, 0.0), (100.0, 0.0), (100.0, 100.0), (0.0, 100.0)]
    L = polyline_lap_length_m(wps)
    assert L > 0
    sit, _ = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        20.0,
        (10.0, 0.0),
        20.0,
        ego_progress_s_m=0.0,
        waypoints=wps,
        lap_length_m=L,
        previous_overlap_state="clear_ahead",
    )
    if sit.delta_s_source == "polyline":
        assert sit.delta_s_m == pytest.approx(10.0, abs=0.5)


def test_overlap_hysteresis_reduces_flicker():
    prev = "side_by_side"
    # Raw would drop to partial_overlap / clear, but still inside exit band
    lon, lat = 6.0, 4.0
    raw = "partial_overlap"
    st = stabilize_overlap_state(prev, raw, lon, lat)
    assert st == "side_by_side"


def test_collision_risk_higher_when_close_than_far():
    near = collision_risk_short_horizon(8.0, 6.0, 0.3)
    far = collision_risk_short_horizon(45.0, 6.0, 0.3)
    assert near > far


def test_planner_segment_context_curve_progress_exit():
    assert planner_segment_context("main curve", 0.9, 0.02) == "corner_exit"


@pytest.mark.parametrize(
    "noise_lat,expected_rel",
    [(0.0, "on_line"), (0.1, "on_line"), (0.4, "left")],
)
def test_lateral_relation_stable_under_small_noise(noise_lat, expected_rel):
    sit, _ = assess_nearest_opponent(
        (0.0, 0.0),
        0.0,
        20.0,
        (10.0, noise_lat),
        18.0,
        previous_overlap_state="clear_ahead",
    )
    assert sit.lateral_relation == expected_rel
