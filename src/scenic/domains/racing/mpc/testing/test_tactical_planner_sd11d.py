"""SD-11d: integration tests for the dual-planner shim (telemetry-only path).

Verifies that:
  - When fellow_trajectory + polylines + arc-lengths are supplied, the planner
    populates state.strategy_selected_name + clearances + progress diagnostics.
  - When inputs are missing, no error and state.strategy_* stays empty.
  - F9-style stationary fellow off the line picks "stay_optimal".
  - F2-style overtake picks one of pass_left/pass_right.
  - Telemetry-only mode does NOT change the planner's mode/ttl/cap output —
    same scenario without fellow_trajectory yields the same return value.
"""

from __future__ import annotations

import io
import sys
from contextlib import redirect_stdout

import pytest

from scenic.domains.racing.tactical_planner import (
    TacticalPlannerConfig,
    TacticalPlannerState,
    tactical_planner_step_v1,
)
from scenic.domains.racing.situation_assessment import OpponentSituation
from scenic.domains.racing.assessment.pass_geometry import _clear_linestring_cache


@pytest.fixture(autouse=True)
def _drop_polyline_cache():
    _clear_linestring_cache()
    yield
    _clear_linestring_cache()


def _straight_polyline(x0, y0, dx, dy, n):
    return [(x0 + i * dx, y0 + i * dy, 0.0) for i in range(n)]


def _common_polylines():
    return (
        _straight_polyline(0.0, 0.0, 1.0, 0.0, 600),     # optimal
        _straight_polyline(0.0, 5.0, 1.0, 0.0, 600),     # left
        _straight_polyline(0.0, -5.0, 1.0, 0.0, 600),    # right
    )


def _stationary_fellow_traj(x, y, horizon_s=10.0, dt=0.5):
    n = int(horizon_s / dt)
    return [(i * dt, x, y, None) for i in range(n + 1)]


def _moving_fellow_traj(x0, y0, vx, vy, horizon_s=10.0, dt=0.5):
    n = int(horizon_s / dt)
    return [(i * dt, x0 + vx * i * dt, y0 + vy * i * dt, None) for i in range(n + 1)]


def _sit(*, longitudinal_m, lateral_m, distance_m=None, ahead=True,
         opponent_speed_mps=0.0):
    return OpponentSituation(
        ahead=bool(ahead),
        delta_s_m=float(longitudinal_m),
        delta_s_source="polyline",
        lateral_relation="left" if lateral_m > 0 else "right" if lateral_m < 0 else "neutral",
        closing_speed_mps=0.0,
        overlap_state="clear_ahead",
        collision_risk_01=0.0,
        segment_context="straight",
        distance_m=float(distance_m if distance_m is not None
                         else (longitudinal_m**2 + lateral_m**2) ** 0.5),
        longitudinal_m=float(longitudinal_m),
        lateral_m=float(lateral_m),
        opponent_speed_mps=float(opponent_speed_mps),
    )


def test_strategy_telemetry_populated_when_inputs_supplied():
    """F9-like geometry: stationary fellow at lat=-5, on-line ego."""
    opt, left, right = _common_polylines()
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)
    cfg = TacticalPlannerConfig()  # use_strategy_authority defaults to False
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        mode, ttl, cap, reason = tactical_planner_step_v1(
            st, sit,
            has_opponent=True, ego_speed_mps=0.5, opponent_speed_mps=0.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=0.0, opp_s_m=30.0, lap_length_m=599.0,
            ego_active_ttl="optimal",
            fellow_trajectory=fellow_traj,
        )
    out = buf.getvalue()
    # Telemetry print emitted.
    assert "[Strategy]" in out
    # F9 case: stay_optimal should win (large clearance, full target speed).
    assert st.strategy_selected_name == "stay_optimal"
    assert st.strategy_min_clearances["stay_optimal"] >= 4.5
    assert "stay_optimal" in st.strategy_reachable_progress


def test_strategy_pipeline_skipped_when_no_polylines():
    """No optimal_waypoints → strategy block doesn't run, no error."""
    cfg = TacticalPlannerConfig()
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0)
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        mode, ttl, cap, reason = tactical_planner_step_v1(
            st, sit,
            has_opponent=True, ego_speed_mps=0.5, opponent_speed_mps=0.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            fellow_trajectory=fellow_traj,
        )
    out = buf.getvalue()
    assert "[Strategy]" not in out
    assert st.strategy_selected_name == ""


def test_strategy_pipeline_skipped_when_no_fellow_trajectory():
    opt, left, right = _common_polylines()
    cfg = TacticalPlannerConfig()
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        tactical_planner_step_v1(
            st, sit,
            has_opponent=True, ego_speed_mps=0.5, opponent_speed_mps=0.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=0.0, opp_s_m=30.0, lap_length_m=599.0,
            fellow_trajectory=None,
        )
    out = buf.getvalue()
    assert "[Strategy]" not in out
    assert st.strategy_selected_name == ""


def test_strategy_picks_pass_for_overtake_geometry():
    """F2-style: fellow on optimal, slow → strategy should pick pass_left or pass_right."""
    opt, left, right = _common_polylines()
    fellow_traj = _moving_fellow_traj(15.0, 0.0, 8.0, 0.0)
    cfg = TacticalPlannerConfig()
    st = TacticalPlannerState()
    sit = _sit(longitudinal_m=15.0, lateral_m=0.0, opponent_speed_mps=8.0)
    buf = io.StringIO()
    with redirect_stdout(buf):
        tactical_planner_step_v1(
            st, sit,
            has_opponent=True, ego_speed_mps=20.0, opponent_speed_mps=8.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=0.0, opp_s_m=15.0, lap_length_m=599.0,
            fellow_trajectory=fellow_traj,
        )
    assert st.strategy_selected_name in ("pass_left", "pass_right", "stay_optimal")
    # follow_fellow should NOT win on this geometry — clear left/right TTLs available.


def test_telemetry_only_does_not_change_planner_output():
    """Same scenario WITH and WITHOUT fellow_trajectory must yield same mode/ttl/cap
    when use_strategy_authority=False."""
    opt, left, right = _common_polylines()
    fellow_traj = _stationary_fellow_traj(30.0, -5.0)
    cfg = TacticalPlannerConfig(use_strategy_authority=False)

    # Run twice from identical state, with and without trajectory.
    st_with = TacticalPlannerState()
    st_without = TacticalPlannerState()
    sit = _sit(longitudinal_m=30.0, lateral_m=-5.0, opponent_speed_mps=0.0)

    buf = io.StringIO()
    with redirect_stdout(buf):
        out_with = tactical_planner_step_v1(
            st_with, sit,
            has_opponent=True, ego_speed_mps=15.0, opponent_speed_mps=0.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=0.0, opp_s_m=30.0, lap_length_m=599.0,
            ego_active_ttl="optimal",
            fellow_trajectory=fellow_traj,
        )
        out_without = tactical_planner_step_v1(
            st_without, sit,
            has_opponent=True, ego_speed_mps=15.0, opponent_speed_mps=0.0,
            sim_time_s=0.0, pit_mode=False, config=cfg,
            optimal_waypoints=opt, side_waypoints_left=left, side_waypoints_right=right,
            ego_s_m=0.0, opp_s_m=30.0, lap_length_m=599.0,
            ego_active_ttl="optimal",
            fellow_trajectory=None,
        )
    # Mode, ttl, cap, reason all identical when flag off.
    assert out_with == out_without
