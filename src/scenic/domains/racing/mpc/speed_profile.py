"""Centralized speed reference and trajectory logic for longitudinal MPC.

This module holds the single source of truth for:
  - What speed we ask the vehicle to track (curvature, CTE, slew, lead-step).
  - How that reference is shaped over the horizon (brake in time, don't throttle into curves).

Used by FollowRacingLineMPCBehavior so the behavior stays a thin orchestration layer:
  waypoints + state -> speed_profile.compute_speed_reference() -> v_ref_profile -> MPC.run_step().
"""

import time
from typing import List, Tuple, Optional, Dict, Any
import numpy as np

from . import timing as _mpc_timing

# --- Constants (can be overridden via config) ---
MAX_SPEED_LIMIT_MS = 62.58
CURVATURE_EPSILON = 0.001
IAC_DECEL_MS2 = 4.0
BACKWARD_PASS_DECEL_MS2 = 6.0
BACKWARD_PASS_SAFETY = 0.85
LOOKAHEAD_MIN_M = 120.0
LOOKAHEAD_TIME_S = 6.0
LOOKAHEAD_MAX_M = 300.0
AX_MAX_MS2 = 6.0
# Below this speed, skip per-step curvature cap so v_ref stays at scalar limit and the car accelerates smoothly from standstill (no throttle/brake flip-flop).
LOW_SPEED_SKIP_CURV_CAP_MS = 2.0
# Ignore curvature from segments shorter than this (m) to avoid TTL artifacts: two close points on a straight can create a false "curve".
MIN_SEGMENT_LENGTH_FOR_CURVATURE_M = 1.0


def _cte_target_speed(
    cte_mag: float,
    target_speed: float,
    current_speed: float,
    cte_stop_threshold: float,
    cte_slowdown_threshold: float,
    cte_throttle_reduction_max: float,
) -> float:
    """CTE-based speed limit for recovery when off-line. Softer bands to avoid slam-to-8m/s then brake-then-throttle."""
    if cte_mag >= 10.0:
        return min(3.0, max(0.0, current_speed - 4.0))
    if cte_mag >= 5.0:
        return min(5.0, current_speed)
    if cte_mag >= 3.0:
        return 5.0
    if cte_mag >= 2.0:
        return 7.0   # was 6.0: softer so we don't over-slow for moderate CTE
    if cte_mag >= 1.5:
        return 8.0   # was 6.0
    if cte_mag >= 1.0:
        return 9.0   # was 7.0
    if cte_mag >= 0.5:
        return 10.0  # was 8.0: avoid slamming ref down for small CTE (brake-then-throttle fix)
    if cte_mag >= cte_stop_threshold:
        return target_speed * 0.1
    if cte_mag >= cte_slowdown_threshold:
        return target_speed * 0.3
    if cte_mag >= cte_throttle_reduction_max:
        factor = 0.5 - ((cte_mag - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.2
        return target_speed * factor
    return target_speed


def compute_speed_reference(
    waypoints: List,
    wp_last_idx: int,
    current_speed: float,
    target_speed: float,
    cte_mag_for_speed: float,
    config: Any,
    last_effective_target_speed: Optional[float] = None,
    cte_stop_threshold: float = 50.0,
    cte_slowdown_threshold: float = 15.0,
    cte_throttle_reduction_max: float = 10.0,
    approach_curve_ahead: bool = False,
) -> Tuple[List[float], Optional[np.ndarray], Dict[str, Any]]:
    """Compute speed reference profile for the longitudinal MPC.

    Centralizes: three-pass velocity profile (lateral/forward/backward), curvature
    and CTE limits, slew-rate limiting, lead-step and first-step floors, and
    grade profile for 3D waypoints.

    Args:
        waypoints: List of [x, y] or [x, y, z] waypoints along the path.
        wp_last_idx: Current waypoint index (nearest ahead).
        current_speed: Current vehicle speed (m/s).
        target_speed: Desired cruising speed (m/s).
        cte_mag_for_speed: |CTE| for off-line speed reduction.
        config: MPCConfig (horizon, mpc_prediction_dt, max_lateral_acceleration, etc.).
        last_effective_target_speed: Previous step slew-limited ref (slew state).
        cte_*: CTE thresholds for speed reduction.

    Returns:
        v_ref_profile: Speed reference for each horizon step (m/s).
        grade_profile: Road grade (rad) per step, or None if 2D waypoints.
        debug: Dict for logging (curvature_speed_limit, curvature_ahead_max, effective_target_speed, ...).
    """
    t0 = time.perf_counter()
    horizon = getattr(config, 'mpc_prediction_horizon', 35)
    dt_slew = getattr(config, 'mpc_prediction_dt', 0.05)
    max_lateral_accel = getattr(config, 'max_lateral_acceleration', 8.0)
    curvature_speed_margin = getattr(config, 'curvature_speed_margin', 0.88)

    curvature_speed_limit = target_speed
    curvature_ahead_max = 0.0
    lookahead_dist = None

    n_wp = len(waypoints) if waypoints else 0
    if n_wp >= 3:
        try:
            if current_speed < 2.0:
                lookahead_dist = 25.0
            else:
                lookahead_dist = max(LOOKAHEAD_MIN_M, current_speed * LOOKAHEAD_TIME_S)
                lookahead_dist = min(lookahead_dist, LOOKAHEAD_MAX_M)

            lookahead_idx = wp_last_idx
            accumulated_dist = 0.0
            sample_points = []  # (waypoint index at segment start, distance at start)

            while accumulated_dist < lookahead_dist:
                next_idx = (lookahead_idx + 1) % n_wp
                x0, y0 = float(waypoints[lookahead_idx][0]), float(waypoints[lookahead_idx][1])
                x1, y1 = float(waypoints[next_idx][0]), float(waypoints[next_idx][1])
                seg_len = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
                if seg_len < 1e-6:
                    lookahead_idx = next_idx
                    continue
                sample_points.append((lookahead_idx, accumulated_dist))
                accumulated_dist += seg_len
                lookahead_idx = next_idx
                if lookahead_idx == wp_last_idx and len(sample_points) > 1:
                    break

            dists = []
            v_lat = []
            for sample_idx, sample_dist in sample_points:
                dists.append(sample_dist)
                v_lat_i = target_speed
                i0 = (sample_idx - 1) % n_wp
                i1 = sample_idx % n_wp
                i2 = (sample_idx + 1) % n_wp
                p0 = (float(waypoints[i0][0]), float(waypoints[i0][1]))
                p1 = (float(waypoints[i1][0]), float(waypoints[i1][1]))
                p2 = (float(waypoints[i2][0]), float(waypoints[i2][1]))
                v1x, v1y = p1[0] - p0[0], p1[1] - p0[1]
                v2x, v2y = p2[0] - p1[0], p2[1] - p1[1]
                cross = v1x * v2y - v1y * v2x
                len1 = (v1x * v1x + v1y * v1y) ** 0.5
                len2 = (v2x * v2x + v2y * v2y) ** 0.5
                if len1 > 1e-6 and len2 > 1e-6:
                    avg_len = (len1 + len2) / 2.0
                    if avg_len > 1e-6:
                        abs_kappa = abs(2.0 * cross / (len1 * len2 * avg_len))
                        # Ignore curvature from very short segments (TTL artifacts: two close points on straight -> false curve)
                        if min(len1, len2) >= MIN_SEGMENT_LENGTH_FOR_CURVATURE_M:
                            if abs_kappa > curvature_ahead_max:
                                curvature_ahead_max = abs_kappa
                            v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + CURVATURE_EPSILON)) ** 0.5
                            if abs_kappa > 0.08:
                                slow_in_margin = 0.78
                            elif abs_kappa > 0.05:
                                slow_in_margin = 0.82
                            else:
                                slow_in_margin = 0.88
                            v_max_slow_in = slow_in_margin * (max_lateral_accel / (abs_kappa + CURVATURE_EPSILON)) ** 0.5
                            v_lat_i = min(v_max_at_kappa, v_max_slow_in, target_speed)
                v_lat.append(v_lat_i)

            ax_brake_ms2 = BACKWARD_PASS_DECEL_MS2 * BACKWARD_PASS_SAFETY
            N = len(dists)
            if N >= 2:
                v_fwd = [float(v_lat[0])]
                for i in range(1, N):
                    ds = max(1e-6, dists[i] - dists[i - 1])
                    v_prev = v_fwd[-1]
                    v_accel_lim = (v_prev * v_prev + 2.0 * AX_MAX_MS2 * ds) ** 0.5
                    v_fwd.append(min(float(v_lat[i]), v_accel_lim))
                v_bwd = [None] * N
                v_bwd[N - 1] = v_fwd[N - 1]
                for i in range(N - 2, -1, -1):
                    ds = max(1e-6, dists[i + 1] - dists[i])
                    v_next = v_bwd[i + 1]
                    v_brake_lim = (v_next * v_next + 2.0 * ax_brake_ms2 * ds) ** 0.5
                    v_bwd[i] = min(v_fwd[i], v_brake_lim)
                curvature_speed_limit = min(target_speed, v_bwd[0])
                if curvature_ahead_max > 0.05 and current_speed is not None:
                    gap_to_limit = current_speed - curvature_speed_limit
                    if gap_to_limit > 12.0:
                        curvature_speed_limit = min(curvature_speed_limit, current_speed - 8.0)
                if curvature_ahead_max > 0.04:
                    if curvature_ahead_max > 0.08:
                        slow_in = 0.78
                    elif curvature_ahead_max > 0.05:
                        slow_in = 0.82
                    else:
                        slow_in = 0.88
                    v_max_at_curv = slow_in * (max_lateral_accel / (curvature_ahead_max + CURVATURE_EPSILON)) ** 0.5
                    curvature_speed_limit = min(curvature_speed_limit, v_max_at_curv)
            else:
                curvature_speed_limit = min(target_speed, v_lat[0]) if N == 1 else target_speed
        except Exception:
            pass

    cte_target_speed_val = _cte_target_speed(
        cte_mag_for_speed, target_speed, current_speed or 0.0,
        cte_stop_threshold, cte_slowdown_threshold, cte_throttle_reduction_max
    )
    effective_target_speed = min(cte_target_speed_val, curvature_speed_limit)
    effective_target_speed = min(effective_target_speed, MAX_SPEED_LIMIT_MS)
    target_before_slew = float(effective_target_speed)
    gap_above_target = (current_speed - target_before_slew) if current_speed is not None and target_before_slew is not None else 0.0

    # Simplified slew: default decel rate; stronger when we need to slow for a curve or when CTE is large.
    slew_down_ms = IAC_DECEL_MS2
    if cte_mag_for_speed >= 3.0:
        slew_down_ms = 12.0
    elif curvature_ahead_max > 0.05 and gap_above_target > 5.0:
        slew_down_ms = 10.0

    # Slew-up: slower recovery in curves so v_ref doesn't jump and cause brake-throttle-brake.
    slew_up_ms = 2.0 if curvature_ahead_max > 0.04 else 5.0

    last_eff = last_effective_target_speed if last_effective_target_speed is not None else float(effective_target_speed)
    effective_target_speed = max(
        last_eff - slew_down_ms * dt_slew,
        min(last_eff + slew_up_ms * dt_slew, float(effective_target_speed))
    )

    v_ref_profile = [float(effective_target_speed)] * horizon
    # Skip per-step curvature cap when speed < threshold: at standstill dist_ahead ~ 0 so we'd cap to curvature at current wp and cause low/oscillating v_ref -> throttle/brake flip-flop.
    if n_wp >= 2 and (current_speed or 0.0) >= LOW_SPEED_SKIP_CURV_CAP_MS:
        try:
            for k in range(horizon):
                dist_ahead = (current_speed or 0.0) * (k + 1) * dt_slew
                wp_idx = wp_last_idx
                accumulated_dist = 0.0
                while wp_idx < n_wp - 1 and accumulated_dist < dist_ahead:
                    x0, y0 = float(waypoints[wp_idx][0]), float(waypoints[wp_idx][1])
                    x1, y1 = float(waypoints[wp_idx + 1][0]), float(waypoints[wp_idx + 1][1])
                    seg_len = ((x1 - x0) ** 2 + (y1 - y0) ** 2) ** 0.5
                    if seg_len < 1e-6:
                        wp_idx += 1
                        continue
                    accumulated_dist += seg_len
                    if accumulated_dist < dist_ahead:
                        wp_idx += 1
                if wp_idx > 0 and wp_idx < n_wp - 1:
                    p0 = (float(waypoints[wp_idx - 1][0]), float(waypoints[wp_idx - 1][1]))
                    p1 = (float(waypoints[wp_idx][0]), float(waypoints[wp_idx][1]))
                    p2 = (float(waypoints[wp_idx + 1][0]), float(waypoints[wp_idx + 1][1]))
                    v1x, v1y = p1[0] - p0[0], p1[1] - p0[1]
                    v2x, v2y = p2[0] - p1[0], p2[1] - p1[1]
                    cross = v1x * v2y - v1y * v2x
                    len1 = (v1x * v1x + v1y * v1y) ** 0.5
                    len2 = (v2x * v2x + v2y * v2y) ** 0.5
                    if len1 > 1e-6 and len2 > 1e-6:
                        avg_len = (len1 + len2) / 2.0
                        if avg_len > 1e-6:
                            abs_kappa = abs(2.0 * cross / (len1 * len2 * avg_len))
                            if min(len1, len2) >= MIN_SEGMENT_LENGTH_FOR_CURVATURE_M:
                                v_max_at_kappa = curvature_speed_margin * (max_lateral_accel / (abs_kappa + CURVATURE_EPSILON)) ** 0.5
                                v_ref_profile[k] = min(v_ref_profile[k], v_max_at_kappa)
        except Exception:
            pass

    v_ref_profile = [min(v, MAX_SPEED_LIMIT_MS) for v in v_ref_profile]
    for k in range(1, horizon):
        max_drop = slew_down_ms * dt_slew
        v_ref_profile[k] = max(v_ref_profile[k], v_ref_profile[k - 1] - max_drop)

    # Lead-step: relax (gap>15, margin 4 m/s) to reduce aggressive brake then throttle
    if gap_above_target > 15.0 and curvature_ahead_max > 0.05 and current_speed is not None and current_speed > 5.0:
        v_ref_profile[0] = min(v_ref_profile[0], current_speed - 4.0)  # was 12/6: now 15/4 for smoother entry
    # First-step floors: raise 12->14 for moderate curve so we don't demand too low too early
    if curvature_ahead_max < 0.04:
        v_ref_profile[0] = max(v_ref_profile[0], 18.0)
    elif curvature_ahead_max < 0.07:
        v_ref_profile[0] = max(v_ref_profile[0], 14.0)  # was 12: smoother into curve, less brake-then-throttle

    grade_profile = None
    if n_wp >= 2 and len(waypoints[0]) >= 3:
        try:
            grade_profile = []
            for k in range(horizon):
                dist_ahead = (current_speed or 0.0) * (k + 1) * dt_slew
                wp_idx = wp_last_idx
                accumulated_dist = 0.0
                while wp_idx < n_wp - 1 and accumulated_dist < dist_ahead:
                    wp0, wp1 = waypoints[wp_idx], waypoints[wp_idx + 1]
                    dx = float(wp1[0]) - float(wp0[0])
                    dy = float(wp1[1]) - float(wp0[1])
                    dz = float(wp1[2]) - float(wp0[2]) if len(wp1) >= 3 and len(wp0) >= 3 else 0.0
                    seg_len = (dx * dx + dy * dy + dz * dz) ** 0.5
                    if seg_len < 1e-6:
                        wp_idx += 1
                        continue
                    accumulated_dist += seg_len
                    if accumulated_dist < dist_ahead:
                        wp_idx += 1
                if wp_idx < n_wp - 1:
                    wp0, wp1 = waypoints[wp_idx], waypoints[wp_idx + 1]
                    dx = float(wp1[0]) - float(wp0[0])
                    dy = float(wp1[1]) - float(wp0[1])
                    dz = float(wp1[2]) - float(wp0[2]) if len(wp1) >= 3 and len(wp0) >= 3 else 0.0
                    seg_len_xy = (dx * dx + dy * dy) ** 0.5
                    grade = np.arctan2(dz, seg_len_xy) if seg_len_xy > 1e-6 else 0.0
                else:
                    grade = 0.0
                grade_profile.append(grade)
            grade_profile = np.array(grade_profile, dtype=np.float64)
        except Exception:
            grade_profile = None

    debug = {
        'curvature_speed_limit': curvature_speed_limit,
        'curvature_ahead_max': curvature_ahead_max,
        'effective_target_speed': float(effective_target_speed),
        'effective_target_before_slew': target_before_slew,
        'slew_down_ms': slew_down_ms,
        'gap_above_target': gap_above_target,
        'lookahead_dist': lookahead_dist,
    }
    _mpc_timing.record_speed_profile_ms((time.perf_counter() - t0) * 1000)
    return v_ref_profile, grade_profile, debug
