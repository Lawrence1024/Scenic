"""SD-26/27 offline regression bank for the strategy simulator.

Calls ``simulate_strategy`` and ``FellowPredictor`` directly with synthetic
geometries (parallel straight polylines, simple fellow trajectories) and
asserts the prediction stack behaves correctly. No Scenic compile, no
dSPACE, no full planner — pure unit-bank style.

Mirrors the SD-24 placement bank pattern: single-command runner, log
output to file + stdout, exit code 0 on full pass / 1 on any failure.

USAGE:

    python src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py
    python src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py --log sd26_sim.log

Cases cover:
- SD-26: _blend_alpha math at t=0, t=tau, t=3*tau (saturation curve)
- SD-26: tau=0 reproduces the legacy instantaneous-switch behaviour
- SD-26: pass_left with tau=2.5 places ego at intermediate y values during lane_change
- SD-26: pass_right symmetric on the opposite side
- SD-26: stay_optimal unaffected by tau
- SD-26: merge_back returns ego smoothly (reverse-blend, SD-27b)
- SD-27a: FellowPredictor.trajectory uses CTR — straight history collapses
  to CV; curving history bends the predicted xy
- SD-27b: simulate_strategy reports OBB edge-to-edge gap, not centroid
- SD-27b: lateral pass with cars side-by-side at 5m centroid → ~3m OBB gap
- SD-27b: full overlap geometry → 0 OBB clearance (hard filter must reject)
"""

from __future__ import annotations

import argparse
import math
import sys
import time
from dataclasses import dataclass
from pathlib import Path
from typing import Callable, List, Optional


try:
    sys.stdout.reconfigure(encoding="utf-8", errors="replace")
    sys.stderr.reconfigure(encoding="utf-8", errors="replace")
except Exception:
    pass


from scenic.domains.racing.prediction.strategy_simulator import (
    _blend_alpha,
    simulate_strategy,
)
from scenic.domains.racing.prediction.fellow_predictor import FellowPredictor
from scenic.domains.racing.eval_geometry import (
    IAC_DALLARA_LENGTH_M,
    IAC_DALLARA_WIDTH_M,
)


# ---------------------------------------------------------------------------
# Synthetic geometry helpers
# ---------------------------------------------------------------------------

def _straight_polyline(y_offset: float, x_max: float = 500.0, dx: float = 5.0):
    """Return a list of (x, y) waypoints along y=y_offset, x from 0 to x_max."""
    n = int(x_max / dx) + 1
    return [(i * dx, float(y_offset)) for i in range(n)]


def _fellow_traj_constant_velocity(
    x0: float, y0: float, vx: float, vy: float, horizon_s: float, dt_s: float = 0.5
):
    """CV-extrapolated fellow trajectory: list of (t, x, y, s_or_None)."""
    n = int(horizon_s / dt_s) + 1
    return [
        (i * dt_s, x0 + vx * i * dt_s, y0 + vy * i * dt_s, None)
        for i in range(n)
    ]


_DEFAULT_KW = dict(
    horizon_s=10.0,
    sample_dt_s=0.5,
    target_speed_mps=20.0,
    accel_mps2=4.0,
    setup_speed_margin_mps=4.5,
    commit_speed_margin_mps=16.0,
    post_pass_buffer_m=5.0,
    lane_change_s=1.5,
)


# ---------------------------------------------------------------------------
# Bank cases
# ---------------------------------------------------------------------------

@dataclass
class BankCase:
    name: str
    description: str
    check: Callable[[], tuple]  # returns (passed: bool, detail: str)


def _approx(a: float, b: float, tol: float = 1e-6) -> bool:
    return abs(a - b) < tol


def _case_blend_alpha_math():
    """_blend_alpha at t=0 → 0; at t=tau → ~0.632; at t=3*tau → ~0.950; saturated."""
    a0 = _blend_alpha(0.0, 2.5)
    a_tau = _blend_alpha(2.5, 2.5)
    a_3tau = _blend_alpha(7.5, 2.5)
    a_inf = _blend_alpha(100.0, 2.5)
    a_tau0 = _blend_alpha(1.0, 0.0)  # tau=0 → always 1.0

    fail = []
    if not _approx(a0, 0.0, 1e-9):
        fail.append(f"alpha(0, tau=2.5)={a0}, expected 0.0")
    if not _approx(a_tau, 1.0 - math.exp(-1.0), 1e-9):
        fail.append(f"alpha(tau, tau)={a_tau}, expected {1.0 - math.exp(-1.0):.6f}")
    if not _approx(a_3tau, 1.0 - math.exp(-3.0), 1e-9):
        fail.append(f"alpha(3*tau, tau)={a_3tau}, expected {1.0 - math.exp(-3.0):.6f}")
    if not _approx(a_inf, 1.0, 1e-9):
        fail.append(f"alpha(100, tau=2.5)={a_inf}, expected 1.0 (saturated)")
    if not _approx(a_tau0, 1.0, 1e-9):
        fail.append(f"alpha(1.0, tau=0)={a_tau0}, expected 1.0 (degenerate tau)")

    if fail:
        return (False, "; ".join(fail))
    return (True, f"a(0)={a0:.3f}, a(tau)={a_tau:.3f}, a(3tau)={a_3tau:.3f}")


def _case_tau_zero_reproduces_instantaneous():
    """With tau=0, ego_xy at every sample is exactly on the side polyline (legacy
    behaviour). With tau>0, ego_xy is between the polylines for early samples."""
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=200.0, y0=5.0, vx=8.0, vy=0.0, horizon_s=10.0)

    out_legacy = simulate_strategy(
        "pass_left",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=200.0,
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=0.0,  # legacy
        **_DEFAULT_KW,
    )
    out_blended = simulate_strategy(
        "pass_left",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=200.0,
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out_legacy.reason != "ok" or out_blended.reason != "ok":
        return (False, f"reason: legacy={out_legacy.reason} blended={out_blended.reason}")

    # Compare the y-coord of ego at t=0.5 s (sample index 1).
    # Legacy: ego on left polyline → y=5.0
    # Blended: ego at α(0.5, 2.5)=0.181 of the way to left → y≈0.91
    s_legacy = out_legacy.samples[1]   # (t, x, y, v, phase, fx, fy, clearance)
    s_blended = out_blended.samples[1]
    y_legacy = s_legacy[2]
    y_blended = s_blended[2]
    expected_alpha = 1.0 - math.exp(-0.5 / 2.5)  # ≈ 0.181
    expected_y_blended = expected_alpha * 5.0

    fail = []
    if not _approx(y_legacy, 5.0, 0.01):
        fail.append(f"legacy ego_y at t=0.5={y_legacy}, expected 5.0")
    if not _approx(y_blended, expected_y_blended, 0.01):
        fail.append(f"blended ego_y at t=0.5={y_blended}, expected ≈{expected_y_blended:.3f}")
    # Critical: legacy and blended produce DIFFERENT ego_y values for early samples
    if abs(y_legacy - y_blended) < 1.0:
        fail.append(f"legacy and blended too close at t=0.5: {y_legacy} vs {y_blended}")

    if fail:
        return (False, "; ".join(fail))
    return (True, f"legacy y(0.5)=5.0, blended y(0.5)={y_blended:.3f} (expected {expected_y_blended:.3f})")


def _case_pass_left_blend_y_trajectory():
    """For pass_left with tau=2.5, ego_y at successive samples should follow
    α(t)·5.0 during the lane_change/alongside phases."""
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=300.0, y0=5.0, vx=8.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=300.0,  # far ahead so we stay in lane_change/alongside
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")

    # Walk first ~5 samples and verify ego_y matches α(t)·5
    fail = []
    for s in out.samples[:6]:  # t = 0, 0.5, 1.0, 1.5, 2.0, 2.5
        t_off = s[0]
        ego_y = s[2]
        phase = s[4]
        if phase == "merge_back":
            continue  # different rule applies; not what this case covers
        expected_y = (1.0 - math.exp(-t_off / 2.5)) * 5.0
        if not _approx(ego_y, expected_y, 0.05):
            fail.append(f"t={t_off:.1f}: ego_y={ego_y:.3f}, expected {expected_y:.3f}")
    if fail:
        return (False, "; ".join(fail))
    return (True, f"y trajectory follows α(t)·5 across {len(out.samples)} samples")


def _case_pass_right_symmetric():
    """For pass_right with tau=2.5, ego_y should drift toward the RIGHT polyline (y=-5)."""
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=300.0, y0=5.0, vx=8.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "pass_right",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=300.0,
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")

    fail = []
    for s in out.samples[:6]:
        t_off = s[0]
        ego_y = s[2]
        phase = s[4]
        if phase == "merge_back":
            continue
        expected_y = (1.0 - math.exp(-t_off / 2.5)) * -5.0  # toward y=-5
        if not _approx(ego_y, expected_y, 0.05):
            fail.append(f"t={t_off:.1f}: ego_y={ego_y:.3f}, expected {expected_y:.3f}")
    if fail:
        return (False, "; ".join(fail))
    return (True, f"y trajectory follows α(t)·(-5) (right side) across {len(out.samples)} samples")


def _case_stay_optimal_unaffected_by_tau():
    """stay_optimal with tau=0 vs tau=2.5 produces identical ego_xy values at every sample."""
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=300.0, y0=5.0, vx=8.0, vy=0.0, horizon_s=10.0)

    out_zero = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=300.0,
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=0.0,
        **_DEFAULT_KW,
    )
    out_blend = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=300.0,
        opp_speed_mps=8.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out_zero.reason != "ok" or out_blend.reason != "ok":
        return (False, f"reason: zero={out_zero.reason} blend={out_blend.reason}")

    fail = []
    for i, (sa, sb) in enumerate(zip(out_zero.samples, out_blend.samples)):
        if not _approx(sa[1], sb[1], 1e-6) or not _approx(sa[2], sb[2], 1e-6):
            fail.append(f"sample {i}: zero=({sa[1]},{sa[2]}) blend=({sb[1]},{sb[2]})")
            break  # one failure is enough
    if fail:
        return (False, "; ".join(fail))
    return (True, f"all {len(out_zero.samples)} samples identical between tau=0 and tau=2.5 for stay_optimal")


def _case_merge_back_reverse_blends():
    """SD-27b: merge_back smoothly reverse-blends ego back to optimal — it does
    NOT snap to y=0 at the first merge_back tick (which the pre-SD-27 SD-26
    code did, breaking OBB clearance for safe passes that complete the merge).

    The first merge_back tick should still have ego_y near the side polyline
    (alpha decays from its alongside-final value, not jumping to 0). After
    several tau_s of decay, ego_y approaches 0.
    """
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=20.0, y0=5.0, vx=5.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0,
        ego_speed_mps=20.0,
        opp_s_m=20.0,  # only 20m ahead, ego closes fast
        opp_speed_mps=5.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")

    merge_back_samples = [s for s in out.samples if s[4] == "merge_back"]
    if not merge_back_samples:
        return (False, "no merge_back phase samples observed; geometry may be wrong")

    fail = []
    # First merge_back tick: ego_y should be NEAR the side polyline (>2.0m),
    # not snapped to 0. Without reverse-blend the y would be 0 here.
    first_mb = merge_back_samples[0]
    if first_mb[2] < 2.0:
        fail.append(f"first merge_back sample at t={first_mb[0]}: ego_y={first_mb[2]:.3f}, "
                    "expected >= 2.0 (reverse-blend, not snap to 0)")
    # Last merge_back tick (if multiple): ego_y should have decayed substantially.
    if len(merge_back_samples) >= 4:
        last_mb = merge_back_samples[-1]
        if last_mb[2] >= first_mb[2] - 0.1:
            fail.append(f"last merge_back ego_y={last_mb[2]:.3f} did not decay below "
                        f"first_mb={first_mb[2]:.3f}")
    if fail:
        return (False, "; ".join(fail))
    return (True, f"{len(merge_back_samples)} merge_back samples reverse-blend "
            f"from y={first_mb[2]:.2f} toward 0")


def _case_pass_into_far_fellow_does_not_collide_in_simulator():
    """Sanity: pass_left into a fellow that's FAR ahead and never gets caught
    should report a healthy min_clearance, regardless of tau. This
    confirms SD-26/27 doesn't make the simulator pathologically pessimistic."""
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=400.0, y0=5.0, vx=15.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "pass_left",
        ego_s_m=0.0,
        ego_speed_mps=15.0,  # same speed as fellow, never catches up
        opp_s_m=400.0,
        opp_speed_mps=15.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")
    # SD-27b OBB metric subtracts up to ~5m worth of vehicle extents.
    if out.min_clearance_m < 90.0:
        return (False, f"min_clearance_m={out.min_clearance_m}, expected >90m for far-fellow")
    return (True, f"min_clearance_m={out.min_clearance_m:.1f}m (fellow stays far ahead)")


# ---------------------------------------------------------------------------
# SD-27a: CTR fellow prediction
# ---------------------------------------------------------------------------

def _case_ctr_straight_history_collapses_to_cv():
    """SD-27a: straight-line history → yaw rate ~0 → CTR falls back to CV.

    Feed FellowPredictor 5 samples along y=0, vx=10. The trajectory()
    output should match the legacy CV result (xy advancing along +x).
    """
    p = FellowPredictor(history_decay_s=0.5, history_max_age_s=2.0)
    for i in range(5):
        p.step(
            sim_time_s=float(i) * 0.1,
            x=float(i) * 1.0,  # vx = 10 m/s
            y=0.0,
            fellow_progress_s_m=None,
            dt_pred_s=0.1,
        )
    samples = p.trajectory(horizon_s=2.0, sample_dt_s=0.5)
    if len(samples) < 5:
        return (False, f"trajectory() returned {len(samples)} samples (expected 5)")
    # Last observation at (4, 0) and t=0.4. trajectory[0] is the current obs.
    # trajectory[i] for i >= 1 should be at (4 + 10*i*0.5, 0).
    fail = []
    for i in range(1, len(samples)):
        t_off, x, y, _ = samples[i]
        expected_x = 4.0 + 10.0 * t_off
        if abs(x - expected_x) > 0.5:
            fail.append(f"sample {i} t={t_off}: x={x:.3f}, expected {expected_x:.3f}")
        if abs(y) > 0.05:
            fail.append(f"sample {i} t={t_off}: y={y:.3f}, expected ~0 (straight)")
    if fail:
        return (False, "; ".join(fail))
    return (True, f"straight history → CV: trajectory matches expected x advance")


def _case_ctr_curving_history_bends_prediction():
    """SD-27a: curving history → non-zero yaw rate → CTR bends predicted xy.

    Feed FellowPredictor a circular arc (constant turn rate of 0.2 rad/s).
    The trajectory() output should follow the same arc, not a straight CV
    line. Compare to a CV baseline: the difference must grow monotonically.
    """
    p_ctr = FellowPredictor(history_decay_s=0.5, history_max_age_s=2.0)
    p_cv = FellowPredictor(history_decay_s=0.5, history_max_age_s=2.0)

    yaw_rate = 0.2  # rad/s
    speed = 10.0  # m/s
    radius = speed / yaw_rate  # 50m
    n_obs = 6
    dt = 0.1
    for i in range(n_obs):
        t = i * dt
        # Circular arc starting at origin, heading +x at t=0.
        angle = yaw_rate * t
        x = radius * math.sin(angle)
        y = radius * (1.0 - math.cos(angle))
        p_ctr.step(sim_time_s=t, x=x, y=y, fellow_progress_s_m=None, dt_pred_s=dt)
        p_cv.step(sim_time_s=t, x=x, y=y, fellow_progress_s_m=None, dt_pred_s=dt)

    s_ctr = p_ctr.trajectory(horizon_s=2.0, sample_dt_s=0.5, use_ctr=True)
    s_cv = p_cv.trajectory(horizon_s=2.0, sample_dt_s=0.5, use_ctr=False)
    if len(s_ctr) != len(s_cv) or len(s_ctr) < 5:
        return (False, f"len mismatch: ctr={len(s_ctr)} cv={len(s_cv)}")

    # CTR should match the true arc; CV diverges. Check the last sample.
    t_last = s_ctr[-1][0]  # 2.0 s
    x_ctr_last, y_ctr_last = s_ctr[-1][1], s_ctr[-1][2]
    x_cv_last, y_cv_last = s_cv[-1][1], s_cv[-1][2]

    # True arc position at t = 0.5 (end of obs) + 2.0 = 2.5 s from arc start.
    t_obs_end = (n_obs - 1) * dt
    angle_at_traj_end = yaw_rate * (t_obs_end + t_last)
    x_true = radius * math.sin(angle_at_traj_end)
    y_true = radius * (1.0 - math.cos(angle_at_traj_end))

    err_ctr = math.hypot(x_ctr_last - x_true, y_ctr_last - y_true)
    err_cv = math.hypot(x_cv_last - x_true, y_cv_last - y_true)

    fail = []
    # CTR should be substantially closer to the true arc than CV.
    if err_ctr >= err_cv:
        fail.append(f"CTR err {err_ctr:.2f} not better than CV err {err_cv:.2f}")
    if err_ctr > 2.0:
        fail.append(f"CTR err {err_ctr:.2f} m too large vs true arc")
    if err_cv < 1.0:
        fail.append(f"CV err {err_cv:.2f} m too small — geometry not curving enough")
    if fail:
        return (False, "; ".join(fail))
    return (True, f"CTR err={err_ctr:.2f}m, CV err={err_cv:.2f}m — CTR tracks the arc, CV diverges")


# ---------------------------------------------------------------------------
# SD-27b: OBB-aware clearance
# ---------------------------------------------------------------------------

def _case_obb_clearance_lateral_pass():
    """SD-27b: side-by-side at 5m centroid → ~3m OBB gap.

    Stay_optimal with fellow at lat=-5. Centroid distance min ≈ 5m. With
    IAC dimensions (1.93m wide), OBB edge-to-edge gap ≈ 5 - 1.93 = 3.07m.
    Pre-SD-27 the metric was centroid (5m); post-SD-27 it's OBB (3m).
    """
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    fellow_traj = _fellow_traj_constant_velocity(x0=30.0, y0=-5.0, vx=0.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0,
        ego_speed_mps=15.0,
        opp_s_m=30.0,
        opp_speed_mps=0.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")
    # OBB lateral gap = 5 - 2*0.965 = 3.07m. Allow ±0.2m for finite samples.
    if not (2.8 <= out.min_clearance_m <= 3.3):
        return (False, f"min_clearance_m={out.min_clearance_m:.3f}, expected ~3.07m (OBB)")
    return (True, f"min_clearance_m={out.min_clearance_m:.3f}m (OBB lateral 5m - 1.93m width)")


def _case_obb_full_overlap_returns_zero():
    """SD-27b: cars at the same xy → OBB clearance 0 (hard filter must reject).

    Sample 2 of the 30-sample CE campaign exhibited a real full-overlap
    collision while the simulator predicted a 3.95m centroid clearance.
    Post-SD-27, the same geometry returns OBB ~0, falling below the
    new 0.5m hard threshold and rejecting the unsafe strategy.
    """
    polylines = {
        "optimal": _straight_polyline(0.0),
        "left": _straight_polyline(5.0),
        "right": _straight_polyline(-5.0),
    }
    # Fellow ON THE OPTIMAL polyline ahead; ego catches up alongside (overlap).
    fellow_traj = _fellow_traj_constant_velocity(x0=10.0, y0=0.0, vx=5.0, vy=0.0, horizon_s=10.0)

    out = simulate_strategy(
        "stay_optimal",
        ego_s_m=0.0,
        ego_speed_mps=20.0,
        opp_s_m=10.0,
        opp_speed_mps=5.0,
        fellow_traj=fellow_traj,
        polylines=polylines,
        lap_length_m=500.0,
        lane_change_tau_s=2.5,
        **_DEFAULT_KW,
    )

    if out.reason != "ok":
        return (False, f"reason={out.reason}")
    if out.min_clearance_m > 0.5:
        return (False, f"min_clearance_m={out.min_clearance_m:.3f}, expected ~0 (full overlap)")
    return (True, f"min_clearance_m={out.min_clearance_m:.3f}m → fails 0.5m hard filter (correct)")


_BANK: List[BankCase] = [
    BankCase(
        name="blend_alpha_math",
        description="_blend_alpha math: 0 at t=0, ~0.632 at t=tau, ~0.950 at t=3*tau, 1.0 saturated, 1.0 if tau≤0.",
        check=_case_blend_alpha_math,
    ),
    BankCase(
        name="tau_zero_vs_tau_blend_differ",
        description="tau=0 places ego on side polyline immediately (legacy); tau=2.5 places ego at intermediate y values for early samples.",
        check=_case_tau_zero_reproduces_instantaneous,
    ),
    BankCase(
        name="pass_left_y_follows_blend_curve",
        description="For pass_left tau=2.5, ego_y at each sample should match α(t)·5 during lane_change/alongside phases.",
        check=_case_pass_left_blend_y_trajectory,
    ),
    BankCase(
        name="pass_right_symmetric",
        description="For pass_right tau=2.5, ego_y drifts toward y=-5 on the same exponential curve. Confirms the fix is symmetric.",
        check=_case_pass_right_symmetric,
    ),
    BankCase(
        name="stay_optimal_unaffected",
        description="stay_optimal with tau=0 vs tau=2.5 produces identical ego_xy. The blending only applies to pass_left/pass_right.",
        check=_case_stay_optimal_unaffected_by_tau,
    ),
    BankCase(
        name="merge_back_reverse_blends",
        description="SD-27b: merge_back uses a reverse-blend back to optimal; first MB tick is still near the side polyline, not snapped to 0.",
        check=_case_merge_back_reverse_blends,
    ),
    BankCase(
        name="far_fellow_no_pathological_clearance",
        description="Sanity check: pass_left against a fellow that's far ahead and never caught reports a healthy (>90m) min_clearance.",
        check=_case_pass_into_far_fellow_does_not_collide_in_simulator,
    ),
    BankCase(
        name="ctr_straight_history_collapses_to_cv",
        description="SD-27a: FellowPredictor.trajectory with straight history extrapolates linearly (CTR yaw_rate ≈ 0 → CV).",
        check=_case_ctr_straight_history_collapses_to_cv,
    ),
    BankCase(
        name="ctr_curving_history_bends_prediction",
        description="SD-27a: FellowPredictor.trajectory with circular-arc history follows the arc (CTR), unlike CV which flies off tangent.",
        check=_case_ctr_curving_history_bends_prediction,
    ),
    BankCase(
        name="obb_clearance_lateral_pass",
        description="SD-27b: ego on optimal y=0, fellow lat=-5 → OBB clearance ~3m (centroid 5m − IAC width 1.93m).",
        check=_case_obb_clearance_lateral_pass,
    ),
    BankCase(
        name="obb_full_overlap_returns_zero",
        description="SD-27b: full vehicle overlap geometry → OBB clearance 0; the new 0.5m hard filter rejects the unsafe strategy.",
        check=_case_obb_full_overlap_returns_zero,
    ),
]


# ---------------------------------------------------------------------------
# Runner
# ---------------------------------------------------------------------------

@dataclass
class CaseResult:
    name: str
    passed: bool
    detail: str


def _write_log(out, line: str = ""):
    print(line)
    out.write(line + "\n")
    out.flush()


def main() -> int:
    parser = argparse.ArgumentParser(
        description="SD-26 offline regression bank for the strategy simulator."
    )
    parser.add_argument(
        "--log", "-l", type=str, default=None,
        help="Output log path. Defaults to "
             "sd26_simulator_unit_bank_<TIMESTAMP>.log in cwd."
    )
    parser.add_argument(
        "--filter", "-f", type=str, default=None,
        help="Only run cases whose name contains this substring."
    )
    args = parser.parse_args()

    log_path = (
        Path(args.log) if args.log else
        Path.cwd() / f"sd26_simulator_unit_bank_{time.strftime('%Y%m%d_%H%M%S')}.log"
    )
    log_path.parent.mkdir(parents=True, exist_ok=True)

    cases = _BANK
    if args.filter:
        cases = [c for c in _BANK if args.filter in c.name]
    if not cases:
        print(f"[Bank] no cases matched filter '{args.filter}'")
        return 2

    with log_path.open("w", encoding="utf-8") as fh:
        _write_log(fh, "=" * 78)
        _write_log(fh, "SD-26 strategy_simulator unit bank")
        _write_log(fh, f"  cases : {len(cases)}")
        _write_log(fh, f"  log   : {log_path}")
        _write_log(fh, "=" * 78)
        _write_log(fh)

        results: List[CaseResult] = []
        for case in cases:
            _write_log(fh, f"[CASE] {case.name}")
            _write_log(fh, f"       {case.description}")
            try:
                passed, detail = case.check()
            except Exception as exc:
                passed, detail = False, f"raised: {exc!r}"
            results.append(CaseResult(case.name, passed, detail))
            tag = "PASS" if passed else "FAIL"
            _write_log(fh, f"       {tag}: {detail}")
            _write_log(fh)

        _write_log(fh, "=" * 78)
        _write_log(fh, "Summary")
        _write_log(fh, "=" * 78)
        n_total = len(results)
        n_pass = sum(1 for r in results if r.passed)
        n_fail = n_total - n_pass
        for r in results:
            tag = "PASS" if r.passed else "FAIL"
            _write_log(fh, f"  [{tag}] {r.name}")
        _write_log(fh)
        _write_log(fh, f"  TOTAL: {n_pass}/{n_total} pass  ({n_fail} fail)")

    print(f"\n[Bank] log written to {log_path}")
    return 0 if n_fail == 0 else 1


if __name__ == "__main__":
    raise SystemExit(main())
