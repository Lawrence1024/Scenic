"""SD-26 offline regression bank for the strategy simulator's lane-change blending.

Calls ``simulate_strategy`` directly with synthetic geometries (parallel
straight polylines, simple fellow trajectories) and asserts the new
blending math behaves correctly. No Scenic compile, no dSPACE, no
trajectory predictor — pure unit-bank style on the new
``_blend_alpha`` helper and the modified ``simulate_strategy``.

Mirrors the SD-24 placement bank pattern: single-command runner, log
output to file + stdout, exit code 0 on full pass / 1 on any failure.

USAGE:

    python src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py
    python src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py --log sd26_sim.log

Cases cover:
- _blend_alpha math at t=0, t=tau, t=3*tau (saturation curve)
- tau=0 reproduces the legacy instantaneous-switch behaviour
- pass_left with tau=2.5 places ego at intermediate y values during lane_change
- pass_right with tau=2.5 places ego at intermediate y values on the opposite side (symmetry)
- stay_optimal with any tau is unaffected (always on optimal polyline)
- merge_back phase routes ego back to optimal regardless of tau
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


def _case_merge_back_uses_optimal():
    """During merge_back phase, ego_y should be 0 (optimal) regardless of tau.

    Setup: small initial gap so ego catches fellow within ~3s, triggering
    merge_back for the latter half of the horizon.
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

    # Find samples in merge_back phase. Verify ego_y == 0 (on optimal).
    merge_back_samples = [s for s in out.samples if s[4] == "merge_back"]
    if not merge_back_samples:
        return (False, "no merge_back phase samples observed; geometry may be wrong")

    fail = []
    for s in merge_back_samples:
        if not _approx(s[2], 0.0, 0.01):  # y should be 0 in merge_back
            fail.append(f"merge_back sample at t={s[0]}: ego_y={s[2]}, expected 0")
            break
    if fail:
        return (False, "; ".join(fail))
    return (True, f"{len(merge_back_samples)} merge_back samples all on optimal (y=0)")


def _case_pass_into_far_fellow_does_not_collide_in_simulator():
    """Sanity: pass_left into a fellow that's FAR ahead and never gets caught
    should report a healthy (>5m) min_clearance, regardless of tau. This
    confirms SD-26 doesn't make the simulator pathologically pessimistic."""
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
    if out.min_clearance_m < 100.0:
        return (False, f"min_clearance_m={out.min_clearance_m}, expected >100m for far-fellow")
    return (True, f"min_clearance_m={out.min_clearance_m:.1f}m (fellow stays far ahead)")


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
        name="merge_back_routes_to_optimal",
        description="During merge_back phase, ego_y returns to 0 (on optimal polyline) regardless of tau.",
        check=_case_merge_back_uses_optimal,
    ),
    BankCase(
        name="far_fellow_no_pathological_clearance",
        description="Sanity check: pass_left against a fellow that's far ahead and never caught reports a healthy (>100m) min_clearance.",
        check=_case_pass_into_far_fellow_does_not_collide_in_simulator,
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
