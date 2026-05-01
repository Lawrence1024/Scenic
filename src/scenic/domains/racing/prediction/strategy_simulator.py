"""SD-11b: ego strategy trajectory simulator.

For each candidate ego strategy (stay_optimal, follow_fellow, pass_left,
pass_right), simulate the ego forward in time over the planning horizon and
report the minimum clearance to a predicted fellow trajectory plus the
ego's reachable progress at horizon end.

Used by SD-11c's strategy selector to pick the fastest strategy whose
predicted clearance stays above the safety threshold across the entire
horizon. The selected strategy then becomes the planner's primary decision
(SD-11d/e).

Design notes:
  - This is FORECAST, not control. Speed profiles are simple piecewise
    constant-acceleration ramps; the actual lateral motion of a lane change
    is handled by the MPC at runtime. SD-26 added lane-change blending
    so the simulator no longer teleports ego onto the side polyline at t=0+.
  - Reuses the cached _xy_at_arclength from assessment/pass_geometry so each
    polyline is constructed at most once per simulator call.
  - Fellow trajectory comes from FellowPredictor.trajectory(); SD-27a made
    that a constant-turn-rate (CTR) extrapolation rather than cartesian CV,
    so curving fellows stop drifting off-line in the prediction.
  - SD-27b: clearance is OBB-aware. Pre-SD-27 used centroid distance, which
    treated 4.88m × 1.93m IAC Dallaras as points; a 2.5 m centroid filter
    failed to catch end-to-end approaches where the cars were already
    overlapping. This module now computes true gap between the predicted
    OBBs at each tick using eval_geometry.obb_separation_distance_m.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from typing import List, Literal, Optional, Sequence, Tuple

from scenic.domains.racing.assessment.pass_geometry import _xy_at_arclength
from scenic.domains.racing.eval_geometry import (
    IAC_DALLARA_LENGTH_M,
    IAC_DALLARA_WIDTH_M,
    obb_separation_distance_m,
)


def _blend_alpha(t_off: float, tau_s: float) -> float:
    """SD-26: first-order saturating ramp from 0 to 1 with time constant tau_s.

    Models the MPC's actual lateral lane-change dynamics — ego does NOT
    teleport onto the side polyline at t=0+; it converges over multiple
    seconds. Empirical fit from sample #8 of the verifai_20260428_165255
    campaign: CTE decay 4.37 m → 4.13 m → 2.75 m over t=0.2 → 1.0 → 2.5 s
    suggests tau ≈ 3–5 s.

    At t = tau_s, alpha ≈ 0.63. At t = 3*tau_s, alpha ≈ 0.95.

    tau_s ≤ 0 collapses to alpha=1 (legacy instantaneous behaviour) so
    callers can disable blending by passing 0 (used by the unit bank to
    verify backward-compat).
    """
    if tau_s <= 1e-6:
        return 1.0
    return float(min(1.0, 1.0 - math.exp(-float(t_off) / float(tau_s))))


Strategy = Literal["stay_optimal", "follow_fellow", "pass_left", "pass_right"]
ALL_STRATEGIES: Tuple[Strategy, ...] = (
    "stay_optimal",
    "follow_fellow",
    "pass_left",
    "pass_right",
)


@dataclass
class StrategyOutcome:
    """Per-strategy simulation result, consumed by strategy_selector."""

    strategy: Strategy
    reachable_progress_at_horizon_m: float
    reachable_speed_at_horizon_mps: float
    min_clearance_m: float
    closest_t_s: float
    completed: bool  # for pass_*: True iff ego_s passed opp_s + buffer by horizon
    samples: List[Tuple[float, float, float, float, str, float, float, float]] = field(
        default_factory=list
    )
    reason: str = ""  # diagnostic ("ok", "no_polyline", "shapely_unavailable", ...)


def _ramp_speed(current_v: float, target_v: float, accel: float, dt: float) -> float:
    """Piecewise-constant accel toward target. Symmetric for accel/decel."""
    if target_v >= current_v:
        return min(target_v, current_v + accel * dt)
    return max(target_v, current_v - accel * dt)


def _polyline_for_pass_phase(
    side: str,
    phase: str,
    polylines: dict,
) -> Optional[Sequence[Sequence[float]]]:
    """Return which polyline ego walks during a pass_* phase.

    side  : "left" or "right"
    phase : "lane_change" | "alongside" | "merge_back"
    """
    if phase in ("lane_change", "alongside"):
        return polylines.get(side)
    return polylines.get("optimal")


def _signed_cross_track_at_s(
    xy: Tuple[float, float],
    polyline,
    s_m: float,
    lap_length_m: float,
) -> Optional[float]:
    """Signed perpendicular distance from ``xy`` to ``polyline`` at arclength ``s_m``.

    Positive = LEFT of the polyline's forward tangent (matches the placement
    convention ``t > 0`` = left of TTL centerline). Negative = RIGHT.
    Returns ``None`` if the polyline tangent at ``s_m`` cannot be computed
    (degenerate segment or shapely unavailable upstream of ``_xy_at_arclength``).

    Used by SD-32C's pass-side sanity guard to express fellow's lateral in the
    TRACK frame instead of the ego-heading frame (``OpponentSituation.lateral_m``
    is heading-frame and flips sign when ego yaws relative to the track).
    """
    xy_on = _xy_at_arclength(polyline, float(s_m), float(lap_length_m))
    if xy_on is None:
        return None
    xy_next = _xy_at_arclength(polyline, float(s_m) + 1.0, float(lap_length_m))
    if xy_next is None:
        return None
    tx = float(xy_next[0]) - float(xy_on[0])
    ty = float(xy_next[1]) - float(xy_on[1])
    tlen = math.hypot(tx, ty)
    if tlen < 1e-6:
        return None
    tx /= tlen
    ty /= tlen
    dx = float(xy[0]) - float(xy_on[0])
    dy = float(xy[1]) - float(xy_on[1])
    # left-normal = tangent rotated 90° CCW = (-ty, tx)
    return dx * (-ty) + dy * tx


def simulate_strategy(
    strategy: Strategy,
    *,
    ego_s_m: float,
    ego_speed_mps: float,
    opp_s_m: float,
    opp_speed_mps: float,
    fellow_traj: Sequence[Tuple[float, float, float, Optional[float]]],
    polylines: dict,
    lap_length_m: float,
    horizon_s: float,
    sample_dt_s: float,
    target_speed_mps: float,
    accel_mps2: float,
    setup_speed_margin_mps: float,
    commit_speed_margin_mps: float,
    post_pass_buffer_m: float,
    lane_change_s: float,
    lane_change_tau_s: float = 2.5,
    ego_length_m: float = IAC_DALLARA_LENGTH_M,
    ego_width_m: float = IAC_DALLARA_WIDTH_M,
    fellow_length_m: float = IAC_DALLARA_LENGTH_M,
    fellow_width_m: float = IAC_DALLARA_WIDTH_M,
) -> StrategyOutcome:
    """Walk ego forward over [0, horizon_s] under the chosen strategy.

    Inputs:
      strategy            : which candidate to simulate
      ego_s_m             : ego's arc-length on the OPTIMAL polyline at t=0
      ego_speed_mps       : ego's current speed (m/s)
      opp_s_m             : opponent's arc-length on the optimal polyline at t=0
      opp_speed_mps       : opponent's speed (m/s) at t=0 — used only for
                            speed-target derivation; opponent's actual xy at
                            future samples comes from fellow_traj
      fellow_traj         : list of (t, x, y, s_or_None) from FellowPredictor.trajectory()
      polylines           : dict with keys "optimal", "left", "right" → waypoint sequences
      lap_length_m        : closed-loop length of the optimal polyline
      horizon_s           : planning horizon (default 10s in SD-11d wiring)
      sample_dt_s         : sample interval (default 0.5s in SD-11d wiring)
      target_speed_mps    : ego's target speed for stay_optimal / max-of follow_fellow
      accel_mps2          : longitudinal accel/decel ramp rate
      setup_speed_margin_mps : pass_* phase A target = opp + this
      commit_speed_margin_mps: pass_* phase B target = opp + this
      post_pass_buffer_m  : pass_* "completed" condition: ego_s >= opp_s + this
      lane_change_s       : pass_* phase A duration
      lane_change_tau_s   : SD-26. Time constant for the lateral-shift blend
                            between the optimal polyline and the side polyline
                            during pass_* simulation. Models the MPC's actual
                            lateral dynamics — ego doesn't teleport to the side
                            polyline at t=0+; it converges over ~tau seconds.
                            Default 2.5 s. Set to 0.0 to reproduce the legacy
                            instantaneous-switch behaviour (used by the unit
                            bank for backward-compat checks).
      ego_length_m, ego_width_m, fellow_length_m, fellow_width_m
                          : SD-27b. Vehicle dimensions used for OBB-aware
                            clearance. Defaults to IAC Dallara (4.88 m ×
                            1.93 m). The reported min_clearance_m is the
                            minimum edge-to-edge gap between the two oriented
                            bounding boxes over the horizon, NOT centroid
                            distance — pre-SD-27 the simulator treated cars
                            as points, so the 2.5 m filter missed end-to-end
                            approaches that were already overlapping.

    Returns StrategyOutcome with reachable progress, min clearance, samples.
    Failure modes (return outcome with reason!="ok"):
      - missing optimal polyline    → reason="no_optimal_polyline"
      - missing side polyline (pass_*) → reason="no_side_polyline"
      - shapely unavailable          → reason="shapely_unavailable"
    """
    n_steps = max(1, int(float(horizon_s) / float(sample_dt_s)))

    optimal_wp = polylines.get("optimal")
    if not optimal_wp or float(lap_length_m) <= 0.0:
        return StrategyOutcome(
            strategy=strategy,
            reachable_progress_at_horizon_m=float(ego_s_m),
            reachable_speed_at_horizon_mps=float(ego_speed_mps),
            min_clearance_m=float("inf"),
            closest_t_s=0.0,
            completed=False,
            reason="no_optimal_polyline",
        )

    if strategy in ("pass_left", "pass_right"):
        side = "left" if strategy == "pass_left" else "right"
        side_wp = polylines.get(side)
        if not side_wp:
            return StrategyOutcome(
                strategy=strategy,
                reachable_progress_at_horizon_m=float(ego_s_m),
                reachable_speed_at_horizon_mps=float(ego_speed_mps),
                min_clearance_m=float("inf"),
                closest_t_s=0.0,
                completed=False,
                reason="no_side_polyline",
            )

        # SD-32B (refined): closing-rate gap-feasibility guard. The lateral
        # shift takes ~tau_s to complete; during that window, ego CLOSES on
        # fellow by (v_ego - v_opp)·tau_s longitudinally. If the available
        # gap is shorter than this closing distance (with a 20% safety
        # factor), ego catches fellow before the shift completes and would
        # clip fellow mid-transition. Clamp clearance to 0 so the hard
        # filter rejects the pass.
        #
        # The previous absolute-speed formula (v_ego·tau·1.2) was too
        # aggressive: it blocked legitimate passes against slow fellows
        # whenever v_ego itself was high (S2 50-sample run: samples 4, 5,
        # 14, 18, 24, ... regressed from OK to COLLISION because the gap
        # guard rejected pass_right against a slow fellow on the opposite
        # side, and the selector then picked stay_optimal over follow_fellow
        # on progress). Closing rate is the correct kinematic signal —
        # fellows that are equal- or faster-than-ego pose no catch-up risk
        # and should not block a pass attempt.
        longitudinal_gap = (float(opp_s_m) - float(ego_s_m)) % float(lap_length_m)
        closing_rate_mps = max(0.0, float(ego_speed_mps) - float(opp_speed_mps))
        min_pass_gap = closing_rate_mps * float(lane_change_tau_s) * 1.2
        if longitudinal_gap > 0.0 and min_pass_gap > 0.0 and longitudinal_gap < min_pass_gap:
            return StrategyOutcome(
                strategy=strategy,
                reachable_progress_at_horizon_m=float(ego_s_m),
                reachable_speed_at_horizon_mps=float(ego_speed_mps),
                min_clearance_m=0.0,
                closest_t_s=0.0,
                completed=False,
                reason="gap_too_short_for_lane_change",
            )

        # SD-32C (refined): pass-side sanity guard in TRACK frame. Refuse to
        # pass on the side the opponent already occupies (signed lateral from
        # optimal centerline > 1.0 m on that side). Compute the lateral
        # internally by projecting fellow's current xy onto the optimal
        # polyline — the original SD-32C accepted sit.lateral_m which is in
        # the EGO HEADING frame, and that frame can flip sign under yaw
        # relative to track tangent (S2 sample 10: fellow on RIGHT TTL but
        # ego frame had fellow on left → guard didn't fire → COMMIT_PASS_RIGHT
        # straight into the fellow).
        FELLOW_LATERAL_THRESHOLD_M = 1.0
        fellow_lat_track: Optional[float] = None
        if fellow_traj and len(fellow_traj) > 0:
            fellow_xy_now = (float(fellow_traj[0][1]), float(fellow_traj[0][2]))
            fellow_lat_track = _signed_cross_track_at_s(
                fellow_xy_now,
                optimal_wp,
                float(opp_s_m),
                float(lap_length_m),
            )
        if fellow_lat_track is not None:
            if strategy == "pass_left" and fellow_lat_track > FELLOW_LATERAL_THRESHOLD_M:
                return StrategyOutcome(
                    strategy=strategy,
                    reachable_progress_at_horizon_m=float(ego_s_m),
                    reachable_speed_at_horizon_mps=float(ego_speed_mps),
                    min_clearance_m=0.0,
                    closest_t_s=0.0,
                    completed=False,
                    reason="fellow_on_left_side",
                )
            if strategy == "pass_right" and fellow_lat_track < -FELLOW_LATERAL_THRESHOLD_M:
                return StrategyOutcome(
                    strategy=strategy,
                    reachable_progress_at_horizon_m=float(ego_s_m),
                    reachable_speed_at_horizon_mps=float(ego_speed_mps),
                    min_clearance_m=0.0,
                    closest_t_s=0.0,
                    completed=False,
                    reason="fellow_on_right_side",
                )
    else:
        side = ""

    # Compute per-strategy target speed schedule.
    if strategy == "stay_optimal":
        def speed_target(t_off: float, opp_s_at_t: float) -> float:
            return float(target_speed_mps)
        def phase_for_t(t_off: float, ego_s_at_t: float, opp_s_at_t: float) -> str:
            return "stay"
    elif strategy == "follow_fellow":
        def speed_target(t_off: float, opp_s_at_t: float) -> float:
            return float(min(target_speed_mps, opp_speed_mps + 0.3))
        def phase_for_t(t_off: float, ego_s_at_t: float, opp_s_at_t: float) -> str:
            return "follow"
    else:
        # pass_left / pass_right 3-phase profile.
        def speed_target(t_off: float, opp_s_at_t: float) -> float:
            phase = phase_for_t(t_off, _ego_s_carry[0], opp_s_at_t)
            if phase == "lane_change":
                return float(min(target_speed_mps, opp_speed_mps + setup_speed_margin_mps))
            if phase == "alongside":
                return float(min(target_speed_mps, opp_speed_mps + commit_speed_margin_mps))
            return float(target_speed_mps)
        def phase_for_t(t_off: float, ego_s_at_t: float, opp_s_at_t: float) -> str:
            if t_off < float(lane_change_s):
                return "lane_change"
            if ego_s_at_t < opp_s_at_t + float(post_pass_buffer_m):
                return "alongside"
            return "merge_back"

    _ego_s_carry = [float(ego_s_m)]
    _ego_v_carry = [float(ego_speed_mps)]

    # Walk the simulation forward.
    min_clear = float("inf")
    closest_t = 0.0
    completed = False
    samples: List[Tuple[float, float, float, float, str, float, float, float]] = []
    fellow_n = len(fellow_traj)
    # SD-27b: cache previous tick xy for each agent so we can take a
    # finite-difference heading at i >= 1. Sample i=0 falls back to a
    # circumradius approximation (safe over-estimate of vehicle extent).
    prev_ego_xy: Optional[Tuple[float, float]] = None
    prev_fellow_xy: Optional[Tuple[float, float]] = None
    half_circ_ego = 0.5 * math.hypot(float(ego_length_m), float(ego_width_m))
    half_circ_fellow = 0.5 * math.hypot(float(fellow_length_m), float(fellow_width_m))
    # SD-27b: track merge_back start so we can reverse-blend ego back to the
    # optimal polyline symmetrically (instead of the SD-26 snap-to-optimal,
    # which made OBB clearance collapse during the post-pass merge tick).
    merge_back_start_t: Optional[float] = None
    last_alpha_pre_merge: float = 0.0

    def _fellow_xy_at(t_off: float) -> Optional[Tuple[float, float]]:
        if fellow_n == 0:
            return None
        # Index by closest sample (fellow_traj is sorted by t_off in SD-11a).
        # Linear search bounded by fellow_n (~21 for 10s @ 0.5dt).
        best_i = 0
        best_dt = abs(fellow_traj[0][0] - t_off)
        for i in range(1, fellow_n):
            d = abs(fellow_traj[i][0] - t_off)
            if d < best_dt:
                best_dt = d
                best_i = i
        return (float(fellow_traj[best_i][1]), float(fellow_traj[best_i][2]))

    for i in range(n_steps + 1):
        t_off = i * float(sample_dt_s)
        ego_s = _ego_s_carry[0]
        ego_v = _ego_v_carry[0]
        opp_s_at_t = float(opp_s_m) + float(opp_speed_mps) * t_off
        phase = phase_for_t(t_off, ego_s, opp_s_at_t)

        # SD-26: blend ego_xy between optimal and side polylines for pass_*
        # strategies during the lateral-shift phase. Pre-SD-26 placed ego at
        # FULL lateral offset on the side polyline from t=0+, which produced
        # optimistic clearance predictions (sample #8 of the pre-SD-25
        # 30-sample run: predicted 7.73 m, actual 0.94 m collision). The
        # blended position uses _blend_alpha(t_off, lane_change_tau_s) to
        # model the MPC's actual lateral dynamics — ego converges to the
        # side polyline over ~tau_s seconds.
        if strategy in ("pass_left", "pass_right"):
            # SD-27b: symmetric reverse-blend during merge_back. Pre-SD-27 the
            # simulator snapped ego to the optimal polyline at the first
            # merge_back tick; with OBB clearance that produces an artificial
            # near-zero gap when ego is one car-length ahead on the same line
            # as fellow, breaking otherwise-safe passes. Reverse-blend keeps
            # ego on the side polyline a moment longer and exponentially
            # decays alpha back to 0 over the same tau_s.
            if phase == "merge_back":
                if merge_back_start_t is None:
                    merge_back_start_t = t_off
                t_since_merge = max(0.0, t_off - merge_back_start_t)
                tau = float(lane_change_tau_s)
                if tau <= 1e-6:
                    alpha = 0.0
                else:
                    alpha = float(last_alpha_pre_merge) * math.exp(-t_since_merge / tau)
            else:
                alpha = _blend_alpha(t_off, float(lane_change_tau_s))
                last_alpha_pre_merge = alpha

            ego_xy_opt = _xy_at_arclength(
                optimal_wp, ego_s, float(lap_length_m)
            )
            ego_xy_side = _xy_at_arclength(
                side_wp, ego_s, float(lap_length_m)
            )
            if ego_xy_opt is None or ego_xy_side is None:
                return StrategyOutcome(
                    strategy=strategy,
                    reachable_progress_at_horizon_m=ego_s,
                    reachable_speed_at_horizon_mps=ego_v,
                    min_clearance_m=float("inf"),
                    closest_t_s=0.0,
                    completed=False,
                    reason="shapely_unavailable",
                )
            ego_xy = (
                alpha * ego_xy_side[0] + (1.0 - alpha) * ego_xy_opt[0],
                alpha * ego_xy_side[1] + (1.0 - alpha) * ego_xy_opt[1],
            )
        else:
            # stay_optimal / follow_fellow: always on optimal.
            ego_xy = _xy_at_arclength(optimal_wp, ego_s, float(lap_length_m))
        if ego_xy is None:
            return StrategyOutcome(
                strategy=strategy,
                reachable_progress_at_horizon_m=ego_s,
                reachable_speed_at_horizon_mps=ego_v,
                min_clearance_m=float("inf"),
                closest_t_s=0.0,
                completed=False,
                reason="shapely_unavailable",
            )

        fellow_xy = _fellow_xy_at(t_off)
        clearance = float("inf")
        if fellow_xy is not None:
            dx = ego_xy[0] - fellow_xy[0]
            dy = ego_xy[1] - fellow_xy[1]
            centroid_dist = math.hypot(dx, dy)
            # SD-27b: report true OBB edge-to-edge gap. Headings come from a
            # finite difference vs the previous tick's xy; on the first tick
            # we don't have one yet, so subtract the circumradius of each
            # vehicle as a conservative (heading-agnostic) fallback.
            if prev_ego_xy is not None and prev_fellow_xy is not None:
                ehx = ego_xy[0] - prev_ego_xy[0]
                ehy = ego_xy[1] - prev_ego_xy[1]
                fhx = fellow_xy[0] - prev_fellow_xy[0]
                fhy = fellow_xy[1] - prev_fellow_xy[1]
                ego_heading = (
                    math.atan2(ehy, ehx) if (ehx * ehx + ehy * ehy) > 1e-9 else 0.0
                )
                fellow_heading = (
                    math.atan2(fhy, fhx) if (fhx * fhx + fhy * fhy) > 1e-9 else 0.0
                )
                clearance = obb_separation_distance_m(
                    float(ego_xy[0]), float(ego_xy[1]), ego_heading,
                    float(ego_length_m), float(ego_width_m),
                    float(fellow_xy[0]), float(fellow_xy[1]), fellow_heading,
                    float(fellow_length_m), float(fellow_width_m),
                )
            else:
                clearance = max(0.0, centroid_dist - half_circ_ego - half_circ_fellow)
            if clearance < min_clear:
                min_clear = clearance
                closest_t = t_off

        samples.append((
            t_off,
            float(ego_xy[0]),
            float(ego_xy[1]),
            float(ego_v),
            phase,
            float(fellow_xy[0]) if fellow_xy is not None else float("nan"),
            float(fellow_xy[1]) if fellow_xy is not None else float("nan"),
            float(clearance),
        ))
        prev_ego_xy = (float(ego_xy[0]), float(ego_xy[1]))
        if fellow_xy is not None:
            prev_fellow_xy = (float(fellow_xy[0]), float(fellow_xy[1]))

        # Advance ego state to next sample.
        if i == n_steps:
            break
        v_target = speed_target(t_off, opp_s_at_t)
        v_next = _ramp_speed(ego_v, v_target, float(accel_mps2), float(sample_dt_s))
        # Trapezoidal s update with current and next v.
        ds = 0.5 * (ego_v + v_next) * float(sample_dt_s)
        _ego_s_carry[0] = ego_s + ds
        _ego_v_carry[0] = v_next

        # Check pass_* completion using NEXT-step opp position to match the
        # next-step ego position we just advanced to.
        if strategy in ("pass_left", "pass_right") and not completed:
            opp_s_next = float(opp_s_m) + float(opp_speed_mps) * (t_off + float(sample_dt_s))
            if _ego_s_carry[0] >= opp_s_next + float(post_pass_buffer_m):
                completed = True

    if strategy not in ("pass_left", "pass_right"):
        completed = True  # follow / stay always "completes" by definition

    if min_clear == float("inf"):
        # No fellow xy was available at any sample (fellow_traj empty).
        # Treat as "no obstacle" — strategy can't be ranked on clearance.
        min_clear = float("inf")
        closest_t = 0.0

    return StrategyOutcome(
        strategy=strategy,
        reachable_progress_at_horizon_m=float(_ego_s_carry[0]),
        reachable_speed_at_horizon_mps=float(_ego_v_carry[0]),
        min_clearance_m=float(min_clear),
        closest_t_s=float(closest_t),
        completed=bool(completed),
        samples=samples,
        reason="ok",
    )


__all__ = [
    "Strategy",
    "ALL_STRATEGIES",
    "StrategyOutcome",
    "simulate_strategy",
]
