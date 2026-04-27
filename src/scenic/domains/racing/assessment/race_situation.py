"""Tactical facts + speed-sensitive safe-gap assessment."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation

# Large lateral: partial_overlap / side_by_side should not imply 0.9 emergency pressure.
OVERLAP_LAT_RELIEF_M = 2.0
# Below this speed, opponent is treated as a static obstacle for risk/closing.
STATIONARY_OPP_SPEED_MPS = 1.5

# Longitudinal vs lateral: fly-by geometry — damp rear-end style gap/TTC when lateral offset
# exists and along-track slot is adequate (speed should not dominate that decision).
FLYBY_LAT_MIN_M = 1.15
FLYBY_MIN_LONG_SLOT_M = 7.5

# SD-2a: corridor-hysteresis hold window. ~5 ticks @ 20 Hz ≈ 0.25 s. The chicken-and-egg
# pattern observed at F2_tactical t=0.70→0.75 (right_open 1→0 within one tick of ego
# starting SETUP_PASS_RIGHT) needs a brief grace period so the planner can commit before
# the lateral re-projection from the TTL switch flips the corridor flag back closed.
CORRIDOR_HOLD_CYCLES = 5


@dataclass(frozen=True)
class RaceSituationAssessment:
    fellow_relation: str
    closing_flag: bool
    actual_gap_m: Optional[float]
    safe_gap_m: float
    gap_ok: bool
    optimal_open: bool
    left_open: bool
    right_open: bool
    overlap_flag: bool
    emergency_risk_01: float
    source: str
    # SD-2a: bitmask of which corridor flags were forced open by hysteresis this tick
    # (raw said closed, but hold_remaining > 0). Used purely for telemetry / grep.
    # 0 = none held; bit 0 = optimal, bit 1 = left, bit 2 = right.
    corridor_held_mask: int = 0


@dataclass(frozen=True)
class RaceSituationState:
    """Small state carrier to stabilize relation/risk labels across cycles."""

    previous_relation: str = "none"
    emergency_latch_steps: int = 0
    last_emergency_risk_01: float = 0.0
    # SD-2a: corridor-flag hysteresis. Once a corridor opens, hold it open for
    # CORRIDOR_HOLD_CYCLES even if the raw computation would close it. Prevents
    # the chicken-and-egg pattern where ego switches TTL toward an open side and
    # the act of switching causes the lateral relationship to flip the corridor
    # closed within one tick (observed F2_tactical t=0.70→0.75: right_open
    # 1→0 immediately after ego started SETUP_PASS_RIGHT). Closes are NOT
    # held — overlap_flag and immediate-danger paths still close instantly via
    # the bypass below.
    left_open_hold_remaining: int = 0
    right_open_hold_remaining: int = 0
    optimal_open_hold_remaining: int = 0



def _clamp01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


def compute_dynamic_safe_gap_m(
    ego_speed_mps: float,
    *,
    time_headway_s: float = 0.80,
    base_gap_m: float = 6.0,
    max_gap_m: float = 70.0,
    lateral_offset_m: float = 0.0,
    parallel_headway_s: float = 0.35,
    parallel_lateral_threshold_m: float = 1.5,
) -> float:
    """Safe-gap baseline: base margin + speed*time-headway.

    When the opponent is laterally separated beyond *parallel_lateral_threshold_m*
    (i.e. on a parallel TTL), uses the shorter *parallel_headway_s* instead of
    the default *time_headway_s* — the cars are side-by-side, not nose-to-tail.
    """

    v = max(0.0, float(ego_speed_mps))
    headway = parallel_headway_s if abs(float(lateral_offset_m)) > float(parallel_lateral_threshold_m) else time_headway_s
    gap = float(base_gap_m) + v * float(headway)
    return min(float(max_gap_m), max(float(base_gap_m), gap))


def _relation_from_delta_s(
    delta_s_m: float,
    *,
    safe_gap_m: float,
    previous_relation: str,
) -> str:
    """Infer ahead/behind using Δs with hysteresis around zero crossing."""

    ds = float(delta_s_m)
    deadband_m = max(2.0, 0.20 * float(safe_gap_m))
    if ds >= deadband_m:
        return "ahead"
    if ds <= -deadband_m:
        return "behind"
    if previous_relation in ("ahead", "behind"):
        return previous_relation
    return "ahead" if ds >= 0.0 else "behind"


def _compute_predicted_ego_frame(
    ego_xy: Tuple[float, float],
    ego_heading_rad: float,
    predicted_opp_xy: Optional[Tuple[float, float]],
    *,
    fallback_long_m: float,
    fallback_lat_m: float,
) -> Tuple[float, float, str]:
    if predicted_opp_xy is None:
        return float(fallback_long_m), float(fallback_lat_m), "current"
    px, py = float(ego_xy[0]), float(ego_xy[1])
    ox, oy = float(predicted_opp_xy[0]), float(predicted_opp_xy[1])
    dx, dy = ox - px, oy - py
    ch, sh = math.cos(float(ego_heading_rad)), math.sin(float(ego_heading_rad))
    pred_long = dx * ch + dy * sh
    pred_lat = dx * (-sh) + dy * ch
    return pred_long, pred_lat, "predicted"


def _compute_corridor_open_flags(
    *,
    relation: str,
    pred_long_m: float,
    pred_lat_m: float,
    overlap_flag: bool,
    closing_speed_mps: float = 0.0,
    opponent_speed_mps: float = 0.0,
    ego_speed_mps: float = 0.0,
) -> Tuple[bool, bool, bool]:
    """Corridor occupancy semantics from projected relative pose.

    SD-2a-bias: centered slow opponents at safe range are converted from
    "both-blocked" to "asymmetric (right-biased) open" so the planner's
    XOR-based ``opening_window_available`` gate can fire and release
    ``protected_follow_active``. Strict both-blocked is still returned for
    close-quarters (long < 12 m) and hot-closing (closing > 8 m/s and
    opponent still ≥ 50% ego speed) geometry — those represent genuine
    rear-end danger that should not be re-labelled as a pass opportunity.
    """

    optimal_open = True
    left_open = True
    right_open = True

    # Corridor occupancy should matter primarily when the fellow is projected ahead/alongside.
    if relation == "ahead" and -2.0 <= float(pred_long_m) <= 45.0:
        # Opponent near centerline blocks optimal corridor.
        if abs(float(pred_lat_m)) <= 1.8:
            optimal_open = False
        # Positive lateral means fellow occupies left side; negative occupies right.
        if float(pred_lat_m) >= 0.8:
            left_open = False
        if float(pred_lat_m) <= -0.8:
            right_open = False
        # SD-2a-bias: near-center opponent. Used to be unconditional both-blocked,
        # but that locks the planner into protected_follow forever (XOR gate at
        # tactical_planner.py:381 needs exactly one side open). Bucket by danger.
        if abs(float(pred_lat_m)) < 1.5:
            close_quarters = float(pred_long_m) < 12.0
            hot_closing = (
                float(closing_speed_mps) > 8.0
                and float(opponent_speed_mps) > 0.5 * float(ego_speed_mps)
            )
            if close_quarters or hot_closing:
                # Genuine rear-end danger — strict both-blocked (original behavior).
                left_open = False
                right_open = False
            else:
                # Far + slow + cold-closing: open EXACTLY one side (asymmetric).
                # Right-biased by default (typical race-track passing convention).
                # Tiny lateral tilt picks the OPPOSITE side (pass on the side the
                # fellow is NOT drifting toward).
                if float(pred_lat_m) > 0.05:
                    left_open = False
                    right_open = True
                elif float(pred_lat_m) < -0.05:
                    left_open = True
                    right_open = False
                else:
                    left_open = False
                    right_open = True
        # Overlap state means neither side should be considered confidently open.
        if overlap_flag and abs(float(pred_lat_m)) <= 1.4:
            left_open = False
            right_open = False
    return optimal_open, left_open, right_open


def _apply_corridor_hysteresis(
    *, raw_open: bool, prior_hold: int, bypass: bool
) -> Tuple[bool, int]:
    """Hold a corridor "open" for CORRIDOR_HOLD_CYCLES after raw computation closes.

    Returns (effective_open, next_hold_remaining).
    - bypass=True (overlap_flag): immediate close, hold reset.
    - raw_open=True: open, hold rearmed to CORRIDOR_HOLD_CYCLES.
    - raw_open=False, prior_hold>0: open (held), hold decremented.
    - raw_open=False, prior_hold==0: closed.
    """
    if bypass:
        return False, 0
    if raw_open:
        return True, int(CORRIDOR_HOLD_CYCLES)
    if prior_hold > 0:
        return True, prior_hold - 1
    return False, 0


def _longitudinal_opening_dampen(
    pred_lat_m: float,
    actual_gap_m: float,
    safe_gap_m: float,
    ego_speed_mps: float,
) -> float:
    """Reduce gap/TTC pressure when lateral offset + along-track slot support a fly-by.

    Longitudinal distance supplies the "opening"; once it clears a modest slot, ego speed
    should not inflate rear-end style fear the way it does when opponents are co-linear.
    """
    lat_abs = abs(float(pred_lat_m))
    if lat_abs < float(FLYBY_LAT_MIN_M):
        return 1.0
    along = max(0.0, float(actual_gap_m))
    slot_req = max(
        float(FLYBY_MIN_LONG_SLOT_M),
        0.55 * float(safe_gap_m),
        5.5 + 0.06 * max(0.0, float(ego_speed_mps)),
    )
    if along >= slot_req:
        return 0.32
    if lat_abs >= float(OVERLAP_LAT_RELIEF_M):
        return 0.58
    if lat_abs >= 1.6:
        return 0.72
    return 0.88


def _compute_emergency_risk(
    *,
    sit: OpponentSituation,
    relation: str,
    closing_flag: bool,
    safe_gap_m: float,
    actual_gap_m: Optional[float],
    overlap_flag: bool,
    pred_lat_m: float,
    opponent_speed_mps: float,
    ego_speed_mps: float,
) -> float:
    """Design-level risk decomposition (not single additive tweak)."""

    base = _clamp01(float(sit.collision_risk_01))
    gap_pressure = 0.0
    ttc_pressure = 0.0
    lat_abs = abs(float(pred_lat_m))
    lateral_clear = lat_abs >= float(OVERLAP_LAT_RELIEF_M)
    stationary_opp = float(opponent_speed_mps) < float(STATIONARY_OPP_SPEED_MPS)
    if relation == "ahead" and actual_gap_m is not None:
        gap_pressure = _clamp01((float(safe_gap_m) - float(actual_gap_m)) / max(1e-6, float(safe_gap_m)))
        if closing_flag and float(sit.closing_speed_mps) > 1e-6:
            ttc_s = max(0.0, float(actual_gap_m)) / float(sit.closing_speed_mps)
            ttc_pressure = _clamp01((4.0 - ttc_s) / 4.0)
        if stationary_opp and lateral_clear:
            gap_pressure *= 0.35
            ttc_pressure *= 0.35
        flyby_d = _longitudinal_opening_dampen(
            pred_lat_m, float(actual_gap_m), float(safe_gap_m), float(ego_speed_mps)
        )
        # Fly-by damping assumes adequate longitudinal headway. Inside the safe-gap envelope,
        # keep full rear-end / closing pressure (e.g. sudden stop while still "gap_ok" false).
        # Exception: when laterally separated (parallel TTL), a rear-end collision is not
        # the primary risk — keep fly-by damping active so risk doesn't spike artificially.
        if float(actual_gap_m) < float(safe_gap_m) and lat_abs < 1.5:
            flyby_d = 1.0
        gap_pressure *= flyby_d
        ttc_pressure *= flyby_d
    # Shoulder / roadside: do not apply full overlap spike when the car is clearly off-axis.
    overlap_pressure = 0.90 if overlap_flag and (not lateral_clear) else 0.0
    # Use max to reflect "any severe mode should dominate".
    return max(
        base,
        gap_pressure,
        ttc_pressure,
        overlap_pressure,
    )


def assess_race_situation(
    *,
    sit: Optional[OpponentSituation],
    ego_speed_mps: float,
    ego_xy: Tuple[float, float],
    ego_heading_rad: float,
    predicted_opp_xy: Optional[Tuple[float, float]] = None,
    state: Optional[RaceSituationState] = None,
) -> Tuple[RaceSituationAssessment, RaceSituationState]:
    """Stateful assessment for stable relation/corridor/risk semantics."""

    st = state if state is not None else RaceSituationState()
    # Initial safe_gap without lateral info (used for no-opponent fallback).
    safe_gap_m = compute_dynamic_safe_gap_m(float(ego_speed_mps))
    if sit is None:
        a_none = RaceSituationAssessment(
            fellow_relation="none",
            closing_flag=False,
            actual_gap_m=None,
            safe_gap_m=safe_gap_m,
            gap_ok=True,
            optimal_open=True,
            left_open=True,
            right_open=True,
            overlap_flag=False,
            emergency_risk_01=0.0,
            source="none",
        )
        return a_none, RaceSituationState(
            previous_relation="none",
            emergency_latch_steps=max(0, int(st.emergency_latch_steps) - 1),
            last_emergency_risk_01=0.0,
            left_open_hold_remaining=0,
            right_open_hold_remaining=0,
            optimal_open_hold_remaining=0,
        )

    relation = (
        _relation_from_delta_s(
            float(sit.delta_s_m),
            safe_gap_m=safe_gap_m,
            previous_relation=str(st.previous_relation or "none"),
        )
        if str(sit.delta_s_source or "") == "polyline"
        else ("ahead" if bool(sit.ahead) else "behind")
    )
    overlap_flag = str(sit.overlap_state or "").lower() in {"side_by_side", "partial_overlap"}

    pred_long, pred_lat, source = _compute_predicted_ego_frame(
        ego_xy,
        ego_heading_rad,
        predicted_opp_xy,
        fallback_long_m=float(sit.longitudinal_m),
        fallback_lat_m=float(sit.lateral_m),
    )
    # Recompute safe_gap with lateral awareness — parallel-TTL opponents use shorter headway.
    safe_gap_m = compute_dynamic_safe_gap_m(float(ego_speed_mps), lateral_offset_m=float(pred_lat))
    actual_gap_m = max(0.0, float(sit.delta_s_m)) if relation == "ahead" else None
    gap_ok = True if actual_gap_m is None else bool(actual_gap_m >= safe_gap_m)
    closing_threshold_mps = 2.0 if abs(float(pred_lat)) > 1.5 else 0.30
    closing_flag = bool(relation == "ahead" and float(sit.closing_speed_mps) > closing_threshold_mps)
    if (
        relation == "ahead"
        and float(getattr(sit, "opponent_speed_mps", 0.0)) < float(STATIONARY_OPP_SPEED_MPS)
        and abs(float(pred_lat)) >= float(OVERLAP_LAT_RELIEF_M)
    ):
        closing_flag = False
        # Stationary and clearly off-axis: do not propagate overlap to guard/emergency logic.
        overlap_flag = False
    # Use current measured lateral (sit.lateral_m) for corridor side occupancy.
    # The predicted position introduces heading-noise at low speed and oscillates around the
    # 0.8 m threshold, causing left_open/right_open to flip every cycle. Current lateral is
    # stable; longitudinal (pred_long) still comes from prediction for gap calc.
    raw_optimal, raw_left, raw_right = _compute_corridor_open_flags(
        relation=relation,
        pred_long_m=pred_long,
        pred_lat_m=float(sit.lateral_m),
        overlap_flag=overlap_flag,
        closing_speed_mps=float(getattr(sit, "closing_speed_mps", 0.0)),
        opponent_speed_mps=float(getattr(sit, "opponent_speed_mps", 0.0)),
        ego_speed_mps=float(ego_speed_mps),
    )
    # SD-2a hysteresis. Raw "open" passes through and arms the hold counter; raw "closed"
    # is overridden to "open" while hold_remaining > 0 (and we are NOT in an overlap-flag
    # safety bypass). Overlap flag closes immediately so true near-contact geometry is
    # never held-open by a stale hysteresis tick.
    optimal_open, opt_hold = _apply_corridor_hysteresis(
        raw_open=raw_optimal,
        prior_hold=int(st.optimal_open_hold_remaining),
        bypass=overlap_flag,
    )
    left_open, left_hold = _apply_corridor_hysteresis(
        raw_open=raw_left,
        prior_hold=int(st.left_open_hold_remaining),
        bypass=overlap_flag,
    )
    right_open, right_hold = _apply_corridor_hysteresis(
        raw_open=raw_right,
        prior_hold=int(st.right_open_hold_remaining),
        bypass=overlap_flag,
    )
    held_mask = (
        (1 if (optimal_open and not raw_optimal) else 0)
        | (2 if (left_open and not raw_left) else 0)
        | (4 if (right_open and not raw_right) else 0)
    )

    risk_now = _compute_emergency_risk(
        sit=sit,
        relation=relation,
        closing_flag=closing_flag,
        safe_gap_m=safe_gap_m,
        actual_gap_m=actual_gap_m,
        overlap_flag=overlap_flag,
        pred_lat_m=pred_lat,
        opponent_speed_mps=float(getattr(sit, "opponent_speed_mps", 0.0)),
        ego_speed_mps=float(ego_speed_mps),
    )
    latch_steps = int(st.emergency_latch_steps)
    if risk_now >= 0.70:
        latch_steps = 3
    elif latch_steps > 0:
        latch_steps -= 1
    risk_out = max(risk_now, float(st.last_emergency_risk_01) * 0.80) if latch_steps > 0 else risk_now
    risk_out = _clamp01(risk_out)

    assessment = RaceSituationAssessment(
        fellow_relation=relation,
        closing_flag=closing_flag,
        actual_gap_m=actual_gap_m,
        safe_gap_m=float(safe_gap_m),
        gap_ok=gap_ok,
        optimal_open=bool(optimal_open),
        left_open=bool(left_open),
        right_open=bool(right_open),
        overlap_flag=overlap_flag,
        emergency_risk_01=risk_out,
        source=source,
        corridor_held_mask=int(held_mask),
    )
    next_state = RaceSituationState(
        previous_relation=relation,
        emergency_latch_steps=latch_steps,
        last_emergency_risk_01=risk_out,
        left_open_hold_remaining=left_hold,
        right_open_hold_remaining=right_hold,
        optimal_open_hold_remaining=opt_hold,
    )
    return assessment, next_state



def format_assessment_log_line(sim_time_s: float, a: RaceSituationAssessment) -> str:
    ag = f"{a.actual_gap_m:.3f}" if a.actual_gap_m is not None else "na"
    held_tag = ""
    if int(a.corridor_held_mask) != 0:
        m = int(a.corridor_held_mask)
        which = []
        if m & 1: which.append("opt")
        if m & 2: which.append("L")
        if m & 4: which.append("R")
        held_tag = f" [CorridorHysteresis] held={','.join(which)}"
    return (
        f"[Assessment] t={sim_time_s:.2f}s fellow_relation={a.fellow_relation} "
        f"closing_flag={1 if a.closing_flag else 0} actual_gap={ag} safe_gap={a.safe_gap_m:.3f} "
        f"gap_ok={1 if a.gap_ok else 0} optimal_open={1 if a.optimal_open else 0} "
        f"left_open={1 if a.left_open else 0} right_open={1 if a.right_open else 0} "
        f"overlap_flag={1 if a.overlap_flag else 0} emergency_risk_01={a.emergency_risk_01:.3f} "
        f"source={a.source}{held_tag}"
    )
