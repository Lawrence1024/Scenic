"""Phase 8: tactical facts + speed-sensitive safe-gap assessment."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation

# Large lateral: partial_overlap / side_by_side should not imply 0.9 emergency pressure.
PHASE8_OVERLAP_LAT_RELIEF_M = 2.0
# Below this speed, opponent is treated as a static obstacle for Phase-8 risk/closing.
PHASE8_STATIONARY_OPP_SPEED_MPS = 1.5

# Longitudinal vs lateral: fly-by geometry — damp rear-end style gap/TTC when lateral offset
# exists and along-track slot is adequate (speed should not dominate that decision).
PHASE8_FLYBY_LAT_MIN_M = 1.15
PHASE8_FLYBY_MIN_LONG_SLOT_M = 7.5


@dataclass(frozen=True)
class Phase8Assessment:
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


@dataclass(frozen=True)
class Phase8AssessmentState:
    """Small state carrier to stabilize relation/risk labels across cycles."""

    previous_relation: str = "none"
    emergency_latch_steps: int = 0
    last_emergency_risk_01: float = 0.0


def _clamp01(x: float) -> float:
    return max(0.0, min(1.0, float(x)))


def compute_dynamic_safe_gap_m(
    ego_speed_mps: float,
    *,
    time_headway_s: float = 1.10,
    base_gap_m: float = 6.0,
    max_gap_m: float = 70.0,
) -> float:
    """Simple Phase 8 safe-gap baseline: base margin + speed*time-headway."""

    v = max(0.0, float(ego_speed_mps))
    gap = float(base_gap_m) + v * float(time_headway_s)
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
) -> Tuple[bool, bool, bool]:
    """Phase 8 corridor occupancy semantics from projected relative pose."""

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
        # Overlap state means neither side should be considered confidently open.
        if overlap_flag and abs(float(pred_lat_m)) <= 1.4:
            left_open = False
            right_open = False
    return optimal_open, left_open, right_open


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
    if lat_abs < float(PHASE8_FLYBY_LAT_MIN_M):
        return 1.0
    along = max(0.0, float(actual_gap_m))
    slot_req = max(
        float(PHASE8_FLYBY_MIN_LONG_SLOT_M),
        0.55 * float(safe_gap_m),
        5.5 + 0.06 * max(0.0, float(ego_speed_mps)),
    )
    if along >= slot_req:
        return 0.32
    if lat_abs >= float(PHASE8_OVERLAP_LAT_RELIEF_M):
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
    lateral_clear = lat_abs >= float(PHASE8_OVERLAP_LAT_RELIEF_M)
    stationary_opp = float(opponent_speed_mps) < float(PHASE8_STATIONARY_OPP_SPEED_MPS)
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
        if float(actual_gap_m) < float(safe_gap_m):
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


def assess_phase8_situation_stateful(
    *,
    sit: Optional[OpponentSituation],
    ego_speed_mps: float,
    ego_xy: Tuple[float, float],
    ego_heading_rad: float,
    predicted_opp_xy: Optional[Tuple[float, float]] = None,
    state: Optional[Phase8AssessmentState] = None,
) -> Tuple[Phase8Assessment, Phase8AssessmentState]:
    """Stateful assessment for stable Phase-8 relation/corridor/risk semantics."""

    st = state if state is not None else Phase8AssessmentState()
    safe_gap_m = compute_dynamic_safe_gap_m(float(ego_speed_mps))
    if sit is None:
        a_none = Phase8Assessment(
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
        return a_none, Phase8AssessmentState(
            previous_relation="none",
            emergency_latch_steps=max(0, int(st.emergency_latch_steps) - 1),
            last_emergency_risk_01=0.0,
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
    actual_gap_m = max(0.0, float(sit.delta_s_m)) if relation == "ahead" else None
    gap_ok = True if actual_gap_m is None else bool(actual_gap_m >= safe_gap_m)

    pred_long, pred_lat, source = _compute_predicted_ego_frame(
        ego_xy,
        ego_heading_rad,
        predicted_opp_xy,
        fallback_long_m=float(sit.longitudinal_m),
        fallback_lat_m=float(sit.lateral_m),
    )
    closing_flag = bool(relation == "ahead" and float(sit.closing_speed_mps) > 0.30)
    if (
        relation == "ahead"
        and float(getattr(sit, "opponent_speed_mps", 0.0)) < float(PHASE8_STATIONARY_OPP_SPEED_MPS)
        and abs(float(pred_lat)) >= float(PHASE8_OVERLAP_LAT_RELIEF_M)
    ):
        closing_flag = False
        # Stationary and clearly off-axis: do not propagate overlap to guard/emergency logic.
        overlap_flag = False
    # Use current measured lateral (sit.lateral_m) for corridor side occupancy.
    # The Phase-7 predicted position introduces heading-noise at low speed and
    # oscillates around the 0.8 m threshold, causing left_open/right_open to
    # flip every cycle. Current lateral is stable because it uses actual measured
    # position; longitudinal (pred_long) still comes from prediction for gap calc.
    optimal_open, left_open, right_open = _compute_corridor_open_flags(
        relation=relation,
        pred_long_m=pred_long,
        pred_lat_m=float(sit.lateral_m),
        overlap_flag=overlap_flag,
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

    assessment = Phase8Assessment(
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
    )
    next_state = Phase8AssessmentState(
        previous_relation=relation,
        emergency_latch_steps=latch_steps,
        last_emergency_risk_01=risk_out,
    )
    return assessment, next_state


def format_phase8_assessment_log_line(sim_time_s: float, a: Phase8Assessment) -> str:
    ag = f"{a.actual_gap_m:.3f}" if a.actual_gap_m is not None else "na"
    return (
        f"[Phase8Assessment] t={sim_time_s:.2f}s fellow_relation={a.fellow_relation} "
        f"closing_flag={1 if a.closing_flag else 0} actual_gap={ag} safe_gap={a.safe_gap_m:.3f} "
        f"gap_ok={1 if a.gap_ok else 0} optimal_open={1 if a.optimal_open else 0} "
        f"left_open={1 if a.left_open else 0} right_open={1 if a.right_open else 0} "
        f"overlap_flag={1 if a.overlap_flag else 0} emergency_risk_01={a.emergency_risk_01:.3f} "
        f"source={a.source}"
    )

