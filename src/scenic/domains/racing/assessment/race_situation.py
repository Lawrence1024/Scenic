"""Phase 8: tactical facts + speed-sensitive safe-gap assessment."""

from __future__ import annotations

from dataclasses import dataclass
import math
from typing import Optional, Tuple

from scenic.domains.racing.situation_assessment import OpponentSituation


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


def _compute_emergency_risk(
    *,
    sit: OpponentSituation,
    relation: str,
    closing_flag: bool,
    safe_gap_m: float,
    actual_gap_m: Optional[float],
    overlap_flag: bool,
) -> float:
    """Design-level risk decomposition (not single additive tweak)."""

    base = _clamp01(float(sit.collision_risk_01))
    gap_pressure = 0.0
    ttc_pressure = 0.0
    if relation == "ahead" and actual_gap_m is not None:
        gap_pressure = _clamp01((float(safe_gap_m) - float(actual_gap_m)) / max(1e-6, float(safe_gap_m)))
        if closing_flag and float(sit.closing_speed_mps) > 1e-6:
            ttc_s = max(0.0, float(actual_gap_m)) / float(sit.closing_speed_mps)
            ttc_pressure = _clamp01((4.0 - ttc_s) / 4.0)
    overlap_pressure = 0.90 if overlap_flag else 0.0
    # Use max to reflect "any severe mode should dominate".
    return max(base, gap_pressure, ttc_pressure, overlap_pressure)


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
    closing_flag = bool(relation == "ahead" and float(sit.closing_speed_mps) > 0.30)
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
    optimal_open, left_open, right_open = _compute_corridor_open_flags(
        relation=relation,
        pred_long_m=pred_long,
        pred_lat_m=pred_lat,
        overlap_flag=overlap_flag,
    )

    risk_now = _compute_emergency_risk(
        sit=sit,
        relation=relation,
        closing_flag=closing_flag,
        safe_gap_m=safe_gap_m,
        actual_gap_m=actual_gap_m,
        overlap_flag=overlap_flag,
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

