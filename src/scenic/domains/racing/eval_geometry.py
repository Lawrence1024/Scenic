"""Evaluation-only geometry for racing logs (not used by planners or control).

Oriented bounding boxes use vehicle **center** as origin; **length** is along +x in the
vehicle frame (forward), **width** along +y (left). Heading ``theta`` is Scenic yaw (rad),
same convention as ``dspaceActor.heading``.
"""

from __future__ import annotations

import math
from typing import List, Optional, Tuple

# Indy Autonomous Challenge Dallara-based vehicle envelope (commonly cited as 192" × 76").
# See e.g. IAC / team technical summaries; both AV-21 and AV-24 share this footprint.
IAC_DALLARA_LENGTH_M = 192.0 * 0.0254
IAC_DALLARA_WIDTH_M = 76.0 * 0.0254


def _dist_point_to_segment_2d(
    px: float,
    py: float,
    ax: float,
    ay: float,
    bx: float,
    by: float,
) -> float:
    abx = bx - ax
    aby = by - ay
    apx = px - ax
    apy = py - ay
    ab2 = abx * abx + aby * aby
    if ab2 < 1e-18:
        return math.hypot(px - ax, py - ay)
    t = max(0.0, min(1.0, (apx * abx + apy * aby) / ab2))
    qx = ax + t * abx
    qy = ay + t * aby
    return math.hypot(px - qx, py - qy)


def _segment_segment_distance_m(
    ax: float,
    ay: float,
    bx: float,
    by: float,
    cx: float,
    cy: float,
    dx: float,
    dy: float,
) -> float:
    """Minimum distance between closed segments AB and CD in 2D."""
    # Disjoint segments: minimum always occurs at an endpoint vs opposite segment.
    return min(
        _dist_point_to_segment_2d(ax, ay, cx, cy, dx, dy),
        _dist_point_to_segment_2d(bx, by, cx, cy, dx, dy),
        _dist_point_to_segment_2d(cx, cy, ax, ay, bx, by),
        _dist_point_to_segment_2d(dx, dy, ax, ay, bx, by),
    )


def _obb_corners(
    cx: float,
    cy: float,
    heading_rad: float,
    length_m: float,
    width_m: float,
) -> List[Tuple[float, float]]:
    half_l = length_m * 0.5
    half_w = width_m * 0.5
    c = math.cos(heading_rad)
    s = math.sin(heading_rad)
    # Forward = +x local, left = +y local
    fx = c * half_l
    fy = s * half_l
    lx = -s * half_w
    ly = c * half_w
    corners = [
        (cx + fx + lx, cy + fy + ly),
        (cx + fx - lx, cy + fy - ly),
        (cx - fx - lx, cy - fy - ly),
        (cx - fx + lx, cy - fy + ly),
    ]
    return corners


def _point_in_obb(
    px: float,
    py: float,
    cx: float,
    cy: float,
    heading_rad: float,
    length_m: float,
    width_m: float,
) -> bool:
    dx = px - cx
    dy = py - cy
    c = math.cos(-heading_rad)
    s = math.sin(-heading_rad)
    lx = c * dx - s * dy
    ly = s * dx + c * dy
    hl = length_m * 0.5 + 1e-9
    hw = width_m * 0.5 + 1e-9
    return abs(lx) <= hl and abs(ly) <= hw


def obb_separation_distance_m(
    cx1: float,
    cy1: float,
    heading1_rad: float,
    length1_m: float,
    width1_m: float,
    cx2: float,
    cy2: float,
    heading2_rad: float,
    length2_m: float,
    width2_m: float,
) -> float:
    """Minimum distance between the edges of two axis-aligned rectangles in world space.

    Each rectangle is centered at ``(cx, cy)`` with ``length`` along vehicle forward and
    ``width`` lateral. Returns ``0.0`` if the interiors overlap (one corner inside the
    other, or edges cross).
    """
    c1 = _obb_corners(cx1, cy1, heading1_rad, length1_m, width1_m)
    c2 = _obb_corners(cx2, cy2, heading2_rad, length2_m, width2_m)

    for x, y in c1:
        if _point_in_obb(x, y, cx2, cy2, heading2_rad, length2_m, width2_m):
            return 0.0
    for x, y in c2:
        if _point_in_obb(x, y, cx1, cy1, heading1_rad, length1_m, width1_m):
            return 0.0

    best = float("inf")
    for i in range(4):
        ax, ay = c1[i]
        bx, by = c1[(i + 1) % 4]
        for j in range(4):
            cx, cy = c2[j]
            dx, dy = c2[(j + 1) % 4]
            d = _segment_segment_distance_m(ax, ay, bx, by, cx, cy, dx, dy)
            if d < best:
                best = d
    return best


def eval_vehicle_length_width_m(obj) -> Tuple[float, float]:
    """Return ``(length_m, width_m)`` for a Scenic object, falling back to IAC defaults."""
    ln = getattr(obj, "length", None)
    w = getattr(obj, "width", None)
    try:
        length_m = float(ln) if ln is not None else IAC_DALLARA_LENGTH_M
        width_m = float(w) if w is not None else IAC_DALLARA_WIDTH_M
    except (TypeError, ValueError):
        length_m, width_m = IAC_DALLARA_LENGTH_M, IAC_DALLARA_WIDTH_M
    if length_m <= 0 or width_m <= 0:
        length_m, width_m = IAC_DALLARA_LENGTH_M, IAC_DALLARA_WIDTH_M
    return length_m, width_m


def eval_heading_rad(obj) -> Optional[float]:
    """Best-effort vehicle yaw (rad) for evaluation geometry."""
    da = getattr(obj, "dspaceActor", None)
    if da is not None:
        h = getattr(da, "heading", None)
        if h is not None:
            return float(h)
    y = getattr(obj, "yaw", None)
    if y is not None:
        return float(y)
    return None


def eval_dspace_dist_object_1_valid(d: Optional[float]) -> bool:
    """True if ``Dist_Object_1`` is a usable nonnegative distance (meters).

    Negative values (e.g. ``-1``) are treated as invalid / no target — not physical distance.
    Zero is valid (touching gap).
    """
    if d is None:
        return False
    try:
        x = float(d)
    except (TypeError, ValueError):
        return False
    if not math.isfinite(x):
        return False
    return x >= 0.0


# Defaults for log-only contact hints (not used for control).
EVAL_DEFAULT_OBB_OVERLAP_EPS_M = 0.02
EVAL_DEFAULT_HULL_NEAR_M = 1.0
EVAL_DEFAULT_SENSOR_CLOSE_M = 0.5


def classify_eval_contact(
    obb_sep_m: Optional[float],
    dspace_dist_m: Optional[float],
    *,
    overlap_eps_m: float = EVAL_DEFAULT_OBB_OVERLAP_EPS_M,
    hull_near_m: float = EVAL_DEFAULT_HULL_NEAR_M,
    sensor_close_m: float = EVAL_DEFAULT_SENSOR_CLOSE_M,
) -> Tuple[str, Dict[str, bool]]:
    """Classify evaluation contact risk from IAC OBB gap and optional dSPACE object distance.

    Returns ``(risk, flags)`` where ``risk`` is one of:
    ``overlap``, ``near``, ``clear``, ``insufficient_data``.

    **overlap** — hull polygons touch/penetrate (OBB gap ≤ eps) *or* sensor gap ≤ eps when valid.
    **near** — not overlap but within ``hull_near_m`` (OBB) or ``sensor_close_m`` (sensor).
    **insufficient_data** — no OBB (e.g. missing heading) and no valid sensor reading.
    """
    vd = eval_dspace_dist_object_1_valid(dspace_dist_m)
    obb = float(obb_sep_m) if obb_sep_m is not None else None

    overlap_obb = obb is not None and obb <= overlap_eps_m
    overlap_sens = vd and float(dspace_dist_m) <= overlap_eps_m
    overlap = overlap_obb or overlap_sens

    near_obb = obb is not None and not overlap and obb < hull_near_m
    near_sens = vd and not overlap and float(dspace_dist_m) < sensor_close_m
    near = near_obb or near_sens

    flags = {
        "overlap_obb": overlap_obb,
        "overlap_sensor": overlap_sens,
        "near_obb": near_obb,
        "near_sensor": near_sens,
        "dspace_valid": vd,
    }

    if overlap:
        return ("overlap", flags)
    if near:
        return ("near", flags)
    if obb is None and not vd:
        return ("insufficient_data", flags)
    return ("clear", flags)
