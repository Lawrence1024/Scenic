"""SD-42L: per-TTL offline velocity profile.

Replaces the runtime cap-composition chain in `behaviors.scenic` with a single
authoritative source of "what speed is right at this point on this TTL."

Algorithm — TUM forward-backward velocity profile pass:

  1. Cornering ceiling per waypoint: vx_max(s) = sqrt(a_lat_max / max(|kappa|, eps))
     This is the friction-limited cornering speed (no longitudinal accel
     budget left over from lateral grip use).

  2. Forward pass (accel limit):
     vx[i+1] = min(vx_max[i+1], sqrt(vx[i]^2 + 2 * a_long_accel * ds))
     "If I'm at vx[i] now, I can't be faster than (kinematics bound) at i+1."

  3. Backward pass (decel limit):
     vx[i] = min(vx[i], sqrt(vx[i+1]^2 + 2 * a_long_decel * ds))
     "If I have to be at vx[i+1] then, I must be slow enough now to brake to it."

  4. Clip to [v_min_mps, v_max_mps] absolute bounds.

  5. ax_optimal = vx · d(vx)/ds (chain rule, since dvx/dt = (dvx/ds)·(ds/dt) = (dvx/ds)·vx).

The result is a `VelocityProfile` cached on the TTL data structure — computed
once per TTL polyline at scene init, looked up per tick by the planner.

Why this replaces 9+ runtime caps:
  - The cornering ceiling subsumes `curvature_speed_cap` (it IS the same formula,
    just precomputed and forward-backward-smoothed for accel/decel realism).
  - The forward-backward pass naturally produces "brake before the corner,
    throttle on the exit" without any RC-7b-style `slew_up_ms = 0.0` hack.
  - The CTE cap is a tracking-error gate on top, not a baseline speed source.
  - Tactical caps (FOLLOW headway, ABORT derate) modulate the profile in the
    planner's mode-shaping step, not as another `min()` layer in behaviors.

Sources:
  - TUM Autonomous Motorsport (Betz et al., JFR 2023, arXiv 2205.15979).
  - TUM `mod_vehicle_dynamics_control` — open-source reference implementation.
  - F1Tenth Unifying Survey (arXiv 2402.18558) — empirical: PP + offline
    velocity profile beats MPCC online primarily because the offline profile
    is better than what MPC can synthesize at runtime.
"""

from __future__ import annotations

from dataclasses import dataclass
from typing import Optional, Sequence

import numpy as np


@dataclass
class VelocityProfile:
    """Per-TTL precomputed racing speed and acceleration as functions of arc length.

    All arrays are 1D length n_waypoints. Sample i corresponds to the i-th
    waypoint of the TTL polyline (same indexing as the source waypoints list).
    """

    s_m: np.ndarray
    """Arc length from waypoint 0 (m). Monotone non-decreasing. Last entry =
    total polyline length."""

    kappa_radpm: np.ndarray
    """Signed curvature at each waypoint (1/m). Source: race_common TTL
    column 4 if available, else 3-point cross-product. Sign convention:
    positive = left turn."""

    vx_optimal_mps: np.ndarray
    """Forward-backward smoothed optimal longitudinal speed (m/s). The
    racing-line speed at this point if you respect both lateral and
    longitudinal grip limits."""

    ax_optimal_mps2: np.ndarray
    """Optimal longitudinal acceleration (m/s²). Numerical derivative of
    vx_optimal: ax = vx · (dvx/ds)."""

    a_lat_max_mps2: float
    """Lateral accel limit used to derive the profile. Tunable per scene
    via the `racing_a_lat_max_mps2` scene param."""

    def lookup(self, s_query_m: float) -> float:
        """Linear-interpolated lookup of `vx_optimal_mps` at arbitrary `s`.

        Wraps via modulo on `s_m[-1]` (total polyline length) for closed-loop
        TTLs. Returns a single float scalar.
        """
        n = int(self.s_m.shape[0])
        if n == 0:
            return 0.0
        L = float(self.s_m[-1])
        if L <= 0.0:
            return float(self.vx_optimal_mps[0])
        s = float(s_query_m) % L
        # np.searchsorted is O(log n); n is typically a few thousand so this
        # is fine even at 20 Hz × 35 horizon × 4 strategies.
        idx = int(np.searchsorted(self.s_m, s, side="right")) - 1
        idx = max(0, min(n - 2, idx))
        s0, s1 = float(self.s_m[idx]), float(self.s_m[idx + 1])
        v0, v1 = float(self.vx_optimal_mps[idx]), float(self.vx_optimal_mps[idx + 1])
        if s1 <= s0:
            return v0
        alpha = (s - s0) / (s1 - s0)
        return float(v0 + alpha * (v1 - v0))


def _compute_kappa_3point(waypoints: Sequence[Sequence[float]]) -> np.ndarray:
    """3-point Menger signed curvature for 3-col TTLs lacking a precomputed
    curvature column.

    Menger curvature for triangle (P0, P1, P2): K = 4·Area / (|P0P1|·|P1P2|·|P0P2|).
    Area = 0.5·|cross(P1-P0, P2-P1)|, so:
      K = 2·cross / (|v1|·|v2|·|v3|)
    where v1=P1-P0, v2=P2-P1, v3=P2-P0 (the third triangle side).

    Sign convention: positive cross = counter-clockwise turn = left turn = K > 0.

    NOTE: behaviors.scenic:1577 uses `avg_len = 0.5·(|v1|+|v2|)` instead of
    |v3| in the denominator. That's an off-by-2× formula (returns 2·K_actual
    on a circle: avg_len ≈ chord_per_segment, while |v3| ≈ 2·chord_per_segment).
    The Menger version here is the correct one. The legacy behaviors.scenic
    formula compensates by using `max_lateral_accel = 8.0` instead of the
    physical ~12-15 m/s² — its `curvature_speed_cap` happens to land near the
    right number for the wrong reason. SD-42N's deletion of that cap removes
    the mismatched calibration alongside the formula.

    Closed-loop indexing: waypoint 0 uses waypoints[-1] and waypoints[1] as
    neighbors so the curvature at the lap-loop closure is well-defined.
    """
    n = len(waypoints)
    kappa = np.zeros(n, dtype=np.float64)
    if n < 3:
        return kappa
    for i in range(n):
        i0 = (i - 1) % n
        i1 = i
        i2 = (i + 1) % n
        x0, y0 = float(waypoints[i0][0]), float(waypoints[i0][1])
        x1, y1 = float(waypoints[i1][0]), float(waypoints[i1][1])
        x2, y2 = float(waypoints[i2][0]), float(waypoints[i2][1])
        v1x, v1y = x1 - x0, y1 - y0
        v2x, v2y = x2 - x1, y2 - y1
        v3x, v3y = x2 - x0, y2 - y0  # P0 → P2 (third triangle side)
        l1 = (v1x * v1x + v1y * v1y) ** 0.5
        l2 = (v2x * v2x + v2y * v2y) ** 0.5
        l3 = (v3x * v3x + v3y * v3y) ** 0.5
        if l1 < 1e-6 or l2 < 1e-6 or l3 < 1e-6:
            continue
        cross = v1x * v2y - v1y * v2x
        # Menger curvature: K = 2·cross / (l1·l2·l3). Sign from cross.
        kappa[i] = 2.0 * cross / (l1 * l2 * l3)
    return kappa


def _compute_arc_length(waypoints: Sequence[Sequence[float]]) -> np.ndarray:
    """Cumulative arc length from waypoint 0. 2D distance (z ignored)."""
    n = len(waypoints)
    s = np.zeros(n, dtype=np.float64)
    for i in range(1, n):
        dx = float(waypoints[i][0]) - float(waypoints[i - 1][0])
        dy = float(waypoints[i][1]) - float(waypoints[i - 1][1])
        s[i] = s[i - 1] + (dx * dx + dy * dy) ** 0.5
    return s


def compute_velocity_profile(
    waypoints: Sequence[Sequence[float]],
    *,
    a_lat_max_mps2: float = 12.0,
    a_long_accel_max_mps2: float = 8.0,
    a_long_decel_max_mps2: float = 12.0,
    v_max_mps: float = 62.58,
    v_min_mps: float = 3.0,
    kappa_per_waypoint: Optional[Sequence[float]] = None,
    kappa_eps: float = 1.0e-3,
) -> VelocityProfile:
    """Build a `VelocityProfile` from a polyline using TUM's forward-backward pass.

    Parameters
    ----------
    waypoints
        Sequence of (x, y) or (x, y, z) tuples. Closed-loop assumed (last
        waypoint connects back to first for curvature computation).
    a_lat_max_mps2
        Lateral acceleration budget (m/s²). Default 12.0 is a conservative IAC
        Dallara number; TUM publishes 14.0 for actual race conditions. The
        SD-41I `commit_max_lateral_accel_mps2` was 8.0 — that was a cornering
        cap not a friction limit, so 12.0 here is consistent.
    a_long_accel_max_mps2
        Longitudinal acceleration limit (m/s²) for the forward pass.
    a_long_decel_max_mps2
        Longitudinal deceleration limit (m/s²) for the backward pass.
        Higher than accel because braking is normally stronger than throttle.
    v_max_mps
        Absolute speed ceiling (m/s). Default matches `MAX_SPEED_LIMIT_MS`.
    v_min_mps
        Floor so we never command full-stop on an apex. The minimum
        sustainable speed in gear 2 (~3 m/s with idle creep).
    kappa_per_waypoint
        If supplied (e.g., race_common TTL column 4), used directly. Otherwise
        computed via `_compute_kappa_3point`.
    kappa_eps
        Floor on |kappa| to prevent division-by-zero on perfectly straight
        segments. 1e-3 corresponds to a 1000 m radius (essentially straight),
        which gives vx_max = sqrt(12 / 0.001) = 109 m/s, well above v_max
        so the actual ceiling becomes v_max.

    Returns
    -------
    VelocityProfile
        Per-waypoint optimal speed and acceleration arrays.
    """
    n = len(waypoints)
    if n < 2:
        # Degenerate input — return a constant-zero profile of the right shape
        # so downstream code never has to None-check. v_min_mps so we don't
        # divide by zero anywhere.
        return VelocityProfile(
            s_m=np.zeros(max(n, 1), dtype=np.float64),
            kappa_radpm=np.zeros(max(n, 1), dtype=np.float64),
            vx_optimal_mps=np.full(max(n, 1), v_min_mps, dtype=np.float64),
            ax_optimal_mps2=np.zeros(max(n, 1), dtype=np.float64),
            a_lat_max_mps2=float(a_lat_max_mps2),
        )

    s = _compute_arc_length(waypoints)
    if kappa_per_waypoint is not None and len(kappa_per_waypoint) == n:
        kappa = np.asarray(kappa_per_waypoint, dtype=np.float64)
    else:
        kappa = _compute_kappa_3point(waypoints)

    # Step 1: cornering ceiling. abs() because vx_max only depends on the
    # *magnitude* of curvature (lateral force is a scalar).
    abs_kappa = np.maximum(np.abs(kappa), float(kappa_eps))
    a_lat = float(a_lat_max_mps2)
    vx_max = np.sqrt(a_lat / abs_kappa)
    vx_max = np.minimum(vx_max, float(v_max_mps))
    vx_max = np.maximum(vx_max, float(v_min_mps))

    # Step 2: forward pass. Initialize ego at the cornering ceiling (assume
    # we entered at racing speed). Walk forward enforcing accel limit.
    vx = vx_max.copy()
    a_a = float(a_long_accel_max_mps2)
    for i in range(n - 1):
        ds = float(s[i + 1] - s[i])
        if ds <= 0.0:
            continue
        # vf^2 = vi^2 + 2·a·ds; vi is current vx[i], cap by lateral ceiling at i+1.
        cap = (vx[i] * vx[i] + 2.0 * a_a * ds) ** 0.5
        vx[i + 1] = min(vx[i + 1], cap)

    # Step 3: backward pass. Walk backward enforcing decel limit.
    a_d = float(a_long_decel_max_mps2)
    for i in range(n - 2, -1, -1):
        ds = float(s[i + 1] - s[i])
        if ds <= 0.0:
            continue
        # vi^2 = vf^2 + 2·a_d·ds; vf is downstream vx[i+1].
        cap = (vx[i + 1] * vx[i + 1] + 2.0 * a_d * ds) ** 0.5
        vx[i] = min(vx[i], cap)

    # Step 4: final clip (forward-backward can occasionally undershoot v_min on
    # very tight corners — the grip-limited cornering is what it is).
    vx = np.minimum(vx, float(v_max_mps))
    vx = np.maximum(vx, float(v_min_mps))

    # Step 5: ax via chain rule. Forward difference; last sample copies previous.
    ax = np.zeros(n, dtype=np.float64)
    for i in range(n - 1):
        ds = float(s[i + 1] - s[i])
        if ds <= 0.0:
            continue
        dvx_ds = (vx[i + 1] - vx[i]) / ds
        ax[i] = vx[i] * dvx_ds
    if n >= 2:
        ax[-1] = ax[-2]

    return VelocityProfile(
        s_m=s,
        kappa_radpm=kappa,
        vx_optimal_mps=vx,
        ax_optimal_mps2=ax,
        a_lat_max_mps2=float(a_lat_max_mps2),
    )


__all__ = [
    "VelocityProfile",
    "compute_velocity_profile",
]
