"""SD-41A: dense reference trajectory — IAC-standard 7-column schema.

The brain (tactical planner) emits a `PlannerReference` per tick. The leg
(MPC) consumes it directly with no override authority. The safety
supervisor (stability guard) sits between them and can swap the reference
to a safe-stop trajectory in emergencies, but never modifies MPC outputs.

Schema source: TUM Autonomous Motorsport / IAC convention
(`TUMFTM/global_racetrajectory_optimization`):
  s_m  x_m  y_m  psi_rad  kappa_radpm  vx_mps  ax_mps2

This contract eliminates the brain-leg disconnects observed in SD-30..40
where the planner's `tactical_speed_cap` had to flow through a slew
limiter, multiple `min()` cap-composition layers, and a hard-ceiling
clamp before reaching the MPC. With this contract:

  - Planner composes ALL caps (cte, curvature, global, tactical) into vx_mps
  - MPC consumes vx_mps as the per-horizon reference, no slew, no clamp
  - Safety guard swaps the WHOLE reference for a safe-stop trajectory
    when predicted_collision fires, rather than clipping MPC commands

Stages A-G of the SD-41 refactor introduce this contract incrementally.
Stage A (this file) defines the type. Stages B-G wire it through.
"""

from __future__ import annotations

from dataclasses import dataclass, field
from typing import Optional

import numpy as np


@dataclass
class PlannerReference:
    """Dense reference trajectory emitted by the tactical planner per tick.

    Per-horizon-step arrays all share length N (matching the MPC prediction
    horizon — typically 35 steps × 0.05 s = 1.75 s). Sample i represents
    the planner's intended ego state at time `t_planner_stamp_s + i*dt`.

    The MPC tracks (x_m[i], y_m[i], psi_rad[i], kappa_radpm[i]) laterally
    and (vx_mps[i], ax_mps2[i]) longitudinally as feedforward references.
    """

    # ----- Per-horizon-step arrays (length N) ---------------------------
    s_m: np.ndarray
    """Arc-length on the chosen TTL polyline (m). Monotone non-decreasing."""

    x_m: np.ndarray
    """Easting (m), in canonical ENU frame. Reference position for lateral MPC."""

    y_m: np.ndarray
    """Northing (m). Reference position for lateral MPC."""

    psi_rad: np.ndarray
    """Heading (rad). Reference heading for lateral MPC."""

    kappa_radpm: np.ndarray
    """Signed curvature (1/m). Feedforward for lateral MPC; positive = left turn."""

    vx_mps: np.ndarray
    """Target longitudinal speed (m/s). Already accounts for ALL caps:
    cte, curvature, global, and the tactical / strategy / commit-mode cap.
    The MPC tracks this directly with no further composition or slew."""

    ax_mps2: np.ndarray
    """Target longitudinal acceleration (m/s²). Numerical derivative of
    vx_mps (positive = accelerating, negative = braking). Used as
    feedforward by longitudinal MPC."""

    # ----- Metadata ------------------------------------------------------
    traj_id: int
    """Monotonically-increasing per-tick trajectory id. MPC uses it to
    detect replans (every tick produces a new traj_id). Safety supervisor
    uses it to detect planner stalls (traj_id frozen for >N ticks)."""

    t_planner_stamp_s: float
    """Sim-time at which the planner produced this reference. Safety
    supervisor rejects references older than ~0.2 s (assumed stale)."""

    mode: str
    """Planner FSM mode: FREE_RUN, FOLLOW, SETUP_LEFT, SETUP_RIGHT,
    COMMIT_PASS_LEFT, COMMIT_PASS_RIGHT, HOLD_PASS_LEFT, HOLD_PASS_RIGHT,
    or ABORT_PASS. ADVISORY ONLY for the MPC — it's used by telemetry
    and by the safety supervisor's escalation logic, but the MPC does
    not branch on mode (only on the kinematic columns above). This
    enforces the contract: strategy stays in the planner."""

    decision_reason: str
    """Free-form telemetry tag explaining why the planner picked this
    mode/strategy this tick (e.g. 'strategy_pass_left', 'abort_hold',
    'commit_pass_right_hold', 'strategy_pass_left_cooldown_1.15s').
    Pure telemetry; no logic depends on this string."""

    # ----- Optional / supervisor-set fields ------------------------------
    is_safe_stop: bool = False
    """Set True by the safety supervisor when the original planner
    reference was swapped for a safe-stop trajectory. MPC behavior is
    identical either way (it just tracks); the flag is for telemetry +
    so the planner can detect 'my output was overridden last tick'."""

    binding_cap_source: str = ""
    """Diagnostic: which cap was binding when the planner composed
    vx_mps[0] (one of: 'cte', 'curvature', 'global', 'tactical', 'none').
    Replaces the prior `_speed_caps` dict telemetry."""

    ttl_key: str = "optimal"
    """Which TTL polyline this reference was built from
    ('optimal', 'left', 'right', 'pit'). Stored for telemetry + so the
    safety supervisor's safe-stop reference can hold the same TTL."""

    def horizon_length(self) -> int:
        """Number of samples in this reference (N)."""
        return int(self.vx_mps.shape[0]) if self.vx_mps is not None else 0

    def is_stale(self, current_sim_time_s: float, max_age_s: float = 0.2) -> bool:
        """True iff the reference is older than `max_age_s` seconds.

        Safety supervisor uses this to detect planner stalls — if the
        last reference is stale, the supervisor swaps to safe-stop and
        sets is_safe_stop=True.
        """
        return float(current_sim_time_s) - float(self.t_planner_stamp_s) > float(max_age_s)


__all__ = [
    "PlannerReference",
]
