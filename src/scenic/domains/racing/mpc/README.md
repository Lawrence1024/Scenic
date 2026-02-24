# MPC Controller Module

**Last Updated:** 2025-02  
**Status:** MPCC lateral + longitudinal MPC integrated with racing behaviors

This document focuses on the **MPC submodule**: formulation, configuration, and how it fits into the racing control stack. For the overall racing library structure, see the parent [racing README](../README.md) and [RACING_CONTROL_CONTRACT.md](../RACING_CONTROL_CONTRACT.md).

---

## Overview

This module implements **Model Predictive Contouring Control (MPCC)** for lateral (steering) control and **MPC for longitudinal** (throttle/brake) control of racing vehicles. The lateral controller uses a 4-state formulation with contouring cost, lag error, progress reward, and curvature feedforward. It is the **single owner** of lateral steering limits (clamp and rate limit); the behavior layer applies only policy overrides (e.g. far-off-path recovery). Steering output is **road wheel angle in radians**; constants (`DELTA_MAX_RAD`, etc.) come from `scenic.domains.racing.constants`.

---

## Role in the racing library

- **Behaviors** (`FollowRacingLineMPCBehavior`) call `getRacingControllers(agent, use_mpc=True)` to obtain lateral and longitudinal MPCs, build waypoints/speed profile, and pass MPC output to `SetSteerAction(rad)` and throttle/brake actions.
- **MPC** owns lateral clamp (±`max_steer_angle`) and rate limit; returns `delta_cmd_rad`. Config default for `max_steer_angle` uses `DELTA_MAX_RAD` from racing constants.
- **Simulator (e.g. dSPACE)** interprets `_control_state['steering']` using `agent._racing_steer_units` (set to `'rad'` when MPC is used); conversion rad → steering wheel deg happens only in `steer_io`. See [RACING_CONTROL_CONTRACT.md](../RACING_CONTROL_CONTRACT.md).

---

## Module structure

```
mpc/
├── __init__.py              # Exports: MPCLateralController, MPCLongitudinalController, load_mpc_config, ReferenceBuilder
├── config.py                # YAML → MPCConfig (uses racing.constants.DELTA_MAX_RAD as default max_steer_angle)
├── reference_builder.py     # Waypoints → psi_ref, kappa_ref, v_ref, s_horizon
├── mpc_lateral.py           # MPCC lateral (state [e_y, e_psi, delta, s]); returns delta_cmd_rad; clamp/rate limit here only
├── mpc_longitudinal.py      # Longitudinal MPC (throttle/brake)
├── speed_profile.py        # Curvature/CTE-based speed profile for v_ref
├── io_adapter.py           # ControlDesk readback (state for MPC); STEER_ACTUAL_SIGN for readback
├── utils.py                # Low-pass filter, etc.
├── calibration.py          # Steering scale calibration (skeleton)
├── vehicle_mpc.yaml        # Default MPC parameters
├── README.md               # This file
├── result_data/            # Log parsing and run analysis
├── testing/                # Unit and integration tests
└── *.md                    # fix.md, steer_restrcutre_plan.md, MPCC_IMPROVEMENT_PLAN.md, etc.
```

---

## Lateral MPC (MPCC) formulation

**State:** `x = [e_y, e_psi, delta, s]`
- `e_y`: Lateral error (m), positive = left of path
- `e_psi`: Heading error (rad)
- `delta`: Front-wheel angle (rad)
- `s`: Progress along path (m), for lag/progress cost

**Control:** `u = delta_fb`. Total steering: `delta = delta_ff + delta_fb`, with `delta_ff = atan(L * kappa_ref)`.

**Dynamics:** Standard bicycle model; `s_{k+1} = s_k + v_ref_k*dt` for progress.

**Cost:** Contouring (`w_ey*e_y^2`, `w_epsi*e_psi^2`), lag, progress reward, feedforward tracking, input/rate/ddu penalties. Weights are adaptive by curvature (low / mid / high).

**Reference continuity (segment selection):** Best segment by perpendicular distance; **gate** (reject switch if too far / backward / s_jump); **stick** when `|prev_e_y| >= segment_stick_cte_m`. Recover mode: when gate rejects and match_dist > `gate_hard_fail_dist_m`, force re-association.

**Conditional deadzone:** Apply CTE deadzone only when association good and curvature low; otherwise do not zero e_y.

---

## Configuration

**File:** `vehicle_mpc.yaml` (ROS-style parameters under `/**/ros__parameters`).

**Key groups:** Timing, vehicle (`wheel_base`, `max_steer_angle` from constants when not overridden), lateral weights, adaptive curvature, feedforward, MPCC (Q_lag, Q_progress), oscillation/deadzone, reference gate, safety, longitudinal (weights, deadbands, curvature speed limits).

After changing config, run a simulation and use `result_data/analyze_racing_log` (and optionally `compare_racing_results`). Set `run_edit_note` in the YAML to tag runs.

---

## Key conventions

- **Heading:** ControlDesk readback for MPC state; yaw normalized to [-π, π]. No >90° flip; e_y is projection-based with no sign flip (avoids spin-induced discontinuity).
- **Steering output:** **Road wheel angle in radians.** Clamp and rate limit are applied inside `mpc_lateral.py` only. Behavior may apply a safety backup clamp using `DELTA_MAX_RAD` from `scenic.domains.racing.constants`.
- **Steering sign:** Validate with logs (CTE positive ⇒ steer right). See [Wiring and debugging](#wiring-and-debugging) for the full chain.
- **Logging:** ASCII only (Windows console compatibility).

---

## Integration

### With Scenic behaviors

- **`FollowRacingLineMPCBehavior`** (in `behaviors.scenic`): Uses lateral + longitudinal MPC, waypoint-based CTE, optional `mpc_config_path`.
- Example: `ego.behavior = FollowRacingLineMPCBehavior(target_speed=30, manage_gears=True, use_waypoints=True, lookahead=20.0, mpc_config_path=None)`

### With simulator

- **`getRacingControllers(agent, use_mpc=True, mpc_config_path=None)`** (in `simulators.py`): Returns `(MPCLongitudinalController, MPCLateralController)` and sets `agent._racing_steer_units = 'rad'` so the simulator interprets steering in radians.

### Controller interface

- **Lateral:** `MPCLateralController.run_step(vehicle_state, waypoints, ...)` returns **front wheel angle in radians** (`delta_cmd_rad`). Behavior passes it via `SetSteerAction(rad)`; simulator converts rad → deg only in `steer_io`.
- **Longitudinal:** `MPCLongitudinalController.run_step(...)` returns (throttle, brake) in [0, 1].

---

## Wiring and debugging (control pipeline, state, reference, kinematics)

Reference: [RACING_CONTROL_CONTRACT.md](../RACING_CONTROL_CONTRACT.md). This section gives MPC-specific locations.

### A) Control pipeline (steering chain)

| Stage | What | Where |
|-------|------|--------|
| **MPC output** | `run_step(...)` returns **delta_cmd_rad** (clamped to ±`max_steer_angle`, rate-limited). | `mpc_lateral.py`: after clamp, rate limit, `return float(delta_cmd_rad)`. |
| **Behavior** | Safety backup clamp to ±`DELTA_MAX_RAD` (from `scenic.domains.racing.constants`); passes rad to `SetSteerAction(final_steer)`. | `behaviors.scenic`. |
| **Rad → dSPACE** | Steering wheel deg ±240. Conversion only in `steer_io.road_rad_to_dspace_value(delta_road_rad)`. Constants from `scenic.domains.racing.constants`. | `simulators/dspace/steer_io.py`; used in `vehicle/controller.py`. |
| **Simulator interpretation** | Ego: if `agent._racing_steer_units == 'rad'`, use value as rad; else (PID) treat as [-1,1] and convert with `steer * DELTA_MAX_RAD`. | `simulators/dspace/vehicle/controller.py`. |

**Logged names:** `_log_delta_cmd_rad` (MPC); simulator may log `theta_sw_deg_sent`. Readback: `io_adapter.read_state_from_controldesk` → `state['steer_actual']` (rad) with `STEER_ACTUAL_SIGN` if needed.

### B) State estimation

Heading from ControlDesk; yaw rate from readback or behavior; speed from actor/behavior. e_psi = wrap(heading - psi_ref). Frame: world (ENU); no >90° flip.

### C) Reference builder and segment selection

Best segment by perpendicular distance; gate (max_wp_match_dist_m, max_s_jump_m, backward); stick (segment_stick_cte_m); recover when gate rejects and match_dist > gate_hard_fail_dist_m. psi_ref = atan2(seg_dy, seg_dx); e_y from projection (no flip). kappa_ref from spline or linear; sign: left turn > 0.

### D) Wheelbase and kinematics

wheel_base (e.g. 2.9718 m); delta_ff = atan(L * kappa_ref); max_steer_angle in rad (default from racing.constants). steer_cmd_max (e.g. 240) is dSPACE steering wheel deg, not used inside MPC.

---

## Testing

**Location:** `mpc/testing/`. Run: `python run_tests.py` or `python -m pytest test_*.py -v`. Coverage: config, reference builder, lateral MPC (state, gate/stick, OSQP), behavior/simulation integration. See `TEST_CASES.md`. Do not run `scenic` from automated scripts; use `--count 1` when running scenarios manually.

---

## Dependencies

`numpy`, `scipy`, `osqp`, `pyyaml` (required). Optional: `matplotlib`.

---

## Usage example

```scenic
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30, manage_gears=True, use_waypoints=True,
    lookahead=20.0, mpc_config_path=None
)
```

```python
from scenic.domains.racing.mpc import MPCLateralController, load_mpc_config
config = load_mpc_config('src/scenic/domains/racing/mpc/vehicle_mpc.yaml')
mpc = MPCLateralController(config, timestep=0.05)
# In loop: steering_rad = mpc.run_step(vehicle_state, waypoints, ...)
```

---

## Related docs

- **../RACING_CONTROL_CONTRACT.md** – Steering units, constants, simulator contract.
- **fix.md** – Log of fixes (no e_y flip, steer sign at write, gate/stick, single-segment).
- **steer_restrcutre_plan.md** – MPC output in rad; single rad→dSPACE conversion in steer_io.
- **MPCC_IMPROVEMENT_PLAN.md** – Reference gate, conditional deadzone, feedforward, stick.
- **result_data/README.md** – Log analysis and comparison.
- **3D_RESPONSIBILITIES.md** – 3D waypoints and projection.
