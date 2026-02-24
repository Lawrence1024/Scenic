# MPC Controller Module

**Last Updated:** 2025-02  
**Status:** MPCC lateral + longitudinal MPC integrated with racing behaviors

---

## Overview

This module implements **Model Predictive Contouring Control (MPCC)** for lateral (steering) control and **MPC for longitudinal** (throttle/brake) control of racing vehicles in the Scenic racing domain. The lateral controller uses a 4-state formulation with contouring cost, lag error, progress reward, and curvature feedforward. It replaces PID for improved performance on racing tracks, especially in high-speed cornering.

---

## Module Structure

```
mpc/
├── __init__.py              # Module exports (MPCLateralController, MPCLongitudinalController, load_mpc_config)
├── config.py                 # Configuration (YAML → MPCConfig)
├── reference_builder.py      # Waypoints → psi_ref, kappa_ref, v_ref, s_horizon
├── mpc_lateral.py            # MPCC lateral controller (state [e_y, e_psi, delta, s])
├── mpc_longitudinal.py       # Longitudinal MPC (throttle/brake)
├── speed_profile.py          # Curvature/CTE-based speed profile for v_ref
├── io_adapter.py             # ControlDesk I/O (readback, steering write)
├── utils.py                  # Low-pass filter, etc.
├── calibration.py            # Steering scale calibration (skeleton)
├── vehicle_mpc.yaml          # Default MPC parameters
├── README.md                 # This file
├── result_data/              # Log parsing and run analysis
│   ├── analyze_racing_log.py
│   ├── compare_racing_results.py
│   ├── parse_commit_metrics.py
│   └── README.md
├── testing/
│   ├── test_config.py, test_utils.py, test_reference_builder.py, test_mpc_lateral.py
│   ├── test_behavior_integration.py, test_simulation_integration.py, test_scenario_compilation.py
│   ├── run_tests.py, TEST_CASES.md
│   └── ...
└── *.md                      # MPCC_MIGRATION_PLAN, MPCC_IMPROVEMENT_PLAN, 3D_RESPONSIBILITIES, etc.
```

---

## Lateral MPC (MPCC) Formulation

**State:** `x = [e_y, e_psi, delta, s]`
- `e_y`: Lateral error (m), positive = left of path
- `e_psi`: Heading error (rad)
- `delta`: Front-wheel angle (rad)
- `s`: Progress along path (m), for lag/progress cost

**Control:** `u = delta_fb` (feedback steering). Total steering: `delta = delta_ff + delta_fb`, with `delta_ff = atan(L * kappa_ref)` (curvature feedforward).

**Dynamics:**
- `e_y_{k+1} = e_y_k + v_k * e_psi_k * dt`
- `e_psi_{k+1} = e_psi_k + (v_k/L)*(delta_ff_k + u_k)*dt - v_k*kappa_k*dt`
- `delta_{k+1} = delta_k + (dt/tau)*((delta_ff_k + u_k) - delta_k)`
- `s_{k+1} = s_k + v_ref_k*dt` (linearized progress)

**Cost:** Contouring (`w_ey*e_y^2`, `w_epsi*e_psi^2`), lag `Q_lag*(s_ref - s)^2`, progress reward `-Q_progress*(s_N - s_0)`, feedforward tracking `w_ff_track*(delta - delta_ff)^2`, input/rate/ddu penalties. Weights are adaptive by curvature (low / mid / high).

**Reference continuity (segment selection):**
- Best segment by perpendicular distance, with forward bias.
- **Gate:** Reject switch if `match_dist > max_wp_match_dist_m`, or progress backward, or along-path `s_jump > max_s_jump_m`; then keep previous segment.
- **Stick:** When `|prev_e_y| >= segment_stick_cte_m`, keep current segment to avoid reference flip.

**Conditional deadzone:** Apply CTE deadzone only when `|e_y| < cte_deadzone`, `match_dist < deadzone_dist_ok_m`, and `curvature_ahead_max < curv_deadzone_max`; otherwise do not zero e_y (avoids “recenter while far off”).

---

## Configuration

**File:** `vehicle_mpc.yaml` (ROS-style parameters under `/**/ros__parameters`).

**Key groups:**
- **Timing:** `ctrl_period`, `mpc_prediction_horizon` (e.g. 35), `mpc_prediction_dt`
- **Vehicle:** `wheel_base`, `max_steer_angle`, `steer_tau`, `steer_rate_lim`, `steer_cmd_max`
- **Lateral weights:** `w_ey`, `w_epsi`, `w_u`, `w_du`, `w_ddu`, `w_ff_track`; terminal `wT_ey`, `wT_epsi`
- **Adaptive curvature:** `use_adaptive_weights`, `low_curvature_threshold`, `high_curvature_threshold`; `w_ey_low_curv`, `w_ey_high_curv`, etc.
- **Feedforward:** `ff_preview_blend`, `ff_chicane_preview_blend`, `ff_chicane_curvature_threshold`
- **MPCC:** `Q_lag`, `Q_progress`
- **Oscillation / deadzone:** `cte_deadzone`, `deadzone_dist_ok_m`, `curv_deadzone_max`, `cte_multiplier_max`
- **Reference gate:** `segment_stick_cte_m`, `max_wp_match_dist_m`, `max_s_jump_m`, `segment_hysteresis_m`
- **Safety:** `admissible_position_error`, `admissible_yaw_error_rad`, `max_invalid_count`
- **Filtering:** `steering_lpf_cutoff_hz`
- **Longitudinal:** `w_v`, `w_a`, `w_u_lon`, `w_du_lon`, throttle/brake LPF, deadbands, curvature-based speed limits

After changing config, run a simulation and use `analyze_racing_log` (and optionally `compare_racing_results`). Set `run_edit_note` in the YAML to tag runs.

---

## Key Conventions

- **Heading:** Use ControlDesk readback (`dspaceActor.heading`) for MPC state; yaw is converted deg→rad and normalized to [-π, π].
- **Waypoints:** XODR/Scenic coordinates (e.g. `assets/ttls/.../transformed/*.csv`). Reference heading is `psi_ref = atan2(seg_dy, seg_dx)` with no >90° flip; e_y is projection-based with no sign flip (avoids spin-induced discontinuity).
- **Steering sign:** Environment-dependent; validate with logs (e.g. CTE positive ⇒ steer right). Current setup uses solver output without negation. See [Wiring and debugging](#wiring-and-debugging-control-pipeline-state-reference-kinematics) for the full chain.
- **Logging:** Use ASCII only in prints (no Unicode) for Windows console compatibility.

---

## Integration

### With Scenic behaviors
- **`FollowRacingLineMPCBehavior`** (in `behaviors.scenic`): Uses lateral MPC and longitudinal MPC (or shared speed profile), waypoint-based CTE, optional `mpc_config_path`.
- Example: `ego.behavior = FollowRacingLineMPCBehavior(target_speed=30, manage_gears=True, use_waypoints=True, lookahead=20.0, mpc_config_path=None)`

### With simulator
- **`getRacingControllers(agent, use_mpc=True, mpc_config_path=None)`** (in `simulators.py`): Returns `(MPCLongitudinalController, MPCLateralController)` when `use_mpc=True`; otherwise PID controllers from the driving domain.

### Controller interface
- **Lateral:** `MPCLateralController.run_step(vehicle_state, waypoints, ...)` returns **front wheel angle in radians** (`delta_cmd_rad`), not normalized. Behavior clamps to ±`max_steer_angle` and passes to `SetSteerAction(rad)`; the dSPACE simulator converts rad → steering wheel deg and writes to ControlDesk.
- **Longitudinal:** `MPCLongitudinalController.run_step(speed, v_ref, ...)` returns throttle/brake commands.

---

## Wiring and debugging (control pipeline, state, reference, kinematics)

This section answers the questions in `fix.md` so that sign/frame/saturation issues (e.g. intermittent 180° spin from reference flip or I/O sign) can be isolated quickly.

### A) Control pipeline (steering chain)

| Stage | What | Where |
|-------|------|--------|
| **MPC output** | `run_step(...)` returns **front wheel angle in rad** (`delta_cmd_rad`), not normalized [-1,1]. | `mpc_lateral.py`: after clamp to ±`current_delta_max`, rate limit (`steer_rate_limit_output_radps`), then `return float(delta_cmd_rad)`. |
| **Normalized steer → delta (rad)** | MPC already outputs rad. Behavior clamps to ±`DELTA_MAX_RAD` (0.2816) and passes that rad value. | `behaviors.scenic`: `final_steer = max(-DELTA_MAX_RAD, min(DELTA_MAX_RAD, float(steer_mpc)))`, then `SetSteerAction(final_steer)`. |
| **Rad → ControlDesk / dSPACE** | **Steering wheel angle** in degrees, ±240. Conversion: `theta_sw_deg = delta_road_rad * R * 180/pi`, with `R = THETA_SW_MAX_DEG / (DELTA_MAX_RAD * 180/pi)` ≈ 14.9. | `simulators/dspace/steer_io.py`: `road_rad_to_dspace_value(delta_road_rad)`; used in `vehicle/controller.py` and simulator write path. |
| **Negations** | None on write. Readback in `io_adapter.py`: `STEER_ACTUAL_SIGN = -1.0` applied to read steering (deg→rad) if ControlDesk sign is opposite to MPC convention. | `io_adapter.py` (read only). |
| **Scales / deadzone / rate / LPF / saturation** | **Saturation:** clip to ±`max_steer_angle` (rad) and ±`steer_rate_limit_output_radps` in `mpc_lateral.py`. No output LPF in lateral controller. **Scale:** rad→deg only in `steer_io` (R as above). | `mpc_lateral.py` (clamp, rate limit); `steer_io.py` (scale). |
| **Logged names** | **delta_cmd:** `_log_delta_cmd_rad` (MPC, post-clamp/rate). **steer_write:** `theta_sw_deg_sent` (simulator) = value written to ControlDesk. **steer_readback:** `state['steer_actual']` (rad) from `io_adapter.read_state_from_controldesk` (ControlDesk path `steer_actual` → deg→rad, then `STEER_ACTUAL_SIGN`). | MPC: `_log_delta_cmd_rad`, `_log_ctrl_*`. Simulator: `[STEER_IO]`, `[ControlDesk]`. |

**I/O write path:** The behavior does not call `write_steering_to_controldesk` with the MPC output directly. It uses `SetSteerAction(final_steer)` with `final_steer` in rad; the simulator reads `_control_state['steering']` (rad) and calls `road_rad_to_dspace_value(delta_rad)` in `vehicle/controller.py` (or the simulator’s write path), then writes the result to the ControlDesk steering variable.

### B) State estimation used by MPC

| Item | Source / convention |
|------|----------------------|
| **Heading** | ControlDesk readback: `dspaceActor.heading` (rad). Yaw from `Angle_Yaw_Vehicle_CoorSys_E[deg]` → rad → normalized to [-π, π] with `atan2(sin(yaw_rad_raw), cos(yaw_rad_raw))`. Used for MPC state and waypoint-ahead search. |
| **Yaw rate** | If available: `actor.angvel.z` (rad/s) in `io_adapter.read_state_from_controldesk`; behavior can pass `vehicle_state['yaw_rate']` into MPC. |
| **Speed** | `actor.linvel.norm()` in readback (m/s), or behavior `self.speed` / current_speed (m/s). |
| **e_psi** | `e_psi = heading - psi_ref`, then wrapped to [-π, π] with `atan2(sin(e_psi), cos(e_psi))`. | `mpc_lateral._compute_errors`. |
| **Frame** | Vehicle yaw is in **world frame** (Vehicle_CoorSys_E = vehicle pose in world). Typically ENU with yaw positive CCW (right-hand rule); readback does not apply 90° or 180° offset unless empirically required. |

### C) Reference builder and segment selection

| Item | Where / how |
|------|-------------|
| **Best-segment selection** | Perpendicular distance to each segment (XY); score = distance + penalty for being behind (u_proj < 0.5). Prefer segment with smallest score. Then **gate:** reject switch if `best_match_dist > max_wp_match_dist_m`, or `best_segment_idx < last_seg` (backward), or along-path jump > `max_s_jump_m`. **Stick:** if `|prev_e_y| >= segment_stick_cte_m`, keep `last_seg`. Hysteresis in curvature: stronger hysteresis in high curvature. | `mpc_lateral._compute_errors` (search, gate, stick). |
| **Reference heading and CTE (no flip)** | `psi_ref = atan2(seg_dy, seg_dx)` only (no >90° flip). `e_y = (px - proj_x)*nx + (py - proj_y)*ny` with no sign flip. Avoids spin-induced discontinuity (heading flip logic removed per Recommendation #1). | `mpc_lateral._compute_errors`. |
| **kappa_ref sign** | **Spline:** `kappa = (x'*y'' - y'*x'') / (x'^2 + y'^2)^(3/2)` — left turn > 0, right turn < 0. **Linear fallback:** 3-point formula; sign from cross product. | `reference_builder.py` (spline and linear). |
| **CTE (e_y)** | **Waypoint projection:** project vehicle (px, py) onto chosen segment; normal `n = (-seg_dy, seg_dx)/seg_len` (left of segment direction). `e_y = (px - proj_x)*nx + (py - proj_y)*ny` (no flip). Positive e_y = left of path. | `mpc_lateral._compute_errors`. |

### D) Wheelbase and kinematics

| Item | Value / convention |
|------|--------------------|
| **wheel_base** | **2.9718 m** (`vehicle_mpc.yaml`). |
| **delta_ff** | **Exactly** `delta_ff = atan(L * kappa_ref)` with L = wheel_base; used per step with optional preview blend (at-proj vs ahead). | `mpc_lateral.py`: `delta_ff_rad = math.atan(L * kappa_ff)`. |
| **max_steer_angle** | **Front wheel angle** in rad (default 0.2816). Used as clamp and in QP constraints. `steer_cmd_max` (e.g. 240) is **ControlDesk steering wheel deg** (dSPACE), not used inside MPC cost/constraints. | `vehicle_mpc.yaml`: `max_steer_angle`, `steer_cmd_max`. |

**Note:** The previous >90° reference flip and e_y sign flip were removed (Recommendation #1) to avoid spin-induced discontinuity. If sign issues remain, use the logged `delta_cmd_rad`, `steer_write`, `steer_readback`, gate/segment, and kappa sign to isolate I/O or kappa convention.

---

## Testing

**Location:** `mpc/testing/`

**Run:** From `mpc/testing/`: `python run_tests.py` or `python -m pytest test_*.py -v`

**Coverage:** Config loading, reference builder (waypoint search, curvature, reference generation), lateral MPC (state computation, gate/stick, OSQP solve), behavior/simulation/scenario integration. See `TEST_CASES.md` for the test guard.

**Note:** Do not run `scenic` commands from automated scripts; use `--count 1` when running scenarios manually to avoid infinite scene generation.

---

## Dependencies

- `numpy`, `scipy`, `osqp`, `pyyaml` (required)
- Optional: `matplotlib` for visualization

---

## Usage Example

```scenic
# Example: ego_mpc_behavior.scenic
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,
    manage_gears=True,
    use_waypoints=True,
    lookahead=20.0,
    mpc_config_path=None
)
```

```python
from scenic.domains.racing.mpc import MPCLateralController, load_mpc_config

config = load_mpc_config('src/scenic/domains/racing/mpc/vehicle_mpc.yaml')
mpc = MPCLateralController(config, timestep=0.05)
# In loop: steering = mpc.run_step(vehicle_state, waypoints, ...)
```

---

## Related Docs

- **fix.md** – Questions used to build the [Wiring and debugging](#wiring-and-debugging-control-pipeline-state-reference-kinematics) section (control pipeline, state, reference, kinematics).
- **MPCC_MIGRATION_PLAN.md** – Phases 1–3 (progress state, lag/progress cost, MPCC).
- **MPCC_IMPROVEMENT_PLAN.md** – Reference continuity gate, conditional deadzone, curve-approach commitment, feedforward, stick-by-match-quality.
- **result_data/README.md** – Log analysis and comparison.
- **3D_RESPONSIBILITIES.md** – 3D waypoints and projection.
