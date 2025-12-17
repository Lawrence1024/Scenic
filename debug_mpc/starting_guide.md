Below is a **precise “spec + checklist”** you can paste into Cursor for an AI agent. It’s written so the agent can build the MPC without needing your exact ControlDesk variable names (you’ll fill those in once you locate them).

---

# Cursor Task Spec: Build a waypoint-tracking MPC for IAC AV-24 sim via dSPACE ControlDesk

## Goal

Implement a **real-time Model Predictive Controller (MPC)** for an Indy Autonomous Challenge **AV-24** simulation controlled through **dSPACE/ControlDesk**. The controller tracks a waypoint list (TTL) and outputs steering (and optionally throttle/brake later). The system runs in simulation with a nominal control period **0.05 s (20 Hz)**.

Phase plan:

1. **Phase 1 (required):** Lateral MPC → steering only, keep existing longitudinal control (or simple speed PID).
2. **Phase 2 (optional):** Add longitudinal control (accel or throttle/brake).
3. **Phase 3 (optional):** Add gear management (rule-based), not hybrid MPC.

---

## Given/assumed vehicle parameters (from config)

Use these constants unless the user overrides:

### Timing

* `CTRL_DT = 0.05` seconds (control update period)
* MPC horizon: start with `N = 30` steps
  → total preview time = `N * CTRL_DT = 1.5 s`

### Geometry / steering limits

* Wheelbase: `L = 2.9718` m
* Max front wheel angle (physical): `DELTA_MAX_RAD = 0.2816` rad (≈16.1°)
* Steering command range in ControlDesk: `STEER_CMD_MAX = 70` (unit likely degrees-like UI)
* Steering rate limit (conservative): `DELTA_DOT_MAX = 1.0 rad/s` (can be tuned)
* Steering actuator first-order lag: `STEER_TAU = 0.3 s`

**Steering mapping placeholder (must be calibrated):**

* Assume front-wheel angle is proportional to steer command:

  * `delta_rad = steer_cmd * STEER_SCALE`
  * initial guess: `STEER_SCALE = DELTA_MAX_RAD / STEER_CMD_MAX`
  * so `STEER_SCALE ≈ 0.2816 / 70 ≈ 0.0040229 rad/unit`
* This will be refined by a quick calibration test (see below).

### Longitudinal command ranges (for later)

* Throttle command: `0..100`
* Brake command: `0..10000` (may saturate lower; verify)
* Acceleration limit for planning: `A_MAX = +2.0 m/s^2`, `A_MIN = -2.0 m/s^2` (initial)

### Gear

* Gear: integer 0..6 (0 neutral). Shifts ±1 only.
* For Phase 1, do **not** command gear; keep current gear manager/auto.

---

## Required I/O signals (to be provided by user by filling in names)

The agent must implement an adapter layer with a **configuration file** (YAML/JSON) holding the variable names.

### Inputs (read each tick)

Minimum:

* `pose_x` [m]
* `pose_y` [m]
* `yaw` [rad] (heading)
* `speed` [m/s]
* `timestamp` [s] (optional)

Recommended:

* `yaw_rate` [rad/s] (if available)
* `steer_actual` [same units as front-wheel angle or command]
* `engine_rpm` [rpm]
* `gear_actual` [int]

### Outputs (write each tick)

Phase 1:

* `steer_cmd` in ControlDesk units (range approx -70..70)

Optional Phase 2:

* `throttle_cmd` [0..100]
* `brake_cmd` [0..10000]

Optional Phase 3:

* `gear_cmd` [0..6] or `gear_up/down` discrete triggers

---

## Coordinate / waypoint handling

Waypoints (“TTL”) are a list of points:

* At minimum: `(x, y)`
* Optional: `(x, y, v_ref)` for speed profile

Requirements:

* Waypoint coordinates must be in the **same frame** as `pose_x, pose_y`.
* The controller must:

  1. find the nearest waypoint index to the current position,
  2. build a **local reference segment** for the horizon,
  3. compute reference heading `psi_ref` and curvature `kappa_ref` from the segment.

Implement resampling if waypoint spacing is uneven:

* default resample spacing: `RESAMPLE_DIST = 0.2 m` (matches config)
* if TTL is already uniformly spaced, can skip.

---

## MPC formulation (Phase 1): Lateral MPC with steering actuator

Use a small QP solved every tick (OSQP or similar).

### State and control

Use path-relative errors and actuator state:

* State: `x = [e_y, e_psi, delta]`

  * `e_y`: lateral error [m]
  * `e_psi`: heading error [rad]
  * `delta`: front-wheel angle [rad] (internal state)
* Control: `u = delta_cmd` (desired front-wheel angle [rad])

### Discrete-time model (linearized around small angles)

For each step k, with speed `v_k` and curvature reference `kappa_ref_k`:

Let `dt = CTRL_DT`, `tau = STEER_TAU`.

Steering actuator:

* `delta_{k+1} = delta_k + (dt/tau) * (u_k - delta_k)`

Error dynamics:

* `e_y_{k+1} = e_y_k + v_k * e_psi_k * dt`
* `e_psi_{k+1} = e_psi_k + (v_k / L) * delta_k * dt - v_k * kappa_ref_k * dt`

This gives linear system:

* `x_{k+1} = A_k x_k + B_k u_k + g_k`
  where `g_k` contains the curvature feedforward term `-v_k*kappa_ref_k*dt` in the `e_psi` equation.

### Constraints

Internal (physical) steering limit:

* `|delta_k| <= DELTA_MAX_RAD`

Steer command limit (convert later):

* `|u_k| <= DELTA_MAX_RAD`

Steering rate constraint (optional but recommended):

* `|delta_{k+1} - delta_k| <= DELTA_DOT_MAX * dt`

### Cost function

Quadratic cost:
Minimize over horizon:

* tracking:

  * `w_ey * e_y^2 + w_epsi * e_psi^2`
* smoothness:

  * `w_u * u^2`
  * `w_du * (u_k - u_{k-1})^2` (rate penalty)
    Optionally terminal weights:
* `wT_ey * e_y_N^2 + wT_epsi * e_psi_N^2`

Initial weights (safe starting point):

* `w_ey = 2.0`
* `w_epsi = 0.5`
* `w_u = 0.2`
* `w_du = 5.0`
* `wT_ey = 5.0`
* `wT_epsi = 1.0`

These are starting values; tune later based on oscillation/lag.

### Output mapping to ControlDesk steering command

MPC produces `delta_cmd_rad`. Convert to steer command units:

* `steer_cmd = clamp(delta_cmd_rad / STEER_SCALE, -STEER_CMD_MAX, +STEER_CMD_MAX)`

Where initial:

* `STEER_SCALE = DELTA_MAX_RAD / STEER_CMD_MAX`

---

## Calibration procedure (must implement as a small script or steps)

### 1) Steering scale calibration (required)

Purpose: determine `STEER_SCALE` accurately.

Procedure:

1. Hold speed low (e.g., 5–10 m/s), straight path.
2. Set `steer_cmd` manually to a small value (e.g., +10, -10).
3. Observe:

   * if `steer_actual` signal exists in radians or degrees, compute scale directly.
   * else infer scale by yaw-rate response (less accurate).
4. Update:

   * `STEER_SCALE = delta_actual_rad / steer_cmd`

The system should store this in a config file.

### 2) Brake saturation check (optional for Phase 2)

Test brake commands 1000/3000/6000/10000 and see if decel saturates; set `BRAKE_MAX_EFFECTIVE`.

---

## Software architecture requirements

Implement as a small project with these components:

1. `config/vehicle.yaml`

   * Contains constants listed above + variable names for ControlDesk I/O + weights.

2. `io/controldesk_adapter.py`

   * Abstract interface:

     * `read_state() -> {x,y,yaw,v,...}`
     * `write_commands(steer_cmd, throttle_cmd?, brake_cmd?, gear_cmd?)`
   * Implementation may be:

     * dSPACE API, or
     * shared memory, UDP, ROS bridge, etc.
   * If direct ControlDesk API is unavailable, implement a stub that reads/writes CSV/logs for offline testing.

3. `planning/reference_builder.py`

   * Nearest waypoint lookup
   * Builds horizon reference arrays:

     * `psi_ref[k]`, `kappa_ref[k]`, `v_ref[k]` (optional)
   * Computes curvature using 3-point method or numerical derivatives.

4. `control/mpc_lateral.py`

   * Builds QP matrices for OSQP each tick (warm-start enabled).
   * Takes state + reference and outputs `delta_cmd_rad`.
   * Enforces constraints and rate limits.
   * Maintains previous `u_{k-1}` for `w_du` penalty.

5. `main.py`

   * Loads config
   * Loop at 20 Hz:

     * read state
     * build reference
     * solve MPC
     * convert to ControlDesk units
     * write command
   * Include safety fallbacks:

     * If pose error too large: hold last steer or set to 0 (use thresholds below)

---

## Safety / fallback behavior (required)

Stop solving MPC if errors are too large (from config):

* `POSITION_ERROR_MAX = 5.0 m`
* `YAW_ERROR_MAX = 1.57 rad`

Fallback policy:

* If invalid / solver fails:

  * hold previous steering command for up to `MAX_INVALID = 10` ticks
  * then set steer_cmd = 0

Also filter steering output:

* Apply lowpass filter cutoff ~3 Hz:

  * `steering_lpf_cutoff_hz = 3.0`

---

## Deliverables

The agent must produce:

1. Working Phase-1 lateral MPC code runnable in sim loop (even with stub adapter).
2. A clear `vehicle.yaml` template with all required parameter fields.
3. A short “how to hook variable names” guide:

   * where to fill `steer_cmd_var`, `pose_x_var`, etc.
4. A tuning checklist (what to change if oscillation/understeer/lag).

---

## Parameter values to expose in config (must include)

* Timing: `CTRL_DT`, `N`
* Vehicle: `L`, `DELTA_MAX_RAD`, `STEER_TAU`, `DELTA_DOT_MAX`
* Mapping: `STEER_CMD_MAX`, `STEER_SCALE`
* Weights: `w_ey`, `w_epsi`, `w_u`, `w_du`, `wT_ey`, `wT_epsi`
* Thresholds: `POSITION_ERROR_MAX`, `YAW_ERROR_MAX`, `MAX_INVALID`
* Filter: `steering_lpf_cutoff_hz`
* Waypoint: `RESAMPLE_DIST`

---

## Notes / assumptions

* Start with kinematic error model; do not use Pacejka/tire forces initially.
* Longitudinal and gear control are out of scope for Phase 1; keep existing controller unless explicitly requested.
* Control frequency 20 Hz is acceptable; do not use dt=1.0 s.

---

If you want, paste what interface you’re using to read/write ControlDesk variables (Python API? UDP? Simulink block I/O?), and I can adapt the adapter spec to that exact method.
