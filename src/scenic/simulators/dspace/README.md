# dSPACE Simulator Integration

This folder contains the **dSPACE simulator integration** for Scenic: how the racing domain connects to dSPACE ModelDesk/ControlDesk, including vehicle placement, coordinate transformation, control application (ego and fellow), and steering IO. The integration follows the [racing control contract](../../domains/racing/README.md#control-contract) so that steering units and constants are consistent with the racing library.

---

## Overview

The dSPACE backend provides:

- **Racing simulation:** `DSpaceSimulation` extends `RacingSimulation`; overrides `getRacingControllers(agent, use_mpc=..., mpc_config_path=...)` and sets `agent._racing_steer_units` so the write path knows whether steering is in radians (MPC) or normalized [-1, 1] (PID).
- **Vehicle placement:** XODR → RD coordinate transform, route detection (Pit/Lap), projection to (s,t), placement in ModelDesk, readback comparison.
- **Control application:** Ego: throttle/brake/steering from `_control_state` → VesiInterface (throttle %, brake, steering wheel deg). Fellow: throttle/brake/steering → physics model → velocity and deviation written to External_Signals.
- **Steering IO:** Single place for rad → steering wheel deg: `steer_io.road_rad_to_dspace_value()`. Constants from `scenic.domains.racing.constants`.

---

## Racing integration

### Control contract

- **Steering units:** PID path outputs normalized [-1, 1]; MPC path outputs road wheel angle in **radians**. The simulator interprets `_control_state['steering']` using `agent._racing_steer_units` (`'rad'` or `'normalized'`), set by `getRacingControllers` when controllers are created.
- **Ego:** If `_racing_steer_units == 'rad'`, treat value as rad and pass to `road_rad_to_dspace_value(delta_rad)`. Otherwise treat as [-1, 1] and convert: `delta_rad = steer * DELTA_MAX_RAD`, then `road_rad_to_dspace_value(delta_rad)`.
- **Fellow:** Physics model expects steering in [-1, 1]. When `_racing_steer_units == 'rad'`, convert rad → normalized before calling physics.
- **Constants:** `DELTA_MAX_RAD`, `THETA_SW_MAX_DEG`, `R` are defined in `scenic.domains.racing.constants`; `steer_io` imports them. Do not hardcode 0.2816 or 240 elsewhere.

See [racing README – Control contract](../../domains/racing/README.md#control-contract).

### Steering IO (single conversion point)

Ego steering is always in **road wheel angle (radians)** by the time it is written. The **only** place that converts rad → steering wheel deg (e.g. ±240°) is **`steer_io.py`** via `road_rad_to_dspace_value()`. Do not add scaling or 240 elsewhere.

- **PID path:** Behavior outputs [-1, 1]; `vehicle/controller.py` converts to rad with `steer * DELTA_MAX_RAD`, then calls `road_rad_to_dspace_value(delta_rad)`.
- **MPC path:** Behavior outputs rad; controller passes rad through to `road_rad_to_dspace_value(delta_rad)`.

**Key files:** `steer_io.py`, `vehicle/controller.py` (apply_ego_control, apply_fellow_control).

---

## Coordinate transformation and placement

1. **XODR → RD:** Apply coordinate transform (rotation + translation).
2. **Route detection:** Determine if vehicle is on pitLane or mainRacing road.
3. **RD → (s,t):** Project RD coordinates to route-relative (s,t) using route-specific road sequences.
4. **Placement:** Set (s,t) in ModelDesk with appropriate route.
5. **Readback:** Read actual position from ControlDesk and compare with expected.

**Key files:**
- `geometry/coordinate_transform.py` – XODR ↔ RD transformation
- `geometry/route_projection.py` – RD → route-specific (s,t) projection
- `geometry/route_mapping.py` – Route detection (pitLane vs mainRacing)
- `modeldesk/placement.py` – Vehicle placement in ModelDesk
- `controldesk/readback.py` – Position readback from ControlDesk

### GPS ↔ dSPACE local (and to Scenic XODR)

A **GPS ↔ dSPACE local** transform is available for converting between GNSS (lon, lat) and dSPACE Cartesian (x, y). It is calibrated from a run that produced `gps_dspace_table.csv` (ego x, y, z, heading plus GNSS Longitude_deg, Latitude_deg, Heading_deg from GPS_CALC).

- **Calibration:** After a run, fit and save calibration (from repo root):
  ```bash
  python src/scenic/simulators/dspace/converters/fit_gps_dspace_calibration.py
  ```
  Default: reads `gps_dspace_table.csv` from repo root, writes `src/scenic/simulators/dspace/geometry/gps_dspace_calibration.json`.

- **Usage in code:**
  ```python
  from scenic.simulators.dspace.geometry.gps_transform import load_calibration, GPSDspaceTransform

  cal = load_calibration(Path(".../dspace/geometry/gps_dspace_calibration.json"))
  x_dspace, y_dspace = cal.gps_to_dspace(longitude_deg, latitude_deg)
  lon_deg, lat_deg = cal.dspace_to_gps(x_dspace, y_dspace)
  ```
  From dSPACE (x, y) you can then use the existing **XODR ↔ RD** transform in `geometry/coordinate_transform.py` (e.g. `apply_inverse_coordinate_transform`) to get Scenic XODR coordinates.

- **Round-trip verify:** Ensures Scenic → (place) → read GPS → GPS→dSPACE → dSPACE→Scenic matches the initial Scenic position:
  ```bash
  python src/scenic/simulators/dspace/converters/verify_gps_round_trip.py
  ```
  With the current table (no `x_rd`/`y_rd`), this checks GPS ↔ Scenic (x_dspace, y_dspace) only. After a run that collects raw dSPACE RD, the CSV will include `x_rd`, `y_rd`; then run `fit_gps_dspace_calibration.py --rd` to produce `gps_rd_calibration.json`, and the verify script will run the full chain (Scenic → RD → GPS → RD → Scenic).

**Key files:** `geometry/gps_transform.py`, `geometry/gps_dspace_calibration.json` (after calibration), `converters/fit_gps_dspace_calibration.py`, `converters/verify_gps_round_trip.py`.

---

## Logging and debugging

### Transformation chain logs

During placement, each vehicle logs:
```
[VehicleName] XODR: (x, y) → RD: (x, y) → Route RouteName (s=s_val, t=t_val)
```
**Location:** `modeldesk/placement.py` – `place_ego()`, `place_fellow()`.

### Readback comparison logs

On first readback:
```
[VehicleName Readback] RD: (actual_rd) [expected: (expected_rd), error: X.XXXm]
[VehicleName Readback] XODR: (actual_xodr) [expected: (expected_xodr), error: X.XXXm]
```
**Location:** `controldesk/readback.py` – `read_ego_state()`, `read_fellow_state()`.

### Removed / reduced logs

Verbose messages (e.g. “Creating EGO/FELLOW vehicle”, step-by-step ControlDesk connection) have been removed or reduced to keep logs focused.

---

## ControlDesk variable access and step polling

When the simulator waits for simulated time to reach the step deadline, it polls `get_var(SIMULATED_TIME_PATH)` in a loop. This has implications for timeout tuning:

- **`get_var` latency:** Each call to `get_var` (ControlDesk/COM) takes approximately **2–3 ms** per read. This dominates the polling loop cost (in addition to the 1 ms `sleep(0.001)` per iteration).
- **Sleep counter vs. wall timeout:** With `poll_timeout_wall = 10 × timestep` and `timestep = 0.01` s (so 0.1 s timeout), each iteration is ~3.5 ms (get_var + sleep). The loop therefore runs about **28–29 iterations** before the wall timeout. When a retry is logged (deadline not reached in time), the last printed `sleep_counter` is typically **26–29**. If you see a much lower sleep_counter, something else (e.g. an exception or early exit) is occurring; if you see a much higher one, get_var may be faster than 2–3 ms in your setup.

Use these numbers when choosing `poll_timeout_wall`: allow enough wall time for the sim to advance one timestep, given that each poll iteration costs ~2–3 ms plus the 1 ms sleep.

- **No explicit wait for dSPACE readiness:** The simulator does **not** wait for dSPACE to be "ready" before or after a step. It calls `advance_simulation_step()`, then polls until simulated time has advanced (or the poll window times out). If the effect is seen during the poll, the step is done. If not, the advance is assumed not to have been accepted (dSPACE not ready), and the simulator retries: it calls `advance_simulation_step()` again and polls again, up to **max_retries** (default 10). Timing breakdown uses **step_time_waiting_ready** (time on failed attempts) and **step_time_ready_until_advance** (time from start of the winning attempt’s poll until the deadline is seen).

- **COM (ControlDesk) timing:** Per-path timing for the ControlDesk/COM backend is not printed at teardown. COM is used mainly for **setup** (e.g. maneuver start, activation flags); the heavy per-step variable traffic uses MAPort. COM timing is negligible in the long run, so it is omitted from the default timing summary.

---

## Known limitations

- **T-coordinate (lateral deviation):** ModelDesk may ignore lateral deviation settings for ego and fellows (centerline placement). This is a dSPACE ModelDesk configuration issue. See `debug_ego_cord/README.md`, `debug_route_code/README.md`.
- **Simulation control:** The simulator pauses the dSPACE run immediately after starting the maneuver (step 10b) so that variable setup time (COM/MAPort, readback, warmup) does not advance simulated time. Warmup and the main loop use step-by-step advance (SingleStep). This keeps `t_start` and initial state consistent across runs regardless of how long dSPACE takes to connect.

---

## Running a scenario

1. Activate virtual environment (e.g. `venv/Scripts/Activate.ps1`).
2. Run Scenic with the dSPACE racing model, e.g.:
   ```powershell
   scenic examples/racing/fellow_fixed_placing.scenic --2d --model scenic.simulators.dspace.racing_model --simulate --time 10
   ```
3. Check logs for transformation chain, readback errors, and (if applicable) steering/control.

---

## File structure

```
src/scenic/simulators/dspace/
├── README.md                    # This file
├── simulator.py                 # DSpaceSimulation, getRacingControllers override, executeActions
├── steer_io.py                  # road_rad_to_dspace_value; only place for rad → steering wheel deg
├── actions.py                   # SetVehicleControl (dSPACE-specific); steer doc per racing README control contract
├── vehicle/
│   ├── controller.py            # apply_ego_control (throttle/brake/steer by _racing_steer_units), apply_fellow_control
│   └── physics.py               # Fellow kinematic model (steering in [-1, 1])
├── geometry/
│   ├── coordinate_transform.py  # XODR ↔ RD
│   ├── route_projection.py      # RD → (s,t)
│   └── route_mapping.py         # Route detection
├── modeldesk/
│   └── placement.py            # place_ego, place_fellow (with transformation logs)
├── controldesk/
│   └── readback.py             # read_ego_state, read_fellow_state (with readback logs)
└── create_new_ttl/             # TTL tooling: generate/close/visualize racing and pitlane TTLs
    ├── README.md                # Full usage and script index
    ├── combine_and_compare_ttl.py
    ├── find_xodr_for_st_coordinates.py
    ├── generate_racing_line.py
    ├── close_ttl_loop.py
    └── ...                      # See create_new_ttl/README.md
```

---

## TTL tooling

Scripts for generating and validating target trajectory lines (TTLs) live in **`create_new_ttl/`**. Run them from the **Scenic repository root**, e.g.:

```bash
python src/scenic/simulators/dspace/create_new_ttl/combine_and_compare_ttl.py
python src/scenic/simulators/dspace/create_new_ttl/visualize_combined_ttl.py --overlap
```

See **`create_new_ttl/README.md`** for the full script index and usage.

---

## Related documentation

- [Racing README – Control contract](../../domains/racing/README.md#control-contract) – Steering units, constants, simulator contract.
- [Racing domain README](../../domains/racing/README.md) – Full racing reference (objects, actions, behaviors, simulator implementation).
- [MPC README](../../domains/racing/mpc/README.md) – MPC formulation, config, wiring; MPC output in rad.
- [Segments README](../../domains/racing/segments/README.md) – Racing library structure and segment map.
- `debug_cord_code/README.md`, `debug_ego_cord/README.md`, `debug_route_code/README.md` – Route and coordinate debugging.
