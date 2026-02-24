# dSPACE Simulator Integration

This folder contains the **dSPACE simulator integration** for Scenic: how the racing domain connects to dSPACE ModelDesk/ControlDesk, including vehicle placement, coordinate transformation, control application (ego and fellow), and steering IO. The integration follows the [racing control contract](../../domains/racing/RACING_CONTROL_CONTRACT.md) so that steering units and constants are consistent with the racing library.

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

See [RACING_CONTROL_CONTRACT.md](../../domains/racing/RACING_CONTROL_CONTRACT.md).

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

## Known limitations

- **T-coordinate (lateral deviation):** ModelDesk may ignore lateral deviation settings for ego and fellows (centerline placement). This is a dSPACE ModelDesk configuration issue. See `debug_ego_cord/README.md`, `debug_route_code/README.md`.
- **Simulation control:** Currently configured for continuous running; pause/step may be disabled in `simulator.py`.

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
├── actions.py                   # SetVehicleControl (dSPACE-specific); steer doc per RACING_CONTROL_CONTRACT
├── vehicle/
│   ├── controller.py            # apply_ego_control (throttle/brake/steer by _racing_steer_units), apply_fellow_control
│   └── physics.py               # Fellow kinematic model (steering in [-1, 1])
├── geometry/
│   ├── coordinate_transform.py  # XODR ↔ RD
│   ├── route_projection.py      # RD → (s,t)
│   └── route_mapping.py         # Route detection
├── modeldesk/
│   └── placement.py            # place_ego, place_fellow (with transformation logs)
└── controldesk/
    └── readback.py             # read_ego_state, read_fellow_state (with readback logs)
```

---

## Related documentation

- [RACING_CONTROL_CONTRACT.md](../../domains/racing/RACING_CONTROL_CONTRACT.md) – Steering units, constants, simulator contract.
- [Racing domain README](../../domains/racing/README.md) – Full racing reference (objects, actions, behaviors, simulator implementation).
- [MPC README](../../domains/racing/mpc/README.md) – MPC formulation, config, wiring; MPC output in rad.
- [Segments README](../../domains/racing/segments/README.md) – Racing library structure and segment map.
- `debug_cord_code/README.md`, `debug_ego_cord/README.md`, `debug_route_code/README.md` – Route and coordinate debugging.
