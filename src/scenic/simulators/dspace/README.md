# dSPACE Simulator Integration

This folder contains the Scenic-side integration for dSPACE, including:

- the main `DSpaceSimulation` implementation
- ModelDesk / ControlDesk integration
- coordinate and route projection helpers
- steering and vehicle-control IO
- the CoSim synchronization hook used to make VEOS step in lock-step with Scenic

This README is for:

```text
src/scenic/simulators/dspace/README.md
```

---

## What this folder is responsible for

At a high level, the dSPACE backend is responsible for four things:

1. **Simulator lifecycle**
   - creating and destroying the dSPACE-backed simulation
   - coordinating ModelDesk / ControlDesk / MAPort setup
   - managing step execution

2. **Vehicle placement**
   - projecting Scenic world positions into route-relative `(s, t)`
   - placing ego and fellow vehicles in ModelDesk
   - setting each fellow’s ModelDesk **Traffic Object** (3D vehicle asset from the Traffic Object browser)
   - comparing expected placement with ControlDesk readback

3. **Control application**
   - applying ego throttle / brake / steering
   - applying fellow control through the physics model
   - converting steering units consistently

4. **CoSim synchronization**
   - hosting the Python-side step gate used by VEOS CoSim
   - allowing Scenic to become the pacing master for simulation stepping

---

## Important folders and files

### Main simulation entry point

- `simulator.py`
  - contains `DSpaceSimulation`
  - this is the most important file on the Scenic side
  - if stepping behavior changes, this is usually the first file to inspect

### Steering conversion

- `steer_io.py`
  - the single conversion point for road-wheel radians → dSPACE steering value

### Vehicle control

- `vehicle/controller.py`
  - ego control application
  - fellow control application
- `vehicle/physics.py`
  - fellow kinematic model

#### Fellow (v, d) plant behaviors (dSPACE External_Signals)

Some traffic fellows do not use throttle/steer from Scenic. Each step their behaviors **take**
`SetFellowPlantAction` (racing domain action), which stages `_fellow_plant_state` (`v_kmh`,
`d_m`) on the agent—same idea as ego driving actions staging `_control_state`. After
`executeActions`, `VehicleController.apply_fellow_control` detects plant fellows by behavior
flag `_fellow_vd_plant_enabled` and writes those values to
`Const_v_Fellows_External` / `Const_d_Fellows_External` without branching on individual
behavior types.

Numeric helpers live in `src/scenic/domains/racing/fellow/commands.py` (`compute_*`); see
`src/scenic/domains/racing/behaviors.scenic` for the behavior bodies.

| Behavior | Role | Example scene |
|----------|------|----------------|
| **FellowConstantSpeedTrackOffsetBehavior** | Constant `speed_mph` and lateral **d** from placement. | `examples/racing/dSPACE/constant_speed_fellow.scenic` |
| **FellowFollowTTLGeometricBehavior** | Constant **v** and lateral **d** from TTL δ(s) (Lap + optimal CSV). | `examples/racing/dSPACE/ttl_fellow.scenic` |
| **FellowSuddenStopIntervalBehavior** | Repeating cruise (`speed_mph`) then commanded **v = 0**; **d** tracks TTL δ(s). | `examples/combined/fellow_sudden_stop.scenic` |
| **FellowSwerveOutOfControlBehavior** | TTL cruise (`speed_mph`), then rate-limited swerve right/left in **d**, then stop; optional **stop_hold_d**. | `examples/combined/fellow_swerve_out_of_control.scenic` |

See also module docstrings in `scenic.domains.racing.fellow.commands`.

### Coordinate and route logic

- `geometry/coordinate_transform.py`
- `geometry/route_projection.py`
- `geometry/route_mapping.py`

### Placement / readback

- `modeldesk/placement.py`
- `modeldesk/traffic_object.py` — fellow **Traffic Object** asset (`TrafficObjectType.Activate` via ModelDesk COM)
- `modeldesk/authoring.py`
- `controldesk/readback.py`

### CoSim integration

- `cosim/`
  - contains the VEOS CoSim SDK-side material and the IPC bridge

---

## Current stepping model

There are now **two conceptual stepping modes**:

### 1. ControlDesk polling step mode
In this mode, Scenic performs a step and then polls simulated time until the step has advanced.

This mode is useful for:
- non-CoSim usage
- dSPACE-only workflows
- debugging ControlDesk timing

### 2. CoSim synchronous step mode
This is the newer approach used for Scenic ↔ VEOS synchronization.

In this mode:

- Scenic hosts a Python-side synchronization bridge
- the external IPC-enabled VEOS client connects back to Scenic
- VEOS blocks on each `TIME_TRIGGER`
- Scenic explicitly releases exactly one VEOS step
- VEOS advances one step and blocks again
- Scenic returns from `step()` only after VEOS is ready for the next step

This is the mode to use when the goal is:

> one Scenic step == one VEOS CoSim step

---

## Where the Scenic-side sync hook lives

The Scenic-side synchronization object is:

```text
cosim/veos_cosim_ipc_bridge/python_listener/sync_step_bridge.py
```

This file is not a direct VEOS SDK binding.  
Instead, it is a **Python coordination layer** that:

- listens for the IPC-enabled CoSim client
- receives `TIME_TRIGGER` notifications
- blocks until Scenic releases the next step
- sends `"STEP"` back to the VEOS-side client

`simulator.py` imports and uses this bridge.

---

## How synchronous stepping works now

### Scenic side
In `simulator.py`, `DSpaceSimulation` starts `SyncStepBridge`.

When `step()` is called in synchronous mode:

1. Scenic waits until VEOS is blocked at a `TIME_TRIGGER`
2. Scenic releases exactly one blocked step
3. VEOS advances
4. VEOS blocks again at the next `TIME_TRIGGER`
5. Scenic `step()` returns

### VEOS side
The external C++ CoSim client:
- receives `VeosCoSim_Command_TimeTrigger`
- sends a JSON message over localhost
- waits for `"STEP"`
- only then calls `VeosCoSim_FinishCommandMI()`

This makes Scenic the pacing master.

---

## Build / run overview for CoSim sync

The dSPACE simulator folder itself is not where the VEOS client is built.  
Instead, the VEOS pieces live under:

```text
src/scenic/simulators/dspace/cosim/
```

See:
- `cosim/README.md`
- `cosim/VeosCoSim_Client/README.md`
- `cosim/veos_cosim_ipc_bridge/README.md`
- `cosim/FINDINGS.md` §10 — Dennis's 2026-04 VEOS switches, VESI-init fix, minimum viable docker stack

### Minimal Scenic-only docker stack

[`dspace_scenic_stack.yml`](dspace_scenic_stack.yml) brings up only what Scenic
needs for CoSim runs: `vesi`, `veos`, `ctun`, `can-netns`, `asm_socketcan_bridge`,
and a `log_collector` sidecar. No `art_driving_stack`/raptor — just enough to
give VEOS's ExternalControl VPU a persistent CLIF client (the socketcan bridge
fills this role). Use it instead of the full `dspace_art_stack.yml` when you only
want Scenic to drive, without version-skew risk against raptor.

Requirements:
- Custom WSL2 kernel with `CONFIG_CAN_VCAN=y` — see `cosim/FINDINGS.md` §10.
- The `asm_socketcan_bridge_override.yaml` next to the compose file (already
  copied; keep in sync if the race_common copy gets updated).
- `dspace_bridge`'s DBC version must match what's shipped in the bridge image
  (`CAN1-INDY-V23.dbc` at time of writing; pinned in
  `asm_socketcan_bridge_override.yaml`).
- `DS_ROUTE_ID` must stay **unset** on the veos service. If set, VEOS overrides
  the ModelDesk-downloaded ego start pose at every MANEUVER_START, teleporting
  ego to a hardcoded route location. Keep the line commented in the yml —
  Scenic's `place_ego()` + `ts.Download()` is the authoritative spawn source.
  See `cosim/FINDINGS.md` §10 for the full write-verification trace.

### Required switch settings for Dennis's 2026-04-29 corrected CoSim VEOS

The canonical target is now `dspace_art_cosim_stack.yml` with
`Cosim-VEOS/ASM_Traffic.osa` (Dennis's 2026-04-29 OSA wiring fix in place).
The old non-CoSim `dspace_art_stack.yml` is retained as fallback only.

Both modes (Scenic-ego and ART-ego) verified end-to-end on the corrected OSA
on 2026-04-29. The switch values below differ from the canonical answers
Dennis gives because **our deployment is external Python + MAPort, not an
in-VEOS Scenic VPU** — see `cosim/FINDINGS.md` §12 for the full deployment-
topology rationale.

| Switch | Scenic-ego | ART-ego | Rationale |
|---|---|---|---|
| `Sw_Activate_CLIF[0\|1]/Value` | `0.0` (or `2.0` under CoSim) | `1.0` (or `2.0` under CoSim) | CoSim path needs `2.0` for `ManeuverTime` to advance (FINDINGS §6) |
| `Sw_Manual_VESI_Overwrite[0bridge\|1extern\|2scenic]/Value` | **`1.0`** (extern / software-joystick path) | **`0.0`** (bridge) | Scenic-ego uses `=1` because per-tick `Const_*_cmd` MAPort writes land at the OSA input port wired to enum `=1`. Dennis's canonical `=2` (scenic) routes through `ScenicControlInterface`, which assumes an in-VEOS Scenic VPU we don't run. ART-ego = bridge as canonical. |
| `Sw_RaceControl[0Intern\|1Extern\|2Orchestrator]/Value` | **`0.0`** (intern) | **`0.0`** (intern) | Dennis canon for Scenic is `=1` (extern) but expects an extern flag source from `ScenicControlInterface`. Intern + `Const_track_flag=1` holds the plant in green. ART canonically uses `=0` (intern) and lets the docker setflag handshake drive race state. |
| `Sw_MultiEgo_Fellows[0const\|1race\|2extern]/Sw_MultiEgo_Fellows` | **`0.0`** (const) | **`0.0`** (const) | `=0` routes fellow inputs to `Const_*_Fellows_External` MAPort arrays — what Scenic's fellow controller writes every tick. Dennis canon `=2` (extern) points at a different external bus on this OSA (verified 2026-04-29: with `=2` ego drove but fellows stayed stationary). |
| `Const_sys_state` (RaceControl) | `9` (running) | **not written** | ART-ego skips this write; the docker `setflag` handshake (`ASM_Maneuver.py vehicleflag_<N+1> trackflag_4`, fired automatically from `author_scenario` via `ExternalControlManager`) converges race-control state instead. |
| `Const_track_flag` (RaceControl) | `1` (green) | **not written** | Same — gated on Scenic-only via `if scenic_drives_ego:` in `initialize_vesi_interface`. |
| `Const_veh_flag` (RaceControl) | `0` (no flag) | **not written** | Same. |
| `Const_enable_*_cmd` (×4: throttle/brake/steer/gear) | `1` | `0` | Manual-MUX selector. `1` = honor `Const_*_cmd` writes (Scenic). `0` = bridge MUX wins (ART) — must be `0` or raptor's CAN inputs are dropped. |

Plus `controldesk/connection.py::initialize_vesi_interface()` must complete
all of its init steps — a stale switch path earlier in the method used to abort
the rest silently. Watch for `[ControlDesk] VesiInterface init summary: N ok,
0 failed` at setup; expect **15 ok / 0 failed** for Scenic-ego and **12 ok /
0 failed** for ART-ego (3 fewer because race-control Const_* writes are
gate-skipped). Any non-zero `failed` count leaves VESI in a half-enabled
state and ego won't respond.

### Running ART-driven ego with Scenic-controlled fellow

Both modes run on the same `dspace_art_cosim_stack.yml` — flip
`scene.params["scenic_control"]` (default `True`) to switch.

```bash
# Bring up the stack (WSL):
docker compose -f src/scenic/simulators/dspace/dspace_art_cosim_stack.yml up -d

# Then on Windows, start ctun.exe yourself.

# Run the scenario (model declared in the .scenic file):
scenic examples/racing/f_shared/F1_fellow_behind_optimal_cruise.scenic \
    --simulate --time 2000 --count 1 --2d -b
```

For ART-ego, edit the scenario file's `param scenic_control = True` to `False`
(or pick / clone a scenario that sets it to `False`). Scenic CLI's `--param`
flag for booleans is fiddly; an in-file flip is deterministic.

**Status (2026-04-29):** Both modes verified end-to-end on the corrected
CoSim OSA.

- **Scenic-ego**: MPC drove ego at ~60 mph; fellow drove alongside at the
  scenario's `speed_mph=58` target via `Const_*_Fellows_External` MAPort
  writes.
- **ART-ego**: raptor drove ego from idle to ~80 mph in 5 s, held the
  straight, bled speed approaching corners; fellow drove the same MAPort
  path as in Scenic mode.
- **Live divergence check on Scenic-ego** (which is the test that proves
  raptor's bridge writes are correctly bypassed): on the old VEOS we saw
  raptor commanding ~38% throttle while the plant ran at 100% under
  Scenic MPC. Same divergence is expected on CoSim and is the canonical
  signal that the manual-MUX gating is working.

The configuration (current code, see `cosim/FINDINGS.md` §12 for the
canonical record):

- `Sw_Manual_VESI_Overwrite` — `simulator.py` step 9b sets `1.0` for
  Scenic-ego and `0.0` for ART-ego via MAPort. Legacy `[0|1]` fallback path
  retained for pre-Dennis VEOS but unused on the canonical 2026-04-29 OSA.
- `Const_enable_*_cmd` — set by
  `initialize_vesi_interface(scenic_drives_ego=...)`: `1` for Scenic, `0`
  for ART.
- `Sw_RaceControl=0` (intern) for both modes. `Const_sys_state=9`,
  `Const_track_flag=1`, `Const_veh_flag=0` are written for Scenic-ego only;
  ART-ego skips them and lets the docker setflag handshake (`ASM_Maneuver.py`
  via `ExternalControlManager.enableExternalControlViaScript`) handle state
  convergence. The 2026-04-24 framing of "intern + green is a workaround
  for broken extern wiring" was rationale-incorrect: it's the canonical
  config for our external-Python+MAPort deployment topology, not a
  workaround. Full diagnosis in `cosim/FINDINGS.md` §12.

---

## Important runtime rule for CoSim sync

When using the synchronous VEOS stepping path:

- Scenic should start first
- Scenic must open the sync bridge before the IPC client connects
- then the IPC-enabled VEOS client should be launched
- only one VEOS client should be connected at a time

Do **not** run both:
- `VeosCoSimTestClient.exe`
- `VeosCoSimTestClientIpc.exe`

against the same VEOS server simultaneously.

---

## Practical startup order for Scenic + CoSim sync

### Option A — let Scenic launch the IPC client (recommended)

In your Scenic program or model, set:

```text
param launch_veos_ipc_client = True
```

(Or pass `launch_veos_ipc_client=True` to `DSpaceSimulator` from Python.)

During `setup()`, Scenic will:

1. Start the CoSim bridge on `127.0.0.1:50555` (override with `sync_bridge_host` / `sync_bridge_port`).
2. Spawn `VeosCoSimTestClientIpc.exe` from `cosim/veos_cosim_ipc_bridge/client/build/` (override with `veos_ipc_client_exe`).
3. Wait until the client connects (VEOS host/name: `veos_host`, `veos_cosim_server_name`).
4. Only then proceed with ModelDesk scenario setup (`SaveAs` / placement / `Download`).

Bridge behavior is selectable:

- `cosim_bridge_mode="sync_step"` (default): Scenic-paced lock-step (`STEP` reply on each blocked `TIME_TRIGGER`).
- `cosim_bridge_mode="print_time_callbacks"`: emulate `print_time_callbacks.py` behavior (reply `ACK` after optional delay).
- `cosim_time_trigger_ack_delay_s` controls the delay used by `print_time_callbacks` mode (default `3.0` seconds).

You should see:

```text
[CoSimSync] SyncStepBridge listening on 127.0.0.1:50555
[CoSimSync] Launching VEOS IPC client: ...
[CoSimSync] VEOS IPC client connected to SyncStepBridge.
```

On simulation shutdown, Scenic terminates that process.

Build the client once: `cosim\veos_cosim_ipc_bridge\build_client.bat`.

### Option B — CoSim off

If `launch_veos_ipc_client` is `False`, Scenic does **not** start `SyncStepBridge` or bind any CoSim socket. Simulation steps use ControlDesk / MAPort only. Use this when you do not need VEOS–Scenic synchronous stepping (for example to avoid Windows port-permission issues on the sync port).

### Option C — manual bridge + client (without a full Scenic run)

To experiment with the IPC client and `SyncStepBridge` outside the normal Scenic lifecycle, use `modeldesk_connection_test/test_modeldesk_connection.py --with-cosim` (see that script’s README) or start `SyncStepBridge` from Python yourself, then run `VeosCoSimTestClientIpc.exe` with matching `--ipc-host` / `--ipc-port`.

---

## Fellow vehicle asset (Traffic Object)

Every fellow Scenic creates in ModelDesk gets an explicit **Traffic Object** selection (the same field as in the ModelDesk Traffic Object browser). Implementation:

- Module: `modeldesk/traffic_object.py`
- API: `apply_fellow_traffic_object(fellow)` resolves a short asset name against `TrafficObjectType.AvailableElements` and calls `TrafficObjectType.Activate(full_path)`.
- Default asset name: `DEFAULT_FELLOW_TRAFFIC_OBJECT_BASENAME` in that file (currently **`IAC_Car_AIRacing`**, resolving to a path such as `Vehicles\IAC_Racecars\IAC_Car_AIRacing.tro` when present in the library).

Called from:

- `modeldesk/placement.py` (`place_fellow`, after route setup)
- `modeldesk/authoring.py` (`configure_fellow`)

To use a different default vehicle for all fellows, change **`DEFAULT_FELLOW_TRAFFIC_OBJECT_BASENAME`** in `traffic_object.py` (use the object name as shown in ModelDesk, not necessarily the full `.tro` path).

---

## ModelDesk / VEOS notes

In practice, the CoSim workflow also depends on the VEOS application / `.osa` used by ModelDesk.

If ModelDesk fails to download the scenario to VEOS or cannot load the correct application / `.sdf`, the VEOS CoSim server may never become reachable even though Scenic’s sync bridge is running correctly.

So if Scenic shows `CoSimSync` readiness but the IPC client cannot connect to VEOS, check:

- ModelDesk download success
- VEOS preload / unload logs
- the selected `.osa`
- whether the `.osa` supports the intended ModelDesk API / CoSim mode

---

## If you want to modify stepping behavior

The most important files are:

### Scenic-side step orchestration
- `simulator.py`

### Scenic-side VEOS sync gate
- `cosim/veos_cosim_ipc_bridge/python_listener/sync_step_bridge.py`

### VEOS-side gate release logic
- `cosim/veos_cosim_ipc_bridge/client/VeosCoSimTestClientIpc.cpp`

If the system is not stepping synchronously, inspect those three files first.

---

## If you want to modify placement / controls instead

Use these files first:

### placement
- `modeldesk/placement.py`
- `modeldesk/traffic_object.py` (fellow Traffic Object asset)

### readback
- `controldesk/readback.py`

### ego / fellow control
- `vehicle/controller.py`

### steering conversion
- `steer_io.py`

---

## Running Scenic with the dSPACE backend

Typical command:

```powershell
scenic examples/racing/fellow_fixed_placing.scenic --2d --model scenic.simulators.dspace.racing_model --simulate --time 10
```

The exact model / scenario can differ, but the general backend entry point remains the same.

---

## Summary

This folder is the Scenic-side control center for dSPACE.

If you are debugging:
- **placement** → inspect `modeldesk/placement.py`
- **fellow 3D asset (Traffic Object)** → inspect `modeldesk/traffic_object.py`
- **readback** → inspect `controldesk/readback.py`
- **control application** → inspect `vehicle/controller.py`
- **synchronous CoSim stepping** → inspect `simulator.py` and `cosim/veos_cosim_ipc_bridge/`
