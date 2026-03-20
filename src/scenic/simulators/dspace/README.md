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

### Coordinate and route logic

- `geometry/coordinate_transform.py`
- `geometry/route_projection.py`
- `geometry/route_mapping.py`

### Placement / readback

- `modeldesk/placement.py`
- `controldesk/readback.py`

### CoSim integration

- `cosim/`
  - contains the VEOS CoSim SDK-side material and the IPC bridge

---

## Current stepping model

There are now **two conceptual stepping modes**:

### 1. Legacy / non-blocking step mode
This is the older approach, where Scenic performs a step and then polls simulated time until the step has advanced.

This mode is still useful for:
- non-CoSim usage
- older dSPACE-only workflows
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

### Step 1 — start Scenic
Launch Scenic normally.

During setup, Scenic should print something like:

```text
[CoSimSync] SyncStepBridge listening on 127.0.0.1:50555
```

This means Scenic is ready to accept the IPC client connection.

### Step 2 — start the IPC-enabled CoSim client
From the IPC bridge build folder, run:

```powershell
.\VeosCoSimTestClientIpc.exe --host 192.168.100.101 --name CoSimServerScenic --ipc-host 127.0.0.1 --ipc-port 50555
```

### Step 3 — verify VEOS-side connection
The IPC client should:
- connect to Scenic’s sync bridge
- then connect to the VEOS CoSim server
- then begin participating in step-by-step synchronization

If VEOS is not ready yet, the client may fail to connect even though Scenic is already listening. In that case, the issue is on the VEOS startup side, not the Python-side sync bridge.

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
- **readback** → inspect `controldesk/readback.py`
- **control application** → inspect `vehicle/controller.py`
- **synchronous CoSim stepping** → inspect `simulator.py` and `cosim/veos_cosim_ipc_bridge/`
