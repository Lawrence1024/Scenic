# CoSim Integration Overview

This folder is the top-level home for the VEOS CoSimulation integration used by Scenic.

This README is for:

```text
src/scenic/simulators/dspace/cosim/README.md
```

The important subfolders are:

- `VeosCoSim_Client/`
  - the vendor CoSim SDK and example client sources
- `veos_cosim_ipc_bridge/`
  - the custom bridge used to synchronize VEOS stepping with Scenic over localhost IPC

## Other docs in this folder

- [`FINDINGS.md`](FINDINGS.md) — empirical notes from the CoSim investigation:
  variable-access backends (MAPort vs ControlDesk COM vs CoSim bus), data-type
  pitfalls (UINT vs FLOAT), why `VESIResultData_Manual` writes don't reach the
  plant under CoSim, why `initialize_vesi_interface` writes don't persist, the
  distinction between `SimulationTime` and `ManeuverTime`, and the open question
  about what COM call corresponds to the "Start" button in ModelDesk. Read this
  if you're debugging "I wrote the value, readback confirms it, but the vehicle
  isn't responding."
- [`veos_cosim_ipc_bridge/README.md`](veos_cosim_ipc_bridge/README.md) —
  step-handshake protocol details, including the JSON reply envelope that carries
  per-tick command values to the bus outports.

---

## Important clarification: what is the “Python API” here?

In the current working setup, there is **not** a direct Python SDK binding that talks to VEOS on its own.

Instead, the Python-facing control path is now split into two layers:

### 1. Scenic-hosted sync bridge
Inside Scenic, the Python-side step gate lives in:

```text
veos_cosim_ipc_bridge/python_listener/sync_step_bridge.py
```

Scenic imports this file and runs it internally.

### 2. Optional standalone Python listener
For debugging / inspection, there is also:

```text
veos_cosim_ipc_bridge/python_listener/print_time_callbacks.py
```

That script is useful for manual testing, but it is **not** the primary stepping path anymore.

So for actual Scenic synchronization:

- there is no separate “Terminal 1 Python listener” anymore
- Scenic itself becomes the Python side of the handshake
- Terminal 2 remains the VEOS-connected C++ client

---

## High-level architecture

### Current synchronous Scenic ↔ VEOS design

```text
Scenic process
  └─ SyncStepBridge (Python)
        ⇅ localhost TCP
VeosCoSimTestClientIpc.exe (C++)
        ⇅ CoSim
VEOS Server
```

This means:

- only the IPC-enabled C++ client talks to VEOS
- Scenic does not create its own CoSim client connection
- Scenic releases exactly one VEOS step at a time

---

## Why this architecture was chosen

Earlier attempts tried to create a direct Python wrapper around the VEOS client API. In practice, that caused problems:

1. the Python wrapper still created a separate logical CoSim client connection
2. in your environment, opening another CoSim connection could interfere with the active VEOS session

So the safer architecture is:

- one VEOS-connected client only
- that client is written in C++
- Scenic communicates with that client locally over IPC
- the step boundary is controlled from Python without creating another CoSim session

---

## What each subfolder is for

### `VeosCoSim_Client/`
This is the vendor side.

It contains:
- `client/x64/Release/include/VeosCoSim.h`
- `client/x64/Release/lib/VeosCoSimApplStatic.lib`
- `examples/client/VeosCoSimTestClient.cpp`
- vendor helper files

This folder is the authoritative SDK / reference implementation.

### `veos_cosim_ipc_bridge/`
This is the custom layer built on top of the vendor SDK.

It contains:
- a modified client executable source
- a small TCP sender used by that client
- the Python-side synchronization / debug listener code

This folder is the main place to extend if you want new Scenic-visible CoSim behavior.

---

## Build instructions

## 1. Build the main example client

Use this when you want to rebuild the original VEOS example client and verify the SDK/source setup works.

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\examples\client
cl /std:c++17 /EHsc /MD ^
  /I "C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\client\x64\Release\include" ^
  VeosCoSimTestClient.cpp ClientServerTestHelper.cpp Generator.cpp ^
  /link ^
  /LIBPATH:"C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\client\x64\Release\lib" ^
  VeosCoSimApplStatic.lib Ws2_32.lib ^
  /OUT:"VeosCoSimTestClient.exe"
```

### What this build proves
If this EXE builds and runs successfully, then:
- your source tree is usable
- your include/lib paths are correct
- the VEOS client SDK is aligned enough to produce a working client in your environment

---

## 2. Build the IPC bridge client

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge
.\build_client.bat
```

This produces the IPC-enabled client EXE in:

```text
veos_cosim_ipc_bridge\client\build\VeosCoSimTestClientIpc.exe
```

---

## CoSim startup recipe (programmatic — no manual ModelDesk clicks)

The full sequence required before the ego vehicle moves under CoSim:

1. **VEOS IPC client connects** and gate enters AUTO mode (TIME_TRIGGERs flow freely).
2. **Read `Sw_Activate_CLIF` baseline** via MAPort (usually `0.0`); store for teardown restore.
3. **Write `Sw_Activate_CLIF = 2.0`** via MAPort — must happen **while gate is AUTO**.
   - The variable name `[0|1]` is misleading; `2.0` is the correct activation value.
   - COM writes (ControlDesk) do not persist under CoSim (see `FINDINGS.md §4`); MAPort is required.
4. **Pulse `MANEUVER_START`** via MAPort (`1.0` → sleep 0.5s → `0.0`).
   - `connection.py::start_maneuver(var_access=maport)` implements this.
   - Without step 3 first, the pulse has no effect.
5. **Verify `ManeuverTime > 0`** — if it stays at 0.0 after 3s, the maneuver engine isn't running.
6. At **teardown**: write `Sw_Activate_CLIF` back to the captured baseline so the next run sees
   a clean `0 → 2` transition.

`simulator.py` step 9 handles steps 2–4 and 6 automatically when `launch_veos_ipc_client=True`.
See `FINDINGS.md §6` for the full empirical story.

---

## Current runtime model

### Scenic synchronous stepping path

#### Scenic side
Scenic starts `SyncStepBridge` internally and listens on localhost.

Scenic should log something like:

```text
[CoSimSync] SyncStepBridge listening on 127.0.0.1:50555
```

That means Scenic is ready for the VEOS IPC client.

#### VEOS side

**Automatic (Scenic):** set `launch_veos_ipc_client=True` on `DSpaceSimulator` (or `param launch_veos_ipc_client = True` in `model.scenic`). That enables the full CoSim path: Scenic starts a localhost bridge, spawns `VeosCoSimTestClientIpc.exe`, and waits for the connection. If `launch_veos_ipc_client=False`, Scenic does **not** start the bridge or any CoSim resources (ControlDesk stepping only). See `simulator.py` and `model.scenic`.

Bridge modes:

- `cosim_bridge_mode="sync_step"` (default): lock-step pacing (`STEP` reply; one Scenic step == one VEOS step).
- `cosim_bridge_mode="print_time_callbacks"`: emulates `python_listener/print_time_callbacks.py` behavior (`ACK` replies, optional delay).
- `cosim_time_trigger_ack_delay_s`: delay before ACK in `print_time_callbacks` mode (default: `3.0`).

**Manual:** launch the IPC-enabled client in a separate terminal:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\client\build
.\VeosCoSimTestClientIpc.exe --host 192.168.100.101 --name CoSimServerScenic --ipc-host 127.0.0.1 --ipc-port 50555
```

What this process does:

- connects to Scenic’s localhost sync bridge
- then connects to the VEOS CoSim server at `192.168.100.101`
- waits on each `TIME_TRIGGER` until Scenic releases the next step

---

## Important runtime rule

Do **not** run both of these at the same time:
- original `VeosCoSimTestClient.exe`
- `VeosCoSimTestClientIpc.exe`

Only one VEOS client should be connected to the VEOS server at a time.

---

## Important files to know

### In `VeosCoSim_Client/`
- `examples/client/VeosCoSimTestClient.cpp`
  - original example client main program
- `examples/client/ClientServerTestHelper.cpp`
- `examples/client/Generator.cpp`
- `client/x64/Release/include/VeosCoSim.h`
  - core SDK header
- `client/x64/Release/lib/VeosCoSimApplStatic.lib`
  - static library used for linking

### In `veos_cosim_ipc_bridge/`
- `build_client.bat`
  - build script for the IPC-enabled client
- `client/VeosCoSimTestClientIpc.cpp`
  - main C++ file for the IPC-enabled client
- `client/TcpEventClient.h`
- `client/TcpEventClient.cpp`
  - local TCP sender / reply receiver used for step gating
- `python_listener/sync_step_bridge.py`
  - Scenic-hosted synchronization gate
- `python_listener/print_time_callbacks.py`
  - standalone debug listener

---

## How to interface with the important files

### If you want to change how VEOS stepping is synchronized with Scenic
Edit:

```text
veos_cosim_ipc_bridge/python_listener/sync_step_bridge.py
veos_cosim_ipc_bridge/client/VeosCoSimTestClientIpc.cpp
```

This pair implements the actual step handshake.

### If you want to change what the IPC client sends or waits for
Edit:

```text
veos_cosim_ipc_bridge/client/TcpEventClient.h
veos_cosim_ipc_bridge/client/TcpEventClient.cpp
veos_cosim_ipc_bridge/client/VeosCoSimTestClientIpc.cpp
```

### If you want to debug messages manually without Scenic
Use:

```text
veos_cosim_ipc_bridge/python_listener/print_time_callbacks.py
```

### If you want to compare against the vendor client
Read:

```text
VeosCoSim_Client/examples/client/VeosCoSimTestClient.cpp
```

---

## Troubleshooting

### Scenic prints `CoSimSync`, but the IPC client cannot connect to VEOS
That means:
- Scenic-side listener is ready
- localhost IPC is probably fine
- but VEOS / ModelDesk / `.osa` is not yet ready for CoSim

Check:
- ModelDesk successfully downloaded the scenario to VEOS
- VEOS successfully loaded the expected application
- the chosen `.osa` supports the intended ModelDesk / CoSim workflow

### The IPC client says `Failed to connect to listener`
That means Scenic has not started listening yet, or is not using the updated sync path.

Check:
- Scenic is actually running the updated `simulator.py`
- Scenic printed the `CoSimSync` line
- localhost port `50555` is open

### The IPC client connects to Scenic, but no steps happen
Then inspect:
- `SyncStepBridge.step()`
- the `TIME_TRIGGER` command branch in `VeosCoSimTestClientIpc.cpp`

---

## Summary

The most important thing to remember now is:

- Scenic itself hosts the Python synchronization gate
- the external IPC-enabled C++ client is the only VEOS-connected process (Scenic **spawns** it when `launch_veos_ipc_client=True`; when `False`, Scenic does not run CoSim at all)
- synchronous stepping is implemented by blocking VEOS at `TIME_TRIGGER` until Scenic releases one step
