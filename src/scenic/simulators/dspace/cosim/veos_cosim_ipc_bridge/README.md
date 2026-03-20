# veos_cosim_ipc_bridge

This folder contains the custom IPC bridge that exposes VEOS CoSimulation events and step boundaries to Scenic/Python.

This README is for:

```text
src/scenic/simulators/dspace/cosim/veos_cosim_ipc_bridge/README.md
```

This is the current Scenic-facing CoSim integration path.

---

## Important clarification

The Python side here is **not** a direct VEOS SDK binding.

The Python side does **not** create its own VEOS client connection.

Instead:

- the C++ IPC-enabled client connects to VEOS
- the C++ client forwards messages and waits for step releases over localhost TCP
- Scenic hosts the Python-side synchronization gate
- Scenic decides when the next VEOS step is allowed

So if you ask “which side is the Python-facing control point now?” the answer is:

- **Scenic process**
  - runs `python_listener/sync_step_bridge.py`
- **External terminal**
  - runs `client/build/VeosCoSimTestClientIpc.exe`

The older manual debug script:

```text
python_listener/print_time_callbacks.py
```

is still useful for inspection, but it is no longer the main stepping path.

---

## Folder layout

### `client/`
Contains the custom C++ side of the bridge.

Important files:
- `VeosCoSimTestClientIpc.cpp`
- `TcpEventClient.h`
- `TcpEventClient.cpp`

### `python_listener/`
Contains the Python-side bridge code.

Important files:
- `sync_step_bridge.py`
- `print_time_callbacks.py`

### `build_client.bat`
Build script for the IPC-enabled client

---

## Runtime architecture

### Current synchronous stepping design

```text
Scenic process
  └─ SyncStepBridge (Python)
        ⇅ localhost TCP
VeosCoSimTestClientIpc.exe
        ⇅ CoSim
VEOS Server
```

This means:

- Scenic is the step authority
- VEOS blocks at `TIME_TRIGGER`
- Scenic releases one step at a time
- VEOS advances one step and blocks again

---

## Build the IPC bridge

Run this from:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge
```

Then build:

```powershell
.\build_client.bat
```

Expected output EXE:

```text
veos_cosim_ipc_bridge\client\build\VeosCoSimTestClientIpc.exe
```

---

## How to run it now

## Step 1 — start Scenic
Launch Scenic first.

Scenic should print something like:

```text
[CoSimSync] SyncStepBridge listening on 127.0.0.1:50555
```

That means Scenic is listening for the IPC client.

## Step 2 — start the IPC-enabled VEOS client
Run:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\client\build
.\VeosCoSimTestClientIpc.exe --host 192.168.100.101 --name CoSimServerScenic --ipc-host 127.0.0.1 --ipc-port 50555
```

What this does:
1. connects to Scenic’s sync bridge
2. connects to VEOS at `192.168.100.101`
3. enters the VEOS non-blocking command loop
4. waits for Scenic to release each `TIME_TRIGGER`

---

## Important files and how to interface with them

### `client/VeosCoSimTestClientIpc.cpp`
This is the most important file on the C++ side.

It is where:
- command-line args are parsed
- the local IPC connection is opened
- the VEOS client handle is created
- `VeosCoSim_ConnectMI()` is called
- callbacks are registered
- the command loop runs
- `TIME_TRIGGER` commands are held until Scenic releases them

If you want to:
- change what VEOS sends to Scenic
- change the handshake protocol
- forward more data
- add retries or better startup behavior

this is the first file to edit.

### `client/TcpEventClient.h` and `client/TcpEventClient.cpp`
These files implement the local TCP client used by the C++ process.

They handle:
- connect to Scenic’s bridge
- send newline-delimited JSON messages
- wait for reply lines such as `STEP`

If you want to:
- change the transport
- add retries
- add buffering
- add more structured protocol behavior

this is where you would work.

### `python_listener/sync_step_bridge.py`
This is the Scenic-hosted synchronization gate.

It is the most important file on the Python side for actual stepping.

It:
- opens the localhost server
- accepts the IPC-enabled client
- receives `TIME_TRIGGER`
- blocks until Scenic calls `step()`
- sends `"STEP"` back to release exactly one VEOS step

If you want to:
- change the step semantics
- add richer synchronization state
- expose more Python-side control

this is the first file to edit.

### `python_listener/print_time_callbacks.py`
This is the debug / inspection listener.

It is useful when you want to:
- manually inspect messages
- test the C++ side without Scenic
- verify logs and timer traffic

It is **not** the primary synchronization path anymore.

---

## Current step handshake

The current design is:

1. VEOS reaches `TIME_TRIGGER`
2. `VeosCoSimTestClientIpc.exe` sends a JSON step-boundary message
3. Scenic’s `SyncStepBridge` records that VEOS is blocked and ready
4. Scenic calls its own `step()`
5. `SyncStepBridge` replies with `STEP`
6. IPC client calls `VeosCoSim_FinishCommandMI()`
7. VEOS advances one step
8. VEOS blocks again at the next `TIME_TRIGGER`

This gives:

> one Scenic step == one VEOS CoSim step

---

## Important rule: do not run the original client at the same time

Do not run both:
- `VeosCoSimTestClient.exe`
- `VeosCoSimTestClientIpc.exe`

at the same time against the same VEOS server.

Use only:

```text
VeosCoSimTestClientIpc.exe
```

for the Scenic-synchronized workflow.

---

## Troubleshooting

### IPC client says `Failed to connect to listener`
That means Scenic has not opened the sync bridge yet.

Check:
- Scenic is running
- Scenic printed the `CoSimSync` message
- port `50555` is listening locally

### IPC client connects to Scenic, but cannot connect to VEOS
That means:
- Scenic side is ready
- localhost IPC is fine
- but the VEOS CoSim server is not accepting connections yet

Check:
- ModelDesk successfully downloaded the scenario to VEOS
- the selected `.osa` is correct
- VEOS successfully loaded the application / `.sdf`
- the CoSim server is actually up

This is often a VEOS startup-order problem, not a Python-side bridge problem.

### Scenic is listening, VEOS is up, but steps do not advance
Then inspect:
- `sync_step_bridge.py`
- `VeosCoSimTestClientIpc.cpp`
- the `TIME_TRIGGER` branch before `VeosCoSim_FinishCommandMI()`

---

## Next extension path

If you eventually want a richer Python / Scenic API, the recommended path is:

1. keep the IPC client as the only VEOS-connected process
2. extend the JSON / reply protocol
3. let Scenic own more of the high-level step, signal, and synchronization logic

That keeps the single-client VEOS constraint intact while still giving Scenic full control over pacing.
