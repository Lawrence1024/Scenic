# CoSim Integration Overview

This folder is the top-level home for the VEOS CoSimulation integration used by Scenic.

The important subfolders are:

- `VeosCoSim_Client/`
  - the vendor CoSim SDK and example client sources
- `veos_cosim_ipc_bridge/`
  - the custom bridge that lets Python observe VEOS events without opening a second CoSim connection

---

## Important clarification: where is the “Python binding”?

In the current working setup, there is **no direct Python binding to VEOS**.

Instead, the Python-facing piece is the **IPC listener script**:

```text
veos_cosim_ipc_bridge/python_listener/print_time_callbacks.py
```

That script runs in **Terminal 1**.

So:

- **Terminal 1** = Python listener
- **Terminal 2** = C++ VEOS client with IPC enabled

This means Python does **not** call `VeosCoSim_ConnectMI()` directly.  
Only the IPC-enabled C++ client connects to VEOS.

That design is intentional, because earlier attempts to connect from Python as a separate CoSim client could interfere with the existing VEOS session.

---

## High-level architecture

The runtime data flow is:

```text
VEOS Server
    ⇅
VeosCoSimTestClientIpc.exe   (C++ client)
    ⇅ localhost TCP
print_time_callbacks.py      (Python listener)
```

### Why this architecture was chosen

We originally explored the idea of building a direct Python wrapper around the VEOS client library. In practice, that caused two problems:

1. The Python wrapper still had to create its own logical CoSim connection.
2. In your environment, a second CoSim connection could disrupt or terminate the active session.

So the safer design is:

- only **one** VEOS client connects to the VEOS server
- that client is written in C++
- Python only listens locally for forwarded events

---

## What each folder is for

### `VeosCoSim_Client/`
This is the vendor side.

It contains:
- `client/x64/Release/include/VeosCoSim.h`
- `client/x64/Release/lib/VeosCoSimApplStatic.lib`
- `examples/client/VeosCoSimTestClient.cpp`
- helper files used by the example client

You should treat this folder as the authoritative SDK / reference implementation.

### `veos_cosim_ipc_bridge/`
This is the custom layer built on top of the vendor SDK.

It contains:
- a modified client executable source
- a small TCP sender used by that client
- a Python listener that receives and prints callback events

This folder is the place to extend if you want Python-facing behavior.

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

## Run instructions

### Terminal 1 — Python listener

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\python_listener
py print_time_callbacks.py --host 127.0.0.1 --port 50555
```

What this terminal does:
- binds a local TCP server at `127.0.0.1:50555`
- waits for the IPC-enabled VEOS client to connect
- prints JSON events it receives

Expected early output:

```text
Starting local IPC listener on 127.0.0.1:50555 ...
Waiting for IPC bridge client to connect...
```

After terminal 2 starts successfully, you should see:

```text
IPC bridge connected from 127.0.0.1:xxxxx
[HELLO] {'event': 'HELLO', 'message': 'ipc connected'}
```

### Terminal 2 — IPC-enabled VEOS client

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\client\build
.\VeosCoSimTestClientIpc.exe --host 192.168.100.101 --name CoSimServerScenic --ipc-host 127.0.0.1 --ipc-port 50555
```

What this terminal does:
- connects to the local Python listener first
- then connects to the VEOS server at `192.168.100.101`
- forwards logs and callback/command events to Terminal 1

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
  - helper utilities from vendor example
- `examples/client/Generator.cpp`
  - vendor example support file
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
  - local TCP sender used to forward events to Python
- `python_listener/print_time_callbacks.py`
  - Python receiver / observer

---

## How to interface with the important files

### If you want to change what Python sees
Edit:

```text
veos_cosim_ipc_bridge/client/VeosCoSimTestClientIpc.cpp
```

This is where:
- VEOS callbacks are registered
- VEOS commands are polled
- JSON events are sent to Python

### If you want to change how Python prints or processes events
Edit:

```text
veos_cosim_ipc_bridge/python_listener/print_time_callbacks.py
```

This is the Python entry point used in Terminal 1.

### If you want to inspect or compare against the vendor client
Read:

```text
VeosCoSim_Client/examples/client/VeosCoSimTestClient.cpp
```

That is the baseline implementation for the vendor client.

---

## Troubleshooting

### Terminal 1 says “Waiting for IPC bridge client to connect...” forever
That means terminal 2 did not connect to the listener.

Check:
- are you running `VeosCoSimTestClientIpc.exe`, not the original EXE?
- did you pass `--ipc-host 127.0.0.1 --ipc-port 50555`?
- did the IPC client print `[ipc] Connected to listener.`?

### Terminal 2 connects to IPC, but no timer events appear
That means:
- VEOS connection succeeded
- Python IPC succeeded
- but no `TIME_TRIGGER` events are being forwarded yet

In that case inspect:
- `OnTimeTriggerCallback`
- the `GetNextCommandMI` loop in `VeosCoSimTestClientIpc.cpp`

### VEOS session stops unexpectedly
That usually means multiple clients touched the same server/session.  
Make sure only the IPC-enabled client is active for the test.

---

## Summary

The most important thing to remember is:

- **Terminal 1 is the Python-facing side**
- **Terminal 2 is the VEOS-connected side**
- Python is currently **not** a direct VEOS binding
- Python receives events through local IPC from the C++ client
