# veos_cosim_ipc_bridge

This folder contains the custom IPC bridge that exposes VEOS CoSimulation events to Python.

This is the current Python-facing integration path.

---

## Important clarification

The Python side here is **not** a direct VEOS binding.

The Python script in Terminal 1 does **not** load `VeosCoSimApplStatic.lib` or call `VeosCoSim_ConnectMI()`.

Instead:

- the C++ IPC-enabled client connects to VEOS
- the C++ client forwards logs and events to Python over localhost TCP
- Python listens and prints / processes those events

So if you ask “which terminal is the Python binding file?” the answer is:

- **Terminal 1 runs the Python-facing file**
  - `python_listener/print_time_callbacks.py`
- **Terminal 2 runs the VEOS-connected file**
  - `client/build/VeosCoSimTestClientIpc.exe`

---

## Folder layout

### `client/`
Contains the custom C++ side of the bridge.

Important files:
- `VeosCoSimTestClientIpc.cpp`
- `TcpEventClient.h`
- `TcpEventClient.cpp`

### `python_listener/`
Contains the Python receiver.

Important file:
- `print_time_callbacks.py`

### `build_client.bat`
Build script for the IPC-enabled client

---

## Runtime architecture

```text
VEOS Server
    ⇅
VeosCoSimTestClientIpc.exe
    ⇅ localhost TCP
print_time_callbacks.py
```

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

## Terminal 1 — Python listener

Run:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\python_listener
py print_time_callbacks.py --host 127.0.0.1 --port 50555
```

What this does:
- opens a local TCP server on `127.0.0.1:50555`
- waits for the IPC-enabled client to connect
- prints JSON events from the client

Expected initial output:

```text
Starting local IPC listener on 127.0.0.1:50555 ...
Waiting for IPC bridge client to connect...
```

Expected after terminal 2 starts:

```text
IPC bridge connected from 127.0.0.1:xxxxx
[HELLO] {'event': 'HELLO', 'message': 'ipc connected'}
```

---

## Terminal 2 — IPC-enabled VEOS client

Run:

```powershell
cd C:\Users\bklfh\Documents\Scenic\Scenic\src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge\client\build
.\VeosCoSimTestClientIpc.exe --host 192.168.100.101 --name CoSimServerScenic --ipc-host 127.0.0.1 --ipc-port 50555
```

What this does:
1. connects to the Python listener
2. sends a `HELLO` event
3. connects to VEOS at `192.168.100.101`
4. starts the VEOS non-blocking loop
5. forwards logs and selected events to Python

---

## Important files and how to interface with them

### `client/VeosCoSimTestClientIpc.cpp`
This is the most important file in this folder.

It is where:
- command-line args are parsed
- the local IPC connection is opened
- the VEOS client handle is created
- `VeosCoSim_ConnectMI()` is called
- callbacks are registered
- the command loop runs
- JSON events are sent to Python

If you want to:
- add more event types
- forward signal values
- forward command metadata
- add Python-to-C++ control later

this is the first file to edit.

### `client/TcpEventClient.h` and `client/TcpEventClient.cpp`
These files implement the local TCP sender used by the C++ client.

If you want to:
- change the transport
- add reconnection logic
- add buffering
- add multi-client support

this is where you would work.

### `python_listener/print_time_callbacks.py`
This is the Python-facing entry point.

It:
- binds the listening socket
- accepts the connection from the C++ client
- parses newline-delimited JSON messages
- prints event information

If you want to:
- log to file
- dispatch to a Python callback system
- expose the data to Scenic
- turn this into a reusable Python module

this is the file to extend first.

---

## Current event flow

The bridge currently forwards events such as:
- `HELLO`
- `LOG`
- `START`
- `STOP`
- `TERMINATE`
- `TIME_TRIGGER` (depending on current C++ implementation)

In practice, the easiest way to verify end-to-end behavior is:

1. start Terminal 1
2. start Terminal 2
3. confirm `HELLO`
4. confirm `LOG`
5. then verify `TIME_TRIGGER`

---

## Important rule: do not run the original client at the same time

Do not run both:
- `VeosCoSimTestClient.exe`
- `VeosCoSimTestClientIpc.exe`

at the same time against the same VEOS server.

Use one or the other.

For the Python-observable workflow, use only:

```text
VeosCoSimTestClientIpc.exe
```

---

## Troubleshooting

### Terminal 1 never shows “IPC bridge connected”
That means terminal 2 never connected to the Python listener.

Check:
- terminal 2 is running the IPC-enabled EXE
- `--ipc-host 127.0.0.1 --ipc-port 50555` are present
- firewall is not blocking localhost

### Terminal 1 shows `HELLO` and `LOG`, but no `TIME_TRIGGER`
That means:
- local IPC is working
- VEOS connection is working
- but timer events are not being forwarded yet

Then inspect:
- `OnTimeTriggerCallback`
- the `GetNextCommandMI` loop
- event sending logic in `VeosCoSimTestClientIpc.cpp`

### Terminal 2 connects to IPC but VEOS connection fails
Check:
- `--host 192.168.100.101`
- `--name CoSimServerScenic`
- VEOS server is running
- no competing client is already connected

---

## Next extension path

If you eventually want a richer Python API, the recommended path is:

1. keep Terminal 2 as the only VEOS-connected process
2. extend the JSON protocol sent over localhost
3. upgrade `print_time_callbacks.py` into a real Python module or service

That keeps the single-client VEOS constraint intact while still giving Python full visibility.
