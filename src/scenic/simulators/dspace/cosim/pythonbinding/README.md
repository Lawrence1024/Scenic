# VeosCoSim Python binding (`veos-cosim`)

This folder provides **Python bindings** to the dSPACE **VeosCoSim** client C API (`VeosCoSimAppl`). You can drive VEOS co-simulation from Python (e.g. Scenic) using the same library as `VeosCoSimTestClient.cpp`, with a blocking `run(..., time_trigger=...)` entry point aligned to `VeosCoSim_RunMI`.

**Scope (current repo):**

- **Platform:** Windows **x64** only (matches `VeosCoSim_Client/client/x64/...`).
- **Native library:** `VeosCoSimAppl.dll` (+ import library `VeosCoSimAppl.lib`), produced with the VeosCoSim client SDK under `../VeosCoSim_Client/`.
- **Binding technology:** [pybind11](https://github.com/pybind/pybind11) (see `pyproject.toml`, `setup.py`, `src/veos_cosim_binding.cpp`).

The extension module is **`veos_cosim._veos_cosim`**; the importable package is **`veos_cosim`**.

---

## Layout

| Path | Purpose |
|------|---------|
| `pyproject.toml` | Project metadata, build-system (`setuptools` + `pybind11`). |
| `setup.py` | Declares the `Pybind11Extension` for `_veos_cosim`. |
| `src/veos_cosim_binding.cpp` | pybind11 module: `CoSimClient` (`connect`, `connect2`, `run` with `time_trigger`, IO/channel queries). |
| `veos_cosim/__init__.py` | Re-exports `CoSimClient` and `__version__`. |
| `scripts/time_trigger_demo.py` | **Manual test** — connect, print IO list, blocking `run()` with decimated control prints. |
| `scripts/cpp_client_bridge_demo.py` | Talk to a running `VeosCoSimTestClient.exe` process over localhost bridge TCP. |
| `MANIFEST.in` | Ensures C++ sources ship in sdists. |

---

## Manual test script (`scripts/time_trigger_demo.py`)

1. Build/install the package (Debug client libs by default; use **`VEOSCOSIM_CONFIG=Release`** for Release)::

       cd src/scenic/simulators/dspace/cosim/pythonbinding
       pip install -e .

2. Start your VEOS co-simulation server (e.g. same host/name as `VeosCoSimTestClient.exe`).

3. Run from the **Scenic repo root** (or any directory **other** than `pythonbinding/` alone, so Python does not pick up the source tree without the `.pyd`)::

       python src/scenic/simulators/dspace/cosim/pythonbinding/scripts/time_trigger_demo.py

   Options: `--host`, `--name`, `--dll-dir` (folder containing `VeosCoSimAppl.dll`), `--remote-port`, `--local-port`.

   For tunnel / custom port setups, use explicit connection ports (maps to `VeosCoSim_ConnectMI2`):

       python src/scenic/simulators/dspace/cosim/pythonbinding/scripts/time_trigger_demo.py --host 192.168.100.101 --name CoSimServerScenic --remote-port 12345

**Expected:** connect succeeds; IO signals and bus channels are listed; `Simulation start` / `[control] …` lines appear during the run (control lines about every **0.05 s** simulated time while the trigger still fires every step); `Simulation stop` and a final `Run finished` message with a **result code** from `VeosCoSim_RunMI` (treat per dSPACE docs — may still indicate error when behavior was fine).

**If the server is unreachable:** `connect()` raises (e.g. `VeosCoSim_Result=1` / connection refused); VeosCoSim log lines appear on **stderr**.

---

## Running client bridge mode (C++ client stays primary)

If you want Scenic/Python to interact with an already running C++ client process (instead of creating a second client connection), use bridge mode.

1. Start the C++ client with bridge enabled:

       .\src\scenic\simulators\dspace\cosim\VeosCoSim_Client\examples\client\VeosCoSimTestClient.exe --host 192.168.100.101 --name CoSimServerScenic --bridge-port 17071

2. Run the Python bridge demo:

       python src/scenic/simulators/dspace/cosim/pythonbinding/scripts/cpp_client_bridge_demo.py --port 17071 --steps 20

Current bridge protocol (line-based TCP on `127.0.0.1`): `PING`, `STEP`, `QUIT`.

- `STEP` blocks until the next `timeTriggerCallback` in the C++ client and returns `STEP <simulation_time_ns>`.
- This keeps synchronization driven by the existing C++ client’s callback loop.

---

## What we learned from `VeosCoSim_Client`

### Official C API location

The public C API is **`VeosCoSim.h`**. In this tree, use the copy next to the prebuilt binaries, e.g.:

- `../VeosCoSim_Client/client/x64/Debug/include/VeosCoSim.h`
- `../VeosCoSim_Client/client/x64/Release/include/VeosCoSim.h`

It declares **C linkage** functions (`VEOSCOSIM_DECL`) and structs for IO, CAN/LIN/ETH messages, and runtime configuration.

### Runtime binary

- **`VeosCoSimAppl.dll`** — load this at runtime (PATH, or place next to the built `.pyd`, or set `os.add_dll_directory(...)` on Python 3.8+).
- **`VeosCoSimAppl.lib`** — link the extension **against** the import library when compiling the pybind11 module on Windows.

Debug vs Release: match CRT / ABI expectations with the toolchain you use to **build** the extension; the DLL you load at runtime should be consistent with your deployment.

### Multi-instance API (“MI”)

The C++ example uses the **MI** (multi-instance) entry points: `VeosCoSim_CreateMI`, `VeosCoSim_ConnectMI`, `VeosCoSim_RunMI`, `VeosCoSim_DisconnectMI`, etc. A `VeosCoSim_Handle` is created first; all MI calls take that handle.

Reference flow (same idea as `examples/client/VeosCoSimTestClient.cpp`):

1. **`VeosCoSim_CreateMI()`** → handle.
2. **`VeosCoSim_ConnectMI(handle, host, serverName, logCallback)`** or **`VeosCoSim_ConnectMI2(handle, config)`** — e.g. host `192.168.100.101`, name `CoSimServerScenic` (must match server configuration). `ConnectMI2` also supports explicit `remotePort` / `localPort`.
3. **`VeosCoSim_GetAvailableChannelsMI`** / **`VeosCoSim_IoGetAvailableSignalsMI`** — discover bus controllers and IO signals (names, ids, `VeosCoSim_DataType`, `VeosCoSim_Direction`, length, `VeosCoSim_SizeKind`).
4. **`VeosCoSim_RunMI(handle, VeosCoSim_RuntimeConfiguration)`** — **blocking**; the library invokes registered callbacks for start/stop/time trigger, IO read, and bus receive.
5. **`VeosCoSim_IoReadMI` / `VeosCoSim_IoWriteMI`** — read/write scalar or vector signals (length is element count; buffer size = `length * sizeof(element)` per type).
6. **`VeosCoSim_DisconnectMI`** when tearing down.

There is also a **non-blocking** API (`StartNonBlockingMI`, `GetNextCommandMI`, `FinishCommandMI`). The intended Scenic integration discussed so far favors **`RunMI` + `timeTriggerCallback`** for step synchronization.

### Time and simulation step

- **`VeosCoSim_Time`** is **`int64_t`**; **`VEOSCOSIM_TIME_RESOLUTION_PER_SECOND`** is **`1e9`** (nanoseconds per second of simulated time).
- Example: `ClientServerTestHelper.cpp` converts to seconds with `simulationTime / 1e9`.

### `timeTriggerCallback` and control cadence

Documentation (bundled `VeosCoSim.html`) states that **`timeTriggerCallback` is invoked every cycle for each step time** and is the **last callback for that virtual simulation time step**.

The reference server JSON `../VeosCoSim_Client/examples/cosim_server_scenic.json` sets **`SampleTime`: `0.01`** (10 ms). If Scenic’s control period is **0.05 s**, implement **decimation inside** `timeTriggerCallback`: e.g. only call `IoReadMI` / `IoWriteMI` and Scenic control logic when simulation time crosses 50 ms boundaries (or every 5th trigger), and return quickly on other steps. Values written typically **hold** until the next write at the coarser rate.

### IO direction naming (easy to confuse)

In `VeosCoSim.h`:

- **`VeosCoSim_Direction_Read`** — **read** signal (client **reads** from the co-simulation / `VeosCoSim_IoReadMI`).
- **`VeosCoSim_Direction_Write`** — **write** signal (client **writes** with `VeosCoSim_IoWriteMI`).

The example `cosim_server_scenic.json` lists **`IoInputSignals`** and **`IoOutputSignals`** from the **server** naming perspective; map them to **Read/Write** in the API by checking **`VeosCoSim_IoSignalInfo.direction`** at runtime rather than assuming JSON names alone.

### `VeosCoSim_RunMI` return value

Per the same bundled documentation, **`VeosCoSim_RunMI` may always return `VeosCoSim_Result_Error` due to a current VEOS limitation**, even when behavior is otherwise correct. Treat logs and observable behavior as the source of truth, not only that return code.

### Example client and helpers

- **`examples/client/VeosCoSimTestClient.cpp`** — end-to-end: connect, list channels/signals, `RunMI`, demonstrates `IoWriteMI` and bus transmit in `timeTriggerCallback`, and IO/bus receive callbacks.
- **`examples/client/ClientServerTestHelper.cpp`** — **`GetDataTypeSize`**, **`TimeToSeconds`**, logging helpers; useful reference when packing/unpacking buffers in Python.

---

## Building (developer)

**Prerequisites**

- Windows x64, **Visual Studio Build Tools** (or VS) with C++ workload.
- **Python** ≥ 3.8 (aligned with Scenic’s `requires-python`).
- **`pybind11`** (pulled in as a build dependency via `pyproject.toml`).

**Steps**

From this directory:

```bash
pip install .
```

Or editable:

```bash
pip install -e .
```

**Linking:** `setup.py` resolves `../VeosCoSim_Client/client/x64/<Config>/include` and `.../lib`, links `VeosCoSimAppl.lib`, and defines `VEOSCOSIM_IMPORT`. Override layout with environment variables **`VEOSCOSIM_PLATFORM`** (default `x64`) and **`VEOSCOSIM_CONFIG`** (`Debug` or `Release`, default `Debug`).

**Runtime:** Ensure `VeosCoSimAppl.dll` is found (same directory as `_veos_cosim.*.pyd`, or on `PATH`, or `os.add_dll_directory`).

**Import gotcha:** If your current working directory is this `pythonbinding` folder, Python may import the local `veos_cosim/` package **without** the compiled `_veos_cosim` extension and fail. Run Python from another directory (e.g. the Scenic repo root) after `pip install .`, or use an editable install and build so the extension sits next to the package as your tooling expects.

---

## Relationship to Scenic

This package is **not** yet wired into Scenic’s main `pyproject.toml`. When integrated, it will likely be an **optional extra** (dSPACE / Windows-only). `CoSimClient.run` releases the GIL around `VeosCoSim_RunMI` and re-acquires it when invoking Python callbacks.

---

## License

Project metadata uses BSD-3-Clause to align with Scenic; **dSPACE headers and binaries remain under their own licenses** — see VeosCoSim distribution terms from dSPACE.
