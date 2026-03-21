# ModelDesk + optional CoSim smoke test

Use this to verify:

1. **ModelDesk COM** — same path as Scenic’s dSPACE simulator (`Dispatch("ModelDesk.Application")`, active project, active experiment) via `ModelDeskConnection`.
2. **Optional CoSim IPC** — same pattern as `src/scenic/simulators/dspace/cosim/README.md`: start `SyncStepBridge` on localhost, then launch `VeosCoSimTestClientIpc.exe` so VEOS can find its client before ModelDesk / VEOS are fully usable. **Full Scenic runs** can do the same automatically with `param launch_veos_ipc_client = True` (see `model.scenic` / `simulator.py`).

## Prerequisites

### ModelDesk

- ModelDesk is running
- A project is open
- An experiment is active

### CoSim (only if using `--with-cosim`)

- Build the IPC client once:

  ```powershell
  cd src\scenic\simulators\dspace\cosim\veos_cosim_ipc_bridge
  .\build_client.bat
  ```

  Output: `veos_cosim_ipc_bridge\client\build\VeosCoSimTestClientIpc.exe`

- VEOS / CoSim server reachable at `--veos-host` with name `--veos-name` (defaults match `cosim/README.md`).

## Run

### ModelDesk only (no CoSim subprocess)

From the Scenic repo root, with `src` on `PYTHONPATH`:

```powershell
$env:PYTHONPATH = "src"
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py
```

### ModelDesk after starting the IPC client + sync bridge (recommended if VEOS needs the client)

```powershell
$env:PYTHONPATH = "src"
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim
```

Override VEOS address / server name if needed:

```powershell
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim --veos-host 192.168.100.101 --veos-name CoSimServerScenic
```

Custom path to the IPC EXE:

```powershell
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim --cosim-exe C:\path\to\VeosCoSimTestClientIpc.exe
```

If the simulation is already running and `TIME_TRIGGER` would block the client until Scenic sends `STEP`, use a background step pump:

```powershell
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim --auto-step
```

### Module form

```powershell
$env:PYTHONPATH = "src"
python -m scenic.simulators.dspace.modeldesk_connection_test --with-cosim
```

Verbose COM details:

```powershell
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim -v
```

### SaveAs + Download to VEOS (Scenic `setup()`-style)

After ModelDesk COM succeeds, you can run the same **SaveAs → activate working copy → Save → Download** path as `DSpaceSimulation.setup()` (see `simulator.py` and `modeldesk/scenario.py`). This creates a new scenario copy (default name `Scenic_veos_test_YYYYMMDD_HHMMSS`) and attempts to push it to VEOS.

```powershell
python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim --test-veos-download
```

Optional:

- `--scenario-src NAME` — scenario to activate before SaveAs (default: current `TrafficScenario` name).
- `--scenario-name NAME` — name for the new copy (default: timestamped `Scenic_veos_test_...`).
- `--no-maneuver-reset` — skip `ManeuverControl.Reset()` after download (normally `setup()` calls it).

Exit code **2** if `Download()` fails or returns `False`.

## What it checks

- **CoSim (with `--with-cosim`)**: `SyncStepBridge` listens on `127.0.0.1:50555` (configurable); `VeosCoSimTestClientIpc.exe` connects to that bridge, then to VEOS — same idea as `DSpaceSimulation.setup()` + Terminal 2 in `cosim/README.md`.
- **ModelDesk**: `ActiveProject` / `ActiveExperiment` / best-effort `TrafficScenario` name.
- **`--test-veos-download`**: `SaveAs` / `ActivateTrafficScenario` / `Save` / `Download` / optional `ManeuverControl.Reset`.

## Implementation references

- ModelDesk: `scenic.simulators.dspace.modeldesk.connection.ModelDeskConnection` and `DSpaceSimulation.setup()` in `simulator.py`.
- CoSim sync: `cosim/veos_cosim_ipc_bridge/python_listener/sync_step_bridge.py` and `cosim/README.md`.
