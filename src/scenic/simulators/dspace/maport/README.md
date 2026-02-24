# MAPort – dSPACE XIL API variable access

This folder provides **MAPort-based read/write** for the dSPACE simulator, mirroring the variable-access API of the ControlDesk COM layer (`controldesk/`). Session control (go online, start maneuver, stepping) remains via **ControlDesk COM**; only variable **get_var** / **set_var** can be performed through MAPort (XIL API) for better performance.

## Original dSPACE demo files

The reference Python demos and helpers in this folder are derived from the dSPACE XIL API .NET demos. The **original files** are installed at:

- **`C:\Program Files\dSPACE XIL API .NET 2023-A\Demos\MAPort\Python`**

That directory contains the official dSPACE samples (e.g. `1_ReadWrite.py`, `18_ReadWriteArrays.py`, `DemoHelpers.py`, port configurations). The copies under this Scenic `maport` folder are adapted for use with the Scenic dSPACE simulator (paths, config, and integration).

## Contents of this folder

| File / module | Description |
|---------------|-------------|
| **connection.py** | `MAPortApp`: wrapper that creates an MAPort instance and exposes **get_var(path)** and **set_var(path, value)** with the same semantics as ControlDesk (scalars and arrays). Use this for variable read/write instead of ControlDesk COM when MAPort is enabled. |
| **session.py** | `connect_and_prepare_maport(sim, config_path=None)`: creates and configures an MAPort instance (load config, configure, optionally start simulation). Returns a `MAPortApp` ready for get_var/set_var. Session control (go online, start maneuver, step) is **not** done here—use ControlDesk for that. |
| **__init__.py** | Package exports for `maport` (e.g. `connection`, `session`). |
| **1_ReadWrite.py** | dSPACE demo: scalar read/write via MAPort (reference). |
| **18_ReadWriteArrays.py** | dSPACE demo: vector/array and element access via MAPort (reference). |
| **DemoHelpers.py** | Helper for XIL API value conversion (e.g. `convertIBaseValue`). |
| **MAPortConfigVEOS.xml** | MAPort configuration for VEOS; points to the ASM_Traffic .sdf. Edit `<SystemDescriptionFile>` to match your installation. |
| **README.md** | This file. |

## Design (mirroring `controldesk/`)

- **controldesk**: `ControlDeskApp` in `connection.py` + `connect_and_prepare()` in `session.py` → provides **get_var** / **set_var** (COM) and session/stepping (go_online, start_maneuver, advance_simulation_step, etc.).
- **maport**: `MAPortApp` in `connection.py` + `connect_and_prepare_maport()` in `session.py` → provides **get_var** / **set_var** only (XIL API). Session and stepping stay in ControlDesk.

So the simulator can:

1. Use **ControlDesk COM** for everything (current default): session + stepping + variable access.
2. Use **ControlDesk COM** for session and stepping, and **MAPort** for variable access only (faster read/write in benchmarks).

Variable paths (ego DISP_Plant, VesiInterface, fellow FellowTrailer / External_Signals) are the same for both; only the backend (COM vs MAPort) changes.

## Prerequisites

- **pythonnet** (clr) and dSPACE XIL API .NET assemblies (e.g. in GAC): required for MAPort.
- **ControlDesk** running with experiment loaded: required for session control and for starting the simulation before using MAPort (or MAPort can start simulation if none is running).
- **MAPortConfigVEOS.xml**: `<SystemDescriptionFile>` must point to your ASM_Traffic .sdf.

## Usage (conceptual)

```python
# Session and stepping: always ControlDesk
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk import session as cd_session
cd = ControlDeskApp().connect()
cd.go_online()
cd.start_measurement()
cd_session.start_maneuver(cd)
# ... later: cd.advance_simulation_step() for stepping

# Variable access: MAPort (drop-in replacement for cd.get_var / cd.set_var)
from scenic.simulators.dspace.maport.connection import MAPortApp
from scenic.simulators.dspace.maport import session as maport_session
mp = maport_session.connect_and_prepare_maport(sim)
x = mp.get_var("Platform()://ASM_Traffic/Model Root/.../Pos_x_.../Out1")
mp.set_var("Platform()://ASM_Traffic/Model Root/.../Const_throttle_cmd/Value", 50.0)
# When done: mp.dispose()
```

See `Scenic/debug_maport/bench_ego_com_vs_maport.py` for a benchmark comparing COM vs MAPort variable access (ego only).
