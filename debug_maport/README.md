# debug_maport

This folder contains a **test script** that verifies the combined use of **ControlDesk (COM)** and **MAPort (XIL API)** with the dSPACE simulator: go online, start the maneuver via ControlDesk, then create a MAPort and perform read/write of ego and fellow variables.

## Purpose

- **ControlDesk**: Session control (go online, start measurement, initialize VesiInterface, start maneuver).
- **MAPort**: Variable access (read/write) using the dSPACE XIL API, matching the patterns from the proven `1_ReadWrite.py` and the array patterns from `18_ReadWriteArrays.py`.

This is useful to:

- Debug or validate MAPort-based read/write without running the full Scenic simulation.
- Confirm that ego (DISP_Plant, VesiInterface) and fellow (FellowTrailer, External_Signals) paths work via MAPort.
- Compare behavior with the existing ControlDesk COM-only pipeline.

## Contents

| File | Description |
|------|-------------|
| `test_controldesk_maport.py` | Main test: ControlDesk connect + MAPort create + ego/fellow read and write. |
| `README.md` | This file. |

## Prerequisites

1. **ControlDesk** running with an experiment loaded (e.g. ASM_Traffic on VEOS).
2. **Python** with:
   - `pythonnet` (clr) for .NET/XIL API.
   - `pywin32` for ControlDesk COM.
3. **dSPACE XIL API .NET** assemblies in the GAC (e.g. `ASAM.XIL.Implementation.TestbenchFactory`, `ASAM.XIL.Interfaces`).
4. **MAPort config**: The script uses the config at  
   `Scenic/src/scenic/simulators/dspace/maport/MAPortConfigVEOS.xml`.  
   Ensure the `<SystemDescriptionFile>` path in that XML points to your `.sdf` (e.g. ASM_Traffic.sdf). You can copy `MAPortConfigVEOS.xml` into `debug_maport` and edit the path if needed.

## How to run

From the repository root (or from a directory where `Scenic` is on `PYTHONPATH`), run the script with the **current working directory** set so that the script can resolve paths to the Scenic package and maport folder. Recommended:

```bash
cd Scenic/debug_maport
python test_controldesk_maport.py
```

Or from the repo root:

```bash
python Scenic/debug_maport/test_controldesk_maport.py
```

The script adds `../src` (relative to the script’s directory) and the maport directory to `sys.path` so it can import `scenic.simulators.dspace.controldesk` and `DemoHelpers` from the maport folder.

## What the script does

1. **ControlDesk**
   - Connects to ControlDesk (COM), goes online, starts measurement.
   - Calls `initialize_vesi_interface()` and sets the simulation step.
   - Starts the maneuver via the MANEUVER_START pulse.

2. **MAPort**
   - Creates a Testbench and MAPort (same pattern as `1_ReadWrite.py`).
   - Loads and applies the MAPort config with `Configure(..., False)` (no download if an app is already loaded).
   - Starts the simulation via MAPort if the state is not already `eSIMULATION_RUNNING`.

3. **Ego**
   - **Read**: DISP_Plant (Pos_x, Pos_y, Pos_z, Angle_Yaw, v_x, v_y) using `CreateGenericVariableRef` + `Read2`.
   - **Write**: VesiInterface (throttle, brake front/rear, steering) using `CreateGenericVariableRef` + `Write2` with `CreateFloatValue`.

4. **Fellow**
   - **Read**: FellowTrailer arrays (`x`, `y`, `z`, `yaw_deg_out`) as full arrays via `CreateGenericVariableRef`, and a single index via `CreateVectorElementRef` (array format from `18_ReadWriteArrays.py`).
   - **Read/Write**: External_Signals (`Const_v_Fellows_External[km|h]/Value`, `Const_d_Fellows_External[m]/Value`) as full arrays with `CreateGenericVariableRef`, then `Read2` / `Write2` with `CreateFloatVectorValue(Array[System.Double](...))`.

At the end, the MAPort is disposed.

## Variable paths (reference)

- **Ego state**: `Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant/...`
- **Ego control**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/...`
- **Fellow state**: `Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer/x`, `/y`, `/z`, `/yaw_deg_out`
- **Fellow external**: `.../External_Signals/Const_v_Fellows_External[km|h]/Value`, `Const_d_Fellows_External[m]/Value`

## References

- `Scenic/src/scenic/simulators/dspace/maport/1_ReadWrite.py` – MAPort creation and scalar read/write (proven working).
- `Scenic/src/scenic/simulators/dspace/maport/18_ReadWriteArrays.py` – Vector/matrix and array element access (`CreateVectorElementRef`, `CreateFloatVectorValue`, etc.).
- `Scenic/src/scenic/simulators/dspace/controldesk/connection.py` – ControlDesk COM API (connect, go_online, start_maneuver, set_var, get_var).
