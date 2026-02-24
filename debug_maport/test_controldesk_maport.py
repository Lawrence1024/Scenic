"""
Test script: ControlDesk (go online, start maneuver) + MAPort (XIL API) read/write.

This script:
  1. Connects to ControlDesk via COM, goes online, starts measurement,
     initializes VesiInterface, sets simulation step, and starts the maneuver.
  2. Creates and configures MAPort (same pattern as 1_ReadWrite.py).
  3. Reads and writes ego vehicle variables via MAPort.
  4. Reads (and optionally writes) fellow vehicle array variables via MAPort,
     using the array access format from 18_ReadWriteArrays.py.

Prerequisites:
  - ControlDesk running with an experiment loaded (ASM_Traffic / VEOS).
  - Python with pythonnet (clr), pywin32 (for ControlDesk COM).
  - dSPACE XIL API .NET assemblies in GAC (ASAM.XIL.*).
  - MAPortConfigVEOS.xml path below points to a valid .sdf; adjust if needed.

Reference: Scenic/src/scenic/simulators/dspace/maport/1_ReadWrite.py (proven working)
          Scenic/src/scenic/simulators/dspace/maport/18_ReadWriteArrays.py (array format)
"""

import clr
import sys
import os
import time

# ---------------------------------------------------------------------------
# Paths: allow importing Scenic controldesk and maport DemoHelpers
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_scenic_src = os.path.join(_script_dir, "..", "src")
_maport_dir = os.path.join(_script_dir, "..", "src", "scenic", "simulators", "dspace", "maport")
_scenic_src = os.path.normpath(_scenic_src)
_maport_dir = os.path.normpath(_maport_dir)
if _scenic_src not in sys.path:
    sys.path.insert(0, _scenic_src)
if _maport_dir not in sys.path:
    sys.path.insert(0, _maport_dir)

# Load ASAM XIL API assemblies (same as 1_ReadWrite.py)
clr.AddReference(
    "ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984"
)
clr.AddReference(
    "ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984"
)

from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.Common.Error import TestbenchPortException
from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState
from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation

import System
from System import Array

from DemoHelpers import convertIBaseValue

# Scenic ControlDesk (COM)
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk import session as cd_session

# MAPort config: use maport folder config; user may copy and edit for their .sdf path
MAPortConfigFile = os.path.join(_maport_dir, "MAPortConfigVEOS.xml")
if not os.path.isfile(MAPortConfigFile):
    MAPortConfigFile = os.path.join(_script_dir, "MAPortConfigVEOS.xml")

# ---------------------------------------------------------------------------
# Variable paths (ASM_Traffic / VesiInterface / DISP_Plant / FellowTrailer)
# ---------------------------------------------------------------------------
BASE = "Platform()://ASM_Traffic/Model Root"

# Ego: VesiInterface inputs (write)
KEY_THROTTLE = f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
KEY_BRAKE_FRONT = f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
KEY_BRAKE_REAR = f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
KEY_STEERING = f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"

# Ego: DISP_Plant state (read)
DISP = f"{BASE}/VehicleDynamics/Plant/UserInterface/DISP_Plant"
EGO_X = f"{DISP}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
EGO_Y = f"{DISP}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"
EGO_Z = f"{DISP}/Positions/Pos_z_Vehicle_CoorSys_E[m]/Out1"
EGO_YAW = f"{DISP}/Positions/Angle_Yaw_Vehicle_CoorSys_E[deg]/Out1"
EGO_VX = f"{DISP}/Velocities/v_x_Vehicle_CoG[km|h]/Out1"
EGO_VY = f"{DISP}/Velocities/v_y_Vehicle_CoG[km|h]/Out1"

# Fellows: FellowTrailer arrays (read) – full array via CreateGenericVariableRef
FELLOW_BASE = f"{BASE}/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
FELLOW_X = f"{FELLOW_BASE}/x"
FELLOW_Y = f"{FELLOW_BASE}/y"
FELLOW_Z = f"{FELLOW_BASE}/z"
FELLOW_YAW = f"{FELLOW_BASE}/yaw_deg_out"

# Fellows: External_Signals (read/write arrays) – array format as in 18_ReadWriteArrays
EXT_BASE = f"{BASE}/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
FELLOW_V_EXT = f"{EXT_BASE}/Const_v_Fellows_External[km|h]/Value"
FELLOW_D_EXT = f"{EXT_BASE}/Const_d_Fellows_External[m]/Value"


def control_desk_connect_and_start(timestep=0.01):
    """Connect to ControlDesk, go online, start measurement, init VesiInterface, start maneuver."""
    print("[Step 1] Connecting to ControlDesk and preparing...")
    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        cd.start_measurement()
        cd.initialize_vesi_interface()
        cd.set_simulation_step(timestep)
        print("[Step 1] ControlDesk online and measurement started.")
    except Exception as e:
        print(f"[Step 1] ControlDesk connection failed: {e}")
        return None
    print("[Step 1] Starting maneuver...")
    if not cd_session.start_maneuver(cd):
        print("[Step 1] WARNING: start_maneuver failed (continuing anyway).")
    else:
        print("[Step 1] Maneuver started.")
    time.sleep(0.5)
    return cd


def create_and_configure_maport():
    """Create MAPort, load config, configure (no download if already loaded), start if needed."""
    print("[Step 2] Creating MAPort instance...")
    MyTestbenchFactory = TestbenchFactory()
    MyTestbench = MyTestbenchFactory.CreateVendorSpecificTestbench("dSPACE GmbH", "XIL API", "2023-A")
    MyMAPortFactory = MyTestbench.MAPortFactory
    MyValueFactory = MyTestbench.ValueFactory
    MyVariableRefFactory = MyTestbench.VariableRefFactory

    DemoMAPort = MyMAPortFactory.CreateMAPort("DemoMAPort")
    print("[Step 2] Configuring MAPort from %s ..." % MAPortConfigFile)
    DemoMAPortConfig = DemoMAPort.LoadConfiguration(MAPortConfigFile)
    DemoMAPort.Configure(DemoMAPortConfig, False)
    if DemoMAPort.State != MAPortState.eSIMULATION_RUNNING:
        print("[Step 2] Starting simulation via MAPort...")
        DemoMAPort.StartSimulation()
    else:
        print("[Step 2] Simulation already running.")
    print("[Step 2] MAPort ready.\n")
    return DemoMAPort, MyValueFactory, MyVariableRefFactory


def read_ego_values(maport, var_factory):
    """Read ego position and velocity via MAPort (CreateGenericVariableRef + Read2)."""
    print("[Step 3a] Reading ego variables via MAPort...")
    VRF = var_factory  # VariableRefFactory
    paths = [
        ("EGO_X", EGO_X),
        ("EGO_Y", EGO_Y),
        ("EGO_Z", EGO_Z),
        ("EGO_YAW", EGO_YAW),
        ("EGO_VX", EGO_VX),
        ("EGO_VY", EGO_VY),
    ]
    for name, path in paths:
        try:
            ref = VRF.CreateGenericVariableRef(path, ValueRepresentation.ePhysicalValue)
            val = convertIBaseValue(maport.Read2(ref))
            print("  %s = %s" % (name, val.Value))
        except Exception as e:
            print("  %s: ERROR %s" % (name, e))


def write_ego_values(maport, value_factory, var_ref_factory, throttle=0.0, brake=0.0, steering=0.0):
    """Write ego VesiInterface inputs via MAPort."""
    print("[Step 3b] Writing ego control via MAPort (throttle=%.2f, brake=%.2f, steering=%.2f)..." % (throttle, brake, steering))
    VRF = var_ref_factory
    throttle_val = float(throttle * 100.0)
    brake_val = float(brake * 10000.0)
    try:
        maport.Write2(VRF.CreateGenericVariableRef(KEY_THROTTLE, ValueRepresentation.ePhysicalValue), value_factory.CreateFloatValue(throttle_val))
        maport.Write2(VRF.CreateGenericVariableRef(KEY_BRAKE_FRONT, ValueRepresentation.ePhysicalValue), value_factory.CreateFloatValue(brake_val))
        maport.Write2(VRF.CreateGenericVariableRef(KEY_BRAKE_REAR, ValueRepresentation.ePhysicalValue), value_factory.CreateFloatValue(brake_val))
        maport.Write2(VRF.CreateGenericVariableRef(KEY_STEERING, ValueRepresentation.ePhysicalValue), value_factory.CreateFloatValue(float(steering)))
        print("  Ego write OK.")
    except Exception as e:
        print("  Ego write ERROR: %s" % e)


def read_fellow_arrays(maport, var_ref_factory):
    """Read fellow arrays (FellowTrailer x, y, z, yaw_deg_out) via MAPort – full array as in 18_ReadWriteArrays."""
    print("[Step 4a] Reading fellow arrays via MAPort (CreateGenericVariableRef for full array)...")
    VRF = var_ref_factory
    for label, path in [("FELLOW_X", FELLOW_X), ("FELLOW_Y", FELLOW_Y), ("FELLOW_Z", FELLOW_Z), ("FELLOW_YAW", FELLOW_YAW)]:
        try:
            ref = VRF.CreateGenericVariableRef(path, ValueRepresentation.ePhysicalValue)
            val = convertIBaseValue(maport.Read2(ref))
            v = val.Value
            if hasattr(v, "__len__") and not isinstance(v, str):
                print("  %s = %s (len=%d)" % (label, v, len(v)))
            else:
                print("  %s = %s" % (label, v))
        except Exception as e:
            print("  %s: ERROR %s" % (label, e))


def read_fellow_vector_element(maport, var_ref_factory, index=0):
    """Read a single fellow by index using CreateVectorElementRef (array format from 18_ReadWriteArrays)."""
    print("[Step 4b] Reading fellow element at index %d (CreateVectorElementRef)..." % index)
    VRF = var_ref_factory
    for label, path in [("FELLOW_X", FELLOW_X), ("FELLOW_Y", FELLOW_Y), ("FELLOW_YAW", FELLOW_YAW)]:
        try:
            ref = VRF.CreateVectorElementRef(path, index, ValueRepresentation.ePhysicalValue)
            val = convertIBaseValue(maport.Read2(ref))
            print("  %s[%d] = %s" % (label, index, val.Value))
        except Exception as e:
            print("  %s[%d]: ERROR %s" % (label, index, e))


def read_write_external_signals(maport, value_factory, var_ref_factory, fellow_index=0):
    """Read and write fellow External_Signals arrays (velocity and deviation)."""
    print("[Step 4c] Read/Write External_Signals (Const_v_Fellows_External, Const_d_Fellows_External)...")
    VRF = var_ref_factory
    try:
        ref_v = VRF.CreateGenericVariableRef(FELLOW_V_EXT, ValueRepresentation.ePhysicalValue)
        ref_d = VRF.CreateGenericVariableRef(FELLOW_D_EXT, ValueRepresentation.ePhysicalValue)
        val_v = convertIBaseValue(maport.Read2(ref_v))
        val_d = convertIBaseValue(maport.Read2(ref_d))
        v_arr = list(val_v.Value) if hasattr(val_v.Value, "__iter__") else [float(val_v.Value)]
        d_arr = list(val_d.Value) if hasattr(val_d.Value, "__iter__") else [float(val_d.Value)]
        print("  Const_v_Fellows_External (km/h) = %s" % v_arr)
        print("  Const_d_Fellows_External (m)    = %s" % d_arr)
        # Optional write: ensure length and write back (e.g. set one element)
        need = fellow_index + 1
        while len(v_arr) < need:
            v_arr.append(0.0)
        while len(d_arr) < need:
            d_arr.append(0.0)
        maport.Write2(ref_v, value_factory.CreateFloatVectorValue(Array[System.Double](v_arr)))
        maport.Write2(ref_d, value_factory.CreateFloatVectorValue(Array[System.Double](d_arr)))
        print("  Write back OK (same values).")
    except Exception as e:
        print("  External_Signals ERROR: %s" % e)


def main():
    print("=" * 60)
    print("ControlDesk + MAPort test: go online, start maneuver, read/write ego & fellow")
    print("=" * 60)

    # 1) ControlDesk: connect, go online, start maneuver
    cd = control_desk_connect_and_start(timestep=0.01)
    if cd is None:
        print("Aborting: ControlDesk not available.")
        return 1

    DemoMAPort = None
    try:
        # 2) Create and configure MAPort (like 1_ReadWrite)
        DemoMAPort, MyValueFactory, MyVariableRefFactory = create_and_configure_maport()

        # 3) Ego: read then write
        read_ego_values(DemoMAPort, MyVariableRefFactory)
        write_ego_values(DemoMAPort, MyValueFactory, MyVariableRefFactory, throttle=0.0, brake=0.0, steering=0.0)
        read_ego_values(DemoMAPort, MyVariableRefFactory)

        # 4) Fellow: read full arrays and optionally single element + External_Signals
        read_fellow_arrays(DemoMAPort, MyVariableRefFactory)
        read_fellow_vector_element(DemoMAPort, MyVariableRefFactory, index=0)
        read_write_external_signals(DemoMAPort, MyValueFactory, MyVariableRefFactory, fellow_index=0)

        print("")
        print("Demo successfully finished.")

    except TestbenchPortException as ex:
        print("TestbenchPortException: %s" % ex.CodeDescription)
        print("VendorCodeDescription: %s" % ex.VendorCodeDescription)
        raise
    finally:
        if DemoMAPort is not None:
            DemoMAPort.Dispose()
            DemoMAPort = None
        print("MAPort disposed.")

    return 0


if __name__ == "__main__":
    sys.exit(main())
