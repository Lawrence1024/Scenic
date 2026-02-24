"""
Benchmark: Ego variable read/write via ControlDesk COM vs MAPort (XIL API).

Runs the same ego workload (6 reads + 4 writes per cycle) using:
  A) ControlDesk COM (get_var / set_var)
  B) MAPort (Read2 / Write2)

Session control (connect, go online, start maneuver) is always via ControlDesk COM.
Only the variable access is compared.

Usage: From Scenic folder:  python debug_maport/bench_ego_com_vs_maport.py
       Or from debug_maport: python bench_ego_com_vs_maport.py

Prerequisites: ControlDesk running, experiment loaded (ASM_Traffic / VEOS).
"""

import clr
import sys
import os
import time

# ---------------------------------------------------------------------------
# Paths for Scenic and maport
# ---------------------------------------------------------------------------
_script_dir = os.path.dirname(os.path.abspath(__file__))
_scenic_src = os.path.normpath(os.path.join(_script_dir, "..", "src"))
_maport_dir = os.path.normpath(os.path.join(_script_dir, "..", "src", "scenic", "simulators", "dspace", "maport"))
if _scenic_src not in sys.path:
    sys.path.insert(0, _scenic_src)
if _maport_dir not in sys.path:
    sys.path.insert(0, _maport_dir)

clr.AddReference("ASAM.XIL.Implementation.TestbenchFactory, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")
clr.AddReference("ASAM.XIL.Interfaces, Version=2.2.0.0, Culture=neutral, PublicKeyToken=bf471dff114ae984")

from ASAM.XIL.Implementation.TestbenchFactory.Testbench import TestbenchFactory
from ASAM.XIL.Interfaces.Testbench.Common.Error import TestbenchPortException
from ASAM.XIL.Interfaces.Testbench.MAPort.Enum import MAPortState
from ASAM.XIL.Interfaces.Testbench.Common.VariableRef.Enum import ValueRepresentation

from DemoHelpers import convertIBaseValue
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk import session as cd_session

MAPortConfigFile = os.path.join(_maport_dir, "MAPortConfigVEOS.xml")
if not os.path.isfile(MAPortConfigFile):
    MAPortConfigFile = os.path.join(_script_dir, "MAPortConfigVEOS.xml")

# ---------------------------------------------------------------------------
# Ego variable paths (read: DISP_Plant; write: VesiInterface)
# ---------------------------------------------------------------------------
BASE = "Platform()://ASM_Traffic/Model Root"
DISP = f"{BASE}/VehicleDynamics/Plant/UserInterface/DISP_Plant"

EGO_READ_PATHS = [
    f"{DISP}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1",
    f"{DISP}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1",
    f"{DISP}/Positions/Pos_z_Vehicle_CoorSys_E[m]/Out1",
    f"{DISP}/Positions/Angle_Yaw_Vehicle_CoorSys_E[deg]/Out1",
    f"{DISP}/Velocities/v_x_Vehicle_CoG[km|h]/Out1",
    f"{DISP}/Velocities/v_y_Vehicle_CoG[km|h]/Out1",
]

EGO_WRITE_PATHS = [
    f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value",
    f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value",
    f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value",
    f"{BASE}/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value",
]
# Write values per cycle: throttle 0–100, brake 0–10000, steering (deg)
EGO_WRITE_VALUES = (0.0, 0.0, 0.0, 0.0)

WARMUP_CYCLES = 20
BENCH_CYCLES = 300


def setup_control_desk(timestep=0.01):
    """Connect ControlDesk, go online, start measurement, init VesiInterface, start maneuver."""
    print("[Setup] Connecting to ControlDesk and preparing...")
    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        cd.start_measurement()
        cd.initialize_vesi_interface()
        cd.set_simulation_step(timestep)
        cd_session.start_maneuver(cd)
        time.sleep(0.5)
        print("[Setup] ControlDesk ready.\n")
        return cd
    except Exception as e:
        print("[Setup] ControlDesk failed: %s" % e)
        return None


def one_cycle_com(cd):
    """One cycle: 6 ego reads + 4 ego writes via ControlDesk COM."""
    for path in EGO_READ_PATHS:
        cd.get_var(path)
    for path, val in zip(EGO_WRITE_PATHS, EGO_WRITE_VALUES):
        cd.set_var(path, val)


def run_bench_com(cd, warmup_cycles, bench_cycles):
    """Run COM benchmark: warmup then timed cycles."""
    for _ in range(warmup_cycles):
        one_cycle_com(cd)
    t0 = time.perf_counter()
    for _ in range(bench_cycles):
        one_cycle_com(cd)
    elapsed = time.perf_counter() - t0
    return elapsed


def one_cycle_maport(maport, read_refs, write_refs, value_factory):
    """One cycle: 6 ego reads + 4 ego writes via MAPort."""
    for ref in read_refs:
        convertIBaseValue(maport.Read2(ref))
    for ref, val in zip(write_refs, EGO_WRITE_VALUES):
        maport.Write2(ref, value_factory.CreateFloatValue(float(val)))


def run_bench_maport(cd, warmup_cycles, bench_cycles):
    """Create MAPort, build refs, warmup, timed cycles, dispose. Returns elapsed time."""
    factory = TestbenchFactory()
    testbench = factory.CreateVendorSpecificTestbench("dSPACE GmbH", "XIL API", "2023-A")
    vrf = testbench.VariableRefFactory
    vf = testbench.ValueFactory
    maport = testbench.MAPortFactory.CreateMAPort("BenchMAPort")
    config = maport.LoadConfiguration(MAPortConfigFile)
    maport.Configure(config, False)
    if maport.State != MAPortState.eSIMULATION_RUNNING:
        maport.StartSimulation()

    read_refs = [vrf.CreateGenericVariableRef(p, ValueRepresentation.ePhysicalValue) for p in EGO_READ_PATHS]
    write_refs = [vrf.CreateGenericVariableRef(p, ValueRepresentation.ePhysicalValue) for p in EGO_WRITE_PATHS]

    for _ in range(warmup_cycles):
        one_cycle_maport(maport, read_refs, write_refs, vf)
    t0 = time.perf_counter()
    for _ in range(bench_cycles):
        one_cycle_maport(maport, read_refs, write_refs, vf)
    elapsed = time.perf_counter() - t0
    maport.Dispose()
    maport = None
    return elapsed


def main():
    print("=" * 60)
    print("Ego read/write benchmark: ControlDesk COM vs MAPort")
    print("=" * 60)
    print("Workload per cycle: 6 reads (x,y,z,yaw,vx,vy) + 4 writes (throttle,brake F/R,steer)")
    print("Warmup cycles: %d  |  Timed cycles: %d" % (WARMUP_CYCLES, BENCH_CYCLES))
    print()

    cd = setup_control_desk(timestep=0.01)
    if cd is None:
        print("Aborting: ControlDesk not available.")
        return 1

    # Benchmark 1: COM
    print("[Bench] Running COM (get_var / set_var)...")
    t_com = run_bench_com(cd, WARMUP_CYCLES, BENCH_CYCLES)
    mean_com_ms = (t_com / BENCH_CYCLES) * 1000.0
    print("  Total: %.3f s  |  Mean per cycle: %.3f ms" % (t_com, mean_com_ms))
    print()

    # Benchmark 2: MAPort
    print("[Bench] Running MAPort (Read2 / Write2)...")
    try:
        t_maport = run_bench_maport(cd, WARMUP_CYCLES, BENCH_CYCLES)
        mean_maport_ms = (t_maport / BENCH_CYCLES) * 1000.0
        print("  Total: %.3f s  |  Mean per cycle: %.3f ms" % (t_maport, mean_maport_ms))
    except TestbenchPortException as e:
        print("  MAPort error: %s" % e.CodeDescription)
        return 1
    print()

    # Comparison
    print("--- Summary ---")
    print("  COM:    %.3f ms/cycle (total %.3f s for %d cycles)" % (mean_com_ms, t_com, BENCH_CYCLES))
    print("  MAPort: %.3f ms/cycle (total %.3f s for %d cycles)" % (mean_maport_ms, t_maport, BENCH_CYCLES))
    if t_maport > 0:
        ratio = t_com / t_maport
        if ratio > 1.0:
            print("  MAPort is %.2fx faster than COM for this workload." % ratio)
        else:
            print("  COM is %.2fx faster than MAPort for this workload." % (1.0 / ratio))
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
