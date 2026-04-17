import json
import time
from pathlib import Path

from .connection import ControlDeskApp


def _external_control_baseline_path():
    """Path to external_control_baseline.json (next to dspace package)."""
    return Path(__file__).resolve().parent.parent / "external_control_baseline.json"


def _apply_external_control_baseline(cd):
    """Apply the 16 VesiInterface variables from external_control_baseline.json (External Control mode)."""
    path = _external_control_baseline_path()
    if not path.is_file():
        print(f"[ControlDesk] External Control baseline not found: {path}")
        print("[ControlDesk] Run read_vesi_interface_vars.py in External Control mode to create it.")
        return
    try:
        with open(path) as f:
            snapshot = json.load(f)
    except Exception as e:
        print(f"[ControlDesk] Failed to load baseline {path}: {e}")
        return
    for var_path, value in snapshot.items():
        try:
            cd.set_var(var_path, value)
        except Exception as e:
            print(f"[ControlDesk] Failed to set {var_path}: {e}")
    print("[ControlDesk] Applied External Control baseline (VesiInterface variables from JSON).")


def connect_and_prepare(sim):
    """Connect to ControlDesk, go online, start measurement, init VESI (or apply baseline), set step.
    
    Args:
        sim: DSpaceSimulation object with timestep attribute; sim.sim.scenic_control determines
             whether to initialize Manual Control (True) or apply External Control baseline (False).
    """
    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        cd.start_measurement()
        scenic_control = getattr(getattr(sim, "sim", None), "scenic_control", True)
        if scenic_control:
            cd.initialize_vesi_interface()
            print("[ControlDesk] Manual Control: VesiInterface initialized for Scenic-controlled ego.")
        else:
            _apply_external_control_baseline(cd)
        
        # Use the simulation's timestep to ensure Scenic and ControlDesk are synchronized
        timestep = getattr(sim, 'timestep', 0.01)  # Default to 0.01 if not found
        cd.set_simulation_step(timestep)
        return cd
    except Exception as e:
        print(f"\n[ControlDesk] Connection failed: {e}")
        print("[ControlDesk] Make sure:")
        print("  1. ControlDesk application is running")
        print("  2. An experiment with a platform is loaded")
        print("  3. The platform is ready for online calibration")
        return None


def start_maneuver(cd, var_access=None):
    if not cd:
        return False
    try:
        cd.start_maneuver(var_access=var_access)
        return True
    except Exception as e:
        print(f"[Maneuver] Failed to start: {e}")
        return False


def pause(cd):
    if not cd:
        return False
    try:
        cd.pause_simulation()
        return True
    except Exception:
        return False


def stop(cd, var_access=None):
    """Stop the active maneuver (pulse MANEUVER_STOP variable)."""
    if not cd:
        return False
    try:
        cd.stop_maneuver(var_access=var_access)
        return True
    except Exception as e:
        print(f"[Maneuver] Failed to stop: {e}")
        return False


def step(cd, dt):
    raise NotImplementedError(
        "controldesk.session.step() is dead. DSpaceSimulation.step() owns "
        "the dual-path (CoSim vs ControlDesk) stepping logic."
    )


