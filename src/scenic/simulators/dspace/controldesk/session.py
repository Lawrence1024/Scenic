import time

from .connection import ControlDeskApp


def connect_and_prepare(sim):
    """Connect to ControlDesk, go online, start measurement, init VESI, set step.
    
    Args:
        sim: DSpaceSimulation object with timestep attribute
    """
    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        cd.start_measurement()
        cd.initialize_vesi_interface()
        
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


def start_maneuver(cd):
    if not cd:
        return False
    try:
        cd.start_maneuver()
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


def step(cd, dt):
    if cd:
        try:
            cd.advance_simulation_step()
            return True
        except Exception:
            pass
    time.sleep(dt)
    return False


