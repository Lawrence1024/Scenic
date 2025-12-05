import time

from .connection import ControlDeskApp


def connect_and_prepare(sim):
    """Connect to ControlDesk, go online, start measurement, init VESI, set step.
    
    Args:
        sim: DSpaceSimulation object with timestep attribute
    """
    try:
        print("[ControlDesk] Attempting to connect to ControlDesk COM interface...")
        cd = ControlDeskApp().connect()
        print("[ControlDesk] Connected to COM interface")
        
        print("[ControlDesk] Starting online calibration...")
        cd.go_online()
        print("[ControlDesk] Online calibration started")
        
        print("[ControlDesk] Starting measurement...")
        cd.start_measurement()
        print("[ControlDesk] Online and measuring")
        
        print("[ControlDesk] Initializing VesiInterface...")
        cd.initialize_vesi_interface()
        
        # Use the simulation's timestep to ensure Scenic and ControlDesk are synchronized
        timestep = getattr(sim, 'timestep', 0.01)  # Default to 0.01 if not found
        cd.set_simulation_step(timestep)
        print(f"[VesiInterface] Initialization complete - ready for manual control (timestep={timestep}s)")
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
        print("[Maneuver] Started via ControlDesk")
        return True
    except Exception as e:
        print(f"[Maneuver] Failed to start: {e}")
        return False


def pause(cd):
    if not cd:
        print("[ControlDesk] Cannot pause - no connection")
        return False
    try:
        print("[ControlDesk] Calling pause_simulation()...")
        cd.pause_simulation()
        print("[ControlDesk] Pause command sent successfully")
        return True
    except Exception as e:
        print(f"[ControlDesk] Pause failed: {e}")
        import traceback
        traceback.print_exc()
        return False


def step(cd, dt):
    """Advance one simulation step; fallback to sleep if not available."""
    if cd:
        try:
            cd.advance_simulation_step()
            return True
        except Exception as e:
            print(f"[ControlDesk] advance_simulation_step failed: {e}")
            print(f"[ControlDesk] Falling back to time.sleep({dt}s)")
            import traceback
            traceback.print_exc()
    else:
        print(f"[ControlDesk] No connection - using time.sleep({dt}s) fallback")
    time.sleep(dt)
    return False


