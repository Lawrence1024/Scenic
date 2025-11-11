import time

from .connection import ControlDeskApp


def connect_and_prepare(sim):
    """Connect to ControlDesk, go online, start measurement, init VESI, set step."""
    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        cd.start_measurement()
        print("[ControlDesk] Online and measuring")
        cd.initialize_vesi_interface()
        cd.set_simulation_step(0.01)
        print("[VesiInterface] Initialization complete - ready for manual control")
        return cd
    except Exception as e:
        print(f"[ControlDesk] Not available: {e}")
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
        return False
    try:
        cd.pause_simulation()
        return True
    except Exception as e:
        print(f"[ControlDesk] Pause failed: {e}")
        return False


def step(cd, dt):
    """Advance one simulation step; fallback to sleep if not available."""
    if cd:
        try:
            cd.advance_simulation_step()
            return True
        except Exception as e:
            print(f"[ControlDesk] advance_simulation_step failed: {e}")
    time.sleep(dt)
    return False


