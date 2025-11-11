from scenic.core.vectors import Vector

def read_ego_state(sim, obj):
    """Read ego vehicle state from ControlDesk into obj.dspaceActor."""
    if not sim._cd:
        return False
    try:
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel"
        try:
            x = sim._cd.get_var(f"{base_path}/Ego_x/Value")
            y = sim._cd.get_var(f"{base_path}/Ego_y/Value")
            z = sim._cd.get_var(f"{base_path}/Ego_z/Value")
            yaw_deg = sim._cd.get_var(f"{base_path}/Ego_yaw/Value")
            velocity = sim._cd.get_var(f"{base_path}/Ego_velocity/Value")
            obj.dspaceActor.position = Vector(float(x), float(y), float(z))
            obj.dspaceActor.heading = float(yaw_deg) * (3.14159265 / 180.0)
            obj.dspaceActor.linvel = Vector(float(velocity), 0, 0)
            return True
        except Exception:
            return False
    except Exception as e:
        print(f"[readback:ego] Error: {e}")
        return False


def read_fellow_state(sim, obj, dutils):
    """Read fellow vehicle state from ControlDesk arrays into obj.dspaceActor."""
    if not sim._cd:
        return False
    sim._ensureFellowArraysInitialized()
    try:
        fellow_index = sim._getFellowIndex(obj)
        if fellow_index is None:
            return False
        eff_index = fellow_index + (sim._fellow_index_base or 0)
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        try:
            x_arr = sim._cd.get_var(f"{base_path}/x")
            x = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(x_arr) else 0.0
        except Exception:
            x = 0.0
        try:
            y_arr = sim._cd.get_var(f"{base_path}/y")
            y = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(y_arr) else 0.0
        except Exception:
            y = 0.0
        try:
            z_arr = sim._cd.get_var(f"{base_path}/z")
            z = z_arr[eff_index] if isinstance(z_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(z_arr) else 0.0
        except Exception:
            z = 0.0
        try:
            yaw_arr = sim._cd.get_var(f"{base_path}/yaw_deg_out")
            yaw_deg = yaw_arr[eff_index] if isinstance(yaw_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(yaw_arr) else 0.0
        except Exception:
            yaw_deg = 0.0
        v = 0.0
        w = 0.0
        try:
            v_arr = sim._cd.get_var(f"{base_path}/v_Fellows")
            if isinstance(v_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(v_arr):
                v = v_arr[eff_index] if v_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        try:
            w_arr = sim._cd.get_var(f"{base_path}/w_Fellows")
            if isinstance(w_arr, (list, tuple)) and isinstance(eff_index, int) and eff_index < len(w_arr):
                w = w_arr[eff_index] if w_arr[eff_index] is not None else 0.0
        except Exception:
            pass
        if hasattr(obj, '_array_bounds_warning_shown'):
            delattr(obj, '_array_bounds_warning_shown')
        import math
        obj.dspaceActor.position = Vector(float(x), float(y), float(z))
        obj.dspaceActor.heading = float(yaw_deg) * (math.pi / 180.0)
        yaw_rad = obj.dspaceActor.heading
        obj.dspaceActor.linvel = Vector(
            float(v) * math.cos(yaw_rad),
            float(v) * math.sin(yaw_rad),
            0
        )
        obj.dspaceActor.angvel = Vector(0, 0, float(w))
        return True
    except Exception as e:
        msg = str(e)
        if "bounds" in msg:
            if not hasattr(obj, '_array_bounds_warning_shown'):
                print(f"[readback:fellow] Warning: Fellow {fellow_index} array not ready yet")
                obj._array_bounds_warning_shown = True
            return False
        print(f"[readback:fellow] Error: {e}")
        return False


