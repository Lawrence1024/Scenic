from scenic.core.vectors import Vector
import math

def read_ego_state(sim, obj):
    """Read ego vehicle state from ControlDesk into obj.dspaceActor."""
    if not sim._cd:
        return False
    
    base_physics = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/VehicleDynamics/VehicleMovement/ASMSignalCollector/Vehicle_Movement_Info_Car"
    
    # 1. POSITION [x, y, z]
    # 'Out1' of the integrator is the calculated position for the next step.
    # Note: The '\n' is required because the block name in Simulink has a line break.
    path_pos_vec = f"{base_physics}/Positions/Discrete-Time\nIntegrator/Out1"
    
    # 2. VELOCITY [vx, vy, vz]
    # We try the Integrator first, then VectorLabel if that fails.
    path_vel_int = f"{base_physics}/Velocities/Discrete-Time\nIntegrator/Out1"
    path_vel_lbl = f"{base_physics}/Velocities/VectorLabel/out"

    try:
        # --- READ POSITION ---
        pos_arr = sim._cd.get_var(path_pos_vec)
        
        if pos_arr and len(pos_arr) >= 2:
            x = float(pos_arr[0])
            y = float(pos_arr[1])
            z = float(pos_arr[2]) if len(pos_arr) > 2 else 0.0
            
            # --- READ VELOCITY ---
            vx = 0.0
            vy = 0.0
            
            # Try Integrator path (Standard physics)
            try:
                vel_arr = sim._cd.get_var(path_vel_int)
                if vel_arr and len(vel_arr) >= 2:
                    vx = float(vel_arr[0])
                    vy = float(vel_arr[1])
            except:
                # Try VectorLabel path (Passthrough signal)
                try:
                    vel_arr = sim._cd.get_var(path_vel_lbl)
                    if vel_arr and len(vel_arr) >= 2:
                        vx = float(vel_arr[0])
                        vy = float(vel_arr[1])
                except:
                    pass 

            # --- CALCULATE YAW ---
            # Since the 'Angles' folder is missing, we derive Heading from Velocity.
            # This is mathematically accurate as long as the car is moving (> 0.1 m/s).
            yaw = 0.0
            
            if (vx*vx + vy*vy) > 0.01:
                yaw = math.atan2(vy, vx)
            elif hasattr(obj, 'dspaceActor'):
                # If stopped, keep the last known heading so we don't snap to 0
                yaw = obj.dspaceActor.heading

            # --- UPDATE ACTOR ---
            obj.dspaceActor.position = Vector(x, y, z)
            obj.dspaceActor.heading = yaw
            obj.dspaceActor.linvel = Vector(vx, vy, 0)
            
            return True

    except Exception as e:
        if not hasattr(obj, '_readback_error_shown'):
            print(f"[readback:physics] Failed to read ego state: {e}")
            obj._readback_error_shown = True
        pass

    return False


def read_fellow_state(sim, obj, dutils):
    """Read fellow vehicle state from ControlDesk arrays into obj.dspaceActor."""
    if not sim._cd:
        return False
    
    # Ensure arrays are initialized (warm-up should have completed during setup)
    from .arrays import ensure_fellow_arrays_initialized
    ensure_fellow_arrays_initialized(sim)
    
    # If arrays still not ready, this is a real problem - warm-up should have fixed this
    if not sim._fellow_arrays_initialized:
        print(f"[readback:fellow] WARNING: Arrays not initialized - warm-up may have failed")
        return False
    
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


