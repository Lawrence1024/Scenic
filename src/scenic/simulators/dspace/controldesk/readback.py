from scenic.core.vectors import Vector
import math

def read_ego_state(sim, obj):
    """Read ego vehicle state from ControlDesk into obj.dspaceActor."""
    if not sim._cd:
        return False
    
    base_path = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant"
    
    # 1. Position (x, y, z)
    path_x = f"{base_path}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
    path_y = f"{base_path}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"
    path_z = f"{base_path}/Positions/Pos_z_Vehicle_CoorSys_E[m]/Out1"
    
    # 2. Orientation (yaw)
    path_yaw = f"{base_path}/Positions/Angle_Yaw_Vehicle_CoorSys_E[deg]/Out1"

    # 3. Velocity [vx, vy, vz]
    path_vx = f"{base_path}/Velocities/v_x_Vehicle_CoG[km|h]/Out1"
    path_vy = f"{base_path}/Velocities/v_y_Vehicle_CoG[km|h]/Out1"

    try:
        # 1. Read Position
        x = float(sim._cd.get_var(path_x))
        y = float(sim._cd.get_var(path_y))
        z = float(sim._cd.get_var(path_z))
        
        # CRITICAL: Transform position from RD/dSPACE coordinates back to Scenic/XODR coordinates
        # Position read from ControlDesk is in RD coordinate system, but Scenic expects XODR coordinates
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_inverse_coordinate_transform
            rd_x, rd_y = float(x), float(y)
            scenic_x, scenic_y = apply_inverse_coordinate_transform(sim._coordinate_transform, (rd_x, rd_y))
            x, y = scenic_x, scenic_y
            
            # Compare with expected position on first read
            if hasattr(obj, '_expected_rd') and not hasattr(obj, '_readback_shown'):
                expected_rd = obj._expected_rd
                error_rd = math.sqrt((rd_x - expected_rd[0])**2 + (rd_y - expected_rd[1])**2)
                expected_xodr = obj._scenic_xodr
                error_xodr = math.sqrt((scenic_x - expected_xodr[0])**2 + (scenic_y - expected_xodr[1])**2)
                print(f"[Ego Readback] RD: ({rd_x:.6f}, {rd_y:.6f}) [expected: ({expected_rd[0]:.6f}, {expected_rd[1]:.6f}), error: {error_rd:.3f}m]")
                print(f"[Ego Readback] XODR: ({scenic_x:.6f}, {scenic_y:.6f}) [expected: ({expected_xodr[0]:.6f}, {expected_xodr[1]:.6f}), error: {error_xodr:.3f}m]")
                obj._readback_shown = True
        
        # 2. Read Orientation
        yaw_deg = float(sim._cd.get_var(path_yaw))
        yaw_rad = yaw_deg * (math.pi / 180.0)
        
        # 3. Read Velocity
        # Inputs are km/h, Scenic uses m/s
        vx_kmh = float(sim._cd.get_var(path_vx))
        vy_kmh = float(sim._cd.get_var(path_vy))
        
        vx_ms = vx_kmh / 3.6
        vy_ms = vy_kmh / 3.6
        
        # 4. Update Actor
        obj.dspaceActor.position = Vector(x, y, z)
        obj.dspaceActor.heading = yaw_rad
        obj.dspaceActor.linvel = Vector(vx_ms, vy_ms, 0)
        
        return True

    except Exception as e:
        print(f"[Ego Readback] Error: {e}")
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
        
        # CRITICAL: Transform position from RD/dSPACE coordinates back to Scenic/XODR coordinates
        # Position read from ControlDesk is in RD coordinate system, but Scenic expects XODR coordinates
        vehicle_name = getattr(obj, "name", f"Fellow_{fellow_index}")
        
        if sim._coordinate_transform is not None:
            from ..geometry.coordinate_transform import apply_inverse_coordinate_transform
            rd_x, rd_y = float(x), float(y)
            scenic_x, scenic_y = apply_inverse_coordinate_transform(sim._coordinate_transform, (rd_x, rd_y))
        else:
            scenic_x, scenic_y = float(x), float(y)
        
        # Compare with expected position on first read
        if hasattr(obj, '_expected_rd') and not hasattr(obj, '_readback_shown'):
            expected_rd = obj._expected_rd
            error_rd = math.sqrt((rd_x - expected_rd[0])**2 + (rd_y - expected_rd[1])**2)
            expected_xodr = obj._scenic_xodr
            error_xodr = math.sqrt((scenic_x - expected_xodr[0])**2 + (scenic_y - expected_xodr[1])**2)
            print(f"[{vehicle_name} Readback] RD: ({rd_x:.6f}, {rd_y:.6f}) [expected: ({expected_rd[0]:.6f}, {expected_rd[1]:.6f}), error: {error_rd:.3f}m]")
            print(f"[{vehicle_name} Readback] XODR: ({scenic_x:.6f}, {scenic_y:.6f}) [expected: ({expected_xodr[0]:.6f}, {expected_xodr[1]:.6f}), error: {error_xodr:.3f}m]")
            obj._readback_shown = True
        
        obj.dspaceActor.position = Vector(scenic_x, scenic_y, float(z))
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
            return False
        return False


