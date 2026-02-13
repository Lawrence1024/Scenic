import time


def ensure_fellow_arrays_initialized(sim):
    """Advance the simulation until fellow arrays are populated (once)."""
    if sim._fellow_arrays_initialized or sim._initializing_fellow_arrays or not sim._cd:
        return
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    array_path = f"{base_path}/x"
    sim._initializing_fellow_arrays = True
    try:
        num_fellows = 0
        try:
            # Count fellows from scene objects (excluding ego). Ego is always present; fellows may be 0.
            if hasattr(sim, 'scene') and hasattr(sim.scene, 'objects'):
                num_fellows = len([o for o in sim.scene.objects if o is not getattr(sim.scene, 'egoObject', None)])
            # Also check _fellow_vehicles dict
            if hasattr(sim, '_fellow_vehicles') and sim._fellow_vehicles:
                num_fellows = max(num_fellows, len(sim._fellow_vehicles))
        except Exception:
            pass
        
        # When there are 0 fellows, no fellow arrays need to be populated; mark ready so ego control runs.
        if num_fellows == 0:
            sim._fellow_arrays_initialized = True
            sim._fellow_index_base = 0
            return

        # More aggressive warm-up: advance more steps and wait for REAL values
        max_attempts = 500  # Increased from 200
        for attempt in range(max_attempts):
            x_arr = y_arr = yaw_arr = None
            try:
                x_arr = sim._cd.get_var(array_path)
            except Exception:
                x_arr = None
            try:
                y_arr = sim._cd.get_var(f"{base_path}/y")
            except Exception:
                y_arr = None
            try:
                yaw_arr = sim._cd.get_var(f"{base_path}/yaw_deg_out")
            except Exception:
                yaw_arr = None

            ready = False
            # Check if arrays exist AND have real values (not just zeros)
            # We need actual position data, not just empty arrays
            if isinstance(x_arr, (list, tuple)) and len(x_arr) >= num_fellows:
                # Check that arrays have REAL values (not just zeros)
                # We need actual position data from the simulation
                def has_real_values(arr, num_indices):
                    """Check if arrays have real non-zero values indicating fellows are spawned."""
                    if not arr or not isinstance(arr, (list, tuple)):
                        return False
                    try:
                        for i in range(min(num_indices, len(arr))):
                            val = arr[i]
                            if isinstance(val, (int, float)) and abs(val) > 0.1:  # Real position, not zero
                                return True
                    except Exception:
                        pass
                    return False
                
                # Arrays are ready if we have real position values (x or y)
                # This means fellows are actually spawned in the simulation
                if has_real_values(x_arr, num_fellows) or (y_arr and has_real_values(y_arr, num_fellows)):
                    ready = True
            if ready:
                sim._fellow_arrays_initialized = True
                sim._fellow_index_base = 0
                break

            if attempt < max_attempts - 1:
                # TEMPORARILY COMMENTED OUT FOR DEBUGGING - no stepping/pausing
                # try:
                #     sim._cd.advance_simulation_step()
                #     time.sleep(sim.timestep * 0.5)
                # except Exception:
                #     time.sleep(sim.timestep)
                # Just wait without stepping for debugging
                time.sleep(sim.timestep * 0.5)
        if not sim._fellow_arrays_initialized:
            print(f"[dSPACE] [ERROR] Fellow arrays not initialized after {max_attempts} warm-up steps")
    finally:
        sim._initializing_fellow_arrays = False


def probe_external_index_base(sim):
    """Probe External_Signals arrays and determine index base (0 or 1)."""
    if sim._ext_probe_done or not sim._cd:
        return sim._ext_probe_done
    base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
    v_candidates = (
        f"{base}/Const_v_Fellows_External[km|h]/Value",
        f"{base}/Const_v_Fellows_External[km/h]/Value",
    )
    d_candidates = (f"{base}/Const_d_Fellows_External[m]/Value",)
    for vpath in v_candidates:
        try:
            arr = sim._cd.get_var(vpath)
            if not isinstance(arr, (list, tuple)):
                continue
            for idx_base in (0, 1):
                try:
                    _ = sim._cd.get_var(f"{vpath}[{idx_base}]")
                    sim._ext_v_path = vpath
                    sim._ext_d_path = d_candidates[0]
                    sim._ext_index_base = idx_base
                    sim._ext_probe_done = True
                    return True
                except Exception:
                    continue
            sim._ext_v_path = vpath
            sim._ext_d_path = d_candidates[0]
            sim._ext_index_base = 0
            sim._ext_probe_done = True
            return True
        except Exception:
            continue
    sim._ext_v_path = v_candidates[0]
    sim._ext_d_path = d_candidates[0]
    sim._ext_index_base = 0
    sim._ext_probe_done = True
    return False


