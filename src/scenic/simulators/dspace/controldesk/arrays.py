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
            # Count fellows from scene objects (excluding ego)
            if hasattr(sim, 'scene') and hasattr(sim.scene, 'objects'):
                num_fellows = len([o for o in sim.scene.objects if o is not getattr(sim.scene, 'egoObject', None)])
            # Also check _fellow_vehicles dict
            if hasattr(sim, '_fellow_vehicles') and sim._fellow_vehicles:
                num_fellows = max(num_fellows, len(sim._fellow_vehicles))
        except Exception:
            pass
        
        # Default to checking first 5 if we can't determine count
        if num_fellows == 0:
            num_fellows = 5

        print(f"[dSPACE] Starting warm-up: waiting for {num_fellows} fellow array(s) to be populated...")
        
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
                    if attempt > 0:
                        # Show what we found
                        try:
                            x_val = x_arr[0] if len(x_arr) > 0 else "N/A"
                            y_val = y_arr[0] if y_arr and len(y_arr) > 0 else "N/A"
                            print(f"[dSPACE] Arrays ready: x[0]={x_val}, y[0]={y_val}")
                        except:
                            pass
            elif attempt < 10 or attempt % 50 == 0:
                # Debug: show what we got periodically
                x_type = type(x_arr).__name__ if x_arr is not None else "None"
                x_len = len(x_arr) if isinstance(x_arr, (list, tuple)) else "N/A"
                print(f"[dSPACE] Warm-up attempt {attempt}: x_arr type={x_type}, len={x_len}, num_fellows={num_fellows}")
                
            if ready:
                sim._fellow_arrays_initialized = True
                sim._fellow_index_base = 0
                print(f"[dSPACE] ✅ Fellow arrays initialized after {attempt} warm-up step(s)")
                break

            if attempt < max_attempts - 1:
                try:
                    sim._cd.advance_simulation_step()
                    # Small delay to let simulation process
                    time.sleep(sim.timestep * 0.5)
                except Exception as step_err:
                    print(f"[dSPACE] Unable to advance simulation during warm-up (attempt {attempt}): {step_err}")
                    # Don't break - keep trying
                    time.sleep(sim.timestep)
        if not sim._fellow_arrays_initialized:
            print(f"[dSPACE] ❌ Fellow arrays still not initialized after {max_attempts} warm-up steps")
            print(f"[dSPACE] This may indicate fellows are not spawning in the simulation. Check ModelDesk configuration.")
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
                    print(f"[dSPACE] ExternalSignals ready at {vpath}[{idx_base}]")
                    return True
                except Exception:
                    continue
            sim._ext_v_path = vpath
            sim._ext_d_path = d_candidates[0]
            sim._ext_index_base = 0
            sim._ext_probe_done = True
            print(f"[dSPACE] ExternalSignals available via bulk at {vpath} (no element addressing)")
            return True
        except Exception:
            continue
    sim._ext_v_path = v_candidates[0]
    sim._ext_d_path = d_candidates[0]
    sim._ext_index_base = 0
    sim._ext_probe_done = True
    print("[dSPACE] ExternalSignals probe failed; using default paths")
    return False


