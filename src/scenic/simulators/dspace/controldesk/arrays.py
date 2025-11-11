import time


def ensure_fellow_arrays_initialized(sim):
    """Advance the simulation until fellow arrays are populated (once)."""
    if sim._fellow_arrays_initialized or sim._initializing_fellow_arrays or not sim._cd:
        return
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    array_path = f"{base_path}/x"
    sim._initializing_fellow_arrays = True
    try:
        for attempt in range(200):
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
            if isinstance(x_arr, (list, tuple)) and len(x_arr) > 0:
                def has_signal(arr):
                    try:
                        for i in range(min(5, len(arr))):
                            val = arr[i]
                            if isinstance(val, (int, float)) and abs(val) > 1e-6:
                                return True
                    except Exception:
                        pass
                    return False
                ready = has_signal(x_arr) or has_signal(y_arr or []) or has_signal(yaw_arr or [])

            if ready:
                sim._fellow_arrays_initialized = True
                sim._fellow_index_base = 0
                if attempt > 0:
                    print(f"[dSPACE] Fellow arrays populated after {attempt} warm-up step(s)")
                break

            if attempt < 199:
                try:
                    sim._cd.advance_simulation_step()
                except Exception as step_err:
                    print(f"[dSPACE] Unable to advance simulation during fellow array initialization: {step_err}")
                    break
                time.sleep(sim.timestep)
        if not sim._fellow_arrays_initialized:
            print("[dSPACE] Fellow arrays still not initialized after warm-up steps")
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


