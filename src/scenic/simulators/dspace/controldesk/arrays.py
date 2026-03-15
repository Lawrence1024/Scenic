import time

# Path for simulated time (used for deterministic warmup; same as simulator.SIMULATED_TIME_PATH)
_SIMULATED_TIME_PATH = "Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime"

# Fixed warmup duration in simulated time (seconds) for deterministic runs
WARMUP_SIM_DURATION = 3


def _advance_one_step(sim):
    """Advance simulation by one timestep (poll until simulated time advances). Used during warmup only."""
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    cd = getattr(sim, "_cd", None)
    if not var:
        return False
    try:
        t_before = float(var.get_var(_SIMULATED_TIME_PATH))
    except Exception:
        return False
    deadline = t_before + sim.timestep * 0.9
    poll_timeout_wall = min(10.0 * sim.timestep, 0.1)
    max_retries = 10
    for _ in range(max_retries):
        if cd:
            cd.advance_simulation_step()
        poll_start = time.perf_counter()
        while time.perf_counter() - poll_start < poll_timeout_wall:
            try:
                t_now = float(var.get_var(_SIMULATED_TIME_PATH))
                if t_now >= deadline:
                    return True
            except Exception:
                pass
            time.sleep(0.001)
    return False


def ensure_fellow_arrays_initialized(sim):
    """Run warmup until 3 s simulated time has elapsed (poll sim time), then mark fellow arrays ready. Deterministic in sim time."""
    if sim._fellow_arrays_initialized or sim._initializing_fellow_arrays:
        return
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return
    sim._initializing_fellow_arrays = True
    try:
        try:
            t_start = float(var.get_var(_SIMULATED_TIME_PATH))
        except Exception:
            t_start = 0.0
        target_sim_time = t_start + WARMUP_SIM_DURATION
        max_steps = int(WARMUP_SIM_DURATION / sim.timestep) + 100  # safeguard
        print(f"[dSPACE] Warmup: advancing until sim time >= {target_sim_time:.2f} s (from {t_start:.2f} s)...")
        step_count = 0
        for _ in range(max_steps):
            if not _advance_one_step(sim):
                print(f"[dSPACE] [WARN] Warmup step {step_count + 1} timed out")
            step_count += 1
            try:
                t_now = float(var.get_var(_SIMULATED_TIME_PATH))
                if t_now >= target_sim_time:
                    break
            except Exception:
                pass
        sim._fellow_arrays_initialized = True
        sim._fellow_index_base = 0
        print(f"[dSPACE] Warmup done: sim time >= {target_sim_time:.2f} s after {step_count} advances.")
    finally:
        sim._initializing_fellow_arrays = False


def probe_external_index_base(sim):
    """Probe External_Signals arrays and determine index base (0 or 1)."""
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if sim._ext_probe_done or not var:
        return sim._ext_probe_done
    base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
    v_candidates = (
        f"{base}/Const_v_Fellows_External[km|h]/Value",
        f"{base}/Const_v_Fellows_External[km/h]/Value",
    )
    d_candidates = (f"{base}/Const_d_Fellows_External[m]/Value",)
    for vpath in v_candidates:
        try:
            arr = var.get_var(vpath)
            if not isinstance(arr, (list, tuple)):
                continue
            for idx_base in (0, 1):
                try:
                    _ = var.get_var(f"{vpath}[{idx_base}]")
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
