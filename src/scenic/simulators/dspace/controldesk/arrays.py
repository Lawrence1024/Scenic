import time

# Path for simulated time (kept for the inner per-step poll only; SimulationTime
# is the free-running VEOS clock that ticks even when the maneuver engine is
# dormant).
_SIMULATED_TIME_PATH = "Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime"

# Path for ManeuverTime — the maneuver-engine clock. Starts at 0 when the
# dSPACE maneuver engine begins running and drives the plant behaviour. This
# is the right reference for warmup so the post-warmup state is independent
# of how the wall-clock-based "did the maneuver start?" poll in
# simulator.py:setup() caught the initial ManeuverTime value (which can vary
# from ~0 to ~0.4 s depending on poll latency).
_MANEUVER_TIME_PATH = (
    "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/"
    "DISP_Plant/ManeuverTime[s]/Out1"
)

# SD-23: warmup runs until ManeuverTime reaches this absolute target (seconds).
# Pre-SD-23 the warmup advanced 3 s of *SimulationTime* from wherever VEOS
# happened to be, which made post-warmup ManeuverTime depend on the wall-clock
# offset of the initial poll. The new target makes MPC tick 1 always start at
# ManeuverTime ∈ [target, target + sim.timestep] (one-tick overshoot only).
WARMUP_MANEUVER_TIME = 3.0
# Backwards-compat alias; some external scripts may import this name.
WARMUP_SIM_DURATION = WARMUP_MANEUVER_TIME


def _advance_one_step(sim):
    """Advance simulation by one timestep (poll until simulated time advances). Used during warmup only."""
    # Under CoSim, VEOS advances via the SyncStepBridge gate, not ControlDesk SingleStep.
    if getattr(sim, "_sync_bridge_step_controlled", False) and getattr(sim, "_sync_bridge", None) is not None:
        try:
            sim._sync_bridge.step(timeout=10.0)
            return True
        except Exception:
            return False

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
    """Advance the simulation until ManeuverTime >= WARMUP_MANEUVER_TIME, then
    mark fellow arrays ready.

    SD-23: target is ManeuverTime (the dSPACE maneuver-engine clock), not
    SimulationTime (the free-running VEOS clock). With a SimulationTime
    target, two runs of the same scenario reached MPC tick 1 at different
    ManeuverTime values because the initial "did the maneuver start?" poll
    in simulator.py setup is wall-clock-paced (time.sleep(0.2)) — it can
    catch ManeuverTime anywhere in [~0, ~0.4 s]. By targeting ManeuverTime
    directly, the post-warmup state is in [target, target + sim.timestep]
    regardless of how late we caught the start. This eliminates the
    biggest source of run-to-run trajectory divergence at fixed seed.
    """
    if sim._fellow_arrays_initialized or sim._initializing_fellow_arrays:
        return
    var = getattr(sim, "_var_access", None) or getattr(sim, "_cd", None)
    if not var:
        return
    sim._initializing_fellow_arrays = True
    try:
        try:
            mt_start = float(var.get_var(_MANEUVER_TIME_PATH))
        except Exception:
            mt_start = 0.0
        # Cap loop count: target/timestep + safety margin. The actual count
        # will be (WARMUP_MANEUVER_TIME - mt_start) / timestep ticks; we add
        # 100 to absorb edge-of-tolerance cases without wedging on a stuck
        # ManeuverTime read.
        max_steps = int(WARMUP_MANEUVER_TIME / sim.timestep) + 100
        print(
            f"[dSPACE] Warmup: advancing until ManeuverTime >= "
            f"{WARMUP_MANEUVER_TIME:.2f} s (from {mt_start:.4f} s)..."
        )
        step_count = 0
        for _ in range(max_steps):
            if not _advance_one_step(sim):
                print(f"[dSPACE] [WARN] Warmup step {step_count + 1} timed out")
            step_count += 1
            try:
                mt_now = float(var.get_var(_MANEUVER_TIME_PATH))
                if mt_now >= WARMUP_MANEUVER_TIME:
                    break
            except Exception:
                pass
        # SD-23 settling block: if the loop broke very close to the target
        # (less than half a timestep past it), advance one more tick so all
        # runs are guaranteed at least half a timestep past the threshold.
        # This collapses the "did we just barely cross or did we overshoot
        # by a full tick?" variance — both runs end in
        # [target + 0.5*timestep, target + 1.5*timestep] regardless of
        # rounding direction. A free-running VEOS clock + floating-point
        # comparison can otherwise put A and B on opposite sides of the
        # break threshold for the same intended target.
        try:
            mt_after_loop = float(var.get_var(_MANEUVER_TIME_PATH))
        except Exception:
            mt_after_loop = float("nan")
        _settle_added = 0
        if (
            mt_after_loop == mt_after_loop  # not NaN
            and (mt_after_loop - WARMUP_MANEUVER_TIME) < 0.5 * sim.timestep
        ):
            if _advance_one_step(sim):
                _settle_added = 1
            else:
                print("[dSPACE] [WARN] Warmup settling step timed out")
                step_count += 0  # leave the count alone; the warning is enough
        sim._fellow_arrays_initialized = True
        sim._fellow_index_base = 0
        try:
            mt_final = float(var.get_var(_MANEUVER_TIME_PATH))
        except Exception:
            mt_final = float("nan")
        # Telemetry: mt_final should agree across two same-seed runs to within
        # roughly half a sim.timestep (5 ms at the 10 ms default). If A and B
        # diverge by more than that on this line, the discrepancy is upstream
        # of the warmup (initial-poll latency or VEOS-side state). Useful
        # diagnostic line for SD-23 follow-up A/B determinism comparisons.
        print(
            f"[dSPACE] Warmup done: ManeuverTime = {mt_final:.4f} s after "
            f"{step_count} advances (+{_settle_added} settle); "
            f"target = {WARMUP_MANEUVER_TIME:.2f} s, "
            f"overshoot = {(mt_final - WARMUP_MANEUVER_TIME)*1000:.2f} ms."
        )
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
