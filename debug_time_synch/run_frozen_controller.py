#!/usr/bin/env python3
"""Frozen-controller time sync test: trigger a step every x seconds (wall clock), record zeroed wall time, ManeuverTime, SimulationTime.

No control logic, no MPC, no readback. Step is triggered every --interval seconds; after each step we record wall_t, ManeuverTime, SimTime (all zeroed at step 0).

Prereqs: ControlDesk running, experiment loaded, platform ready for online calibration.

Run from Scenic repo root:
  python debug_time_synch/run_frozen_controller.py --steps 60 --interval 0.5
"""

import argparse
import sys
import time
from pathlib import Path

# Ensure we can import scenic.simulators.dspace
def _ensure_import():
    repo_root = Path(__file__).resolve().parent.parent
    src = repo_root / "src"
    if src.is_dir():
        sys.path.insert(0, str(src))
    else:
        sys.path.insert(0, str(repo_root))


_ensure_import()

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk import session as cd_session
from scenic.simulators.dspace.simulator import MANEUVER_TIME_PATH

# ControlDesk variable path for simulation time (Simulation and RTOS)
SIMULATION_TIME_PATH = "Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime"


def main():
    parser = argparse.ArgumentParser(
        description="Trigger a step every x seconds (wall clock); record zeroed wall time, ManeuverTime, SimTime."
    )
    parser.add_argument("--steps", type=int, default=60, help="Number of steps")
    parser.add_argument("--interval", type=float, default=0.5, help="Trigger one step every N seconds (wall clock)")
    parser.add_argument("--hold-duration", type=float, default=30.0, help="After steps: read-only phase duration (s); 0 = skip")
    parser.add_argument("--hold-interval", type=float, default=1.0, help="Read sampling interval (s) during hold phase")
    parser.add_argument("--no-vesi", action="store_true", help="Skip VESI init")
    parser.add_argument("--quiet", action="store_true", help="Only print summary, not every step")
    args = parser.parse_args()
    # SingleStepTime in dSPACE: set to same as interval so each step advances sim by that amount
    args.timestep = args.interval

    # Minimal "sim" object for connect_and_prepare
    class MinimalSim:
        timestep = args.timestep

    sim = MinimalSim()

    print("[FrozenController] Connecting to ControlDesk...")
    cd = cd_session.connect_and_prepare(sim)
    if not cd:
        print("[FrozenController] ERROR: ControlDesk connection failed.")
        return 1

    # Ensure dSPACE SingleStepTime matches our timestep (connect_and_prepare already sets it;
    # this makes the script explicitly set it so each advance_simulation_step() advances by args.timestep)
    print(f"[FrozenController] Setting dSPACE SingleStepTime = {args.timestep}s ({1.0 / args.timestep:.0f} Hz)...")
    cd.set_simulation_step(args.timestep)

    print("[FrozenController] Starting maneuver...")
    if not cd_session.start_maneuver(cd):
        print("[FrozenController] ERROR: start_maneuver failed.")
        return 1
    print("[FrozenController] Starting simulation (so time advances), then pausing for step-by-step...")
    cd.start_simulation()
    time.sleep(0.5)  # allow simulation to enter running state
    cd_session.pause(cd)

    print(f"[FrozenController] Trigger one step every {args.interval}s (wall clock), {args.steps} steps.")
    print(f"[FrozenController] ManeuverTime path: {MANEUVER_TIME_PATH}")
    print(f"[FrozenController] SimulationTime path: {SIMULATION_TIME_PATH}")
    print()

    def read_var(path, name):
        try:
            return float(cd.get_var(path))
        except Exception as e:
            print(f"[FrozenController] get_var({name}) failed: {e}")
            return float("nan")

    def read_maneuver_time():
        return read_var(MANEUVER_TIME_PATH, "ManeuverTime")

    def read_simulation_time():
        return read_var(SIMULATION_TIME_PATH, "SimulationTime")

    t0_maneuver = None
    t0_simulation = None
    t0_wall = None  # set at step 0 to zero wall/maneuver/sim
    prev_maneuver_time = None
    print("[FrozenController] Sleeping 3s before trigger loop...")
    time.sleep(3)
    t_start = time.perf_counter()
    if not args.quiet:
        print(f"  {'step':>4}  {'wall_t_0':>8}  {'ManeuverTime_0':>14}  {'SimTime_0':>10}  {'dManeuver':>10}")
    for step in range(args.steps):
        # Wait until next trigger time (every interval seconds from start)
        next_trigger = t_start + (step + 1) * args.interval
        now = time.perf_counter()
        sleep_duration = next_trigger - now
        if sleep_duration > 0:
            time.sleep(sleep_duration)
        cd.advance_simulation_step()
        wall_t = time.perf_counter() - t_start
        maneuver_time = read_maneuver_time()
        simulation_time = read_simulation_time()
        # Delta: ManeuverTime(i) - ManeuverTime(i-1)
        if prev_maneuver_time is not None and maneuver_time == maneuver_time and prev_maneuver_time == prev_maneuver_time:
            delta_maneuver = maneuver_time - prev_maneuver_time
        else:
            delta_maneuver = float("nan")
        prev_maneuver_time = maneuver_time
        # Zero all at step 0
        if step == 0:
            t0_wall = wall_t
            t0_maneuver = maneuver_time
            t0_simulation = simulation_time
        zeroed_wall_t = (wall_t - t0_wall) if t0_wall is not None else 0.0
        zeroed_maneuver = (maneuver_time - t0_maneuver) if (t0_maneuver == t0_maneuver and maneuver_time == maneuver_time) else float("nan")
        zeroed_simulation = (simulation_time - t0_simulation) if (t0_simulation == t0_simulation and simulation_time == simulation_time) else float("nan")
        if not args.quiet:
            d_str = f"{delta_maneuver:10.4f}" if delta_maneuver == delta_maneuver else "       -  "
            print(f"  {step:4d}  {zeroed_wall_t:8.4f}  {zeroed_maneuver:14.4f}  {zeroed_simulation:10.4f}  {d_str}")

    # Phase 2: no steps, just read SimTime/ManeuverTime to see if they catch up to wall time
    if args.hold_duration > 0 and t0_wall is not None and t0_maneuver is not None and t0_simulation is not None:
        print()
        print("[FrozenController] No more steps; reading wall_t, ManeuverTime, SimTime every {:.1f}s for {:.0f}s (see if sim time catches up).".format(args.hold_interval, args.hold_duration))
        if not args.quiet:
            print(f"  {'read':>4}  {'wall_t_0':>8}  {'ManeuverTime_0':>14}  {'SimTime_0':>10}")
        t_hold_start = time.perf_counter() - t_start
        n_hold = 0
        while True:
            next_read = t_start + t_hold_start + (n_hold + 1) * args.hold_interval
            now = time.perf_counter()
            if now - (t_start + t_hold_start) >= args.hold_duration:
                break
            sleep_duration = next_read - now
            if sleep_duration > 0:
                time.sleep(sleep_duration)
            wall_t = time.perf_counter() - t_start
            maneuver_time = read_maneuver_time()
            simulation_time = read_simulation_time()
            zeroed_wall_t = wall_t - t0_wall
            zeroed_maneuver = (maneuver_time - t0_maneuver) if (t0_maneuver == t0_maneuver and maneuver_time == maneuver_time) else float("nan")
            zeroed_simulation = (simulation_time - t0_simulation) if (t0_simulation == t0_simulation and simulation_time == simulation_time) else float("nan")
            if not args.quiet:
                print(f"  {n_hold:4d}  {zeroed_wall_t:8.4f}  {zeroed_maneuver:14.4f}  {zeroed_simulation:10.4f}")
            n_hold += 1

    print()
    print("=== Time sync (trigger every {:.2f}s wall clock, {} steps) ===".format(args.interval, args.steps))
    if args.hold_duration > 0:
        print("  Hold phase: no steps, read-only. If SimTime_0 stays flat, sim is paused; if it increases, something else is advancing it.")
    print("  All times zeroed at step 0. Compare wall_t_0 vs ManeuverTime_0 vs SimTime_0 for drift.")
    return 0


if __name__ == "__main__":
    sys.exit(main())
