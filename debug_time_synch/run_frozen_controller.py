#!/usr/bin/env python3
"""Frozen-controller time sync test: only advance_simulation_step(), log Scenic t vs dSPACE ManeuverTime.

No control logic, no MPC, no readback. Used to see what causes time discrepancy.

Prereqs: ControlDesk running, experiment loaded, platform ready for online calibration.

Run from Scenic repo root:
  python debug_time_synch/run_frozen_controller.py --steps 60 --timestep 0.01
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


def main():
    parser = argparse.ArgumentParser(
        description="Frozen controller: step only, log Scenic t vs dSPACE ManeuverTime."
    )
    parser.add_argument("--steps", type=int, default=60, help="Number of steps")
    parser.add_argument("--timestep", type=float, default=0.01, help="Simulation timestep (s)")
    parser.add_argument("--no-vesi", action="store_true", help="Skip VESI init")
    parser.add_argument("--quiet", action="store_true", help="Only print summary, not every step")
    args = parser.parse_args()

    # Minimal "sim" object for connect_and_prepare
    class MinimalSim:
        timestep = args.timestep

    sim = MinimalSim()

    print("[FrozenController] Connecting to ControlDesk...")
    cd = cd_session.connect_and_prepare(sim)
    if not cd:
        print("[FrozenController] ERROR: ControlDesk connection failed.")
        return 1

    print("[FrozenController] Starting maneuver and pausing for step-by-step...")
    if not cd_session.start_maneuver(cd):
        print("[FrozenController] ERROR: start_maneuver failed.")
        return 1
    time.sleep(0.3)
    cd_session.pause(cd)
    time.sleep(0.2)

    print(f"[FrozenController] Running {args.steps} steps (timestep={args.timestep}s).")
    print(f"[FrozenController] ManeuverTime path: {MANEUVER_TIME_PATH}")
    print()

    def read_maneuver_time():
        try:
            return float(cd.get_var(MANEUVER_TIME_PATH))
        except Exception as e:
            print(f"[FrozenController] get_var(ManeuverTime) failed: {e}")
            return float("nan")

    diffs = []
    for step in range(args.steps):
        if step < 20:
            # High-signal check: read ManeuverTime before and after step for first 20 steps
            dspace_t_before = read_maneuver_time()
            cd.advance_simulation_step()
            dspace_t_after = read_maneuver_time()
            delta_after_before = dspace_t_after - dspace_t_before if (dspace_t_before == dspace_t_before and dspace_t_after == dspace_t_after) else float("nan")
            print(
                f"  [before/after] step_idx={step}  dspace_t_before={dspace_t_before:.9f}s  "
                f"dspace_t_after={dspace_t_after:.9f}s  delta_after_before={delta_after_before:+.9f}s"
            )
            maneuver_time = dspace_t_after
        else:
            cd.advance_simulation_step()
            maneuver_time = read_maneuver_time()
        # After this step, Scenic would expect time = (step + 1) * timestep
        expected_scenic_t = (step + 1) * args.timestep
        diff = maneuver_time - expected_scenic_t
        diffs.append(diff)
        if not args.quiet:
            print(
                f"  step={step:4d}  Scenic_t={expected_scenic_t:.4f}s  "
                f"ManeuverTime={maneuver_time:.9f}s  diff={diff:+.4f}s"
            )

    # Summary
    valid = [d for d in diffs if d == d]  # exclude nan
    print()
    print("=== Time sync summary (frozen controller, no MPC/readback) ===")
    print(f"  Steps:        {len(diffs)}")
    if valid:
        print(f"  Diff (dSPACE - Scenic): min={min(valid):+.4f}s  max={max(valid):+.4f}s")
        avg = sum(valid) / len(valid)
        print(f"  Mean diff:   {avg:+.4f}s")
        if len(valid) >= 2:
            variance = sum((d - avg) ** 2 for d in valid) / (len(valid) - 1)
            print(f"  Std dev:     {variance ** 0.5:.4f}s")
        print()
        print("  If diff grows over steps -> drift (e.g. step count vs dSPACE time base).")
        print("  If diff ~0 -> Scenic and dSPACE time bases agree in this minimal loop.")
    else:
        print("  No valid ManeuverTime reads (check path / experiment).")
    return 0


if __name__ == "__main__":
    sys.exit(main())
