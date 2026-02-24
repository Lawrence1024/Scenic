#!/usr/bin/env python3
"""Minimal script to measure pure SingleStep() round-trip time (no get_var/set_var).

Use this to see how much of step_time is VEOS vs COM/coordination overhead.
Prereqs: ControlDesk running, experiment loaded and ready for online calibration.

Run from Scenic repo root with PYTHONPATH including src, e.g.:
  PYTHONPATH=src python src/scenic/simulators/dspace/bench_single_step.py
Or from Scenic/src:
  python -m scenic.simulators.dspace.bench_single_step

Options:
  --steps N       Number of SingleStep() calls (default 200).
  --timestep T    SingleStepTime in seconds (default 0.01).
  --no-vesi       Skip VESI init (use if your experiment doesn't need it).
"""

import argparse
import statistics
import sys
import time
from pathlib import Path

# Allow running as script when scenic is not on path
def _ensure_import():
    try:
        import scenic.simulators.dspace.controldesk.connection  # noqa: F401
        return
    except ImportError:
        pass
    repo_root = Path(__file__).resolve()
    for _ in range(4):
        repo_root = repo_root.parent
    src = repo_root / "src"
    if src.is_dir():
        sys.path.insert(0, str(src))
    else:
        # Try Scenic/ as repo root (e.g. Scenic/src/scenic/...)
        repo_root = Path(__file__).resolve()
        for _ in range(3):
            repo_root = repo_root.parent
        sys.path.insert(0, str(repo_root))


_ensure_import()

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.controldesk import session as cd_session


def main():
    parser = argparse.ArgumentParser(description="Benchmark SingleStep() round-trip time only.")
    parser.add_argument("--steps", type=int, default=200, help="Number of SingleStep calls")
    parser.add_argument("--timestep", type=float, default=0.01, help="SingleStepTime in seconds")
    parser.add_argument("--no-vesi", action="store_true", help="Skip VESI interface init")
    args = parser.parse_args()

    print("[bench_single_step] Connecting to ControlDesk...")
    cd = ControlDeskApp().connect()
    cd.go_online()
    cd.start_measurement()
    if not args.no_vesi:
        print("[bench_single_step] Initializing VESI interface...")
        cd.initialize_vesi_interface()
    cd.set_simulation_step(args.timestep)
    print("[bench_single_step] Pausing simulation for step-by-step mode...")
    cd_session.pause(cd)
    if not cd:
        print("[bench_single_step] ERROR: No ControlDesk connection. Aborting.")
        return 1

    print(f"[bench_single_step] Running {args.steps} SingleStep() calls (SingleStepTime={args.timestep}s)...")
    times = []
    for i in range(args.steps):
        t0 = time.perf_counter()
        cd.advance_simulation_step()
        t1 = time.perf_counter()
        times.append(t1 - t0)

    # Stats
    n = len(times)
    mean_s = statistics.mean(times)
    min_s = min(times)
    max_s = max(times)
    std_s = statistics.stdev(times) if n > 1 else 0.0
    median_s = statistics.median(times)
    p95 = sorted(times)[int(0.95 * n) - 1] if n >= 20 else max_s

    print()
    print("=== SingleStep() only (no get_var/set_var) ===")
    print(f"  Steps:        {n}")
    print(f"  Mean:         {mean_s*1000:.2f} ms")
    print(f"  Median:       {median_s*1000:.2f} ms")
    print(f"  Min / Max:    {min_s*1000:.2f} / {max_s*1000:.2f} ms")
    print(f"  Std dev:      {std_s*1000:.2f} ms")
    print(f"  95th %ile:    {p95*1000:.2f} ms")
    print()
    sim_time_per_step_s = args.timestep
    wall_per_sim_s = mean_s / sim_time_per_step_s
    print(f"  Wall time per {args.timestep}s sim step: {mean_s*1000:.2f} ms")
    print(f"  Ratio (wall/sim): {wall_per_sim_s:.2f}x  (e.g. 0.25 = 4x faster than real time)")
    print()
    return 0


if __name__ == "__main__":
    sys.exit(main())
