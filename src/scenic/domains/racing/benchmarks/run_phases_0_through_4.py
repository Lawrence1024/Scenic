#!/usr/bin/env python3
"""Run Phase 0–4 benchmark runners sequentially (single command).

Forwards all CLI arguments to **each** runner (e.g. ``--time 2000``, ``--out-dir``,
``--scenario``, ``--inter-run-delay-s``). Stops at the first non-zero exit code.

Usage (repo root)::

    python -m scenic.domains.racing.benchmarks.run_phases_0_through_4
    python -m scenic.domains.racing.benchmarks.run_phases_0_through_4 --time 2000 --inter-run-delay-s 5

Order: phase0_runner → phase1_runner → phase2_runner → phase3_runner → phase4_runner.
"""

from __future__ import annotations

import subprocess
import sys


_RUNNERS = (
    "scenic.domains.racing.benchmarks.phase0_runner",
    "scenic.domains.racing.benchmarks.phase1_runner",
    "scenic.domains.racing.benchmarks.phase2_runner",
    "scenic.domains.racing.benchmarks.phase3_runner",
    "scenic.domains.racing.benchmarks.phase4_runner",
)


def main() -> int:
    argv = sys.argv[1:]
    for mod in _RUNNERS:
        cmd = [sys.executable, "-m", mod, *argv]
        print(f"\n========== Running: {' '.join(cmd)} ==========\n", flush=True)
        proc = subprocess.run(cmd)
        if proc.returncode != 0:
            print(
                f"\n[run_phases_0_through_4] Stopping: {mod} exited with {proc.returncode}",
                file=sys.stderr,
            )
            return int(proc.returncode)
    print("\n[run_phases_0_through_4] All phase 0-4 runners completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
