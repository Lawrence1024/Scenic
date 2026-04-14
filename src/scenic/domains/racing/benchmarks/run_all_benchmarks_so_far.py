#!/usr/bin/env python3
"""Run all implemented racing benchmark runners sequentially (single command).

Forwards all other CLI arguments to each runner (e.g. ``--time 2000``, ``--out-dir``,
``--scenario``, ``--inter-run-delay-s``). Stops at the first non-zero exit code.

This module consumes ``--from START`` itself; that flag is not passed to child runners.

Usage (repo root)::

    python -m scenic.domains.racing.benchmarks.run_all_benchmarks_so_far
    python -m scenic.domains.racing.benchmarks.run_all_benchmarks_so_far --time 2000 --inter-run-delay-s 5
    python -m scenic.domains.racing.benchmarks.run_all_benchmarks_so_far --from phase1 --time 2000

Order (default start is ``fellow_smoke``):
fellow_runner -> fellow_placement_debug_runner ->
phase0_runner -> phase1_runner -> phase2_runner -> phase3_runner -> phase4_runner -> phase5_runner -> phase6_runner.
"""

from __future__ import annotations

import argparse
import subprocess
import sys
from typing import List, Tuple


# (cli_label, python -m module) — single source of truth for sequencing.
_RUNNERS_ORDERED: Tuple[Tuple[str, str], ...] = (
    ("fellow_smoke", "scenic.domains.racing.benchmarks.fellow_runner"),
    ("fellow_placement", "scenic.domains.racing.benchmarks.fellow_placement_debug_runner"),
    ("phase0", "scenic.domains.racing.benchmarks.phase0_runner"),
    ("phase1", "scenic.domains.racing.benchmarks.phase1_runner"),
    ("phase2", "scenic.domains.racing.benchmarks.phase2_runner"),
    ("phase3", "scenic.domains.racing.benchmarks.phase3_runner"),
    ("phase4", "scenic.domains.racing.benchmarks.phase4_runner"),
    ("phase5", "scenic.domains.racing.benchmarks.phase5_runner"),
    ("phase6", "scenic.domains.racing.benchmarks.phase6_runner"),
)

_START_LABELS = tuple(label for label, _mod in _RUNNERS_ORDERED)

# Short aliases accepted for ``--from``.
_START_ALIASES = {
    "smoke": "fellow_smoke",
    "placement": "fellow_placement",
}


def _parse_start_label(raw: str) -> str:
    key = str(raw).strip().lower().replace("-", "_")
    key = _START_ALIASES.get(key, key)
    if key not in _START_LABELS:
        raise argparse.ArgumentTypeError(
            f"invalid --from {raw!r}; choose one of: {', '.join(_START_LABELS)} "
            f"(aliases: smoke -> fellow_smoke, placement -> fellow_placement)"
        )
    return key


def _modules_from(start_label: str) -> List[str]:
    idx = _START_LABELS.index(start_label)
    return [mod for _lbl, mod in _RUNNERS_ORDERED[idx:]]


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Run racing benchmark runners in sequence; forwards remaining args to each runner.",
    )
    parser.add_argument(
        "--from",
        dest="start_at",
        type=_parse_start_label,
        default="fellow_smoke",
        metavar="START",
        help=(
            "First runner to execute (inclusive); skips all earlier runners in the stack. "
            f"Choices: {', '.join(_START_LABELS)}. "
            "Aliases: smoke -> fellow_smoke, placement -> fellow_placement. "
            "Example: --from phase1 runs phase1 through phase6 only."
        ),
    )
    args, forwarded = parser.parse_known_args()
    modules = _modules_from(args.start_at)
    print(
        f"[run_all_benchmarks_so_far] Starting at {args.start_at!r} "
        f"({len(modules)} runner(s)); forwarding args: {forwarded!r}",
        flush=True,
    )
    for mod in modules:
        cmd = [sys.executable, "-m", mod, *forwarded]
        print(f"\n========== Running: {' '.join(cmd)} ==========\n", flush=True)
        proc = subprocess.run(cmd)
        if proc.returncode != 0:
            print(
                f"\n[run_all_benchmarks_so_far] Stopping: {mod} exited with {proc.returncode}",
                file=sys.stderr,
            )
            return int(proc.returncode)
    print("\n[run_all_benchmarks_so_far] All configured runners completed successfully.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())

