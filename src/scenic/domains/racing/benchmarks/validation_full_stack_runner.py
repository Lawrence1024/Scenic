#!/usr/bin/env python3
"""Full-stack validation runner: sequences phase0–phase5 and fellow harness runners, merges results.

Runs each child benchmark in subprocesses (same pattern as ``run_all_benchmarks_so_far``),
writes a **parent** folder under ``--out-dir`` with one subdirectory per child, then merges
all ``summary.json`` result rows into ``merged_summary.json`` and prints a single
``BENCHMARK_AI_DIGEST_*`` block.

Usage (repo root)::

    python -m scenic.domains.racing.benchmarks.validation_full_stack_runner
    python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --time 3000
    python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --suite phases_only --time 2000
    python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --continue-on-failure
    python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --repeats 3 --time 3000

Forwarded flags (e.g. ``--time``, ``--repeats``, ``--inter-run-delay-s``) are passed to every child; all phase runners support ``--repeats``.

See ``src/scenic/domains/racing/plans/comprehensive-planner-validation-runner.md``.
"""

from __future__ import annotations

import argparse
import json
import subprocess
import sys
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple

from scenic.domains.racing.benchmarks.phase_run_common import (
    build_benchmark_ai_digest_payload,
    benchmark_digest_aggregate,
    print_benchmark_ai_digest,
    repo_root,
    standard_benchmark_digest_keys_with_fellow,
)

# (short label, python -m module name)
_CHILD_PHASES: Tuple[Tuple[str, str], ...] = (
    ("phase0", "scenic.domains.racing.benchmarks.phase0_runner"),
    ("phase1", "scenic.domains.racing.benchmarks.phase1_runner"),
    ("phase2", "scenic.domains.racing.benchmarks.phase2_runner"),
    ("phase3", "scenic.domains.racing.benchmarks.phase3_runner"),
    ("phase4", "scenic.domains.racing.benchmarks.phase4_runner"),
    ("phase5", "scenic.domains.racing.benchmarks.phase5_runner"),
)

_CHILD_FELLOW: Tuple[Tuple[str, str], ...] = (
    ("fellow_smoke", "scenic.domains.racing.benchmarks.fellow_runner"),
    ("fellow_placement", "scenic.domains.racing.benchmarks.fellow_placement_debug_runner"),
)

_SUITE_MODULES: Dict[str, Tuple[Tuple[str, str], ...]] = {
    "all": _CHILD_PHASES + _CHILD_FELLOW,
    "phases_only": _CHILD_PHASES,
    "minimal": (
        ("phase0", "scenic.domains.racing.benchmarks.phase0_runner"),
        ("phase5", "scenic.domains.racing.benchmarks.phase5_runner"),
        ("fellow_smoke", "scenic.domains.racing.benchmarks.fellow_runner"),
    ),
    "fellow_only": _CHILD_FELLOW,
}


def _find_latest_child_summary(bundle_phase_dir: Path) -> Optional[Path]:
    """Child writes ``bundle_phase_dir / <run_id> / summary.json``."""
    if not bundle_phase_dir.is_dir():
        return None
    candidates: List[Path] = []
    for sub in bundle_phase_dir.iterdir():
        if sub.is_dir():
            sj = sub / "summary.json"
            if sj.is_file():
                candidates.append(sj)
    if not candidates:
        return None
    candidates.sort(key=lambda p: p.stat().st_mtime)
    return candidates[-1]


def _load_summary(path: Path) -> Optional[Dict[str, Any]]:
    try:
        with open(path, "r", encoding="utf-8") as f:
            return json.load(f)
    except (OSError, json.JSONDecodeError):
        return None


def _merge_rows(
    summaries: Sequence[Tuple[str, str, Dict[str, Any]]],
) -> List[Dict[str, Any]]:
    """Attach ``source_child`` and ``child_run_id`` to each result row."""
    merged: List[Dict[str, Any]] = []
    for label, _mod, payload in summaries:
        run_id = payload.get("run_id")
        for row in payload.get("results") or []:
            if not isinstance(row, dict):
                continue
            r = dict(row)
            r["source_child"] = label
            r["child_run_id"] = run_id
            merged.append(r)
    return merged


def _parse_time_from_forwarded(forwarded: List[str], default: int) -> int:
    for i, tok in enumerate(forwarded):
        if tok == "--time" and i + 1 < len(forwarded):
            try:
                return int(forwarded[i + 1])
            except ValueError:
                pass
    return default


def _parse_time_step_from_forwarded(forwarded: List[str], default: float) -> float:
    for i, tok in enumerate(forwarded):
        if tok == "--time-step-s" and i + 1 < len(forwarded):
            try:
                return float(forwarded[i + 1])
            except ValueError:
                pass
    return default


def _parse_inter_run_delay_from_forwarded(forwarded: List[str], default: float) -> float:
    for i, tok in enumerate(forwarded):
        if tok == "--inter-run-delay-s" and i + 1 < len(forwarded):
            try:
                return float(forwarded[i + 1])
            except ValueError:
                pass
    return default


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run phase0–phase5 and fellow benchmark runners under one parent folder; "
            "merge summary rows and print a combined digest."
        ),
    )
    parser.add_argument(
        "--suite",
        choices=tuple(_SUITE_MODULES.keys()),
        default="all",
        help="Which child runners to include (default: all).",
    )
    parser.add_argument(
        "--out-dir",
        default="src/scenic/domains/racing/benchmarks/results",
        help="Base results directory (relative to repo root); parent bundle is created inside it.",
    )
    parser.add_argument(
        "--continue-on-failure",
        action="store_true",
        help="Run remaining children after a non-zero exit (merged report may be partial).",
    )
    parser.add_argument(
        "--skip-placement",
        action="store_true",
        help="When suite is 'all', skip fellow_placement_debug_runner.",
    )
    args, forwarded = parser.parse_known_args()

    modules = list(_SUITE_MODULES[args.suite])
    if args.suite == "all" and args.skip_placement:
        modules = [m for m in modules if m[0] != "fellow_placement"]

    root = repo_root()
    parent_id = datetime.now(timezone.utc).strftime("validation_full_stack_%Y%m%d_%H%M%S")
    parent_rel = Path(args.out_dir) / parent_id
    parent_dir = (root / parent_rel).resolve()
    parent_dir.mkdir(parents=True, exist_ok=True)

    digest_keys: Tuple[str, ...] = tuple(standard_benchmark_digest_keys_with_fellow()) + (
        "source_child",
        "child_run_id",
    )

    sim_steps = _parse_time_from_forwarded(forwarded, 2000)
    time_step_s = _parse_time_step_from_forwarded(forwarded, 0.01)
    inter_run_delay_s = _parse_inter_run_delay_from_forwarded(forwarded, 15.0)

    child_meta: List[Dict[str, Any]] = []
    loaded: List[Tuple[str, str, Dict[str, Any]]] = []
    any_failure = False

    print(
        f"[ValidationFullStackRunner] bundle={parent_id} suite={args.suite!r} "
        f"children={len(modules)} root={parent_dir}",
        flush=True,
    )

    for label, mod in modules:
        child_out_rel = parent_rel / label
        cmd = [
            sys.executable,
            "-m",
            mod,
            "--out-dir",
            child_out_rel.as_posix(),
            *forwarded,
        ]
        print(f"\n========== [{label}] {' '.join(cmd)} ==========\n", flush=True)
        proc = subprocess.run(cmd, cwd=str(root))
        child_bundle_dir = (root / child_out_rel).resolve()
        summary_path = _find_latest_child_summary(child_bundle_dir)
        payload: Optional[Dict[str, Any]] = None
        if summary_path:
            payload = _load_summary(summary_path)
        n_results = len((payload or {}).get("results") or [])
        meta = {
            "label": label,
            "module": mod,
            "return_code": int(proc.returncode),
            "child_bundle_dir": str(child_bundle_dir),
            "summary_json": str(summary_path) if summary_path else None,
            "scenario_count": n_results,
        }
        child_meta.append(meta)

        if proc.returncode != 0:
            any_failure = True
            print(
                f"[ValidationFullStackRunner] WARNING: {label} exited {proc.returncode}",
                file=sys.stderr,
                flush=True,
            )
            if not args.continue_on_failure:
                break
        if payload:
            loaded.append((label, mod, payload))

    merged_results = _merge_rows(loaded)

    merged_payload: Dict[str, Any] = {
        "run_id": parent_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "runner": "ValidationFullStackRunner",
        "suite": args.suite,
        "base_out_dir": str(args.out_dir),
        "parent_dir": str(parent_dir),
        "continue_on_failure": bool(args.continue_on_failure),
        "children": child_meta,
        "results": merged_results,
    }
    merged_path = parent_dir / "merged_summary.json"
    with open(merged_path, "w", encoding="utf-8") as f:
        json.dump(merged_payload, f, indent=2)

    print(f"\n[ValidationFullStackRunner] Wrote {merged_path}", flush=True)

    scenario_dir_placeholder = root / "examples" / "racing"
    digest_extra: Dict[str, Any] = {
        "validation_suite": args.suite,
        "child_modules_run": [m[1] for m in modules],
        "children_detail": child_meta,
    }
    print_benchmark_ai_digest(
        build_benchmark_ai_digest_payload(
            runner_label="ValidationFullStackRunner",
            run_id=parent_id,
            run_dir=parent_dir,
            scenario_dir=scenario_dir_placeholder,
            sim_steps=int(sim_steps),
            assumed_time_step_s=float(time_step_s),
            inter_run_delay_s=float(inter_run_delay_s),
            results=merged_results,
            digest_keys=list(digest_keys),
            extra=digest_extra,
        )
    )

    # Re-print aggregate in human form (digest aggregate is inside build_benchmark_ai_digest)
    agg = benchmark_digest_aggregate(merged_results)
    print(f"[ValidationFullStackRunner] merged_rows={len(merged_results)} aggregate={agg}", flush=True)

    if any_failure and not args.continue_on_failure:
        return 1
    if any_failure:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
