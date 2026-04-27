#!/usr/bin/env python3
"""Phase 6–12 validation runner: sequences phase6–phase12 runners, merges results.

Runs each child benchmark in subprocesses (same pattern as
``validation_full_stack_runner``), writes a parent folder under ``--out-dir``
with one subdirectory per child, then merges all ``summary.json`` result rows
into ``merged_summary.json`` and prints a single ``BENCHMARK_AI_DIGEST_*`` block.

Usage (repo root)::

    python -m scenic.domains.racing.benchmarks.validation_phase6_12_runner
    python -m scenic.domains.racing.benchmarks.validation_phase6_12_runner --time 1000
    python -m scenic.domains.racing.benchmarks.validation_phase6_12_runner --suite phases_only
    python -m scenic.domains.racing.benchmarks.validation_phase6_12_runner --continue-on-failure
    python -m scenic.domains.racing.benchmarks.validation_phase6_12_runner --repeats 3

Forwarded flags (e.g. ``--time``, ``--repeats``, ``--inter-run-delay-s``) are
passed to every child runner.
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
    benchmark_digest_aggregate,
    build_benchmark_ai_digest_payload,
    print_benchmark_ai_digest,
    repo_root,
    standard_benchmark_digest_keys_with_fellow,
)

# (short label, python -m module name)
_CHILD_PHASES: Tuple[Tuple[str, str], ...] = (
    ("phase6", "scenic.domains.racing.benchmarks.phase6_runner"),
    ("prediction", "scenic.domains.racing.benchmarks.prediction_runner"),
    ("phase8", "scenic.domains.racing.benchmarks.assessment_runner"),
    ("phase9", "scenic.domains.racing.benchmarks.phase9_runner"),
    ("phase10", "scenic.domains.racing.benchmarks.guard_runner"),
    ("phase11", "scenic.domains.racing.benchmarks.phase11_runner"),
    ("phase12", "scenic.domains.racing.benchmarks.phase12_runner"),
)

_SUITE_MODULES: Dict[str, Tuple[Tuple[str, str], ...]] = {
    "all": _CHILD_PHASES,
    "tactical": (
        ("phase10", "scenic.domains.racing.benchmarks.guard_runner"),
        ("phase11", "scenic.domains.racing.benchmarks.phase11_runner"),
        ("phase12", "scenic.domains.racing.benchmarks.phase12_runner"),
    ),
    "corner": (
        ("phase11", "scenic.domains.racing.benchmarks.phase11_runner"),
        ("phase12", "scenic.domains.racing.benchmarks.phase12_runner"),
    ),
}


def _find_latest_child_summary(bundle_phase_dir: Path) -> Optional[Path]:
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


def _parse_int_flag(forwarded: List[str], flag: str, default: int) -> int:
    for i, tok in enumerate(forwarded):
        if tok == flag and i + 1 < len(forwarded):
            try:
                return int(forwarded[i + 1])
            except ValueError:
                pass
    return default


def _parse_float_flag(forwarded: List[str], flag: str, default: float) -> float:
    for i, tok in enumerate(forwarded):
        if tok == flag and i + 1 < len(forwarded):
            try:
                return float(forwarded[i + 1])
            except ValueError:
                pass
    return default


def main() -> int:
    parser = argparse.ArgumentParser(
        description=(
            "Run phase6–phase12 benchmark runners under one parent folder; "
            "merge summary rows and print a combined digest."
        ),
    )
    parser.add_argument(
        "--suite",
        choices=tuple(_SUITE_MODULES.keys()),
        default="all",
        help=(
            "Which child runners to include. "
            "'all': phase6-12; 'tactical': phase10-12; 'corner': phase11-12."
        ),
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
    args, forwarded = parser.parse_known_args()

    modules = list(_SUITE_MODULES[args.suite])

    root = repo_root()
    parent_id = datetime.now(timezone.utc).strftime("phase6_12_%Y%m%d_%H%M%S")
    parent_rel = Path(args.out_dir) / parent_id
    parent_dir = (root / parent_rel).resolve()
    parent_dir.mkdir(parents=True, exist_ok=True)

    digest_keys: Tuple[str, ...] = tuple(standard_benchmark_digest_keys_with_fellow()) + (
        "source_child",
        "child_run_id",
    )

    sim_steps = _parse_int_flag(forwarded, "--time", 1000)
    time_step_s = _parse_float_flag(forwarded, "--time-step-s", 0.01)
    inter_run_delay_s = _parse_float_flag(forwarded, "--inter-run-delay-s", 15.0)

    child_meta: List[Dict[str, Any]] = []
    loaded: List[Tuple[str, str, Dict[str, Any]]] = []
    any_failure = False

    print(
        f"[Phase6_12Runner] bundle={parent_id} suite={args.suite!r} "
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
                f"[Phase6_12Runner] WARNING: {label} exited {proc.returncode}",
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
        "runner": "Phase6_12Runner",
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

    print(f"\n[Phase6_12Runner] Wrote {merged_path}", flush=True)

    scenario_dir_placeholder = root / "examples" / "racing"
    digest_extra: Dict[str, Any] = {
        "validation_suite": args.suite,
        "child_modules_run": [m[1] for m in modules],
        "children_detail": child_meta,
    }
    print_benchmark_ai_digest(
        build_benchmark_ai_digest_payload(
            runner_label="Phase6_12Runner",
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

    agg = benchmark_digest_aggregate(merged_results)
    print(f"[Phase6_12Runner] merged_rows={len(merged_results)} aggregate={agg}", flush=True)

    if any_failure and not args.continue_on_failure:
        return 1
    if any_failure:
        return 1
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
