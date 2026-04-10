#!/usr/bin/env python3
"""Run Phase 1 planner-MPC integration scenarios and produce metrics.

Usage (repo root):
    python -m scenic.domains.racing.benchmarks.phase1_runner --time 2000
    python -m scenic.domains.racing.benchmarks.phase1_runner --time 2000 --scenario 01_optimal_to_left.scenic
"""

from __future__ import annotations

import argparse
import csv
import json
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List

from scenic.domains.racing.benchmarks.phase_run_common import (
    FELLOW_HARNESS_SUMMARY_KEYS,
    analyze_waypoint_timing,
    build_benchmark_ai_digest_payload,
    collect_fellow_harness_metrics_from_log,
    collect_metrics_from_log,
    finalize_row,
    print_benchmark_ai_digest,
    repo_root,
    run_scenic_scenario,
    standard_benchmark_digest_keys_with_fellow,
)


def _run_one_scenario(root: Path, scenario: Path, out_log: Path, sim_steps: int) -> Dict[str, Any]:
    base = run_scenic_scenario(root, scenario, out_log, sim_steps)
    base.update(
        collect_metrics_from_log(out_log, phase1_switches=True),
    )
    base.update(collect_fellow_harness_metrics_from_log(out_log))
    base.update(analyze_waypoint_timing(out_log))
    return finalize_row(base)


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Phase 1 planner-MPC scenarios and emit metrics summary.")
    parser.add_argument(
        "--scenario-dir",
        default="examples/racing/phase1_planner",
        help="Directory with phase1 .scenic files (default: examples/racing/phase1_planner)",
    )
    parser.add_argument("--scenario", action="append", default=[], help="Scenario filename(s) in scenario-dir.")
    parser.add_argument(
        "--scenario-glob",
        default=None,
        help='Glob pattern within scenario-dir, e.g. "01_*.scenic".',
    )
    parser.add_argument(
        "--time",
        type=int,
        default=2000,
        help="Simulation steps per scenario (Scenic --time is step count, not seconds).",
    )
    parser.add_argument(
        "--time-step-s",
        type=float,
        default=0.01,
        help="Approximate simulation step size in seconds for display only (default 0.01).",
    )
    parser.add_argument(
        "--out-dir",
        default="src/scenic/domains/racing/benchmarks/results",
        help="Output directory root for logs and summaries.",
    )
    parser.add_argument(
        "--inter-run-delay-s",
        type=float,
        default=15.0,
        help="Delay (seconds) between scenarios; clamped to [0, 15]. Default is 15s.",
    )
    args = parser.parse_args()

    root = repo_root()
    scenario_dir = root / args.scenario_dir
    if not scenario_dir.is_dir():
        print(f"Scenario directory not found: {scenario_dir}", file=sys.stderr)
        return 2

    scenarios = sorted(scenario_dir.glob("*.scenic"))
    if not scenarios:
        print(f"No .scenic files found in {scenario_dir}", file=sys.stderr)
        return 2

    if args.scenario:
        wanted = set(args.scenario)
        scenarios = [s for s in scenarios if s.name in wanted]
        missing = sorted(wanted - {s.name for s in scenarios})
        if missing:
            print(
                f"Warning: requested --scenario value(s) not found in {scenario_dir}: {', '.join(missing)}",
                file=sys.stderr,
            )
    if args.scenario_glob:
        allowed = {p.resolve() for p in scenario_dir.glob(str(args.scenario_glob))}
        scenarios = [s for s in scenarios if s.resolve() in allowed]

    if not scenarios:
        print(
            f"No scenarios selected after filtering in {scenario_dir}. "
            "Check --scenario and/or --scenario-glob values.",
            file=sys.stderr,
        )
        return 2

    run_id = datetime.now(timezone.utc).strftime("phase1_%Y%m%d_%H%M%S")
    run_dir = (root / args.out_dir / run_id).resolve()
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    inter_run_delay_s = max(0.0, min(15.0, float(args.inter_run_delay_s)))
    if float(args.inter_run_delay_s) != inter_run_delay_s:
        print(f"[Phase1Runner] inter-run delay clamped to {inter_run_delay_s:.2f}s (requested {args.inter_run_delay_s}).")

    results: List[Dict[str, Any]] = []
    for idx, scenario in enumerate(scenarios):
        log_path = logs_dir / f"{scenario.stem}.log"
        print(f"[Phase1Runner] Running {scenario.name} ...")
        row = _run_one_scenario(root, scenario, log_path, int(args.time))
        results.append(row)
        approx_sim_s = args.time * max(0.0, float(args.time_step_s))
        print(
            f"[Phase1Runner] {scenario.stem}: rc={row['return_code']} "
            f"lap={row['lap_completion_status']} lap_time_s={row['lap_time_s']} "
            f"(requested_steps={int(args.time)} ~= {approx_sim_s:.2f}s) "
            f"phase1_switch_observed={row['phase1_switch_observed']} "
            f"ttl_switches={row['ttl_switch_count']} collision={row['collision']} off_track={row['off_track']}"
        )
        if inter_run_delay_s > 0 and idx < (len(scenarios) - 1):
            print(f"[Phase1Runner] Waiting {inter_run_delay_s:.2f}s before next scenario...")
            time.sleep(inter_run_delay_s)

    summary_json = run_dir / "summary.json"
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(
            {
                "run_id": run_id,
                "created_utc": datetime.now(timezone.utc).isoformat(),
                "scenario_dir": str(scenario_dir),
                "sim_steps": args.time,
                "assumed_time_step_s": args.time_step_s,
                "approx_requested_sim_time_s": args.time * max(0.0, float(args.time_step_s)),
                "inter_run_delay_s": inter_run_delay_s,
                "results": results,
            },
            f,
            indent=2,
        )

    summary_csv = run_dir / "summary.csv"
    csv_fields = [
        "scenario",
        "return_code",
        "lap_completion_status",
        "lap_time_s",
        "phase1_switch_observed",
        "phase1_switch_count",
        "ttl_switch_count",
        "min_opponent_distance_m",
        "collision",
        "off_track",
        "near_miss_count",
        "waypoint_hits",
        "phase0_samples",
        *FELLOW_HARNESS_SUMMARY_KEYS,
    ]
    with open(summary_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        writer.writeheader()
        for row in results:
            writer.writerow({k: row.get(k) for k in csv_fields})

    print(f"\n[Phase1Runner] Wrote {summary_json}")
    print(f"[Phase1Runner] Wrote {summary_csv}")
    print_benchmark_ai_digest(
        build_benchmark_ai_digest_payload(
            runner_label="Phase1Runner",
            run_id=run_id,
            run_dir=run_dir,
            scenario_dir=scenario_dir,
            sim_steps=int(args.time),
            assumed_time_step_s=float(args.time_step_s),
            inter_run_delay_s=inter_run_delay_s,
            results=results,
            digest_keys=standard_benchmark_digest_keys_with_fellow(),
        )
    )
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
