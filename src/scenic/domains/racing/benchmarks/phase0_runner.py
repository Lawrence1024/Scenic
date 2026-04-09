#!/usr/bin/env python3
"""Run Phase 0 benchmark scenarios and produce an automatic metrics report.

Outputs (per benchmark run set):
- one log file per scenario
- summary.json (all required Phase 0 KPIs)
- summary.csv (same core KPIs)

Usage (repo root):
    python -m scenic.domains.racing.benchmarks.phase0_runner --time 4500
    python -m scenic.domains.racing.benchmarks.phase0_runner --time 3000 --scenario 02_slower_opponent_left.scenic
    python -m scenic.domains.racing.benchmarks.phase0_runner --time 3000 --scenario-glob "0[0-2]_*.scenic"
    python -m scenic.domains.racing.benchmarks.phase0_runner --time 3000 --inter-run-delay-s 5
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import re
import subprocess
import sys
import time
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional

from scenic.domains.racing.mpc.result_data.analyze_racing_log import (
    compute_segment_times,
    parse_log,
)


RE_PHASE0_LINE = re.compile(
    r"\[Phase0\]\s+t=(?P<t>\d+\.?\d*)s\s+ttl=(?P<ttl>\S+)\s+planner_mode=(?P<mode>\S+)\s+"
    r"ego_s=(?P<ego_s>\S+)\s+ego_speed=(?P<ego_speed>\S+)\s+"
    r"nearest_opp_ds=(?P<opp_ds>\S+)\s+nearest_opp_rel_speed=(?P<opp_rel_v>\S+)\s+nearest_opp_dist=(?P<opp_dist>\S+)"
)
RE_PHASE0_EVENT = re.compile(r"\[Phase0Event\]\s+t=(?P<t>\d+\.?\d*)s\s+type=(?P<event>\S+)")


def _repo_root() -> Path:
    p = Path(__file__).resolve()
    for _ in range(12):
        if (p / "src").is_dir() and (p / "examples").is_dir():
            return p
        p = p.parent
    return Path.cwd()


def _parse_float_or_none(value: str) -> Optional[float]:
    if value is None:
        return None
    v = value.strip().lower()
    if v in ("na", "none", "nan", "?"):
        return None
    try:
        f = float(value)
    except (TypeError, ValueError):
        return None
    if math.isfinite(f):
        return f
    return None


def _collect_phase0_metrics_from_log(log_path: Path) -> Dict[str, Any]:
    min_opp_dist: Optional[float] = None
    ttl_seen: List[str] = []
    planner_modes: List[str] = []
    event_counts: Dict[str, int] = {}

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = RE_PHASE0_LINE.search(line)
            if m:
                ttl = m.group("ttl")
                mode = m.group("mode")
                ttl_seen.append(ttl)
                planner_modes.append(mode)
                d = _parse_float_or_none(m.group("opp_dist"))
                if d is not None:
                    min_opp_dist = d if min_opp_dist is None else min(min_opp_dist, d)
                continue
            e = RE_PHASE0_EVENT.search(line)
            if e:
                ev = e.group("event")
                event_counts[ev] = event_counts.get(ev, 0) + 1

    return {
        "min_opponent_distance_m": min_opp_dist,
        "phase0_samples": len(ttl_seen),
        "ttls_observed": sorted(set(ttl_seen)),
        "planner_modes_observed": sorted(set(planner_modes)),
        "ttl_switch_count": int(event_counts.get("ttl_switch", 0)),
        "near_miss_count": int(event_counts.get("near_miss", 0)),
        "collision_count": int(event_counts.get("collision", 0)),
        "off_track_count": int(event_counts.get("off_track", 0)),
    }


def _analyze_waypoint_timing(log_path: Path) -> Dict[str, Any]:
    try:
        events, _mpc, _run_info = parse_log(log_path)
    except Exception as e:
        return {
            "lap_completion_status": "parse_error",
            "lap_time_s": None,
            "waypoint_hits": 0,
            "parse_error": str(e),
        }
    if not events:
        return {
            "lap_completion_status": "no_waypoint_events",
            "lap_time_s": None,
            "waypoint_hits": 0,
        }
    seg_time_s, _seg_name_map, seg_waypoint_count, total_time_s = compute_segment_times(events)
    _ = seg_time_s
    return {
        "lap_completion_status": "completed",
        "lap_time_s": float(total_time_s),
        "waypoint_hits": int(sum(seg_waypoint_count.values())),
    }


def _run_one_scenario(repo_root: Path, scenario: Path, out_log: Path, sim_steps: int) -> Dict[str, Any]:
    cmd = [
        sys.executable,
        "-m",
        "scenic",
        str(scenario),
        "--2d",
        "--model",
        "scenic.simulators.dspace.racing_model",
        "--simulate",
        "-b",
        "--count",
        "1",
        "--time",
        str(int(sim_steps)),
    ]
    out_log.parent.mkdir(parents=True, exist_ok=True)
    with open(out_log, "w", encoding="utf-8") as f:
        proc = subprocess.run(
            cmd,
            cwd=str(repo_root),
            stdout=f,
            stderr=subprocess.STDOUT,
            text=True,
        )
    base = {
        "scenario": str(scenario.relative_to(repo_root)),
        "log": str(out_log),
        "return_code": int(proc.returncode),
    }
    base.update(_collect_phase0_metrics_from_log(out_log))
    base.update(_analyze_waypoint_timing(out_log))
    base["collision"] = bool(base.get("collision_count", 0) > 0)
    base["off_track"] = bool(base.get("off_track_count", 0) > 0)
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Phase 0 racing benchmarks and emit metrics summary.")
    parser.add_argument(
        "--scenario-dir",
        default="examples/racing/phase0_benchmark",
        help="Directory with benchmark .scenic files (default: examples/racing/phase0_benchmark)",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help=(
            "Specific scenario filename(s) to run from scenario-dir. "
            "Repeat flag for multiple values, e.g. --scenario 00_no_opponent.scenic --scenario 01_slower_opponent_optimal.scenic"
        ),
    )
    parser.add_argument(
        "--scenario-glob",
        default=None,
        help='Glob pattern (within scenario-dir) to select subset, e.g. "02_*.scenic" or "0[0-2]_*.scenic".',
    )
    parser.add_argument(
        "--time",
        type=int,
        default=4500,
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
        default=0.0,
        help="Delay (seconds) between scenarios; clamped to [0, 15].",
    )
    args = parser.parse_args()

    root = _repo_root()
    scenario_dir = root / args.scenario_dir
    if not scenario_dir.is_dir():
        print(f"Scenario directory not found: {scenario_dir}", file=sys.stderr)
        return 2

    scenarios = sorted(scenario_dir.glob("*.scenic"))
    if not scenarios:
        print(f"No .scenic files found in {scenario_dir}", file=sys.stderr)
        return 2

    # Optional filtering by exact filename(s) and/or glob.
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
        pattern = str(args.scenario_glob)
        allowed = {p.resolve() for p in scenario_dir.glob(pattern)}
        scenarios = [s for s in scenarios if s.resolve() in allowed]

    if not scenarios:
        print(
            f"No scenarios selected after filtering in {scenario_dir}. "
            "Check --scenario and/or --scenario-glob values.",
            file=sys.stderr,
        )
        return 2

    run_id = datetime.now(timezone.utc).strftime("phase0_%Y%m%d_%H%M%S")
    run_dir = (root / args.out_dir / run_id).resolve()
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    inter_run_delay_s = max(0.0, min(15.0, float(args.inter_run_delay_s)))
    if float(args.inter_run_delay_s) != inter_run_delay_s:
        print(f"[Phase0Runner] inter-run delay clamped to {inter_run_delay_s:.2f}s (requested {args.inter_run_delay_s}).")

    results: List[Dict[str, Any]] = []
    for idx, scenario in enumerate(scenarios):
        log_path = logs_dir / f"{scenario.stem}.log"
        print(f"[Phase0Runner] Running {scenario.name} ...")
        row = _run_one_scenario(root, scenario, log_path, int(args.time))
        results.append(row)
        approx_sim_s = args.time * max(0.0, float(args.time_step_s))
        print(
            f"[Phase0Runner] {scenario.stem}: rc={row['return_code']} "
            f"lap={row['lap_completion_status']} lap_time_s={row['lap_time_s']} "
            f"(requested_steps={int(args.time)} ~= {approx_sim_s:.2f}s) "
            f"ttl_switches={row['ttl_switch_count']} min_opp_dist={row['min_opponent_distance_m']} "
            f"collision={row['collision']} off_track={row['off_track']}"
        )
        if inter_run_delay_s > 0 and idx < (len(scenarios) - 1):
            print(f"[Phase0Runner] Waiting {inter_run_delay_s:.2f}s before next scenario...")
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
        "ttl_switch_count",
        "min_opponent_distance_m",
        "collision",
        "off_track",
        "near_miss_count",
        "waypoint_hits",
        "phase0_samples",
    ]
    with open(summary_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        writer.writeheader()
        for row in results:
            writer.writerow({k: row.get(k) for k in csv_fields})

    print(f"\n[Phase0Runner] Wrote {summary_json}")
    print(f"[Phase0Runner] Wrote {summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
