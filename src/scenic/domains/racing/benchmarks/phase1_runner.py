#!/usr/bin/env python3
"""Run Phase 1 planner-MPC integration scenarios and produce metrics.

Outputs (per run):
- one log file per scenario
- summary.json (Phase 1 + baseline KPIs)
- summary.csv (core KPIs)

Usage (repo root):
    python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000
    python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000 --scenario 01_optimal_to_left.scenic
    python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000 --scenario-glob "0[1-2]_*.scenic"
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
RE_PHASE1_SWITCH = re.compile(
    r"\[Phase1Planner\]\s+t=(?P<t>\d+\.?\d*)s\s+ttl_switch\s+(?P<from>\w+)->(?P<to>\w+)"
)


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


def _collect_metrics_from_log(log_path: Path) -> Dict[str, Any]:
    min_opp_dist: Optional[float] = None
    ttl_seen: List[str] = []
    planner_modes: List[str] = []
    event_counts: Dict[str, int] = {}
    phase1_switches: List[Dict[str, Any]] = []

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
                continue
            p1 = RE_PHASE1_SWITCH.search(line)
            if p1:
                phase1_switches.append(
                    {
                        "t_s": float(p1.group("t")),
                        "from": p1.group("from"),
                        "to": p1.group("to"),
                    }
                )

    return {
        "min_opponent_distance_m": min_opp_dist,
        "phase0_samples": len(ttl_seen),
        "ttls_observed": sorted(set(ttl_seen)),
        "planner_modes_observed": sorted(set(planner_modes)),
        "ttl_switch_count": int(event_counts.get("ttl_switch", 0)),
        "near_miss_count": int(event_counts.get("near_miss", 0)),
        "collision_count": int(event_counts.get("collision", 0)),
        "off_track_count": int(event_counts.get("off_track", 0)),
        "phase1_switch_observed": bool(len(phase1_switches) > 0),
        "phase1_switch_count": len(phase1_switches),
        "phase1_switches": phase1_switches,
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
    _seg_time_s, _seg_name_map, seg_waypoint_count, total_time_s = compute_segment_times(events)
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
    base.update(_collect_metrics_from_log(out_log))
    base.update(_analyze_waypoint_timing(out_log))
    base["collision"] = bool(base.get("collision_count", 0) > 0)
    base["off_track"] = bool(base.get("off_track_count", 0) > 0)
    return base


def main() -> int:
    parser = argparse.ArgumentParser(description="Run Phase 1 planner-MPC scenarios and emit metrics summary.")
    parser.add_argument(
        "--scenario-dir",
        default="examples/racing/phase1_planner",
        help="Directory with phase1 .scenic files (default: examples/racing/phase1_planner)",
    )
    parser.add_argument(
        "--scenario",
        action="append",
        default=[],
        help=(
            "Specific scenario filename(s) to run from scenario-dir. "
            "Repeat flag for multiple values."
        ),
    )
    parser.add_argument(
        "--scenario-glob",
        default=None,
        help='Glob pattern (within scenario-dir) to select subset, e.g. "01_*.scenic".',
    )
    parser.add_argument(
        "--time",
        type=int,
        default=3000,
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

    root = _repo_root()
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
    ]
    with open(summary_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=csv_fields)
        writer.writeheader()
        for row in results:
            writer.writerow({k: row.get(k) for k in csv_fields})

    print(f"\n[Phase1Runner] Wrote {summary_json}")
    print(f"[Phase1Runner] Wrote {summary_csv}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
