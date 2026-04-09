"""Shared helpers for phase benchmark runners (phase0–phase6).

Scenario discovery for `run_phase_main`: every ``*.scenic`` in the chosen
``--scenario-dir`` (default from `PhaseRunnerSpec.default_scenario_dir`) is run,
in sorted filename order. Adding a new benchmark file under that directory does
not require editing the phase runner module.

When a planned phase (4–6) is implemented, update that phase's `PhaseRunnerSpec`
in ``phase4_runner.py`` / ``phase5_runner.py`` / ``phase6_runner.py`` (CSV
columns, log-parser flags) and extend `collect_metrics_from_log` here if new log
tags need KPIs. See ``examples/racing/README.md`` (Phases 4–6).

Runners print a ``BENCHMARK_AI_DIGEST_*`` JSON block for copy-paste summaries.
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
from dataclasses import dataclass, field
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Sequence, Tuple, Union

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
RE_PHASE2_OVERLAP = re.compile(r"overlap=(?P<v>\S+)")
RE_PHASE2_SEG = re.compile(r"seg_ctx=(?P<v>\S+)")
RE_PHASE2_OPP_NONE = re.compile(r"\[Phase2\].*opponent=none")
RE_PHASE3_TTL_SWITCH = re.compile(
    r"\[Phase3Tactical\]\s+t=(?P<t>\d+\.?\d*)s\s+ttl_switch\s+(?P<from>\S+)->(?P<to>\S+)\s+mode=(?P<mode>\S+)"
)
RE_PHASE3_STATUS = re.compile(
    r"\[Phase3Tactical\]\s+t=(?P<t>\d+\.?\d*)s\s+mode=(?P<mode>\S+)\s+ttl=(?P<ttl>\S+)\s+cap=(?P<cap>\S+)"
)


JsonScalar = Union[str, int, float, bool, None]

# Flat keys copied into BENCHMARK_AI_DIGEST for copy-paste / tooling (one JSON object per run).
STANDARD_BENCHMARK_DIGEST_KEYS: Tuple[str, ...] = (
    "scenario",
    "return_code",
    "lap_completion_status",
    "lap_time_s",
    "waypoint_hits",
    "collision",
    "off_track",
    "near_miss_count",
    "collision_count",
    "off_track_count",
    "ttl_switch_count",
    "min_opponent_distance_m",
    "phase0_samples",
    "phase1_switch_count",
    "phase1_switch_observed",
    "phase2_line_count",
    "phase2_overlap_count",
    "phase2_seg_ctx_count",
    "phase2_assess_errors",
    "phase2_opponent_none_lines",
    "phase3_ttl_switch_count",
    "phase3_tactical_status_count",
)


def _default_digest_keys() -> List[str]:
    return list(STANDARD_BENCHMARK_DIGEST_KEYS)


def _digest_scalar(value: Any) -> JsonScalar:
    if value is None or isinstance(value, (bool, int, float, str)):
        return value
    if isinstance(value, Path):
        return str(value)
    return str(value)


def benchmark_digest_rows(results: List[Dict[str, Any]], keys: Sequence[str]) -> List[Dict[str, JsonScalar]]:
    rows: List[Dict[str, JsonScalar]] = []
    for row in results:
        rows.append({k: _digest_scalar(row.get(k)) for k in keys})
    return rows


def benchmark_digest_aggregate(results: List[Dict[str, Any]]) -> Dict[str, JsonScalar]:
    if not results:
        return {
            "scenario_count": 0,
            "all_return_codes_zero": True,
            "any_collision": False,
            "any_off_track": False,
            "max_phase3_ttl_switch_count": 0,
            "sum_near_miss_count": 0,
            "max_phase2_assess_errors": 0,
        }
    return {
        "scenario_count": len(results),
        "all_return_codes_zero": all(r.get("return_code") == 0 for r in results),
        "any_collision": any(r.get("collision") for r in results),
        "any_off_track": any(r.get("off_track") for r in results),
        "max_phase3_ttl_switch_count": max(
            int(r.get("phase3_ttl_switch_count") or 0) for r in results
        ),
        "sum_near_miss_count": sum(int(r.get("near_miss_count") or 0) for r in results),
        "max_phase2_assess_errors": max(int(r.get("phase2_assess_errors") or 0) for r in results),
    }


def build_benchmark_ai_digest_payload(
    *,
    runner_label: str,
    run_id: str,
    run_dir: Path,
    scenario_dir: Path,
    sim_steps: int,
    assumed_time_step_s: float,
    inter_run_delay_s: float,
    results: List[Dict[str, Any]],
    digest_keys: Sequence[str],
    extra: Optional[Dict[str, Any]] = None,
) -> Dict[str, Any]:
    approx_sim_s = float(sim_steps) * max(0.0, float(assumed_time_step_s))
    payload: Dict[str, Any] = {
        "schema": "benchmark_ai_digest_v1",
        "runner": runner_label,
        "run_id": run_id,
        "paths": {"run_dir": str(run_dir), "scenario_dir": str(scenario_dir)},
        "sim_steps": int(sim_steps),
        "assumed_time_step_s": float(assumed_time_step_s),
        "approx_requested_sim_time_s": approx_sim_s,
        "inter_run_delay_s": float(inter_run_delay_s),
        "aggregate": benchmark_digest_aggregate(results),
        "rows": benchmark_digest_rows(results, digest_keys),
    }
    if extra:
        payload["extra"] = extra
    return payload


def print_benchmark_ai_digest(payload: Dict[str, Any]) -> None:
    """Print a single-line JSON block between markers (easy to paste for analysis)."""
    print("\nBENCHMARK_AI_DIGEST_BEGIN")
    print(json.dumps(payload, ensure_ascii=True, separators=(",", ":")))
    print("BENCHMARK_AI_DIGEST_END")
    print(
        "Tip: attach this block or paths[\"run_dir\"]/summary.json when sharing results.\n"
    )


def repo_root() -> Path:
    p = Path(__file__).resolve()
    for _ in range(14):
        if (p / "src").is_dir() and (p / "examples").is_dir():
            return p
        p = p.parent
    return Path.cwd()


def parse_float_or_none(value: str) -> Optional[float]:
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


def collect_metrics_from_log(
    log_path: Path,
    *,
    phase1_switches: bool = False,
    phase2_lines: bool = False,
    phase3_tactical: bool = False,
) -> Dict[str, Any]:
    min_opp_dist: Optional[float] = None
    ttl_seen: List[str] = []
    planner_modes: List[str] = []
    event_counts: Dict[str, int] = {}
    phase1_switch_list: List[Dict[str, Any]] = []

    phase2_count = 0
    phase2_overlaps: List[str] = []
    phase2_seg_ctx: List[str] = []
    phase2_opponent_none_lines = 0
    phase2_assess_errors = 0

    phase3_switches: List[Dict[str, Any]] = []
    phase3_modes: List[str] = []
    phase3_ttls: List[str] = []

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = RE_PHASE0_LINE.search(line)
            if m:
                ttl_seen.append(m.group("ttl"))
                planner_modes.append(m.group("mode"))
                d = parse_float_or_none(m.group("opp_dist"))
                if d is not None:
                    min_opp_dist = d if min_opp_dist is None else min(min_opp_dist, d)
                continue
            e = RE_PHASE0_EVENT.search(line)
            if e:
                ev = e.group("event")
                event_counts[ev] = event_counts.get(ev, 0) + 1
                continue
            if phase1_switches:
                p1 = RE_PHASE1_SWITCH.search(line)
                if p1:
                    phase1_switch_list.append(
                        {
                            "t_s": float(p1.group("t")),
                            "from": p1.group("from"),
                            "to": p1.group("to"),
                        }
                    )
                    continue
            if phase2_lines and "[Phase2]" in line:
                phase2_count += 1
                if RE_PHASE2_OPP_NONE.search(line):
                    phase2_opponent_none_lines += 1
                if "[Phase2]" in line and "assess_error" in line:
                    phase2_assess_errors += 1
                om = RE_PHASE2_OVERLAP.search(line)
                if om:
                    phase2_overlaps.append(om.group("v"))
                sm = RE_PHASE2_SEG.search(line)
                if sm:
                    phase2_seg_ctx.append(sm.group("v"))
                continue
            if phase3_tactical:
                p3s = RE_PHASE3_TTL_SWITCH.search(line)
                if p3s:
                    phase3_switches.append(
                        {
                            "t_s": float(p3s.group("t")),
                            "from": p3s.group("from"),
                            "to": p3s.group("to"),
                            "mode": p3s.group("mode"),
                        }
                    )
                    continue
                p3st = RE_PHASE3_STATUS.search(line)
                if p3st:
                    phase3_modes.append(p3st.group("mode"))
                    phase3_ttls.append(p3st.group("ttl"))
                    continue

    out: Dict[str, Any] = {
        "min_opponent_distance_m": min_opp_dist,
        "phase0_samples": len(ttl_seen),
        "ttls_observed": sorted(set(ttl_seen)),
        "planner_modes_observed": sorted(set(planner_modes)),
        "ttl_switch_count": int(event_counts.get("ttl_switch", 0)),
        "near_miss_count": int(event_counts.get("near_miss", 0)),
        "collision_count": int(event_counts.get("collision", 0)),
        "off_track_count": int(event_counts.get("off_track", 0)),
    }
    if phase1_switches:
        out["phase1_switch_observed"] = bool(len(phase1_switch_list) > 0)
        out["phase1_switch_count"] = len(phase1_switch_list)
        out["phase1_switches"] = phase1_switch_list
    if phase2_lines:
        out["phase2_line_count"] = phase2_count
        out["phase2_opponent_none_lines"] = phase2_opponent_none_lines
        out["phase2_assess_errors"] = phase2_assess_errors
        out["phase2_overlap_count"] = len(phase2_overlaps)
        out["phase2_seg_ctx_count"] = len(phase2_seg_ctx)
        out["phase2_overlaps_observed"] = sorted(set(phase2_overlaps))
        out["phase2_seg_ctx_observed"] = sorted(set(phase2_seg_ctx))
    if phase3_tactical:
        out["phase3_ttl_switch_count"] = len(phase3_switches)
        out["phase3_tactical_status_count"] = len(phase3_modes)
        out["phase3_switches"] = phase3_switches
        out["phase3_modes_observed"] = sorted(set(phase3_modes))
        out["phase3_ttls_observed"] = sorted(set(phase3_ttls))
    return out


def analyze_waypoint_timing(log_path: Path) -> Dict[str, Any]:
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


def run_scenic_scenario(repo_root: Path, scenario: Path, out_log: Path, sim_steps: int) -> Dict[str, Any]:
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
    return {
        "scenario": str(scenario.relative_to(repo_root)),
        "log": str(out_log),
        "return_code": int(proc.returncode),
    }


def finalize_row(row: Dict[str, Any]) -> Dict[str, Any]:
    row["collision"] = bool(row.get("collision_count", 0) > 0)
    row["off_track"] = bool(row.get("off_track_count", 0) > 0)
    return row


def run_one_scenario_with_collect(
    repo_root: Path,
    scenario: Path,
    out_log: Path,
    sim_steps: int,
    *,
    phase1_switches: bool = False,
    phase2_lines: bool = False,
    phase3_tactical: bool = False,
) -> Dict[str, Any]:
    base = run_scenic_scenario(repo_root, scenario, out_log, sim_steps)
    base.update(
        collect_metrics_from_log(
            out_log,
            phase1_switches=phase1_switches,
            phase2_lines=phase2_lines,
            phase3_tactical=phase3_tactical,
        )
    )
    base.update(analyze_waypoint_timing(out_log))
    return finalize_row(base)


@dataclass
class PhaseRunnerSpec:
    """Configuration for `run_phase_main` (phases 2–6).

    ``default_scenario_dir`` is the folder whose ``*.scenic`` files form the
    default benchmark bank; filenames are not listed in code.
    """

    runner_label: str
    run_id_prefix: str
    default_scenario_dir: str
    default_sim_steps: int = 3000
    phase1_switches: bool = False
    phase2_lines: bool = False
    phase3_tactical: bool = False
    csv_fields: Sequence[str] = field(default_factory=tuple)
    extra_summary_keys: Sequence[str] = field(default_factory=tuple)
    digest_keys: Sequence[str] = field(default_factory=_default_digest_keys)


def run_phase_main(spec: PhaseRunnerSpec) -> int:
    """CLI entry for phase 2–6 benchmark runners (same flags as phase1).

    Runs all ``*.scenic`` files in the scenario directory (after optional
    ``--scenario`` / ``--scenario-glob`` filters). New scenario files are picked
    up automatically; only metrics and defaults need code changes when a phase
    gains new log lines or KPIs.
    """
    parser = argparse.ArgumentParser(
        description=f"Run {spec.runner_label} scenarios and emit metrics summary."
    )
    parser.add_argument("--scenario-dir", default=spec.default_scenario_dir)
    parser.add_argument("--scenario", action="append", default=[])
    parser.add_argument("--scenario-glob", default=None)
    parser.add_argument("--time", type=int, default=spec.default_sim_steps)
    parser.add_argument("--time-step-s", type=float, default=0.01)
    parser.add_argument("--out-dir", default="src/scenic/domains/racing/benchmarks/results")
    parser.add_argument(
        "--inter-run-delay-s",
        type=float,
        default=15.0,
        help="Delay between scenarios; clamped to [0, 15].",
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
                f"Warning: requested --scenario not found in {scenario_dir}: {', '.join(missing)}",
                file=sys.stderr,
            )
    if args.scenario_glob:
        allowed = {p.resolve() for p in scenario_dir.glob(str(args.scenario_glob))}
        scenarios = [s for s in scenarios if s.resolve() in allowed]

    if not scenarios:
        print("No scenarios selected after filtering.", file=sys.stderr)
        return 2

    run_id = datetime.now(timezone.utc).strftime(f"{spec.run_id_prefix}_%Y%m%d_%H%M%S")
    run_dir = (root / args.out_dir / run_id).resolve()
    logs_dir = run_dir / "logs"
    logs_dir.mkdir(parents=True, exist_ok=True)
    inter_run_delay_s = max(0.0, min(15.0, float(args.inter_run_delay_s)))
    if float(args.inter_run_delay_s) != inter_run_delay_s:
        print(
            f"[{spec.runner_label}] inter-run delay clamped to {inter_run_delay_s:.2f}s "
            f"(requested {args.inter_run_delay_s})."
        )

    results: List[Dict[str, Any]] = []
    for idx, scenario in enumerate(scenarios):
        log_path = logs_dir / f"{scenario.stem}.log"
        print(f"[{spec.runner_label}] Running {scenario.name} ...")
        row = run_one_scenario_with_collect(
            root,
            scenario,
            log_path,
            int(args.time),
            phase1_switches=spec.phase1_switches,
            phase2_lines=spec.phase2_lines,
            phase3_tactical=spec.phase3_tactical,
        )
        results.append(row)
        approx_sim_s = args.time * max(0.0, float(args.time_step_s))
        stem = scenario.stem
        extra = ""
        for k in spec.extra_summary_keys:
            if k in row and row[k] is not None:
                extra += f" {k}={row[k]}"
        print(
            f"[{spec.runner_label}] {stem}: rc={row['return_code']} "
            f"lap={row['lap_completion_status']} lap_time_s={row['lap_time_s']} "
            f"(steps={int(args.time)} ~= {approx_sim_s:.2f}s) "
            f"collision={row['collision']} off_track={row['off_track']}{extra}"
        )
        if inter_run_delay_s > 0 and idx < (len(scenarios) - 1):
            print(f"[{spec.runner_label}] Waiting {inter_run_delay_s:.2f}s before next scenario...")
            time.sleep(inter_run_delay_s)

    summary_payload: Dict[str, Any] = {
        "run_id": run_id,
        "created_utc": datetime.now(timezone.utc).isoformat(),
        "scenario_dir": str(scenario_dir),
        "sim_steps": args.time,
        "assumed_time_step_s": args.time_step_s,
        "approx_requested_sim_time_s": args.time * max(0.0, float(args.time_step_s)),
        "inter_run_delay_s": inter_run_delay_s,
        "runner": spec.runner_label,
        "results": results,
    }
    summary_json = run_dir / "summary.json"
    with open(summary_json, "w", encoding="utf-8") as f:
        json.dump(summary_payload, f, indent=2)

    summary_csv = run_dir / "summary.csv"
    fields = list(spec.csv_fields)
    with open(summary_csv, "w", newline="", encoding="utf-8") as f:
        writer = csv.DictWriter(f, fieldnames=fields)
        writer.writeheader()
        for row in results:
            writer.writerow({k: row.get(k) for k in fields})

    print(f"\n[{spec.runner_label}] Wrote {summary_json}")
    print(f"[{spec.runner_label}] Wrote {summary_csv}")
    print_benchmark_ai_digest(
        build_benchmark_ai_digest_payload(
            runner_label=spec.runner_label,
            run_id=run_id,
            run_dir=run_dir,
            scenario_dir=scenario_dir,
            sim_steps=int(args.time),
            assumed_time_step_s=float(args.time_step_s),
            inter_run_delay_s=inter_run_delay_s,
            results=results,
            digest_keys=list(spec.digest_keys),
        )
    )
    return 0
