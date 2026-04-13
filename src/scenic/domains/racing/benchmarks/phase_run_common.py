"""Shared helpers for benchmark runners (phase0–phase5 + fellow harness).

Scenario discovery for `run_phase_main`: every ``*.scenic`` in the chosen
``--scenario-dir`` (default from `PhaseRunnerSpec.default_scenario_dir`) is run,
in sorted filename order. Adding a new benchmark file under that directory does
not require editing the phase runner module.

When a planned phase (4–5) is implemented, update that phase's `PhaseRunnerSpec`
in ``phase4_runner.py`` / ``phase5_runner.py`` (CSV
columns, log-parser flags) and extend `collect_metrics_from_log` here if new log
tags need KPIs. See ``examples/racing/README.md`` (Phases 4–5).

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
RE_PHASE4_EVENT = re.compile(
    r"\[Phase4Event\]\s+t=(?P<t>\d+\.?\d*)s\s+event=(?P<event>\S+)"
)
RE_EVAL_CONTACT_EVENT = re.compile(
    r"\[EvalEvent\]\s+t=(?P<t>\d+\.?\d*)s\s+type=eval_contact\s+severity=(?P<sev>\S+)"
)
RE_PHASE5_EVENT = re.compile(
    r"\[Phase5Event\]\s+t=(?P<t>\d+\.?\d*)s\s+event=(?P<event>\S+)"
)
RE_PHASE5_TTL_SWITCH = re.compile(
    r"\[Phase5Tactical\]\s+t=(?P<t>\d+\.?\d*)s\s+ttl_switch\s+(?P<from>\S+)->(?P<to>\S+)\s+mode_in=(?P<mode_in>\S+)\s+mode_out=(?P<mode_out>\S+)\s+seg=(?P<seg>\S+)\s+overlap=(?P<ov>\S+)\s+reason=(?P<reason>\S+)"
)
RE_PHASE5_STATUS = re.compile(
    r"\[Phase5Tactical\]\s+t=(?P<t>\d+\.?\d*)s\s+mode_in=(?P<mode_in>\S+)\s+mode_out=(?P<mode_out>\S+)\s+ttl=(?P<ttl>\S+)\s+cap=(?P<cap>\S+)\s+seg=(?P<seg>\S+)\s+overlap=(?P<ov>\S+)\s+reason=(?P<reason>\S+)"
)
# Fellow harness: placement from ego offset ([placement.py])
RE_FELLOW_PLACEMENT_FROM_EGO = re.compile(
    r"\[Placement\][^\n]*racing \(s,t\) from ego[^\n]*-> s=([\d.+-]+),\s*t=([\d.+-]+)"
)
# [Fellow s,t] Fellow_0: route=... xy=(...) -> s=..., t=...
RE_FELLOW_ST_LINE = re.compile(r"\[Fellow s,t\][^\n]*-> s=([\d.+-]+),\s*t=([\d.+-]+)")
RE_FELLOW_HARNESS_LINE = re.compile(
    r"\[FellowHarness\]\s+t=([\d.+-]+)s\s+idx=(\d+)\s+speed_mps=([\d.eE+-]+)\s+x=([\d.eE+-]+)\s+y=([\d.eE+-]+)"
)
RE_PLACEMENT_COMMAND_OFFSET = re.compile(
    r"racing \(s,t\) from ego \+ \('(?P<kind>ahead|behind|left|right)',\s*(?P<value>[\d.+-]+)\)"
)
RE_EGO_ST_DEBUG = re.compile(r"\[Ego debug\].*-> s=([\d.+-]+),\s*t=([\d.+-]+)")
RE_EGO_ROAD_ID = re.compile(r"\[Ego debug\].*projected onto road_id=(\d+)")
RE_FELLOW_ROAD_ID = re.compile(r"\[Fellow s,t\].*projected onto road_id=(\d+)")
RE_FELLOW_DISTANCE_FROM_EGO = re.compile(r"\[Fellow s,t\].*distance_from_ego=([\d.+-]+)m")

FELLOW_HARNESS_T_OUT_OF_BAND_M = 15.0

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
    "phase4_abort_pass_count",
    "phase4_emergency_avoid_count",
    "phase4_commit_pass_count",
    "phase4_event_commit_pass_left",
    "phase4_event_commit_pass_right",
    "phase4_event_shield_release",
    "phase5_tactical_line_count",
    "phase5_ttl_switch_count",
    "phase5_event_segment_override",
    "phase5_event_segment_release",
    "phase5_override_count",
    "eval_contact_overlap_count",
    "eval_contact_near_count",
    "collision_eval_hull_overlap",
)

# Fellow traffic harness digest (see examples/racing/fellow_smoke, fellow_runner.py).
FELLOW_HARNESS_DIGEST_KEYS: Tuple[str, ...] = (
    "scenario",
    "return_code",
    "lap_completion_status",
    "lap_time_s",
    "waypoint_hits",
    "collision",
    "off_track",
    "near_miss_count",
    "min_opponent_distance_m",
    "phase0_samples",
    "phase2_line_count",
    "phase2_assess_errors",
    "fellow_placement_from_ego_offset_observed",
    "fellow_st_log_present",
    "fellow_s0",
    "fellow_t0",
    "fellow_t_out_of_band",
    "fellow_harness_line_count",
    "fellow_speed_min_mps",
    "fellow_speed_max_mps",
    "fellow_max_speed_step_jump_mps",
    "fellow_speed_stuck_near_zero",
    "fellow_position_range_m",
)

# Columns merged into phase 0–4 benchmark summaries when parsing logs (with or without fellow in scene).
FELLOW_HARNESS_SUMMARY_KEYS: Tuple[str, ...] = (
    "fellow_placement_from_ego_offset_observed",
    "fellow_st_log_present",
    "fellow_s0",
    "fellow_t0",
    "fellow_t_out_of_band",
    "fellow_harness_line_count",
    "fellow_speed_min_mps",
    "fellow_speed_max_mps",
    "fellow_max_speed_step_jump_mps",
    "fellow_speed_stuck_near_zero",
    "fellow_position_range_m",
)

FELLOW_PLACEMENT_DEBUG_SUMMARY_KEYS: Tuple[str, ...] = (
    "placement_command_observed",
    "placement_command_kind",
    "requested_delta_s_m",
    "requested_delta_t_m",
    "ego_s0",
    "ego_t0",
    "fellow_s0",
    "fellow_t0",
    "observed_delta_s_m",
    "observed_delta_t_m",
    "placement_s_error_m",
    "placement_t_error_m",
    "ego_road_id",
    "fellow_road_id",
    "road_id_mismatch",
    "unexpected_pit_projection",
    "spawn_distance_from_ego_m",
)

FELLOW_PLACEMENT_DEBUG_DIGEST_KEYS: Tuple[str, ...] = (
    "scenario",
    "return_code",
    "lap_completion_status",
    "lap_time_s",
    "collision",
    "off_track",
    "min_opponent_distance_m",
    "fellow_placement_from_ego_offset_observed",
    "fellow_st_log_present",
    "fellow_harness_line_count",
    "placement_command_observed",
    "placement_command_kind",
    "requested_delta_s_m",
    "requested_delta_t_m",
    "ego_s0",
    "ego_t0",
    "fellow_s0",
    "fellow_t0",
    "observed_delta_s_m",
    "observed_delta_t_m",
    "placement_s_error_m",
    "placement_t_error_m",
    "ego_road_id",
    "fellow_road_id",
    "road_id_mismatch",
    "unexpected_pit_projection",
    "spawn_distance_from_ego_m",
)


def standard_benchmark_digest_keys_with_fellow() -> List[str]:
    """`STANDARD_BENCHMARK_DIGEST_KEYS` plus fellow-harness fields for combined digest rows."""
    out: List[str] = list(STANDARD_BENCHMARK_DIGEST_KEYS)
    for k in FELLOW_HARNESS_SUMMARY_KEYS:
        if k not in out:
            out.append(k)
    return out


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
            "any_collision_eval_hull": False,
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
        "any_collision_eval_hull": any(r.get("collision_eval_hull_overlap") for r in results),
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
    """Parse standard racing benchmark tags; does **not** read ``[EvalGT]`` / dSPACE sensor ground truth."""
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

    phase4_abort_pass_count = 0
    phase4_emergency_count = 0
    phase4_commit_count = 0
    phase4_event_commit_left = 0
    phase4_event_commit_right = 0
    phase4_event_abort = 0
    phase4_event_emergency = 0
    phase4_event_shield_release = 0
    phase5_line_count = 0
    phase5_ttl_switch_count = 0
    phase5_event_segment_override = 0
    phase5_event_segment_release = 0
    phase5_override_count = 0
    phase5_mode_out: List[str] = []
    phase5_reasons: List[str] = []
    eval_contact_overlap_count = 0
    eval_contact_near_count = 0

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            evc = RE_EVAL_CONTACT_EVENT.search(line)
            if evc:
                sev = evc.group("sev")
                if sev == "overlap":
                    eval_contact_overlap_count += 1
                elif sev == "near":
                    eval_contact_near_count += 1
                continue
            if "[Phase4Event]" in line:
                pe = RE_PHASE4_EVENT.search(line)
                if pe:
                    ev = pe.group("event")
                    if ev == "commit_pass_left":
                        phase4_event_commit_left += 1
                    elif ev == "commit_pass_right":
                        phase4_event_commit_right += 1
                    elif ev == "abort_pass":
                        phase4_event_abort += 1
                    elif ev == "emergency_avoid":
                        phase4_event_emergency += 1
                    elif ev == "shield_release":
                        phase4_event_shield_release += 1
            if "[Phase5Event]" in line:
                p5e = RE_PHASE5_EVENT.search(line)
                if p5e:
                    ev5 = p5e.group("event")
                    if ev5 == "segment_override":
                        phase5_event_segment_override += 1
                    elif ev5 == "segment_release":
                        phase5_event_segment_release += 1
            if "[Phase5Tactical]" in line:
                phase5_line_count += 1
                p5s = RE_PHASE5_TTL_SWITCH.search(line)
                if p5s:
                    phase5_ttl_switch_count += 1
                    _mout = p5s.group("mode_out")
                    _reason = p5s.group("reason")
                    phase5_mode_out.append(_mout)
                    phase5_reasons.append(_reason)
                    if _reason != "none":
                        phase5_override_count += 1
                p5st = RE_PHASE5_STATUS.search(line)
                if p5st:
                    _mout = p5st.group("mode_out")
                    _reason = p5st.group("reason")
                    phase5_mode_out.append(_mout)
                    phase5_reasons.append(_reason)
                    if _reason != "none":
                        phase5_override_count += 1
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
    out["phase4_abort_pass_count"] = phase4_event_abort
    out["phase4_emergency_avoid_count"] = phase4_event_emergency
    out["phase4_commit_pass_count"] = phase4_event_commit_left + phase4_event_commit_right
    out["phase4_event_commit_pass_left"] = phase4_event_commit_left
    out["phase4_event_commit_pass_right"] = phase4_event_commit_right
    out["phase4_event_shield_release"] = phase4_event_shield_release
    out["phase5_tactical_line_count"] = phase5_line_count
    out["phase5_ttl_switch_count"] = phase5_ttl_switch_count
    out["phase5_event_segment_override"] = phase5_event_segment_override
    out["phase5_event_segment_release"] = phase5_event_segment_release
    out["phase5_override_count"] = phase5_override_count
    out["phase5_modes_observed"] = sorted(set(phase5_mode_out))
    out["phase5_reasons_observed"] = sorted(set(phase5_reasons))
    out["eval_contact_overlap_count"] = eval_contact_overlap_count
    out["eval_contact_near_count"] = eval_contact_near_count
    out["collision_eval_hull_overlap"] = bool(eval_contact_overlap_count > 0)
    return out


def collect_fellow_harness_metrics_from_log(log_path: Path) -> Dict[str, Any]:
    """Parse fellow placement lines and optional ``[FellowHarness]`` runtime samples."""
    placement_seen = False
    fellow_s0: Optional[float] = None
    fellow_t0: Optional[float] = None
    fellow_st_seen = False
    speeds: List[float] = []
    positions: List[Tuple[float, float]] = []
    prev_speed: Optional[float] = None
    max_jump = 0.0

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            pm = RE_FELLOW_PLACEMENT_FROM_EGO.search(line)
            if pm:
                placement_seen = True
            sm = RE_FELLOW_ST_LINE.search(line)
            if sm:
                fellow_st_seen = True
                try:
                    fellow_s0 = float(sm.group(1))
                    fellow_t0 = float(sm.group(2))
                except (TypeError, ValueError):
                    pass
            hm = RE_FELLOW_HARNESS_LINE.search(line)
            if hm:
                try:
                    v = float(hm.group(3))
                    x = float(hm.group(4))
                    y = float(hm.group(5))
                except (TypeError, ValueError):
                    continue
                speeds.append(v)
                positions.append((x, y))
                if prev_speed is not None:
                    max_jump = max(max_jump, abs(v - prev_speed))
                prev_speed = v

    t_oob = False
    if fellow_t0 is not None and abs(fellow_t0) > FELLOW_HARNESS_T_OUT_OF_BAND_M:
        t_oob = True

    pos_range: Optional[float] = None
    if len(positions) >= 2:
        x0, y0 = positions[0]
        pos_range = max(math.hypot(x - x0, y - y0) for x, y in positions)

    v_min = min(speeds) if speeds else None
    v_max = max(speeds) if speeds else None
    stuck = False
    if speeds and len(speeds) >= 10 and v_max is not None and v_max < 0.5:
        stuck = True

    return {
        "fellow_placement_from_ego_offset_observed": placement_seen,
        "fellow_st_log_present": fellow_st_seen,
        "fellow_s0": fellow_s0,
        "fellow_t0": fellow_t0,
        "fellow_t_out_of_band": t_oob,
        "fellow_harness_line_count": len(speeds),
        "fellow_speed_min_mps": v_min,
        "fellow_speed_max_mps": v_max,
        "fellow_max_speed_step_jump_mps": max_jump if speeds else None,
        "fellow_speed_stuck_near_zero": stuck,
        "fellow_position_range_m": pos_range,
    }


def collect_fellow_placement_debug_metrics_from_log(log_path: Path) -> Dict[str, Any]:
    """Parse spawn command/observation deltas and coarse road consistency at placement."""
    placement_command_observed = False
    placement_command_kind: Optional[str] = None
    requested_delta_s_m: Optional[float] = None
    requested_delta_t_m: Optional[float] = None
    ego_s0: Optional[float] = None
    ego_t0: Optional[float] = None
    fellow_s0: Optional[float] = None
    fellow_t0: Optional[float] = None
    ego_road_id: Optional[int] = None
    fellow_road_id: Optional[int] = None
    spawn_distance_from_ego_m: Optional[float] = None

    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            cm = RE_PLACEMENT_COMMAND_OFFSET.search(line)
            if cm and not placement_command_observed:
                placement_command_observed = True
                placement_command_kind = cm.group("kind")
                try:
                    mag = float(cm.group("value"))
                except (TypeError, ValueError):
                    mag = None
                if mag is not None:
                    if placement_command_kind == "ahead":
                        requested_delta_s_m = mag
                        requested_delta_t_m = 0.0
                    elif placement_command_kind == "behind":
                        requested_delta_s_m = -mag
                        requested_delta_t_m = 0.0
                    elif placement_command_kind == "left":
                        requested_delta_s_m = 0.0
                        requested_delta_t_m = mag
                    elif placement_command_kind == "right":
                        requested_delta_s_m = 0.0
                        requested_delta_t_m = -mag
            em = RE_EGO_ST_DEBUG.search(line)
            if em and ego_s0 is None and ego_t0 is None:
                try:
                    ego_s0 = float(em.group(1))
                    ego_t0 = float(em.group(2))
                except (TypeError, ValueError):
                    pass
            erm = RE_EGO_ROAD_ID.search(line)
            if erm and ego_road_id is None:
                try:
                    ego_road_id = int(erm.group(1))
                except (TypeError, ValueError):
                    pass
            fm = RE_FELLOW_ST_LINE.search(line)
            if fm and fellow_s0 is None and fellow_t0 is None:
                try:
                    fellow_s0 = float(fm.group(1))
                    fellow_t0 = float(fm.group(2))
                except (TypeError, ValueError):
                    pass
            frm = RE_FELLOW_ROAD_ID.search(line)
            if frm and fellow_road_id is None:
                try:
                    fellow_road_id = int(frm.group(1))
                except (TypeError, ValueError):
                    pass
            dm = RE_FELLOW_DISTANCE_FROM_EGO.search(line)
            if dm and spawn_distance_from_ego_m is None:
                try:
                    spawn_distance_from_ego_m = float(dm.group(1))
                except (TypeError, ValueError):
                    pass

    observed_delta_s_m: Optional[float] = None
    observed_delta_t_m: Optional[float] = None
    placement_s_error_m: Optional[float] = None
    placement_t_error_m: Optional[float] = None
    if (ego_s0 is not None) and (fellow_s0 is not None):
        observed_delta_s_m = float(fellow_s0 - ego_s0)
        if requested_delta_s_m is not None:
            placement_s_error_m = abs(observed_delta_s_m - requested_delta_s_m)
    if (ego_t0 is not None) and (fellow_t0 is not None):
        observed_delta_t_m = float(fellow_t0 - ego_t0)
        if requested_delta_t_m is not None:
            placement_t_error_m = abs(observed_delta_t_m - requested_delta_t_m)

    road_id_mismatch = (
        bool((ego_road_id is not None) and (fellow_road_id is not None) and (ego_road_id != fellow_road_id))
    )
    unexpected_pit_projection = bool(
        placement_command_observed
        and (placement_command_kind in ("ahead", "behind", "left", "right"))
        and road_id_mismatch
    )

    return {
        "placement_command_observed": placement_command_observed,
        "placement_command_kind": placement_command_kind,
        "requested_delta_s_m": requested_delta_s_m,
        "requested_delta_t_m": requested_delta_t_m,
        "ego_s0": ego_s0,
        "ego_t0": ego_t0,
        "fellow_s0": fellow_s0,
        "fellow_t0": fellow_t0,
        "observed_delta_s_m": observed_delta_s_m,
        "observed_delta_t_m": observed_delta_t_m,
        "placement_s_error_m": placement_s_error_m,
        "placement_t_error_m": placement_t_error_m,
        "ego_road_id": ego_road_id,
        "fellow_road_id": fellow_road_id,
        "road_id_mismatch": road_id_mismatch,
        "unexpected_pit_projection": unexpected_pit_projection,
        "spawn_distance_from_ego_m": spawn_distance_from_ego_m,
    }


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
    """Normalize safety metrics using eval-contact as canonical source."""

    eval_overlap = int(row.get("eval_contact_overlap_count", 0) or 0)
    eval_near = int(row.get("eval_contact_near_count", 0) or 0)

    row["collision_count"] = eval_overlap
    row["near_miss_count"] = eval_near
    row["collision"] = bool(row.get("collision_count", 0) > 0)
    row["off_track"] = bool(row.get("off_track_count", 0) > 0)
    row["collision_eval_hull_overlap"] = bool(eval_overlap > 0)
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
    fellow_harness: bool = False,
    fellow_placement_debug: bool = False,
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
    if fellow_harness:
        base.update(collect_fellow_harness_metrics_from_log(out_log))
    if fellow_placement_debug:
        base.update(collect_fellow_placement_debug_metrics_from_log(out_log))
    base.update(analyze_waypoint_timing(out_log))
    return finalize_row(base)


@dataclass
class PhaseRunnerSpec:
    """Configuration for `run_phase_main` (phase and fellow benchmark runners).

    ``default_scenario_dir`` is the folder whose ``*.scenic`` files form the
    default benchmark bank; filenames are not listed in code.
    """

    runner_label: str
    run_id_prefix: str
    default_scenario_dir: str
    default_sim_steps: int = 2000
    phase1_switches: bool = False
    phase2_lines: bool = False
    phase3_tactical: bool = False
    fellow_harness: bool = False
    fellow_placement_debug: bool = False
    default_repeats: int = 1
    csv_fields: Sequence[str] = field(default_factory=tuple)
    extra_summary_keys: Sequence[str] = field(default_factory=tuple)
    digest_keys: Sequence[str] = field(default_factory=_default_digest_keys)


def run_phase_main(spec: PhaseRunnerSpec) -> int:
    """CLI entry for benchmark runners sharing the common runner flow.

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
    parser.add_argument(
        "--repeats",
        type=int,
        default=int(spec.default_repeats),
        help="Repeat each selected scenario this many times (default: 1).",
    )
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

    repeats = max(1, int(args.repeats))
    if int(args.repeats) != repeats:
        print(f"[{spec.runner_label}] repeats clamped to {repeats}.")

    scenario_runs: List[Tuple[int, int, Path]] = []
    for scenario in scenarios:
        for rep in range(1, repeats + 1):
            scenario_runs.append((rep, repeats, scenario))

    results: List[Dict[str, Any]] = []
    for idx, (rep_idx, rep_total, scenario) in enumerate(scenario_runs):
        rep_suffix = "" if rep_total == 1 else f"__r{rep_idx:02d}"
        log_path = logs_dir / f"{scenario.stem}{rep_suffix}.log"
        rep_label = "" if rep_total == 1 else f" [repeat {rep_idx}/{rep_total}]"
        print(f"[{spec.runner_label}] Running {scenario.name}{rep_label} ...")
        row = run_one_scenario_with_collect(
            root,
            scenario,
            log_path,
            int(args.time),
            phase1_switches=spec.phase1_switches,
            phase2_lines=spec.phase2_lines,
            phase3_tactical=spec.phase3_tactical,
            fellow_harness=spec.fellow_harness,
            fellow_placement_debug=spec.fellow_placement_debug,
        )
        row["repeat_index"] = int(rep_idx)
        row["repeat_count"] = int(rep_total)
        if rep_total > 1:
            row["scenario_instance"] = f"{row['scenario']}#r{rep_idx:02d}"
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
        print(f"[{spec.runner_label}] Log file: {log_path.resolve()}")
        if inter_run_delay_s > 0 and idx < (len(scenario_runs) - 1):
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
