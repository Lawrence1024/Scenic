"""Shared helpers for benchmark runners (phase0–phase11 + fellow harness).

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
    r"\[EvalEvent\]\s+t=(?P<t>\d+\.?\d*)s\s+type=eval_contact\s+severity=(?P<sev>\S+)\s+"
    r"bbox_gap_m=(?P<bbox>\S+)\s+dspace_obj1_m=(?P<ds>\S+)\s+dspace_valid=(?P<dsv>[01])"
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
RE_PHASE6_STATE = re.compile(
    r"\[Phase6State\]\s+t=(?P<t>\d+\.?\d*)s\s+has_opponent=(?P<opp>[01])\s+pit_mode=(?P<pit>[01])\s+ttl=(?P<ttl>\S+)\s+ego_speed=(?P<ego_v>\S+)\s+opp_speed=(?P<opp_v>\S+)\s+opp_dist=(?P<opp_d>\S+)\s+overlap=(?P<ov>\S+)\s+seg=(?P<seg>\S+)\s+ahead=(?P<ahead>[01])"
)
RE_PHASE6_PLANNER = re.compile(
    r"\[Phase6Planner\]\s+t=(?P<t>\d+\.?\d*)s\s+planner_state=(?P<state>\S+)\s+active_ttl=(?P<ttl>\S+)\s+target_speed_cap=(?P<cap>\S+)\s+decision_reason=(?P<reason>\S+)"
)
RE_PHASE6_GUARD = re.compile(
    r"\[Phase6Guard\]\s+t=(?P<t>\d+\.?\d*)s\s+guard_active=(?P<active>[01])\s+guard_reason=(?P<reason>\S+)\s+steer_limited=(?P<steer>[01])\s+brake_limited=(?P<brake>[01])\s+ttl_switch_blocked=(?P<ttl_block>[01])\s+emergency_stable_mode=(?P<emerg>[01])"
)
RE_PHASE6_EXECUTOR = re.compile(
    r"\[Phase6Executor\]\s+t=(?P<t>\d+\.?\d*)s\s+executor_call=(?P<call>[01])\s+planner_state=(?P<state>\S+)\s+active_ttl=(?P<ttl>\S+)\s+decision_reason=(?P<reason>\S+)\s+steer=(?P<steer>\S+)\s+throttle=(?P<throttle>\S+)\s+brake=(?P<brake>\S+)"
)
RE_PHASE7_PREDICTION = re.compile(
    r"\[Phase7Prediction\]\s+t=(?P<t>\d+\.?\d*)s\s+"
    r"fellow_pred_x=(?P<fpx>\S+)\s+fellow_pred_y=(?P<fpy>\S+)\s+fellow_pred_s=(?P<fps>\S+)\s+"
    r"prediction_error_next_step=(?P<e_next>\S+)\s+"
    r"prediction_error_zero_motion=(?P<e0>\S+)\s+"
    r"prediction_error_hold_last=(?P<eh>\S+)"
)
RE_PHASE8_ASSESSMENT = re.compile(
    r"\[Phase8Assessment\]\s+t=(?P<t>\d+\.?\d*)s\s+"
    r"fellow_relation=(?P<rel>\S+)\s+closing_flag=(?P<closing>[01])\s+"
    r"actual_gap=(?P<actual>\S+)\s+safe_gap=(?P<safe>\S+)\s+gap_ok=(?P<gap_ok>[01])\s+"
    r"optimal_open=(?P<opt>[01])\s+left_open=(?P<left>[01])\s+right_open=(?P<right>[01])\s+"
    r"overlap_flag=(?P<ov>[01])\s+emergency_risk_01=(?P<risk>\S+)\s+source=(?P<src>\S+)"
)
RE_PHASE9_PLANNER = re.compile(
    r"\[Phase9Planner\]\s+t=(?P<t>\d+\.?\d*)s\s+planner_state=(?P<state>\S+)\s+"
    r"chosen_ttl=(?P<ttl>\S+)\s+target_speed_cap=(?P<cap>\S+)\s+decision_reason=(?P<reason>\S+)\s+"
    r"assessment_relation=(?P<arel>\S+)\s+assessment_gap_ok=(?P<agap>\S+)\s+"
    r"assessment_optimal_open=(?P<aopt>\S+)\s+assessment_left_open=(?P<aleft>\S+)\s+assessment_right_open=(?P<aright>\S+)"
)
RE_PHASE10_GUARD = re.compile(
    r"\[Phase10Guard\]\s+t=(?P<t>\d+\.?\d*)s\s+guard_active=(?P<active>[01])\s+"
    r"guard_reason=(?P<reason>\S+)\s+steer_limited=(?P<steer>[01])\s+"
    r"brake_limited=(?P<brake>[01])\s+ttl_switch_blocked=(?P<ttl_block>[01])\s+"
    r"emergency_stable_mode=(?P<emerg>[01])\s+planner_state=(?P<state>\S+)\s+"
    r"active_ttl=(?P<ttl>\S+)\s+decision_reason=(?P<dec>\S+)\s+"
    r"steer=(?P<cmd_steer>\S+)\s+throttle=(?P<cmd_thr>\S+)\s+brake=(?P<cmd_brk>\S+)"
)
RE_PHASE11_PLANNER = re.compile(
    r"\[Phase11Planner\]\s+t=(?P<t>\d+\.?\d*)s\s+planner_state=(?P<state>\S+)\s+"
    r"chosen_ttl=(?P<ttl>\S+)\s+decision_reason=(?P<reason>\S+)\s+"
    r"commit_trigger=(?P<commit>\S+)\s+abort_trigger=(?P<abort>\S+)\s+"
    r"pass_success=(?P<pass>[01])\s+abort_success=(?P<abort_ok>[01])\s+"
    r"post_event_state=(?P<post>\S+)"
)
RE_PHASE12_PLANNER = re.compile(
    r"\[Phase11Planner\]\s+t=(?P<t>\d+\.?\d*)s\s+planner_state=(?P<state>\S+)\s+"
    r"chosen_ttl=(?P<ttl>\S+)\s+decision_reason=(?P<reason>\S+)\s+"
    r"commit_trigger=(?P<commit>\S+)\s+abort_trigger=(?P<abort>\S+)\s+"
    r"pass_success=(?P<pass>[01])\s+abort_success=(?P<abort_ok>[01])\s+"
    r"post_event_state=(?P<post>\S+)"
    r".*seg_ctx=(?P<seg_ctx>\S+)\s+seg_modifier=(?P<seg_modifier>\S+)"
)
RE_LOG_TIME_S = re.compile(r"\bt=(?P<t>\d+\.?\d*)s\b")
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
    "phase6_state_line_count",
    "phase6_planner_line_count",
    "phase6_guard_line_count",
    "phase6_executor_line_count",
    "phase6_guard_active_count",
    "eval_contact_overlap_count",
    "eval_contact_near_count",
    "eval_contact_overlap_dspace_invalid_count",
    "eval_contact_near_dspace_invalid_count",
    "collision_eval_hull_overlap",
    "phase7_prediction_line_count",
    "phase7_prediction_error_next_step_mean",
    "phase7_prediction_error_next_step_max",
    "phase7_prediction_error_zero_motion_mean",
    "phase7_prediction_error_hold_last_mean",
    "phase7_prediction_gain_vs_zero_mean",
    "phase7_prediction_regret_vs_hold_mean",
    "phase7_prediction_ratio_vs_hold_mean",
    "phase8_assessment_line_count",
    "phase8_fellow_relation_ahead_count",
    "phase8_fellow_relation_behind_count",
    "phase8_gap_ok_rate",
    "phase8_safe_gap_mean",
    "phase8_actual_gap_mean",
    "phase8_optimal_open_rate",
    "phase8_left_open_rate",
    "phase8_right_open_rate",
    "phase8_closing_flag_rate",
    "phase8_emergency_risk_mean",
    "phase9_planner_line_count",
    "phase9_free_run_count",
    "phase9_follow_count",
    "phase9_setup_pass_left_count",
    "phase9_setup_pass_right_count",
    "phase9_state_change_count",
    "phase9_gap_ok_rate",
    "phase10_guard_line_count",
    "phase10_guard_active_count",
    "phase10_steer_limited_count",
    "phase10_brake_limited_count",
    "phase10_ttl_switch_blocked_count",
    "phase10_emergency_stable_count",
    "phase11_planner_line_count",
    "phase11_commit_trigger_count",
    "phase11_abort_trigger_count",
    "phase11_pass_success_count",
    "phase11_abort_success_count",
    "phase11_commit_pass_left_count",
    "phase11_commit_pass_right_count",
    "phase11_abort_pass_count",
    "phase12_seg_straight_count",
    "phase12_seg_corner_entry_count",
    "phase12_seg_corner_body_count",
    "phase12_seg_corner_exit_count",
    "phase12_seg_modifier_blocked_count",
    "phase12_seg_modifier_conservative_count",
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
    ignore_before_s: float = 1.0,
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
    phase6_state_line_count = 0
    phase6_planner_line_count = 0
    phase6_guard_line_count = 0
    phase6_executor_line_count = 0
    phase6_guard_active_count = 0
    phase6_states: List[str] = []
    phase6_ttls: List[str] = []
    phase6_reasons: List[str] = []
    eval_contact_overlap_count = 0
    eval_contact_near_count = 0
    eval_contact_overlap_dspace_invalid_count = 0
    eval_contact_near_dspace_invalid_count = 0
    phase7_line_count = 0
    phase7_err_next: List[float] = []
    phase7_err_zero: List[float] = []
    phase7_err_hold: List[float] = []
    phase8_line_count = 0
    phase8_rel_ahead_count = 0
    phase8_rel_behind_count = 0
    phase8_gap_ok_count = 0
    phase8_opt_open_count = 0
    phase8_left_open_count = 0
    phase8_right_open_count = 0
    phase8_closing_count = 0
    phase8_safe_gap_vals: List[float] = []
    phase8_actual_gap_vals: List[float] = []
    phase8_risk_vals: List[float] = []
    phase9_line_count = 0
    phase9_states: List[str] = []
    phase9_ttls: List[str] = []
    phase9_reasons: List[str] = []
    phase9_gap_ok_count = 0
    phase9_gap_ok_known = 0
    phase10_guard_line_count = 0
    phase10_guard_active_count = 0
    phase10_steer_limited_count = 0
    phase10_brake_limited_count = 0
    phase10_ttl_switch_blocked_count = 0
    phase10_emergency_stable_count = 0
    phase11_planner_line_count = 0
    phase11_commit_trigger_count = 0
    phase11_abort_trigger_count = 0
    phase11_pass_success_count = 0
    phase11_abort_success_count = 0
    phase11_commit_pass_left_count = 0
    phase11_commit_pass_right_count = 0
    phase11_abort_pass_count = 0
    phase12_seg_straight_count = 0
    phase12_seg_corner_entry_count = 0
    phase12_seg_corner_body_count = 0
    phase12_seg_corner_exit_count = 0
    phase12_seg_modifier_blocked_count = 0
    phase12_seg_modifier_conservative_count = 0

    _ignore_before = max(0.0, float(ignore_before_s))
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            _line_t: Optional[float] = None
            _tm = RE_LOG_TIME_S.search(line)
            if _tm:
                _line_t = parse_float_or_none(_tm.group("t"))
            if _line_t is not None and _line_t < _ignore_before:
                continue
            if "[Phase6State]" in line:
                p6s = RE_PHASE6_STATE.search(line)
                if p6s:
                    phase6_state_line_count += 1
                    phase6_ttls.append(p6s.group("ttl"))
            if "[Phase6Planner]" in line:
                p6p = RE_PHASE6_PLANNER.search(line)
                if p6p:
                    phase6_planner_line_count += 1
                    phase6_states.append(p6p.group("state"))
                    phase6_ttls.append(p6p.group("ttl"))
                    phase6_reasons.append(p6p.group("reason"))
            if "[Phase6Guard]" in line:
                p6g = RE_PHASE6_GUARD.search(line)
                if p6g:
                    phase6_guard_line_count += 1
                    if p6g.group("active") == "1":
                        phase6_guard_active_count += 1
            if "[Phase6Executor]" in line:
                p6e = RE_PHASE6_EXECUTOR.search(line)
                if p6e:
                    phase6_executor_line_count += 1
            if "[Phase7Prediction]" in line:
                p7 = RE_PHASE7_PREDICTION.search(line)
                if p7:
                    phase7_line_count += 1
                    _en = parse_float_or_none(p7.group("e_next"))
                    _ez = parse_float_or_none(p7.group("e0"))
                    _eh = parse_float_or_none(p7.group("eh"))
                    if _en is not None:
                        phase7_err_next.append(_en)
                    if _ez is not None:
                        phase7_err_zero.append(_ez)
                    if _eh is not None:
                        phase7_err_hold.append(_eh)
            if "[Phase8Assessment]" in line:
                p8 = RE_PHASE8_ASSESSMENT.search(line)
                if p8:
                    phase8_line_count += 1
                    _rel = str(p8.group("rel") or "")
                    if _rel == "ahead":
                        phase8_rel_ahead_count += 1
                    elif _rel == "behind":
                        phase8_rel_behind_count += 1
                    if p8.group("gap_ok") == "1":
                        phase8_gap_ok_count += 1
                    if p8.group("opt") == "1":
                        phase8_opt_open_count += 1
                    if p8.group("left") == "1":
                        phase8_left_open_count += 1
                    if p8.group("right") == "1":
                        phase8_right_open_count += 1
                    if p8.group("closing") == "1":
                        phase8_closing_count += 1
                    _safe = parse_float_or_none(p8.group("safe"))
                    _actual = parse_float_or_none(p8.group("actual"))
                    _risk = parse_float_or_none(p8.group("risk"))
                    if _safe is not None:
                        phase8_safe_gap_vals.append(_safe)
                    if _actual is not None:
                        phase8_actual_gap_vals.append(_actual)
                    if _risk is not None:
                        phase8_risk_vals.append(_risk)
            if "[Phase9Planner]" in line:
                p9 = RE_PHASE9_PLANNER.search(line)
                if p9:
                    phase9_line_count += 1
                    phase9_states.append(p9.group("state"))
                    phase9_ttls.append(p9.group("ttl"))
                    phase9_reasons.append(p9.group("reason"))
                    _ag = str(p9.group("agap") or "na").lower()
                    if _ag in ("0", "1"):
                        phase9_gap_ok_known += 1
                        if _ag == "1":
                            phase9_gap_ok_count += 1
            if "[Phase10Guard]" in line:
                p10 = RE_PHASE10_GUARD.search(line)
                if p10:
                    phase10_guard_line_count += 1
                    if p10.group("active") == "1":
                        phase10_guard_active_count += 1
                    if p10.group("steer") == "1":
                        phase10_steer_limited_count += 1
                    if p10.group("brake") == "1":
                        phase10_brake_limited_count += 1
                    if p10.group("ttl_block") == "1":
                        phase10_ttl_switch_blocked_count += 1
                    if p10.group("emerg") == "1":
                        phase10_emergency_stable_count += 1
            if "[Phase11Planner]" in line:
                p11 = RE_PHASE11_PLANNER.search(line)
                if p11:
                    phase11_planner_line_count += 1
                    if p11.group("commit") != "none":
                        phase11_commit_trigger_count += 1
                    if p11.group("abort") != "none":
                        phase11_abort_trigger_count += 1
                    if p11.group("pass") == "1":
                        phase11_pass_success_count += 1
                    if p11.group("abort_ok") == "1":
                        phase11_abort_success_count += 1
                    _p11_state = str(p11.group("state") or "")
                    if _p11_state == "COMMIT_PASS_LEFT":
                        phase11_commit_pass_left_count += 1
                    elif _p11_state == "COMMIT_PASS_RIGHT":
                        phase11_commit_pass_right_count += 1
                    elif _p11_state == "ABORT_PASS":
                        phase11_abort_pass_count += 1
                p12 = RE_PHASE12_PLANNER.search(line)
                if p12:
                    _seg_ctx = str(p12.group("seg_ctx") or "none")
                    _seg_mod = str(p12.group("seg_modifier") or "normal")
                    if _seg_ctx == "straight":
                        phase12_seg_straight_count += 1
                    elif _seg_ctx == "corner_entry":
                        phase12_seg_corner_entry_count += 1
                    elif _seg_ctx == "corner_body":
                        phase12_seg_corner_body_count += 1
                    elif _seg_ctx == "corner_exit":
                        phase12_seg_corner_exit_count += 1
                    if _seg_mod == "blocked":
                        phase12_seg_modifier_blocked_count += 1
                    elif _seg_mod == "conservative":
                        phase12_seg_modifier_conservative_count += 1
            evc = RE_EVAL_CONTACT_EVENT.search(line)
            if evc:
                sev = evc.group("sev")
                dsv = evc.group("dsv")
                if sev == "overlap":
                    eval_contact_overlap_count += 1
                    if dsv == "0":
                        eval_contact_overlap_dspace_invalid_count += 1
                elif sev == "near":
                    eval_contact_near_count += 1
                    if dsv == "0":
                        eval_contact_near_dspace_invalid_count += 1
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
    out["phase6_state_line_count"] = phase6_state_line_count
    out["phase6_planner_line_count"] = phase6_planner_line_count
    out["phase6_guard_line_count"] = phase6_guard_line_count
    out["phase6_executor_line_count"] = phase6_executor_line_count
    out["phase6_guard_active_count"] = phase6_guard_active_count
    out["phase6_states_observed"] = sorted(set(phase6_states))
    out["phase6_ttls_observed"] = sorted(set(phase6_ttls))
    out["phase6_reasons_observed"] = sorted(set(phase6_reasons))
    out["eval_contact_overlap_count"] = eval_contact_overlap_count
    out["eval_contact_near_count"] = eval_contact_near_count
    out["eval_contact_overlap_dspace_invalid_count"] = eval_contact_overlap_dspace_invalid_count
    out["eval_contact_near_dspace_invalid_count"] = eval_contact_near_dspace_invalid_count
    out["collision_eval_hull_overlap"] = bool(eval_contact_overlap_count > 0)
    out["phase7_prediction_line_count"] = phase7_line_count
    out["phase7_prediction_error_next_step_mean"] = (
        (sum(phase7_err_next) / len(phase7_err_next)) if phase7_err_next else None
    )
    out["phase7_prediction_error_next_step_max"] = max(phase7_err_next) if phase7_err_next else None
    out["phase7_prediction_error_zero_motion_mean"] = (
        (sum(phase7_err_zero) / len(phase7_err_zero)) if phase7_err_zero else None
    )
    out["phase7_prediction_error_hold_last_mean"] = (
        (sum(phase7_err_hold) / len(phase7_err_hold)) if phase7_err_hold else None
    )
    if out["phase7_prediction_error_next_step_mean"] is not None and out["phase7_prediction_error_zero_motion_mean"] is not None:
        out["phase7_prediction_gain_vs_zero_mean"] = (
            float(out["phase7_prediction_error_zero_motion_mean"])
            - float(out["phase7_prediction_error_next_step_mean"])
        )
    else:
        out["phase7_prediction_gain_vs_zero_mean"] = None
    if out["phase7_prediction_error_next_step_mean"] is not None and out["phase7_prediction_error_hold_last_mean"] is not None:
        _next_m = float(out["phase7_prediction_error_next_step_mean"])
        _hold_m = float(out["phase7_prediction_error_hold_last_mean"])
        out["phase7_prediction_regret_vs_hold_mean"] = _next_m - _hold_m
        out["phase7_prediction_ratio_vs_hold_mean"] = (_next_m / _hold_m) if _hold_m > 1e-12 else None
    else:
        out["phase7_prediction_regret_vs_hold_mean"] = None
        out["phase7_prediction_ratio_vs_hold_mean"] = None
    out["phase8_assessment_line_count"] = phase8_line_count
    out["phase8_fellow_relation_ahead_count"] = phase8_rel_ahead_count
    out["phase8_fellow_relation_behind_count"] = phase8_rel_behind_count
    out["phase8_gap_ok_rate"] = (
        (float(phase8_gap_ok_count) / float(phase8_line_count)) if phase8_line_count > 0 else None
    )
    out["phase8_safe_gap_mean"] = (
        (sum(phase8_safe_gap_vals) / len(phase8_safe_gap_vals)) if phase8_safe_gap_vals else None
    )
    out["phase8_actual_gap_mean"] = (
        (sum(phase8_actual_gap_vals) / len(phase8_actual_gap_vals)) if phase8_actual_gap_vals else None
    )
    out["phase8_optimal_open_rate"] = (
        (float(phase8_opt_open_count) / float(phase8_line_count)) if phase8_line_count > 0 else None
    )
    out["phase8_left_open_rate"] = (
        (float(phase8_left_open_count) / float(phase8_line_count)) if phase8_line_count > 0 else None
    )
    out["phase8_right_open_rate"] = (
        (float(phase8_right_open_count) / float(phase8_line_count)) if phase8_line_count > 0 else None
    )
    out["phase8_closing_flag_rate"] = (
        (float(phase8_closing_count) / float(phase8_line_count)) if phase8_line_count > 0 else None
    )
    out["phase8_emergency_risk_mean"] = (
        (sum(phase8_risk_vals) / len(phase8_risk_vals)) if phase8_risk_vals else None
    )
    out["phase9_planner_line_count"] = phase9_line_count
    out["phase9_free_run_count"] = sum(1 for s in phase9_states if s == "FREE_RUN")
    out["phase9_follow_count"] = sum(1 for s in phase9_states if s == "FOLLOW")
    out["phase9_setup_pass_left_count"] = sum(
        1 for s in phase9_states if s in ("SETUP_LEFT", "SETUP_PASS_LEFT")
    )
    out["phase9_setup_pass_right_count"] = sum(
        1 for s in phase9_states if s in ("SETUP_RIGHT", "SETUP_PASS_RIGHT")
    )
    _changes = 0
    for i in range(1, len(phase9_states)):
        if phase9_states[i] != phase9_states[i - 1]:
            _changes += 1
    out["phase9_state_change_count"] = _changes
    out["phase9_states_observed"] = sorted(set(phase9_states))
    out["phase9_ttls_observed"] = sorted(set(phase9_ttls))
    out["phase9_reasons_observed"] = sorted(set(phase9_reasons))
    out["phase9_gap_ok_rate"] = (
        (float(phase9_gap_ok_count) / float(phase9_gap_ok_known))
        if phase9_gap_ok_known > 0
        else None
    )
    out["phase10_guard_line_count"] = phase10_guard_line_count
    out["phase10_guard_active_count"] = phase10_guard_active_count
    out["phase10_steer_limited_count"] = phase10_steer_limited_count
    out["phase10_brake_limited_count"] = phase10_brake_limited_count
    out["phase10_ttl_switch_blocked_count"] = phase10_ttl_switch_blocked_count
    out["phase10_emergency_stable_count"] = phase10_emergency_stable_count
    out["phase11_planner_line_count"] = phase11_planner_line_count
    out["phase11_commit_trigger_count"] = phase11_commit_trigger_count
    out["phase11_abort_trigger_count"] = phase11_abort_trigger_count
    out["phase11_pass_success_count"] = phase11_pass_success_count
    out["phase11_abort_success_count"] = phase11_abort_success_count
    out["phase11_commit_pass_left_count"] = phase11_commit_pass_left_count
    out["phase11_commit_pass_right_count"] = phase11_commit_pass_right_count
    out["phase11_abort_pass_count"] = phase11_abort_pass_count
    out["phase12_seg_straight_count"] = phase12_seg_straight_count
    out["phase12_seg_corner_entry_count"] = phase12_seg_corner_entry_count
    out["phase12_seg_corner_body_count"] = phase12_seg_corner_body_count
    out["phase12_seg_corner_exit_count"] = phase12_seg_corner_exit_count
    out["phase12_seg_modifier_blocked_count"] = phase12_seg_modifier_blocked_count
    out["phase12_seg_modifier_conservative_count"] = phase12_seg_modifier_conservative_count
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


def run_scenic_scenario(
    repo_root: Path,
    scenario: Path,
    out_log: Path,
    sim_steps: int,
    *,
    scenic_extra_args: Optional[Sequence[str]] = None,
) -> Dict[str, Any]:
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
    if scenic_extra_args:
        cmd.extend(list(scenic_extra_args))
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
    scenic_extra_args: Optional[Sequence[str]] = None,
    analysis_ignore_before_s: float = 1.0,
) -> Dict[str, Any]:
    base = run_scenic_scenario(
        repo_root, scenario, out_log, sim_steps, scenic_extra_args=scenic_extra_args
    )
    base.update(
        collect_metrics_from_log(
            out_log,
            phase1_switches=phase1_switches,
            phase2_lines=phase2_lines,
            phase3_tactical=phase3_tactical,
            ignore_before_s=analysis_ignore_before_s,
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
    default_scenario_names: Sequence[str] = field(default_factory=tuple)
    phase1_switches: bool = False
    phase2_lines: bool = False
    phase3_tactical: bool = False
    fellow_harness: bool = False
    fellow_placement_debug: bool = False
    scenic_extra_args: Tuple[str, ...] = ()
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
        "--analysis-ignore-before-s",
        type=float,
        default=1.0,
        help="Ignore parsed log metrics before this sim time (startup transient filter).",
    )
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

    # Apply phase-default scenario subsets only when the caller did not
    # explicitly request scenarios/globs. Explicit CLI filters must win.
    if spec.default_scenario_names and (not args.scenario) and (not args.scenario_glob):
        wanted_default = set(spec.default_scenario_names)
        scenarios = [s for s in scenarios if s.name in wanted_default]
        missing_default = sorted(wanted_default - {s.name for s in scenarios})
        if missing_default:
            print(
                f"Warning: default scenarios missing in {scenario_dir}: {', '.join(missing_default)}",
                file=sys.stderr,
            )

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
            scenic_extra_args=spec.scenic_extra_args or None,
            analysis_ignore_before_s=float(args.analysis_ignore_before_s),
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
        "analysis_ignore_before_s": float(max(0.0, float(args.analysis_ignore_before_s))),
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
