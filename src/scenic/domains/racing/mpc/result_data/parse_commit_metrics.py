#!/usr/bin/env python3
"""
Parse logs from To-Do 3 (commit mode) runs and compute Task 6 success metrics.

Reads log lines:
- [FollowRacingLineMPC] commit_log ... curv_ahead_raw=... curv_ahead_filt=... approaching_curve=...
  commit_active=... commit_t_remain=... s_ref_raw=... s_ref_used=... ds_ref=... seg_id_raw=...
  seg_id_used=... curv_regime_raw=... curv_regime_used=...
- [FollowRacingLineMPC] ref_log ... s_ref=... dS_ref=... cte_behavior=...
- [FollowRacingLineMPC] ff_log ... delta_cmd_rad=...

Computes for the 2-second window before each curve entry (approaching_curve True):
  1. # segment_id switches
  2. std(ds_ref)
  3. max |CTE|
  4. max |delta_cmd(t) - delta_cmd(t-1)| (jerkiness proxy)

Usage:
  python -m scenic.domains.racing.mpc.result_data.parse_commit_metrics --log run.log
  python parse_commit_metrics.py run.log
"""

import argparse
import re
import statistics
import sys
from pathlib import Path

# Optional: assume fixed dt between lines if no timestamp (e.g. 0.05 s per step)
DEFAULT_DT = 0.05
PRE_CURVE_WINDOW_S = 2.0


def _parse_commit_log(line: str) -> dict:
    d = {}
    for key in (
        "curv_ahead_raw", "curv_ahead_filt", "approaching_curve",
        "commit_active", "commit_t_remain", "commit_anchor_s_ref",
        "commit_anchor_seg_id", "commit_curv_regime",
        "s_ref_raw", "s_ref_used", "ds_ref", "seg_id_raw", "seg_id_used",
        "curv_regime_raw", "curv_regime_used", "ref_switch_count_in_commit",
    ):
        m = re.search(rf"\b{key}=(\S+)", line)
        if m:
            v = m.group(1)
            if key == "approaching_curve" or key == "commit_active":
                d[key] = v.lower() == "true"
            elif key in ("commit_t_remain", "s_ref_raw", "s_ref_used", "ds_ref") and v != "?":
                try:
                    d[key] = float(v)
                except ValueError:
                    pass
            elif key in ("commit_anchor_seg_id", "seg_id_raw", "seg_id_used", "ref_switch_count_in_commit") and v != "?":
                try:
                    d[key] = int(v)
                except ValueError:
                    pass
            else:
                d[key] = v
    return d


def _parse_ref_log(line: str) -> dict:
    d = {}
    m = re.search(r"\bs_ref=(\S+)", line)
    if m and m.group(1) != "?":
        try:
            d["s_ref"] = float(m.group(1))
        except ValueError:
            pass
    m = re.search(r"\bdS_ref=(\S+)", line)
    if m and m.group(1) != "?":
        try:
            d["ds_ref"] = float(m.group(1))
        except ValueError:
            pass
    m = re.search(r"\bcte_behavior=(\S+)", line)
    if m and m.group(1) != "?":
        try:
            d["cte"] = float(m.group(1))
        except ValueError:
            pass
    return d


def _parse_ff_log(line: str) -> dict:
    d = {}
    m = re.search(r"\bdelta_cmd_rad=(\S+)", line)
    if m and m.group(1) != "?":
        try:
            d["delta_cmd"] = float(m.group(1))
        except ValueError:
            pass
    return d


def _parse_time(line: str) -> float:
    m = re.search(r"t=(\d+\.?\d*)s", line)
    if m:
        return float(m.group(1))
    return None


def run_parser(log_path: Path, dt: float = DEFAULT_DT) -> dict:
    """Parse log and return per-run lists and curve windows."""
    rows = []  # list of (t, commit_d, ref_d, ff_d)
    t_cur = 0.0
    commit_d = {}
    ref_d = {}
    ff_d = {}
    with open(log_path, encoding="utf-8", errors="replace") as f:
        for line in f:
            t = _parse_time(line)
            if t is not None:
                t_cur = t
            if "[FollowRacingLineMPC] commit_log" in line:
                commit_d = _parse_commit_log(line)
                rows.append((t_cur, dict(commit_d), dict(ref_d), dict(ff_d)))
            elif "[FollowRacingLineMPC] ref_log" in line:
                ref_d = _parse_ref_log(line)
            elif "[FollowRacingLineMPC] ff_log" in line:
                ff_d = _parse_ff_log(line)

    # Build time series for seg_id_used, ds_ref, cte, delta_cmd
    seg_ids = []
    ds_refs = []
    ctes = []
    delta_cmds = []
    times = []
    for t, cd, rd, fd in rows:
        times.append(t)
        seg_ids.append(cd.get("seg_id_used", cd.get("seg_id_raw")))
        ds_refs.append(cd.get("ds_ref", rd.get("ds_ref")))
        ctes.append(rd.get("cte"))
        delta_cmds.append(fd.get("delta_cmd"))
    # Curve entries: where approaching_curve becomes True (simplified: any line with approaching_curve=True)
    curve_entry_indices = []
    for i, (t, cd, _, _) in enumerate(rows):
        if cd.get("approaching_curve") is True:
            curve_entry_indices.append(i)
    # Dedupe consecutive True -> one curve entry per "segment"
    entries = []
    for i in curve_entry_indices:
        if not entries or i > entries[-1] + 5:
            entries.append(i)

    # For each curve entry, take 2 s window before (by time or by step count)
    window_dt = PRE_CURVE_WINDOW_S
    metrics_per_window = []
    for idx in entries:
        t_entry = times[idx] if idx < len(times) else 0.0
        t_start = max(0.0, t_entry - window_dt)
        start_idx = 0
        for j, t in enumerate(times):
            if t >= t_start:
                start_idx = j
                break
        end_idx = idx
        if start_idx >= end_idx:
            continue
        seg_slice = [s for s in seg_ids[start_idx:end_idx] if s is not None]
        ds_slice = [d for d in ds_refs[start_idx:end_idx] if d is not None]
        cte_slice = [c for c in ctes[start_idx:end_idx] if c is not None]
        dc_slice = [d for d in delta_cmds[start_idx:end_idx] if d is not None]

        n_switches = sum(1 for i in range(1, len(seg_slice)) if seg_slice[i] != seg_slice[i - 1])
        std_ds = statistics.stdev(ds_slice) if len(ds_slice) > 1 else 0.0
        max_cte = max(abs(c) for c in cte_slice) if cte_slice else 0.0
        jerk = 0.0
        if len(dc_slice) > 1:
            jerk = max(abs(dc_slice[i] - dc_slice[i - 1]) for i in range(1, len(dc_slice)))
        metrics_per_window.append({
            "segment_id_switches": n_switches,
            "std_ds_ref": std_ds,
            "max_abs_cte": max_cte,
            "max_delta_cmd_jerk": jerk,
        })

    # Aggregate over all windows (e.g. mean or max)
    if not metrics_per_window:
        return {
            "n_curve_windows": 0,
            "segment_id_switches": None,
            "std_ds_ref": None,
            "max_abs_cte": None,
            "max_delta_cmd_jerk": None,
        }
    return {
        "n_curve_windows": len(metrics_per_window),
        "segment_id_switches": sum(m["segment_id_switches"] for m in metrics_per_window) / len(metrics_per_window),
        "std_ds_ref": sum(m["std_ds_ref"] for m in metrics_per_window) / len(metrics_per_window),
        "max_abs_cte": max(m["max_abs_cte"] for m in metrics_per_window),
        "max_delta_cmd_jerk": max(m["max_delta_cmd_jerk"] for m in metrics_per_window),
        "per_window": metrics_per_window,
    }


def main():
    ap = argparse.ArgumentParser(description="Parse commit-mode logs and compute Task 6 metrics")
    ap.add_argument("--log", "-l", type=Path, required=True, help="Log file path")
    ap.add_argument("--dt", type=float, default=DEFAULT_DT, help="Assumed time step (s) when t= not in line")
    args = ap.parse_args()
    if not args.log.exists():
        print(f"File not found: {args.log}", file=sys.stderr)
        sys.exit(1)
    result = run_parser(args.log, args.dt)
    print("Task 6 metrics (2 s pre-curve windows):")
    print(f"  # curve windows: {result['n_curve_windows']}")
    print(f"  # segment_id switches (avg per window): {result.get('segment_id_switches')}")
    print(f"  std(ds_ref) (avg): {result.get('std_ds_ref')}")
    print(f"  max |CTE|: {result.get('max_abs_cte')}")
    print(f"  max |delta_cmd(t)-delta_cmd(t-1)|: {result.get('max_delta_cmd_jerk')}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
