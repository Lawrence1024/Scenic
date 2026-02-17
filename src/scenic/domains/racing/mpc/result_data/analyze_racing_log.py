#!/usr/bin/env python3
"""
Parse racing run logs and evaluate performance, with focus on time spent per segment.

Reads log lines that contain:
- "[FollowRacingLineMPC] t=X.Xs ... segment N name" (MPC step, gives segment at time t)
- "[FollowRacingLineMPCBehavior] t=X.Xs WAYPOINT HIT: ... segment N name"

Computes:
- Time (s) spent in each segment (from first to last waypoint hit in that segment).
- Waypoint hits per segment.
- Optional: speed/CTE summary from MPC lines.

Output includes pandas DataFrames (when pandas is installed) for easy visualization:
- segments_df: one row per segment (segment_id, segment_name, time_s, pct, waypoint_hits).
- events_df: one row per waypoint hit (t, segment_id, segment_name).
- mpc_df: one row per MPC sample (t, segment_id, segment_name, speed_mps, cte_m).

Results for each log are written to result_data/<run_id>/ (this folder; segments.csv,
events.csv, mpc.csv, summary.json). Use compare_racing_results.py in this folder to compare runs.

Usage (from repo root):
    python -m scenic.domains.racing.mpc.result_data.analyze_racing_log [--log PATH] [--csv] [--no-table] [--output FILE] [--no-result-dir]

Example:
    python -m scenic.domains.racing.mpc.result_data.analyze_racing_log --log run.log
    python -m scenic.domains.racing.mpc.result_data.compare_racing_results

Programmatic:
    from scenic.domains.racing.mpc.result_data.analyze_racing_log import run_analysis
    result = run_analysis("run.log")
    result.segments_df.plot.bar(x="segment_id", y="time_s")  # if pandas available
"""

import argparse
import json
import re
import sys
from collections import defaultdict
from datetime import datetime, timezone
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

# This folder (result_data): per-log results go into subfolders result_data/<run_id>/
RESULT_DATA_DIR = Path(__file__).resolve().parent

try:
    import pandas as pd
    _PANDAS_AVAILABLE = True
except ImportError:
    pd = None  # type: ignore
    _PANDAS_AVAILABLE = False

# Regex for time: t=12.34s or t=0.00s
RE_TIME = re.compile(r"t=(\d+\.?\d*)s")
# Regex for segment: "segment 44 straight" or "segment 2 curve"
RE_SEGMENT = re.compile(r"segment\s+(\d+)\s*(\w*)")
# Run identifier from dSPACE TTL loader: [RacingRun] TTL=... run_timestamp=... edit_note=... (edit_note optional, may contain spaces)
RE_RACING_RUN = re.compile(r"\[RacingRun\]\s+TTL=(\S+)(?:\s+run_timestamp=(\S+))?(?:\s+edit_note=(.*))?")
# Fallback for logs before [RacingRun] existed: [TTL] Assigned TTL PolylineRegion to ego (ttl_racing_line_xodr.csv)
RE_TTL_ASSIGNED_EGO = re.compile(r"\[TTL\]\s+Assigned TTL PolylineRegion to ego\s+\(([^)]+)\)")


def _find_repo_root() -> Path:
    """Find repo root (directory containing 'src' and 'assets')."""
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent


def _parse_log_with_encoding(
    path: Path,
    encoding: str,
    events: List[Tuple[float, int, str]],
    mpc_samples: List[Tuple[float, int, str, float, float]],
    run_info: Optional[Dict[str, str]] = None,
) -> Optional[Tuple[int, str]]:
    """Parse log with given encoding. Fills events, mpc_samples, and run_info (if provided). Returns first_mpc_segment or None."""
    re_speed = re.compile(r"speed=(\d+\.?\d*)m/s")
    re_cte = re.compile(r"CTE=(-?\d+\.?\d*)m")
    first: Optional[Tuple[int, str]] = None
    run_info_filled = False  # set True once we've parsed run identifier from log
    try:
        with open(path, "r", encoding=encoding, errors="replace") as f:
            for line in f:
                if not run_info_filled and run_info is not None:
                    rm = RE_RACING_RUN.search(line)
                    if rm:
                        run_info["ttl_name"] = rm.group(1).strip()
                        run_info["run_timestamp"] = (rm.group(2) or "").strip()
                        edit_note = (rm.group(3) or "").strip()
                        if edit_note:
                            run_info["edit_note"] = edit_note
                        run_info_filled = True  # only stop looking when we have [RacingRun] (so we get edit_note)
                    else:
                        tm = RE_TTL_ASSIGNED_EGO.search(line)
                        if tm:
                            run_info["ttl_name"] = tm.group(1).strip()
                            # do not set run_info_filled: [RacingRun] often appears next and has edit_note/run_timestamp
                time_m = RE_TIME.search(line)
                seg_m = RE_SEGMENT.search(line)
                if not time_m or not seg_m:
                    continue
                t = float(time_m.group(1))
                seg_id = int(seg_m.group(1))
                seg_name = (seg_m.group(2) or "").strip()

                if "WAYPOINT HIT" in line:
                    events.append((t, seg_id, seg_name))
                elif "[FollowRacingLineMPC]" in line and "Step" in line:
                    speed = 0.0
                    cte = 0.0
                    sm = re_speed.search(line)
                    if sm:
                        speed = float(sm.group(1))
                    cm = re_cte.search(line)
                    if cm:
                        cte = float(cm.group(1))
                    mpc_samples.append((t, seg_id, seg_name, speed, cte))
                    if first is None:
                        first = (seg_id, seg_name)
    except (UnicodeDecodeError, LookupError):
        return None
    return first


def parse_log(path: Path) -> Tuple[List[Tuple[float, int, str]], List[Tuple[float, int, str, float, float]], Dict[str, str]]:
    """
    Parse log file. Returns:
    - events: list of (t, segment_id, segment_name) from WAYPOINT HIT and first MPC line
    - mpc_samples: list of (t, segment_id, segment_name, speed_mps, cte_m) from FollowRacingLineMPC lines
    - run_info: dict with ttl_name, run_timestamp if [RacingRun] line found (else empty)

    Tries UTF-8 (with BOM) first; if no matching lines are found, retries with UTF-16 (Windows logs).
    """
    events: List[Tuple[float, int, str]] = []
    mpc_samples: List[Tuple[float, int, str, float, float]] = []
    run_info: Dict[str, str] = {}
    first_mpc_segment: Optional[Tuple[int, str]] = None

    first_mpc_segment = _parse_log_with_encoding(path, "utf-8-sig", events, mpc_samples, run_info)
    if not events and not mpc_samples:
        events.clear()
        mpc_samples.clear()
        run_info.clear()
        first_mpc_segment = _parse_log_with_encoding(path, "utf-16", events, mpc_samples, run_info)

    # Prepend initial segment from first MPC step so time from t=0 to first waypoint hit is attributed
    if first_mpc_segment and events and events[0][0] > 0:
        events.insert(0, (0.0, first_mpc_segment[0], first_mpc_segment[1]))

    return events, mpc_samples, run_info


def compute_segment_times(
    events: List[Tuple[float, int, str]]
) -> Tuple[Dict[int, float], Dict[int, str], Dict[int, int], float]:
    """
    From (t, seg_id, seg_name) events, compute time spent in each segment.
    Time between consecutive waypoint hits is attributed to the segment of the *previous* event.
    Waypoint hits per segment: count events (excluding synthetic t=0) by segment.
    Returns: seg_time_s, seg_name_map, seg_waypoint_count, total_time_s
    """
    seg_time_s: Dict[int, float] = defaultdict(float)
    seg_name_map: Dict[int, str] = {}
    seg_waypoint_count: Dict[int, int] = defaultdict(int)
    total_time_s = 0.0

    for i in range(1, len(events)):
        t_prev, seg_id_prev, seg_name_prev = events[i - 1]
        t_curr, seg_id_curr, seg_name_curr = events[i]
        dt = t_curr - t_prev
        seg_time_s[seg_id_prev] += dt
        seg_name_map[seg_id_prev] = seg_name_prev
        seg_name_map[seg_id_curr] = seg_name_curr
        total_time_s += dt
        # This waypoint hit (at t_curr) is in segment seg_id_curr
        seg_waypoint_count[seg_id_curr] += 1

    # Ensure any segment with time or waypoint hits has an entry
    for seg_id in set(seg_time_s) | set(seg_waypoint_count):
        if seg_id not in seg_name_map and seg_id in seg_time_s:
            seg_name_map[seg_id] = ""
        if seg_id not in seg_waypoint_count:
            seg_waypoint_count[seg_id] = 0

    return seg_time_s, seg_name_map, seg_waypoint_count, total_time_s


def _segment_cte_stats(
    mpc_samples: List[Tuple[float, int, str, float, float]],
) -> Tuple[Dict[int, float], Dict[int, float]]:
    """From MPC samples (t, seg_id, seg_name, speed, cte_m), compute per-segment mean |CTE| and max |CTE|."""
    seg_abs_ctes: Dict[int, List[float]] = defaultdict(list)
    for (_t, seg_id, _name, _speed, cte_m) in mpc_samples:
        seg_abs_ctes[seg_id].append(abs(cte_m))
    mean_abs: Dict[int, float] = {}
    max_abs: Dict[int, float] = {}
    for seg_id, vals in seg_abs_ctes.items():
        mean_abs[seg_id] = sum(vals) / len(vals)
        max_abs[seg_id] = max(vals)
    return mean_abs, max_abs


def build_dataframes(
    seg_time_s: Dict[int, float],
    seg_name_map: Dict[int, str],
    seg_waypoint_count: Dict[int, int],
    total_time_s: float,
    events: List[Tuple[float, int, str]],
    mpc_samples: List[Tuple[float, int, str, float, float]],
) -> Tuple[Optional[Any], Optional[Any], Optional[Any]]:
    """
    Build pandas DataFrames for visualization. Returns (segments_df, events_df, mpc_df).
    segments_df includes time_s, pct, waypoint_hits and (when MPC samples exist) mean_abs_cte_m, max_abs_cte_m.
    Any element is None if pandas is not installed or if the corresponding data is empty.
    """
    if not _PANDAS_AVAILABLE or pd is None:
        return (None, None, None)

    seg_mean_abs_cte, seg_max_abs_cte = _segment_cte_stats(mpc_samples) if mpc_samples else ({}, {})

    seg_ids_sorted = sorted(set(seg_time_s.keys()) | set(seg_waypoint_count.keys()))
    rows = []
    for seg_id in seg_ids_sorted:
        name = seg_name_map.get(seg_id, "")
        t = seg_time_s.get(seg_id, 0.0)
        pct = (t / total_time_s * 100.0) if total_time_s > 0 else 0.0
        hits = seg_waypoint_count.get(seg_id, 0)
        row = {
            "segment_id": seg_id,
            "segment_name": name or "(unknown)",
            "time_s": t,
            "pct": pct,
            "waypoint_hits": hits,
        }
        if seg_id in seg_mean_abs_cte:
            row["mean_abs_cte_m"] = seg_mean_abs_cte[seg_id]
            row["max_abs_cte_m"] = seg_max_abs_cte[seg_id]
        else:
            row["mean_abs_cte_m"] = None
            row["max_abs_cte_m"] = None
        rows.append(row)
    segments_df = pd.DataFrame(rows) if rows else None

    events_df = None
    if events:
        events_df = pd.DataFrame(
            events,
            columns=["t", "segment_id", "segment_name"],
        )

    mpc_df = None
    if mpc_samples:
        mpc_df = pd.DataFrame(
            mpc_samples,
            columns=["t", "segment_id", "segment_name", "speed_mps", "cte_m"],
        )

    return (segments_df, events_df, mpc_df)


def write_result_data(
    log_path: Path,
    segments_df: Optional[Any],
    events_df: Optional[Any],
    mpc_df: Optional[Any],
    summary: Dict[str, Any],
    run_info: Optional[Dict[str, str]] = None,
) -> Path:
    """
    Write analysis results for this log into result_data/<run_id>/.
    run_id is derived from run_info (ttl_name stem) when present, else log_path stem.
    Always writes summary.json; writes segments.csv, events.csv, mpc.csv when DataFrames are available.
    Returns the output directory path.
    """
    run_info = run_info or {}
    if run_info.get("ttl_name"):
        stem = Path(run_info["ttl_name"]).stem
    else:
        stem = log_path.stem or "log"
    out_dir = RESULT_DATA_DIR / stem
    out_dir.mkdir(parents=True, exist_ok=True)

    summary_out = {
        "log_path": str(log_path),
        "run_timestamp_utc": datetime.now(timezone.utc).isoformat(),
        **summary,
    }
    if run_info.get("ttl_name"):
        summary_out["ttl_name"] = run_info["ttl_name"]
    if run_info.get("run_timestamp"):
        summary_out["run_timestamp"] = run_info["run_timestamp"]
    if run_info.get("edit_note"):
        summary_out["edit_note"] = run_info["edit_note"]
    with open(out_dir / "summary.json", "w", encoding="utf-8") as f:
        json.dump(summary_out, f, indent=2)

    if segments_df is not None:
        segments_df.to_csv(out_dir / "segments.csv", index=False)
    if events_df is not None:
        events_df.to_csv(out_dir / "events.csv", index=False)
    if mpc_df is not None:
        mpc_df.to_csv(out_dir / "mpc.csv", index=False)

    return out_dir


def print_report(
    seg_time_s: Dict[int, float],
    seg_name_map: Dict[int, str],
    seg_waypoint_count: Dict[int, int],
    total_time_s: float,
    events: List[Tuple[float, int, str]],
    mpc_samples: List[Tuple[float, int, str, float, float]],
    csv: bool = False,
    no_table: bool = False,
    run_info: Optional[Dict[str, str]] = None,
    segments_df: Optional[Any] = None,
) -> None:
    """Print summary and per-segment table (or CSV). If segments_df has mean_abs_cte_m/max_abs_cte_m, they are printed."""
    n_waypoints = sum(seg_waypoint_count.values())
    if events:
        t_end = events[-1][0]
    else:
        t_end = 0.0

    run_info = run_info or {}
    print("=== Racing run summary ===")
    if run_info:
        ttl = run_info.get("ttl_name", "")
        ts = run_info.get("run_timestamp", "")
        if ttl:
            print(f"  TTL (from log):                   {ttl}")
        if ts:
            print(f"  Run timestamp (from log):         {ts}")
        note = run_info.get("edit_note", "")
        if note:
            print(f"  Edit note (from log):             {note}")
    print(f"  Total time (from waypoint events): {total_time_s:.2f} s")
    print(f"  Last event time:                   {t_end:.2f} s")
    print(f"  Waypoint hits:                     {n_waypoints}")
    if mpc_samples:
        speeds = [s[3] for s in mpc_samples]
        ctes = [s[4] for s in mpc_samples]
        print(f"  MPC samples:                      {len(mpc_samples)}")
        print(f"  Speed (from MPC): min={min(speeds):.1f} max={max(speeds):.1f} m/s")
        print(f"  |CTE| (from MPC): max={max(abs(c) for c in ctes):.3f} m")
    print()

    if no_table:
        return

    # Segments in order of first appearance (then by seg_id); include any with waypoint hits
    seg_ids_sorted = sorted(set(seg_time_s.keys()) | set(seg_waypoint_count.keys()))
    has_cte = (
        segments_df is not None
        and _PANDAS_AVAILABLE
        and "mean_abs_cte_m" in getattr(segments_df, "columns", [])
        and "max_abs_cte_m" in getattr(segments_df, "columns", [])
    )

    if csv:
        header = "segment_id,segment_name,time_s,pct,waypoint_hits"
        if has_cte:
            header += ",mean_abs_cte_m,max_abs_cte_m"
        print(header)
        for seg_id in seg_ids_sorted:
            name = seg_name_map.get(seg_id, "")
            t = seg_time_s.get(seg_id, 0.0)
            pct = (t / total_time_s * 100.0) if total_time_s > 0 else 0.0
            hits = seg_waypoint_count.get(seg_id, 0)
            line = f"{seg_id},{name},{t:.3f},{pct:.2f},{hits}"
            if has_cte:
                row = segments_df.loc[segments_df["segment_id"] == seg_id]
                if not row.empty and pd.notna(row["mean_abs_cte_m"].iloc[0]):
                    line += f",{row['mean_abs_cte_m'].iloc[0]:.4f},{row['max_abs_cte_m'].iloc[0]:.4f}"
                else:
                    line += ",,"
            print(line)
        return

    print("=== Time per segment ===")
    head = f"  {'Seg':<5} {'Name':<10} {'Time (s)':>10} {'%':>8} {'WP hits':>8}"
    if has_cte:
        head += "  mean_|CTE|  max_|CTE|"
    print(head)
    print("  " + ("-" * 60 if has_cte else "-" * 45))
    for seg_id in seg_ids_sorted:
        name = seg_name_map.get(seg_id, "") or "(unknown)"
        t = seg_time_s.get(seg_id, 0.0)
        pct = (t / total_time_s * 100.0) if total_time_s > 0 else 0.0
        hits = seg_waypoint_count.get(seg_id, 0)
        line = f"  {seg_id:<5} {name:<10} {t:>10.2f} {pct:>7.1f}% {hits:>8}"
        if has_cte:
            row = segments_df.loc[segments_df["segment_id"] == seg_id]
            if not row.empty and pd.notna(row["mean_abs_cte_m"].iloc[0]):
                line += f"  {row['mean_abs_cte_m'].iloc[0]:>8.4f}  {row['max_abs_cte_m'].iloc[0]:>8.4f}"
            else:
                line += "       -       -"
        print(line)
    print("  " + ("-" * 60 if has_cte else "-" * 45))
    print(f"  {'Total':<5} {'':<10} {total_time_s:>10.2f} {100.0:>7.1f}% {n_waypoints:>8}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Parse racing run log and report performance (time per segment, summary)."
    )
    parser.add_argument(
        "--log",
        default=None,
        help="Path to log file (default: run.log in repo root)",
    )
    parser.add_argument(
        "--csv",
        action="store_true",
        help="Output segment table as CSV",
    )
    parser.add_argument(
        "--no-table",
        action="store_true",
        help="Only print summary, no per-segment table",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        metavar="FILE",
        help="Write segment summary DataFrame to CSV (requires pandas)",
    )
    parser.add_argument(
        "--no-result-dir",
        action="store_true",
        help="Do not write results into result_data/<run_id>/",
    )
    args = parser.parse_args()

    repo_root = _find_repo_root()
    log_path = args.log
    if log_path is None:
        log_path = repo_root / "run.log"
    else:
        log_path = Path(log_path)
    if not log_path.is_absolute():
        log_path = repo_root / log_path
    if not log_path.exists():
        print(f"Log file not found: {log_path}", file=sys.stderr)
        return 1

    events, mpc_samples, run_info = parse_log(log_path)
    if not events:
        print("No WAYPOINT HIT or FollowRacingLineMPC segment lines found in log.", file=sys.stderr)
        return 1

    seg_time_s, seg_name_map, seg_waypoint_count, total_time_s = compute_segment_times(events)
    segments_df, events_df, mpc_df = build_dataframes(
        seg_time_s, seg_name_map, seg_waypoint_count, total_time_s, events, mpc_samples
    )

    print_report(
        seg_time_s,
        seg_name_map,
        seg_waypoint_count,
        total_time_s,
        events,
        mpc_samples,
        csv=args.csv,
        no_table=args.no_table,
        run_info=run_info,
        segments_df=segments_df,
    )

    t_end = events[-1][0] if events else 0.0
    summary_dict = {
        "total_time_s": total_time_s,
        "t_end": t_end,
        "n_waypoints": sum(seg_waypoint_count.values()),
        "n_mpc_samples": len(mpc_samples),
    }
    if mpc_samples:
        ctes_abs = [abs(s[4]) for s in mpc_samples]
        summary_dict["mean_abs_cte_m"] = sum(ctes_abs) / len(ctes_abs)
        summary_dict["max_abs_cte_m"] = max(ctes_abs)

    if not args.no_result_dir:
        out_dir = write_result_data(
            log_path, segments_df, events_df, mpc_df, summary_dict, run_info=run_info
        )
        print(f"\n[Result data] Wrote results to {out_dir}")

    if segments_df is not None:
        n_seg = len(segments_df)
        print(f"\n[DataFrame] Segment summary: {n_seg} rows (segments_df)")
        if events_df is not None:
            print(f"[DataFrame] Waypoint events: {len(events_df)} rows (events_df)")
        if mpc_df is not None:
            print(f"[DataFrame] MPC samples: {len(mpc_df)} rows (mpc_df)")
        if args.output:
            out_path = Path(args.output)
            if not out_path.is_absolute():
                out_path = repo_root / out_path
            segments_df.to_csv(out_path, index=False)
            print(f"[DataFrame] Wrote segment summary to {out_path}")
    elif args.output:
        print("Warning: pandas not installed; --output ignored. Install pandas for DataFrame/CSV output.", file=sys.stderr)

    return 0


def run_analysis(log_path: Optional[Path] = None) -> "AnalysisResult":
    """
    Run analysis on a log file and return an object with DataFrames and summary.

    Use for programmatic access and visualization, e.g.:
        result = run_analysis("run.log")
        result.segments_df.plot.bar(x="segment_id", y="time_s")
        result.mpc_df.plot(x="t", y="speed_mps")

    Attributes:
        segments_df: DataFrame with columns segment_id, segment_name, time_s, pct, waypoint_hits (None if no pandas).
        events_df: DataFrame with columns t, segment_id, segment_name (None if no pandas or no events).
        mpc_df: DataFrame with columns t, segment_id, segment_name, speed_mps, cte_m (None if no pandas or no MPC lines).
        summary: Dict with total_time_s, n_waypoints, t_end, etc.
    """
    root = _find_repo_root()
    path = Path(log_path) if log_path else root / "run.log"
    if not path.is_absolute():
        path = root / path
    if not path.exists():
        raise FileNotFoundError(f"Log file not found: {path}")
    events, mpc_samples, run_info = parse_log(path)
    if not events:
        raise ValueError("No WAYPOINT HIT or FollowRacingLineMPC segment lines found in log.")
    seg_time_s, seg_name_map, seg_waypoint_count, total_time_s = compute_segment_times(events)
    segments_df, events_df, mpc_df = build_dataframes(
        seg_time_s, seg_name_map, seg_waypoint_count, total_time_s, events, mpc_samples
    )
    t_end = events[-1][0] if events else 0.0
    summary = {
        "total_time_s": total_time_s,
        "t_end": t_end,
        "n_waypoints": sum(seg_waypoint_count.values()),
        "n_mpc_samples": len(mpc_samples),
    }
    if mpc_samples:
        ctes_abs = [abs(s[4]) for s in mpc_samples]
        summary["mean_abs_cte_m"] = sum(ctes_abs) / len(ctes_abs)
        summary["max_abs_cte_m"] = max(ctes_abs)
    if run_info.get("ttl_name"):
        summary["ttl_name"] = run_info["ttl_name"]
    if run_info.get("run_timestamp"):
        summary["run_timestamp"] = run_info["run_timestamp"]
    if run_info.get("edit_note"):
        summary["edit_note"] = run_info["edit_note"]
    return AnalysisResult(
        segments_df=segments_df,
        events_df=events_df,
        mpc_df=mpc_df,
        summary=summary,
    )


class AnalysisResult:
    """Result of run_analysis(); holds DataFrames and summary dict."""

    __slots__ = ("segments_df", "events_df", "mpc_df", "summary")

    def __init__(
        self,
        segments_df: Optional[Any] = None,
        events_df: Optional[Any] = None,
        mpc_df: Optional[Any] = None,
        summary: Optional[Dict[str, Any]] = None,
    ):
        self.segments_df = segments_df
        self.events_df = events_df
        self.mpc_df = mpc_df
        self.summary = summary or {}


if __name__ == "__main__":
    sys.exit(main())
