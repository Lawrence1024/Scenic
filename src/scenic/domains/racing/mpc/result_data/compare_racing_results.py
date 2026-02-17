#!/usr/bin/env python3
"""
Compare racing run results from the result_data folder.

Each subfolder of this folder (e.g. ttl_fellow_test_xodr_all/, run/) is treated as one run
(from analyze_racing_log.py for a specific log). Loads summary.json and segments.csv from each
and prints a comparison: total time, waypoint hits, and per-segment time/hits/CTE across runs.

Usage (from repo root):
    python -m scenic.domains.racing.mpc.result_data.compare_racing_results [--results-dir PATH] [--output FILE]

Example:
    python -m scenic.domains.racing.mpc.result_data.compare_racing_results
    python -m scenic.domains.racing.mpc.result_data.compare_racing_results --output comparison.csv
"""

import argparse
import json
import sys
from pathlib import Path
from typing import Any, Dict, List, Optional, Tuple

try:
    import pandas as pd
    _PANDAS_AVAILABLE = True
except ImportError:
    pd = None  # type: ignore
    _PANDAS_AVAILABLE = False

# Default: this folder (result_data); run subfolders are alongside this script
DEFAULT_RESULTS_DIR = Path(__file__).resolve().parent


def load_run(results_dir: Path, run_name: str) -> Tuple[Optional[Dict], Optional[Any]]:
    """Load summary.json and segments.csv for one run. Returns (summary_dict, segments_df or None)."""
    run_dir = results_dir / run_name
    if not run_dir.is_dir():
        return (None, None)
    summary_path = run_dir / "summary.json"
    summary = None
    if summary_path.exists():
        try:
            with open(summary_path, "r", encoding="utf-8") as f:
                summary = json.load(f)
        except (json.JSONDecodeError, OSError):
            pass
    segments_df = None
    if _PANDAS_AVAILABLE and pd is not None:
        seg_path = run_dir / "segments.csv"
        if seg_path.exists():
            try:
                segments_df = pd.read_csv(seg_path)
            except Exception:
                pass
    return (summary, segments_df)


def list_runs(results_dir: Path) -> List[str]:
    """Return sorted list of subdirectory names that contain at least summary.json."""
    if not results_dir.is_dir():
        return []
    runs = []
    for p in results_dir.iterdir():
        if p.is_dir() and (p / "summary.json").exists():
            runs.append(p.name)
    return sorted(runs)


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Compare racing run results from result_data subfolders."
    )
    parser.add_argument(
        "--results-dir",
        default=None,
        metavar="PATH",
        help=f"Path to result_data folder (default: {DEFAULT_RESULTS_DIR})",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        metavar="FILE",
        help="Write segment comparison to CSV (segment_id x runs; requires pandas)",
    )
    args = parser.parse_args()

    results_dir = Path(args.results_dir) if args.results_dir else DEFAULT_RESULTS_DIR
    if not results_dir.is_absolute():
        # Resolve relative to cwd
        results_dir = Path.cwd() / results_dir
    if not results_dir.is_dir():
        print(f"Results directory not found: {results_dir}", file=sys.stderr)
        print("Run analyze_racing_log on one or more logs first.", file=sys.stderr)
        return 1

    runs = list_runs(results_dir)
    if not runs:
        print(f"No runs found in {results_dir} (expected subfolders with summary.json).", file=sys.stderr)
        return 1

    # Load summary and segments for each run
    summaries: Dict[str, Dict] = {}
    segments_dfs: Dict[str, Any] = {}
    for name in runs:
        summary, seg_df = load_run(results_dir, name)
        if summary is not None:
            summaries[name] = summary
        if seg_df is not None:
            segments_dfs[name] = seg_df

    # --- Summary comparison ---
    print("=== Run summary comparison ===")
    print(f"  {'Run':<18} {'TTL':<24} {'Edit note':<36} {'Time (s)':>10} {'WP hits':>8} {'MPC':>6} {'t_end':>8}")
    print("  " + "-" * 122)
    for name in runs:
        s = summaries.get(name) or {}
        ttl = (s.get("ttl_name") or "")[:22]
        if len((s.get("ttl_name") or "")) > 22:
            ttl = ttl + "..."
        note = (s.get("edit_note") or "").strip()
        if len(note) > 34:
            note = note[:31] + "..."
        total = s.get("total_time_s", 0)
        wp = s.get("n_waypoints", 0)
        mpc = s.get("n_mpc_samples", 0)
        t_end = s.get("t_end", 0)
        print(f"  {name:<18} {ttl:<24} {note:<36} {total:>10.2f} {wp:>8} {mpc:>6} {t_end:>8.2f}")
    print()

    # --- Per-segment comparison (requires pandas and at least one segments.csv) ---
    if _PANDAS_AVAILABLE and segments_dfs:
        seg_ids = set()
        for df in segments_dfs.values():
            if "segment_id" in df.columns:
                seg_ids.update(df["segment_id"].astype(int).tolist())
        seg_ids_sorted = sorted(seg_ids)

        # Build table: segment_id, segment_name, time_s_<r>, waypoint_hits_<r>, mean_abs_cte_m_<r>, max_abs_cte_m_<r>
        time_cols = [f"time_s_{r}" for r in runs]
        hit_cols = [f"waypoint_hits_{r}" for r in runs]
        has_cte = any(
            df is not None and "mean_abs_cte_m" in df.columns and "max_abs_cte_m" in df.columns
            for df in segments_dfs.values()
        )
        mean_cte_cols = [f"mean_abs_cte_m_{r}" for r in runs] if has_cte else []
        max_cte_cols = [f"max_abs_cte_m_{r}" for r in runs] if has_cte else []

        rows = []
        for seg_id in seg_ids_sorted:
            row = {"segment_id": seg_id, "segment_name": ""}
            for r in runs:
                df = segments_dfs.get(r)
                if df is not None:
                    match = df.loc[df["segment_id"] == seg_id]
                    if not match.empty:
                        row[f"time_s_{r}"] = match["time_s"].iloc[0]
                        row[f"waypoint_hits_{r}"] = match["waypoint_hits"].iloc[0]
                        if not row["segment_name"] and "segment_name" in df.columns:
                            row["segment_name"] = match["segment_name"].iloc[0]
                        if has_cte and "mean_abs_cte_m" in df.columns and "max_abs_cte_m" in df.columns:
                            row[f"mean_abs_cte_m_{r}"] = match["mean_abs_cte_m"].iloc[0]
                            row[f"max_abs_cte_m_{r}"] = match["max_abs_cte_m"].iloc[0]
                    else:
                        row[f"time_s_{r}"] = None
                        row[f"waypoint_hits_{r}"] = None
                        if has_cte:
                            row[f"mean_abs_cte_m_{r}"] = None
                            row[f"max_abs_cte_m_{r}"] = None
                else:
                    row[f"time_s_{r}"] = None
                    row[f"waypoint_hits_{r}"] = None
                    if has_cte:
                        row[f"mean_abs_cte_m_{r}"] = None
                        row[f"max_abs_cte_m_{r}"] = None
            rows.append(row)

        compare_df = pd.DataFrame(rows)
        compare_df = compare_df[["segment_id", "segment_name"] + time_cols + hit_cols + mean_cte_cols + max_cte_cols]

        print("=== Segment time (s) comparison ===")
        disp_cols = ["segment_id", "segment_name"] + time_cols
        if compare_df[time_cols].notna().any().any():
            print(compare_df[disp_cols].to_string(index=False))
        else:
            print("  (no segment time data)")
        if has_cte and (compare_df[mean_cte_cols].notna().any().any() or compare_df[max_cte_cols].notna().any().any()):
            print("\n=== Segment CTE (m) comparison ===")
            print(compare_df[["segment_id", "segment_name"] + mean_cte_cols + max_cte_cols].to_string(index=False))
        print()

        if args.output:
            out_path = Path(args.output)
            if not out_path.is_absolute():
                out_path = Path.cwd() / out_path
            compare_df.to_csv(out_path, index=False)
            print(f"Wrote comparison to {out_path}")
    elif args.output:
        print("Warning: pandas not installed or no segments.csv found; --output ignored.", file=sys.stderr)

    return 0


if __name__ == "__main__":
    sys.exit(main())
