#!/usr/bin/env python3
"""
Visualize TTL centerlines as two segmented lines (main and pit) using the same
OpenDRIVE centerline segmentation code. Draw a polyline through waypoints in each
CSV, then segment each with _build_curve_straight_segments; show main and pit
in two separate subplots.

Usage (from repo root):
    python -m scenic.domains.racing.segments.visualize_ttl_segments
    python -m scenic.domains.racing.segments.visualize_ttl_segments --ttl-folder assets/ttls/LS_ENU_TTL_CSV -o ttl_segments.png

Or use: python -m scenic.domains.racing.segments.visualize_racing_segments --ttl-folder PATH
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
import matplotlib.patheffects as path_effects

from scenic.domains.racing.segments.segment_map import (
    _build_curve_straight_segments,
    _ttl_polyline_from_waypoints,
    CURVATURE_THRESHOLD,
)


def _find_repo_root() -> Path:
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent


def _load_ttl_csv(folder: Path, filename: str) -> List[Tuple[float, float, float]]:
    path = folder / filename
    if not path.exists():
        return []
    pts: List[Tuple[float, float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        first = next(r, None)
        if first and len(first) >= 2 and first[0].strip().lower() == "x":
            pass
        elif first and len(first) >= 2:
            try:
                x, y = float(first[0]), float(first[1])
                z = float(first[2]) if len(first) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                pass
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x, y = float(row[0]), float(row[1])
                z = float(row[2]) if len(row) >= 3 else 0.0
                pts.append((x, y, z))
            except (ValueError, IndexError):
                continue
    return pts


def _dist2(p, q):
    dx = float(q[0]) - float(p[0])
    dy = float(q[1]) - float(p[1])
    return (dx * dx + dy * dy) ** 0.5


def _interp_at_s(coords, s_cum, s):
    n = len(coords)
    if s <= s_cum[0]:
        return (float(coords[0][0]), float(coords[0][1]))
    if s >= s_cum[n - 1]:
        return (float(coords[n - 1][0]), float(coords[n - 1][1]))
    for i in range(n - 1):
        if s_cum[i] <= s <= s_cum[i + 1]:
            t = (s - s_cum[i]) / (s_cum[i + 1] - s_cum[i]) if s_cum[i + 1] > s_cum[i] else 0
            x = float(coords[i][0]) + t * (float(coords[i + 1][0]) - float(coords[i][0]))
            y = float(coords[i][1]) + t * (float(coords[i + 1][1]) - float(coords[i][1]))
            return (x, y)
    return (float(coords[-1][0]), float(coords[-1][1]))


def get_segment_polyline(centerline, s_start: float, s_end: float):
    ls = getattr(centerline, "lineString", None)
    if ls is None:
        return []
    coords = list(getattr(ls, "coords", []))
    if len(coords) < 2:
        return []
    n = len(coords)
    s_cum = [0.0]
    for i in range(1, n):
        s_cum.append(s_cum[-1] + _dist2(coords[i - 1], coords[i]))
    points = []
    points.append(_interp_at_s(coords, s_cum, s_start))
    for j in range(n):
        if s_start < s_cum[j] < s_end:
            points.append((float(coords[j][0]), float(coords[j][1])))
    if s_end > s_start:
        points.append(_interp_at_s(coords, s_cum, s_end))
    return points


def main():
    parser = argparse.ArgumentParser(
        description="Visualize TTL centerlines as two segmented lines (main and pit), same segmentation as OpenDRIVE"
    )
    parser.add_argument(
        "--ttl-folder",
        default=None,
        help="Folder with ttl_main_road.csv and ttl_pitlane.csv (default: assets/ttls/LS_ENU_TTL_CSV)",
    )
    parser.add_argument("--output", "-o", default=None, help="Save figure to path")
    parser.add_argument(
        "--threshold",
        type=float,
        default=CURVATURE_THRESHOLD,
        help=f"Curvature threshold 1/m (default: {CURVATURE_THRESHOLD})",
    )
    args = parser.parse_args()

    repo_root = _find_repo_root()
    ttl_folder = args.ttl_folder
    if ttl_folder is None:
        ttl_folder = repo_root / "assets" / "ttls" / "LS_ENU_TTL_CSV"
    else:
        ttl_folder = Path(ttl_folder)
    if not ttl_folder.is_absolute():
        ttl_folder = repo_root / ttl_folder
    if not ttl_folder.exists():
        print(f"TTL folder not found: {ttl_folder}")
        sys.exit(1)

    main_pts = _load_ttl_csv(ttl_folder, "ttl_main_road.csv")
    pit_pts = _load_ttl_csv(ttl_folder, "ttl_pitlane.csv")
    if not main_pts or len(main_pts) < 2:
        print("Need ttl_main_road.csv with at least 2 points.")
        sys.exit(1)

    main_poly = _ttl_polyline_from_waypoints(main_pts)
    if main_poly is None:
        print("Failed to build main TTL polyline.")
        sys.exit(1)
    main_segs = _build_curve_straight_segments(main_poly, curvature_threshold=args.threshold)
    pit_poly = None
    pit_segs = []
    if pit_pts and len(pit_pts) >= 2:
        pit_poly = _ttl_polyline_from_waypoints(pit_pts)
        if pit_poly is not None:
            pit_segs = _build_curve_straight_segments(pit_poly, curvature_threshold=args.threshold)

    COLOR_MAIN_STRAIGHT = "tab:blue"
    COLOR_MAIN_CURVE = "tab:orange"
    COLOR_PIT_STRAIGHT = "tab:green"
    COLOR_PIT_CURVE = "tab:purple"
    LINE_WIDTH = 2.0

    def segment_color(is_pit: bool, seg_type: str):
        if is_pit:
            return COLOR_PIT_STRAIGHT if seg_type == "straight" else COLOR_PIT_CURVE
        return COLOR_MAIN_STRAIGHT if seg_type == "straight" else COLOR_MAIN_CURVE

    def draw_segments(ax, centerline, segs, is_pit: bool, title: str):
        for i, (s_start, s_end, seg_type) in enumerate(segs):
            pts = get_segment_polyline(centerline, s_start, s_end)
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            color = segment_color(is_pit, seg_type)
            seg_id = i + 1
            ax.plot(xs, ys, color=color, linewidth=LINE_WIDTH, solid_capstyle="round", linestyle="-")
            mid = len(pts) // 2
            ax.text(
                pts[mid][0], pts[mid][1], str(seg_id),
                fontsize=8, ha="center", va="center", color="white", fontweight="normal",
                path_effects=[path_effects.withStroke(linewidth=2, foreground="black")],
            )
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(title)
        ax.legend(
            handles=[
                Line2D([0], [0], color=COLOR_MAIN_STRAIGHT, linewidth=LINE_WIDTH, label="Main straight"),
                Line2D([0], [0], color=COLOR_MAIN_CURVE, linewidth=LINE_WIDTH, label="Main curve"),
                Line2D([0], [0], color=COLOR_PIT_STRAIGHT, linewidth=LINE_WIDTH, label="Pit straight"),
                Line2D([0], [0], color=COLOR_PIT_CURVE, linewidth=LINE_WIDTH, label="Pit curve"),
            ],
            loc="upper left",
        )

    if pit_poly is not None and pit_segs:
        fig, (ax_main, ax_pit) = plt.subplots(1, 2, figsize=(16, 8))
        draw_segments(ax_main, main_poly, main_segs, False, f"Main TTL segments (n={len(main_segs)})")
        draw_segments(ax_pit, pit_poly, pit_segs, True, f"Pit TTL segments (n={len(pit_segs)})")
        print(f"Main TTL: {len(main_segs)} segments. Pit TTL: {len(pit_segs)} segments.")
    else:
        fig, ax_main = plt.subplots(1, 1, figsize=(12, 10))
        draw_segments(ax_main, main_poly, main_segs, False, f"Main TTL segments (n={len(main_segs)})")
        print(f"Main TTL: {len(main_segs)} segments. (No pit CSV or failed to build pit polyline.)")

    plt.tight_layout()
    if args.output:
        out_path = Path(args.output)
        if not out_path.is_absolute():
            out_path = repo_root / out_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_path, dpi=150)
        print(f"Saved: {out_path}")
        plt.close(fig)
    else:
        plt.show()


if __name__ == "__main__":
    main()
