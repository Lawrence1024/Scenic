#!/usr/bin/env python3
"""
Visualize segment types (main straight, main curve, pit straight, pit curve) from the
OpenDRIVE map or from TTL centerline CSVs.

OpenDRIVE: one plot with all roads. TTL: draw polylines through each CSV, segment each
with the same OpenDRIVE code, then overlay on one plot — main road full (includes
Andretti, Corkscrew); pit lane only pit-only parts (overlap with main removed).

Usage (from repo root):
    python -m scenic.domains.racing.segments.visualize_racing_segments [--map PATH]
    python -m scenic.domains.racing.segments.visualize_racing_segments --ttl-folder PATH [-o FILE]
"""

import argparse
import csv
import sys
from pathlib import Path
from typing import List, Tuple

import matplotlib
import matplotlib.pyplot as plt
from matplotlib.lines import Line2D
from matplotlib.patches import Patch
import matplotlib.patheffects as path_effects

from scenic.core.vectors import Vector
from scenic.domains.racing.tracks import createRacingTrack
from scenic.domains.racing.segments.segment_map import (
    _get_road_centerline,
    _build_curve_straight_segments,
    _ttl_polyline_from_waypoints,
    CURVATURE_THRESHOLD,
    TTL_OVERLAP_MAIN_WINS_M,
)


def _find_repo_root() -> Path:
    """Find repo root (directory containing 'src' and 'assets')."""
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent.parent


def _load_ttl_csv(folder: Path, filename: str) -> List[Tuple[float, float, float]]:
    """Load TTL CSV (x,y,z) as list of (x,y,z). Returns [] on failure."""
    path = folder / filename
    if not path.exists():
        return []
    pts: List[Tuple[float, float, float]] = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        first = next(r, None)
        if first and len(first) >= 2 and first[0].strip().lower() == "x":
            pass  # skip header
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
    """Interpolate (x, y) at arc length s along coords. s_cum[i] = cumulative length to coords[i]."""
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
    """Return list of (x, y) for centerline from s_start to s_end, including exact endpoints so segments connect."""
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
    # Include exact start point (so this segment meets the previous one)
    points.append(_interp_at_s(coords, s_cum, s_start))
    for j in range(n):
        if s_start < s_cum[j] < s_end:
            points.append((float(coords[j][0]), float(coords[j][1])))
    # Include exact end point (so this segment meets the next one)
    if s_end > s_start:
        points.append(_interp_at_s(coords, s_cum, s_end))
    return points


# Use same overlap threshold as segment_map so visualization matches build_waypoint_segment_map_from_ttl.
OVERLAP_THRESHOLD_M = TTL_OVERLAP_MAIN_WINS_M


def _pit_only_runs(
    pts: List[Tuple[float, float]],
    main_poly,
    threshold_m: float,
) -> List[Tuple[int, int]]:
    """Return list of (i_start, i_end) index ranges of contiguous pit-only points.

    A point is overlap if distance to main_poly < threshold_m. We return runs of
    indices where distance >= threshold_m (pit-only). Empty list if all overlap.
    """
    if not pts:
        return []
    overlap = []
    for i in range(len(pts)):
        x, y = float(pts[i][0]), float(pts[i][1])
        try:
            d = main_poly.distanceTo(Vector(x, y, 0.0))
        except Exception:
            d = float("inf")
        overlap.append(d < threshold_m)
    runs = []
    i = 0
    n = len(pts)
    while i < n:
        if overlap[i]:
            i += 1
            continue
        j = i
        while j < n and not overlap[j]:
            j += 1
        runs.append((i, j))
        i = j
    return runs


def main():
    parser = argparse.ArgumentParser(description="Visualize racing track curve/straight segments")
    parser.add_argument(
        "--map",
        default=None,
        help="Path to OpenDRIVE .xodr (default: assets/maps/dSPACE/LGS_v1.xodr); ignored if --ttl-folder is set",
    )
    parser.add_argument(
        "--ttl-folder",
        default=None,
        help="Folder with ttl_main_road.csv and ttl_pitlane.csv; if set, use TTL centerlines instead of OpenDRIVE (same visualization)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=CURVATURE_THRESHOLD,
        help=f"Curvature threshold 1/m (default: {CURVATURE_THRESHOLD})",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Save figure to path (e.g. segment_visualization.png) instead of only showing",
    )
    parser.add_argument(
        "--no-pit",
        action="store_true",
        help="Do not include pit roads in segment list or visualization",
    )
    parser.add_argument(
        "--main-loop-ids",
        nargs="*",
        type=int,
        default=None,
        help="OpenDRIVE road IDs of junction links for outer loop (e.g. 24 34). If omitted, chosen by smoothness.",
    )
    parser.add_argument(
        "--pit-ids",
        nargs="*",
        type=int,
        default=None,
        help="OpenDRIVE road IDs of junction links to pit (e.g. 25 30). If omitted, auto-detected by topology.",
    )
    args = parser.parse_args()
    repo_root = _find_repo_root()

    ttl_two_subplots = False
    if args.ttl_folder:
        # ---- TTL path: polyline through each CSV waypoints, then same OpenDRIVE segment code ----
        ttl_folder = Path(args.ttl_folder)
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
        centerlines = [main_poly]
        ttl_main_poly = main_poly
        ttl_pit_poly = None
        if pit_pts and len(pit_pts) >= 2:
            pit_poly = _ttl_polyline_from_waypoints(pit_pts)
            if pit_poly is not None:
                centerlines.append(pit_poly)
                ttl_pit_poly = pit_poly
                ttl_two_subplots = True
        n_main_roads = 1
        all_segments = []
        seg_id = 1
        for road_idx, centerline in enumerate(centerlines):
            is_pit = road_idx >= n_main_roads
            segs = _build_curve_straight_segments(centerline, curvature_threshold=args.threshold)
            for s_start, s_end, seg_type in segs:
                all_segments.append((seg_id, road_idx, s_start, s_end, seg_type, centerline, False, is_pit))
                seg_id += 1
        n_pit_roads = len(centerlines) - n_main_roads
        print(f"TTL centerlines from {ttl_folder} (threshold={args.threshold})")
    else:
        ttl_main_poly = None
        ttl_pit_poly = None
        # ---- OpenDRIVE path ----
        map_path = args.map
        if map_path is None:
            map_path = repo_root / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"
        else:
            map_path = Path(map_path)
        if not map_path.is_absolute():
            map_path = repo_root / map_path
        if not map_path.exists():
            print(f"Map not found: {map_path}")
            sys.exit(1)

        print(f"Loading track from {map_path} ...")
        main_loop_ids = tuple(args.main_loop_ids) if args.main_loop_ids else None
        pit_ids = tuple(args.pit_ids) if args.pit_ids else None
        track = createRacingTrack(
            str(map_path),
            direction="counterclockwise",
            pitLaneRoadName="pit",
            main_loop_connecting_road_ids=main_loop_ids,
            pit_connecting_road_ids=pit_ids,
        )
        main_roads = getattr(track, "_mainRacingRoads", None) or []
        pit_roads = (getattr(track, "_pitRoads", None) or []) if not args.no_pit else []
        roads = main_roads + pit_roads
        if not roads:
            print("No main racing roads found.")
            sys.exit(1)

        conn_set = set(getattr(track.network, "connectingRoads", ()))
        n_main_roads = len(main_roads)
        junction_main_road_ids = [getattr(r, "id", None) for r in main_roads if r in conn_set]
        junction_pit_road_ids = [getattr(r, "id", None) for r in pit_roads if r in conn_set]
        print(f"Junction assignment: main_loop={'explicit IDs ' + str(main_loop_ids) if main_loop_ids else 'smoothness'}, pit={'explicit IDs ' + str(pit_ids) if pit_ids else 'topology'}")
        if junction_main_road_ids:
            print(f"  Main-loop junction links (road IDs): {junction_main_road_ids}")
        if junction_pit_road_ids:
            print(f"  Pit junction links (road IDs): {junction_pit_road_ids}")

        # Build segments per road (main first, then pit) — final segmenting used by behaviors/MPC
        all_segments = []  # (segment_id, road_idx, s_start, s_end, seg_type, centerline, is_junction_link, is_pit)
        seg_id = 1
        for road_idx, road in enumerate(roads):
            centerline = _get_road_centerline(road)
            if centerline is None:
                continue
            is_pit = road_idx >= n_main_roads
            is_junction_link = road in conn_set

            if is_junction_link:
                L = centerline.length
                # Classify junction as curve or straight from geometry (no hardcoded segment ids)
                sub_segs = _build_curve_straight_segments(centerline, curvature_threshold=args.threshold)
                junc_type = "curve" if any(ss[2] == "curve" for ss in sub_segs) else "straight"
                segs = [(0.0, L, junc_type)]
            else:
                segs = _build_curve_straight_segments(centerline, curvature_threshold=args.threshold)

            for s_start, s_end, seg_type in segs:
                all_segments.append((seg_id, road_idx, s_start, s_end, seg_type, centerline, is_junction_link, is_pit))
                seg_id += 1

        n_pit_roads = len(pit_roads)

    # ---- Common: segment summary, endpoints, and visualization ----
    total = len(all_segments)
    n_main_seg = sum(1 for s in all_segments if not s[7])  # not is_pit
    n_pit_seg = total - n_main_seg
    print(f"\nFinal segmenting: {total} segments (threshold={args.threshold})")
    print(f"  Main loop: segments 1–{n_main_seg}  ({n_main_roads} roads, junction links included)")
    if n_pit_seg:
        print(f"  Pit lane:  segments {n_main_seg + 1}–{total}  ({n_pit_roads} roads, junction links included)")
    print("Segment lengths (m): id  type      length  main/pit  junction?")
    for seg in all_segments:
        seg_id, road_idx, s_start, s_end, seg_type, _, is_junc, is_pit = seg
        length = s_end - s_start
        part = "pit " if is_pit else "main"
        j = "yes" if is_junc else ""
        print(f"  {seg_id:2d}   {seg_type:8s}  {length:7.1f}  {part:4s}    {j}")

    # Compute endpoints for all segments for logging
    seg_endpoints = []  # (seg_id, road_idx, s_start, s_end, start_pt, end_pt)
    for seg in all_segments:
        seg_id, road_idx, s_start, s_end, seg_type, centerline, is_junc, is_pit = seg
        pts = get_segment_polyline(centerline, s_start, s_end)
        if len(pts) >= 2:
            seg_endpoints.append((seg_id, road_idx, s_start, s_end, pts[0], pts[-1]))
        elif len(pts) == 1:
            seg_endpoints.append((seg_id, road_idx, s_start, s_end, pts[0], pts[0]))

    # Log endpoints of junction segments
    print("\nJunction segment endpoints:")
    for seg_id, road_idx, s_start, s_end, start_pt, end_pt in seg_endpoints:
        seg = next((s for s in all_segments if s[0] == seg_id), None)
        if seg is not None and seg[6]:  # is_junction_link
            print(f"  Segment {seg_id}: start=({start_pt[0]:.2f}, {start_pt[1]:.2f}), end=({end_pt[0]:.2f}, {end_pt[1]:.2f})")

    # Log endpoints of sections that connect with junctions (endpoint within 2m of a junction segment endpoint)
    def _dist_pt(a, b):
        return ((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2) ** 0.5

    junction_pts = []
    for seg_id, road_idx, s_start, s_end, start_pt, end_pt in seg_endpoints:
        seg = next((s for s in all_segments if s[0] == seg_id), None)
        if seg is not None and seg[6]:
            junction_pts.append((start_pt, end_pt))

    print("\nEndpoints of sections connecting with junctions:")
    TOL_M = 2.0
    for seg_id, road_idx, s_start, s_end, start_pt, end_pt in seg_endpoints:
        seg = next((s for s in all_segments if s[0] == seg_id), None)
        if seg is None or seg[6]:
            continue  # skip junction segments themselves
        connects = False
        for j_start, j_end in junction_pts:
            if _dist_pt(start_pt, j_start) <= TOL_M or _dist_pt(start_pt, j_end) <= TOL_M or _dist_pt(end_pt, j_start) <= TOL_M or _dist_pt(end_pt, j_end) <= TOL_M:
                connects = True
                break
        if connects:
            print(f"  Segment {seg_id} (connects to junction): start=({start_pt[0]:.2f}, {start_pt[1]:.2f}), end=({end_pt[0]:.2f}, {end_pt[1]:.2f})")

    # Four segment types: main straight, main curve, pit straight, pit curve (one color each, no bold).
    COLOR_MAIN_STRAIGHT = "tab:blue"
    COLOR_MAIN_CURVE = "tab:orange"
    COLOR_PIT_STRAIGHT = "tab:green"
    COLOR_PIT_CURVE = "tab:purple"
    LINE_WIDTH = 2.0  # uniform, no bold

    def segment_color(is_pit: bool, seg_type: str):
        if is_pit:
            return COLOR_PIT_STRAIGHT if seg_type == "straight" else COLOR_PIT_CURVE
        return COLOR_MAIN_STRAIGHT if seg_type == "straight" else COLOR_MAIN_CURVE

    default_legend_handles = [
        Line2D([0], [0], color=COLOR_MAIN_STRAIGHT, linewidth=LINE_WIDTH, label="Main straight"),
        Line2D([0], [0], color=COLOR_MAIN_CURVE, linewidth=LINE_WIDTH, label="Main curve"),
        Line2D([0], [0], color=COLOR_PIT_STRAIGHT, linewidth=LINE_WIDTH, label="Pit straight"),
        Line2D([0], [0], color=COLOR_PIT_CURVE, linewidth=LINE_WIDTH, label="Pit curve"),
    ]

    def draw_segments_on_axis(axis, segments_to_draw, subplot_title: str, extra_legend_handles=None):
        for seg in segments_to_draw:
            seg_id, road_idx, s_start, s_end, seg_type, centerline, is_junction_link, is_pit = seg
            pts = get_segment_polyline(centerline, s_start, s_end)
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            color = segment_color(is_pit, seg_type)
            axis.plot(xs, ys, color=color, linewidth=LINE_WIDTH, solid_capstyle="round", linestyle="-")
            frac = 0.4 if is_junction_link and (seg_id % 2) == 1 else (0.6 if is_junction_link else 0.5)
            idx = max(0, min(int(frac * (len(pts) - 1)), len(pts) - 1))
            x_mid, y_mid = pts[idx][0], pts[idx][1]
            axis.text(
                x_mid, y_mid, str(seg_id),
                fontsize=8, ha="center", va="center", color="white", fontweight="normal",
                path_effects=[path_effects.withStroke(linewidth=2, foreground="black")],
            )
        axis.set_aspect("equal")
        axis.set_xlabel("x (m)")
        axis.set_ylabel("y (m)")
        axis.set_title(subplot_title)
        handles = list(default_legend_handles)
        if extra_legend_handles:
            handles.extend(extra_legend_handles)
        axis.legend(handles=handles, loc="upper left")

    if ttl_two_subplots:
        # TTL: one overlaid plot. Main road = full (includes Andretti, Corkscrew).
        # Pit lane = only pit-only parts (overlap with main removed per segment).
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        main_segs = [s for s in all_segments if not s[7]]
        pit_segs = [s for s in all_segments if s[7]]
        # Draw main road in full.
        for seg in main_segs:
            seg_id, road_idx, s_start, s_end, seg_type, centerline, is_junction_link, is_pit = seg
            pts = get_segment_polyline(centerline, s_start, s_end)
            if len(pts) < 2:
                continue
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            color = segment_color(False, seg_type)
            ax.plot(xs, ys, color=color, linewidth=LINE_WIDTH, solid_capstyle="round", linestyle="-")
            frac = 0.5
            idx = max(0, min(int(frac * (len(pts) - 1)), len(pts) - 1))
            x_mid, y_mid = pts[idx][0], pts[idx][1]
            ax.text(
                x_mid, y_mid, str(seg_id),
                fontsize=8, ha="center", va="center", color="white", fontweight="normal",
                path_effects=[path_effects.withStroke(linewidth=2, foreground="black")],
            )
        # Draw pit lane: only pit-only portions (remove overlap with main).
        for seg in pit_segs:
            seg_id, road_idx, s_start, s_end, seg_type, centerline, is_junction_link, is_pit = seg
            pts = get_segment_polyline(centerline, s_start, s_end)
            if len(pts) < 2:
                continue
            runs = _pit_only_runs(pts, ttl_main_poly, OVERLAP_THRESHOLD_M)
            if not runs:
                continue  # whole segment overlaps main — skip (case 2)
            color = segment_color(True, seg_type)
            for run_idx, (i, j) in enumerate(runs):
                run_pts = pts[i:j]
                if len(run_pts) < 2:
                    continue
                xs = [p[0] for p in run_pts]
                ys = [p[1] for p in run_pts]
                ax.plot(xs, ys, color=color, linewidth=LINE_WIDTH, solid_capstyle="round", linestyle="-")
                mid = len(run_pts) // 2
                x_mid, y_mid = run_pts[mid][0], run_pts[mid][1]
                ax.text(
                    x_mid, y_mid, str(seg_id),
                    fontsize=8, ha="center", va="center", color="white", fontweight="normal",
                    path_effects=[path_effects.withStroke(linewidth=2, foreground="black")],
                )
        ax.set_aspect("equal")
        ax.set_xlabel("x (m)")
        ax.set_ylabel("y (m)")
        ax.set_title(
            "TTL segments overlaid: main road full (incl. Andretti, Corkscrew); pit lane pit-only (overlap removed)"
        )
        ax.legend(handles=default_legend_handles, loc="upper left")
    else:
        fig, ax = plt.subplots(1, 1, figsize=(12, 10))
        n_junc = sum(1 for s in all_segments if s[6])
        junc_ids = sorted(s[0] for s in all_segments if s[6])
        title = f"Segment types (n={total}) — main 1–{n_main_seg}, pit {n_main_seg + 1}–{total}. Four colors: main/pit × straight/curve."
        extra = [Patch(facecolor="none", edgecolor="none", label=f"Junction segment IDs: {junc_ids}")] if n_junc else None
        draw_segments_on_axis(ax, all_segments, title, extra_legend_handles=extra)
    plt.tight_layout()
    if args.output:
        out_path = Path(args.output)
        if not out_path.is_absolute():
            out_path = repo_root / out_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_path, dpi=150)
        print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
