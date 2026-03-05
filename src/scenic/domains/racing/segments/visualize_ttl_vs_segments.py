#!/usr/bin/env python3
"""
Visualize track boundaries, all lane middle lines (centerlines), and TTL waypoints.

- Track boundaries: outer left/right edge of the full road (leftmost lane's left edge,
  rightmost lane's right edge). Not just lane 0 — the full envelope for each road.
- Lanes: every lane's centerline. Lane order (Scenic/driving): index 0 = rightmost lane,
  index increases toward left (lane 1 is left of lane 0). See RoadSection "lane 0 rightmost".
- Main road TTL and Pit lane TTL: waypoints from CSV.
- Optional: four GPS-converted XODR TTLs (ttl_left_xodr, ttl_right_xodr, ttl_optimal_xodr, ttl_pit_xodr)
  to compare with boundaries and existing TTLs.

Usage (from repo root):
    python -m scenic.domains.racing.segments.visualize_ttl_vs_segments [--map PATH] [--ttl-folder PATH]
    python -m scenic.domains.racing.segments.visualize_ttl_vs_segments --save   # save to ttl_vs_segments_latest.png
    python -m scenic.domains.racing.segments.visualize_ttl_vs_segments -o my.png
"""

import argparse
import csv
import sys
from pathlib import Path

import matplotlib.pyplot as plt
from matplotlib import colormaps

from scenic.domains.racing.tracks import createRacingTrack


def _find_repo_root() -> Path:
    """Find repo root (directory containing 'src' and 'assets')."""
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent


def get_centerline_points(centerline):
    """Return list of (x, y) for the full centerline."""
    ls = getattr(centerline, "lineString", None)
    if ls is None:
        return []
    coords = list(getattr(ls, "coords", []))
    if len(coords) < 2:
        return []
    return [(float(c[0]), float(c[1])) for c in coords]


def get_all_lane_centerlines(roads):
    """Return list of polylines: one per lane across all roads (each lane's middle line)."""
    all_lane_pts = []
    for road in roads:
        lanes = getattr(road, "lanes", None) or []
        for lane in lanes:
            cl = getattr(lane, "centerline", None)
            if cl is None:
                continue
            pts = get_centerline_points(cl)
            if len(pts) >= 2:
                all_lane_pts.append(pts)
    return all_lane_pts


def get_road_boundaries(roads):
    """Return (left_edges, right_edges): list of polylines from each road's outer left/right edge.
    Uses road.leftEdge and road.rightEdge: leftmost lane's left boundary and rightmost lane's
    right boundary (full track envelope, not just lane 0)."""
    left_edges = []
    right_edges = []
    for road in roads:
        left_edge = getattr(road, "leftEdge", None)
        right_edge = getattr(road, "rightEdge", None)
        if left_edge is not None:
            pts = get_centerline_points(left_edge)
            if len(pts) >= 2:
                left_edges.append(pts)
        if right_edge is not None:
            pts = get_centerline_points(right_edge)
            if len(pts) >= 2:
                right_edges.append(pts)
    return left_edges, right_edges


def load_ttl_csv(ttl_folder: str, filename: str, dx: float = 0.0, dy: float = 0.0):
    """Load TTL CSV as list of (x, y) with optional offset. Returns [] on failure."""
    path = Path(ttl_folder) / filename
    if not path.exists():
        print(f"[TTL] File not found: {path}")
        return []
    pts = []
    with open(path, newline="", encoding="utf-8") as f:
        r = csv.reader(f)
        try:
            first = next(r)
            if first and len(first) >= 2 and ("x" in first[0].lower() or "X" in first[0]):
                pass  # skip header
            else:
                try:
                    x, y = float(first[0]) + dx, float(first[1]) + dy
                    pts.append((x, y))
                except (ValueError, IndexError):
                    pass
        except StopIteration:
            return []
        for row in r:
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0]) + dx
                y = float(row[1]) + dy
                pts.append((x, y))
            except (ValueError, IndexError):
                continue
    return pts


def main():
    parser = argparse.ArgumentParser(
        description="Visualize segment lines (OpenDRIVE centerlines) vs TTL waypoints (4 lines)"
    )
    parser.add_argument(
        "--map",
        default=None,
        help="Path to OpenDRIVE .xodr (default: assets/maps/dSPACE/LagunaSeca.xodr)",
    )
    parser.add_argument(
        "--ttl-folder",
        default=None,
        help="Folder containing ttl_main_road.csv and ttl_pitlane.csv (default: assets/ttls/LS_ENU_TTL_CSV)",
    )
    parser.add_argument(
        "--output",
        "-o",
        default=None,
        help="Save figure to this path (e.g. my_plot.png)",
    )
    parser.add_argument(
        "--save",
        action="store_true",
        help="Save to ttl_vs_segments_latest.png (overwrites with most recent run)",
    )
    parser.add_argument(
        "--no-pit",
        action="store_true",
        help="Do not load or plot pit segment line and pit TTL",
    )
    parser.add_argument(
        "--no-xodr-ttls",
        action="store_true",
        help="Do not load or plot the 4 GPS-converted XODR TTLs (left, right, optimal, pit)",
    )
    args = parser.parse_args()

    repo_root = _find_repo_root()
    map_path = args.map
    if map_path is None:
        map_path = repo_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    else:
        map_path = Path(map_path)
    if not map_path.is_absolute():
        map_path = repo_root / map_path
    if not map_path.exists():
        print(f"Map not found: {map_path}")
        sys.exit(1)

    ttl_folder = args.ttl_folder
    if ttl_folder is None:
        ttl_folder = repo_root / "assets" / "ttls" / "LS_ENU_TTL_CSV"
    else:
        ttl_folder = Path(ttl_folder)
    if not ttl_folder.is_absolute():
        ttl_folder = repo_root / ttl_folder
    ttl_folder_str = str(ttl_folder)

    print(f"Loading track from {map_path} ...")
    track = createRacingTrack(
        str(map_path),
        direction="counterclockwise",
        pitLaneRoadName="pit",
    )
    main_roads = getattr(track, "_mainRacingRoads", None) or []
    pit_roads = (getattr(track, "_pitRoads", None) or []) if not args.no_pit else []

    # All lane middle lines (one polyline per lane, for each road)
    main_lane_pts = get_all_lane_centerlines(main_roads)
    pit_lane_pts = get_all_lane_centerlines(pit_roads) if not args.no_pit else []

    # Track boundaries: outer left/right edge of first/last lane per road
    main_left_edges, main_right_edges = get_road_boundaries(main_roads)
    pit_left_edges, pit_right_edges = get_road_boundaries(pit_roads) if not args.no_pit else ([], [])

    # TTL waypoints (LS_ENU_TTL_CSV is XODR coordinates → offset 0,0)
    dx, dy = 0.0, 0.0
    main_ttl = load_ttl_csv(ttl_folder_str, "ttl_main_road.csv", dx, dy)
    pit_ttl = [] if args.no_pit else load_ttl_csv(ttl_folder_str, "ttl_pitlane.csv", dx, dy)

    # Four GPS-converted XODR TTLs (left, right, optimal, pit) for comparison with boundaries
    xodr_ttl_files = [
        ("ttl_left_xodr.csv", "Left (XODR)"),
        ("ttl_right_xodr.csv", "Right (XODR)"),
        ("ttl_optimal_xodr.csv", "Optimal (XODR)"),
        ("ttl_pit_xodr.csv", "Pit (XODR)"),
    ]
    xodr_ttls = [] if args.no_xodr_ttls else [(load_ttl_csv(ttl_folder_str, fn, dx, dy), label) for fn, label in xodr_ttl_files]

    print(f"Main road: {len(main_lane_pts)} lane centerlines, {len(main_left_edges)} left + {len(main_right_edges)} right boundaries")
    if pit_lane_pts:
        print(f"Pit lane:  {len(pit_lane_pts)} lane centerlines, {len(pit_left_edges)} left + {len(pit_right_edges)} right boundaries")
    print(f"Main road TTL: {len(main_ttl)} waypoints")
    if pit_ttl:
        print(f"Pit lane TTL:  {len(pit_ttl)} waypoints")
    for pts, label in xodr_ttls:
        if pts:
            print(f"  {label}: {len(pts)} waypoints")

    COLOR_MAIN_TTL = "crimson"
    COLOR_PIT_TTL = "darkorange"
    # Colors for the 4 GPS-converted XODR TTLs
    COLORS_XODR_TTL = ("dodgerblue", "forestgreen", "purple", "darkcyan")
    COLOR_BOUNDARY_MAIN = "0.15"   # dark gray
    COLOR_BOUNDARY_PIT = "0.35"    # medium gray
    LW_BOUNDARY = 1.5
    LW_LANE = 1.2
    LW_TTL = 1.5

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    # Track boundaries first (drawn behind lane centerlines)
    for idx, pts_list in enumerate(main_left_edges):
        if len(pts_list) < 2:
            continue
        xs, ys = [p[0] for p in pts_list], [p[1] for p in pts_list]
        ax.plot(xs, ys, color=COLOR_BOUNDARY_MAIN, linewidth=LW_BOUNDARY, label="Main track boundary (left)" if idx == 0 else None)
    for idx, pts_list in enumerate(main_right_edges):
        if len(pts_list) < 2:
            continue
        xs, ys = [p[0] for p in pts_list], [p[1] for p in pts_list]
        ax.plot(xs, ys, color=COLOR_BOUNDARY_MAIN, linewidth=LW_BOUNDARY, label="Main track boundary (right)" if idx == 0 else None)
    for idx, pts_list in enumerate(pit_left_edges):
        if len(pts_list) < 2:
            continue
        xs, ys = [p[0] for p in pts_list], [p[1] for p in pts_list]
        ax.plot(xs, ys, color=COLOR_BOUNDARY_PIT, linewidth=LW_BOUNDARY, label="Pit track boundary (left)" if idx == 0 else None)
    for idx, pts_list in enumerate(pit_right_edges):
        if len(pts_list) < 2:
            continue
        xs, ys = [p[0] for p in pts_list], [p[1] for p in pts_list]
        ax.plot(xs, ys, color=COLOR_BOUNDARY_PIT, linewidth=LW_BOUNDARY, label="Pit track boundary (right)" if idx == 0 else None)

    # Main road lanes: shades of blue (one line per lane)
    n_main = len(main_lane_pts)
    cmap_main = colormaps.get_cmap("Blues")
    for idx, pts_list in enumerate(main_lane_pts):
        if len(pts_list) < 2:
            continue
        t = (idx + 1) / (n_main + 1)  # 0 < t < 1
        color = cmap_main(0.3 + 0.6 * t)
        label = "Main road lanes" if idx == 0 else None
        xs = [p[0] for p in pts_list]
        ys = [p[1] for p in pts_list]
        ax.plot(xs, ys, color=color, linewidth=LW_LANE, label=label)
    # Pit lanes: shades of green
    n_pit = len(pit_lane_pts)
    cmap_pit = colormaps.get_cmap("Greens")
    for idx, pts_list in enumerate(pit_lane_pts):
        if len(pts_list) < 2:
            continue
        t = (idx + 1) / (n_pit + 1) if n_pit else 0.5
        color = cmap_pit(0.3 + 0.6 * t)
        label = "Pit road lanes" if idx == 0 else None
        xs = [p[0] for p in pts_list]
        ys = [p[1] for p in pts_list]
        ax.plot(xs, ys, color=color, linewidth=LW_LANE, label=label)

    if len(main_ttl) >= 2:
        mx = [p[0] for p in main_ttl]
        my = [p[1] for p in main_ttl]
        ax.plot(mx, my, color=COLOR_MAIN_TTL, linewidth=LW_TTL, linestyle="--", label="Main road TTL")
    if len(pit_ttl) >= 2:
        px = [p[0] for p in pit_ttl]
        py = [p[1] for p in pit_ttl]
        ax.plot(px, py, color=COLOR_PIT_TTL, linewidth=LW_TTL, linestyle="--", label="Pit lane TTL")

    # Four GPS-converted XODR TTLs (lighter weight so boundaries/other TTLs remain visible)
    for idx, (pts, label) in enumerate(xodr_ttls):
        if len(pts) >= 2:
            color = COLORS_XODR_TTL[idx % len(COLORS_XODR_TTL)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, color=color, linewidth=LW_TTL * 0.9, linestyle=":", alpha=0.9, label=label)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Track boundaries, lane centerlines, and TTL waypoints (incl. GPS→XODR TTLs)")
    ax.text(0.02, 0.02, "Lane order: index 0 = rightmost, index increases leftward.", transform=ax.transAxes, fontsize=8, verticalalignment="bottom", bbox=dict(boxstyle="round", facecolor="wheat", alpha=0.8))
    handles, labels = ax.get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    ax.legend(by_label.values(), by_label.keys(), loc="upper left")
    plt.tight_layout()

    if args.save or args.output is not None:
        out_path = Path(args.output) if args.output is not None else Path("ttl_vs_segments_latest.png")
        if not out_path.is_absolute():
            out_path = repo_root / out_path
        out_path.parent.mkdir(parents=True, exist_ok=True)
        plt.savefig(out_path, dpi=150)
        print(f"Saved: {out_path}")
    plt.show()


if __name__ == "__main__":
    main()
