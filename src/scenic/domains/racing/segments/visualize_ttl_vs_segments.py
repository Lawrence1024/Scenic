#!/usr/bin/env python3
"""
Visualize track boundaries and the four GPS-converted XODR TTLs (left, right, optimal, pit).

- Track boundaries: outer left/right edge of the full road (leftmost lane's left edge,
  rightmost lane's right edge). Full envelope for each road.
- Four TTLs: ttl_left_xodr.csv, ttl_right_xodr.csv, ttl_optimal_xodr.csv, ttl_pit_xodr.csv.
  Lane centerlines are not shown.

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
        description="Visualize track boundaries and the 4 XODR TTLs (left, right, optimal, pit)"
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
        help="Do not load or plot the 4 XODR TTLs (left, right, optimal, pit)",
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

    # Track boundaries: outer left/right edge of first/last lane per road
    main_left_edges, main_right_edges = get_road_boundaries(main_roads)
    pit_left_edges, pit_right_edges = get_road_boundaries(pit_roads) if not args.no_pit else ([], [])

    dx, dy = 0.0, 0.0
    # Four GPS-converted XODR TTLs (left, right, optimal, pit)
    xodr_ttl_files = [
        ("ttl_left_xodr.csv", "Left (XODR)"),
        ("ttl_right_xodr.csv", "Right (XODR)"),
        ("ttl_optimal_xodr.csv", "Optimal (XODR)"),
        ("ttl_pit_xodr.csv", "Pit (XODR)"),
    ]
    xodr_ttls = [] if args.no_xodr_ttls else [(load_ttl_csv(ttl_folder_str, fn, dx, dy), label) for fn, label in xodr_ttl_files]

    print(f"Main road: {len(main_left_edges)} left + {len(main_right_edges)} right boundaries")
    if not args.no_pit:
        print(f"Pit lane:  {len(pit_left_edges)} left + {len(pit_right_edges)} right boundaries")
    for pts, label in xodr_ttls:
        if pts:
            print(f"  {label}: {len(pts)} waypoints")

    COLORS_XODR_TTL = ("dodgerblue", "forestgreen", "purple", "darkcyan")
    COLOR_BOUNDARY_MAIN = "0.15"   # dark gray
    COLOR_BOUNDARY_PIT = "0.35"    # medium gray
    LW_BOUNDARY = 1.5
    LW_TTL = 1.5

    fig, ax = plt.subplots(1, 1, figsize=(12, 10))

    # Track boundaries
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

    # Four XODR TTLs (left, right, optimal, pit)
    for idx, (pts, label) in enumerate(xodr_ttls):
        if len(pts) >= 2:
            color = COLORS_XODR_TTL[idx % len(COLORS_XODR_TTL)]
            xs = [p[0] for p in pts]
            ys = [p[1] for p in pts]
            ax.plot(xs, ys, color=color, linewidth=LW_TTL * 0.9, linestyle=":", alpha=0.9, label=label)

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title("Track boundaries and 4 XODR TTLs (left, right, optimal, pit)")
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
