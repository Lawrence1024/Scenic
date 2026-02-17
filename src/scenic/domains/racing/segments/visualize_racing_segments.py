#!/usr/bin/env python3
"""
Visualize curve/straight segments derived from the OpenDRIVE map.

Segments are deterministic for the same map: same centerline geometry and
CURVATURE_THRESHOLD yield the same segment boundaries every run. This script
loads the same track as the racing scenario, builds the segments, and shows
them in a window.

Usage (from repo root):
    python -m scenic.domains.racing.segments.visualize_racing_segments [--map PATH]

Example:
    python -m scenic.domains.racing.segments.visualize_racing_segments --map assets/maps/dSPACE/LagunaSeca.xodr
"""

import argparse
import sys
from pathlib import Path

import matplotlib
import matplotlib.pyplot as plt
import matplotlib.cm as cm
import matplotlib.colors as mcolors
from matplotlib.lines import Line2D
from matplotlib.patches import Patch

from scenic.domains.racing.tracks import createRacingTrack
from scenic.domains.racing.segments.segment_map import (
    _get_road_centerline,
    _build_curve_straight_segments,
    CURVATURE_THRESHOLD,
)


def _find_repo_root() -> Path:
    """Find repo root (directory containing 'src' and 'assets')."""
    p = Path(__file__).resolve().parent
    for _ in range(10):
        if (p / "src").is_dir() and (p / "assets").is_dir():
            return p
        p = p.parent
    return Path(__file__).resolve().parent.parent.parent.parent.parent.parent


def _dist2(p, q):
    dx = float(q[0]) - float(p[0])
    dy = float(q[1]) - float(p[1])
    return (dx * dx + dy * dy) ** 0.5


def get_segment_polyline(centerline, s_start: float, s_end: float):
    """Return list of (x, y) for centerline points with s in [s_start, s_end]."""
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
    for j in range(n):
        if s_start <= s_cum[j] <= s_end:
            points.append((float(coords[j][0]), float(coords[j][1])))
    return points


def main():
    parser = argparse.ArgumentParser(description="Visualize racing track curve/straight segments")
    parser.add_argument(
        "--map",
        default=None,
        help="Path to OpenDRIVE .xodr (default: assets/maps/dSPACE/LagunaSeca.xodr)",
    )
    parser.add_argument(
        "--threshold",
        type=float,
        default=CURVATURE_THRESHOLD,
        help=f"Curvature threshold 1/m (default: {CURVATURE_THRESHOLD})",
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

    print(f"Loading track from {map_path} ...")
    track = createRacingTrack(
        str(map_path),
        direction="counterclockwise",
        pitLaneRoadName="pit",
    )
    roads = getattr(track, "_mainRacingRoads", None)
    if not roads:
        print("No main racing roads found.")
        sys.exit(1)

    # Build segments per road (same logic as segment_map)
    all_segments = []  # (segment_id, road_idx, s_start, s_end, type, centerline)
    seg_id = 1
    for road_idx, road in enumerate(roads):
        centerline = _get_road_centerline(road)
        if centerline is None:
            continue
        segs = _build_curve_straight_segments(centerline, curvature_threshold=args.threshold)
        for s_start, s_end, seg_type in segs:
            all_segments.append((seg_id, road_idx, s_start, s_end, seg_type, centerline))
            seg_id += 1

    total = len(all_segments)
    print(f"Total segments: {total} (threshold={args.threshold})")
    print("Segments are deterministic for the same OpenDRIVE map and threshold.")

    # One color per segment (cyclic colormap)
    fig, ax = plt.subplots(1, 1, figsize=(12, 10))
    try:
        cmap = matplotlib.colormaps.get_cmap("turbo")
    except AttributeError:
        cmap = cm.get_cmap("turbo")
    except Exception:
        cmap = cm.get_cmap("rainbow")
    colors = [cmap(i / max(1, total - 1)) for i in range(total)]

    for seg_id, road_idx, s_start, s_end, seg_type, centerline in all_segments:
        pts = get_segment_polyline(centerline, s_start, s_end)
        if len(pts) < 2:
            continue
        xs = [p[0] for p in pts]
        ys = [p[1] for p in pts]
        color = colors[seg_id - 1]
        ax.plot(xs, ys, color=color, linewidth=2.5, solid_capstyle="round")

    ax.set_aspect("equal")
    ax.set_xlabel("x (m)")
    ax.set_ylabel("y (m)")
    ax.set_title(f"Racing track segments (n={total}) — curve/straight from curvature threshold={args.threshold}")
    ax.legend(
        handles=[
            Line2D([0], [0], color="tab:red", linewidth=2, label="curve"),
            Line2D([0], [0], color="tab:blue", linewidth=2, label="straight"),
            Patch(facecolor="none", edgecolor="none", label=f"{total} segments total"),
        ],
        loc="upper left",
    )
    plt.tight_layout()
    plt.show()


if __name__ == "__main__":
    main()
