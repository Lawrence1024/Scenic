#!/usr/bin/env python3
"""
Visualize TTL (racing line) with track inner and outer boundaries.

Draws:
  1. Inner and outer track boundaries (black) from XODR
  2. TTL CSV (racing line) in a distinct color, in between the boundaries

Use the matplotlib toolbar to zoom in/out and pan. Run from Scenic repo root:
  python src/scenic/simulators/dspace/create_new_ttl/visualize_ttl_boundaries.py
  python src/scenic/simulators/dspace/create_new_ttl/visualize_ttl_boundaries.py --ttl path/to/ttl.csv --save
"""

import argparse
import csv
import sys
from pathlib import Path

_CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = _CREATE_NEW_TTL.parent.parent.parent.parent.parent
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib.pyplot as plt
import numpy as np

# Main track road names (exclude junctions for a single track envelope)
MAIN_TRACK_ROAD_NAMES = ("The Corkscrew1", "Andretti Hairpin1_3")


def load_csv(path: Path) -> np.ndarray:
    """Load x,y (and optional z) from CSV. Returns (N, 2+) array."""
    points = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None:
            return np.array(points)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                x, y = float(row[0]), float(row[1])
                if len(row) >= 3:
                    points.append([x, y, float(row[2])])
                else:
                    points.append([x, y])
            except (ValueError, IndexError):
                continue
    return np.array(points) if points else np.empty((0, 2))


def extract_track_boundaries(xodr_path: Path, main_track_road_names=None):
    """Extract left/right (inner/outer) boundary segments from XODR via Scenic."""
    if not xodr_path.exists():
        return None
    try:
        from tools.compare_ttls import extract_track_boundaries as _extract
        return _extract(str(xodr_path), main_track_road_names=main_track_road_names)
    except Exception as e:
        print(f"[WARN] Could not extract boundaries: {e}")
        return None


def _plot_segments(ax, segments, color, linewidth=1.5, label=None, alpha=0.95):
    """Plot a list of segments (each segment = list of (x,y) or Nx2 array)."""
    first = True
    for seg in segments:
        if not seg:
            continue
        pts = np.asarray(seg)
        if pts.ndim == 2 and pts.shape[1] >= 2:
            ax.plot(
                pts[:, 0], pts[:, 1], "-",
                color=color, linewidth=linewidth, alpha=alpha,
                label=label if first else None,
            )
            first = False


def main():
    default_xodr = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"
    default_ttl = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_racing_line_xodr.csv"
    default_save = _CREATE_NEW_TTL / "ttl_boundaries_visualization.png"

    parser = argparse.ArgumentParser(
        description="Visualize TTL with track inner/outer boundaries (black). Use toolbar to zoom/pan."
    )
    parser.add_argument("--xodr", type=Path, default=default_xodr, help="XODR file for track boundaries")
    parser.add_argument("--ttl", type=Path, default=default_ttl, help="TTL CSV (racing line)")
    parser.add_argument("--save", type=Path, default=None, metavar="PATH",
                        help=f"Save figure (default: {default_save.name})")
    parser.add_argument("--no-show", action="store_true", help="Do not open interactive window (only save)")
    args = parser.parse_args()

    xodr_path = args.xodr.resolve()
    ttl_path = args.ttl.resolve()

    if not xodr_path.exists():
        print(f"[ERROR] XODR not found: {xodr_path}")
        return 1
    if not ttl_path.exists():
        print(f"[ERROR] TTL not found: {ttl_path}")
        return 1

    print(f"XODR: {xodr_path.name}")
    print("Loading track boundaries...")
    use_main_track_only = "LagunaSeca" in xodr_path.name
    boundaries = extract_track_boundaries(
        xodr_path,
        main_track_road_names=MAIN_TRACK_ROAD_NAMES if use_main_track_only else None,
    )

    if not boundaries:
        print("[WARN] No boundaries extracted; plot will show TTL only.")

    print(f"TTL: {ttl_path.name}")
    ttl_pts = load_csv(ttl_path)
    if len(ttl_pts) < 2:
        print("[ERROR] TTL CSV has too few points.")
        return 1

    fig, ax = plt.subplots(figsize=(12, 10))

    # 1. Inner and outer boundaries (black)
    if boundaries:
        left_segs = boundaries.get("left_boundary_segments", []) or []
        right_segs = boundaries.get("right_boundary_segments", []) or []
        # Plot all left segments as one boundary (inner/left edge)
        _plot_segments(
            ax, left_segs, color="black", linewidth=2.0,
            label="Inner boundary", alpha=0.95,
        )
        # Plot all right segments as other boundary (outer/right edge)
        _plot_segments(
            ax, right_segs, color="black", linewidth=2.0,
            label="Outer boundary", alpha=0.95,
        )
    else:
        ax.set_title("(No XODR boundaries loaded)")

    # 2. TTL (racing line) – in between the boundaries
    ax.plot(
        ttl_pts[:, 0], ttl_pts[:, 1],
        "-", color="C0", linewidth=1.8, label="TTL (racing line)", alpha=0.95,
    )
    ax.scatter(
        ttl_pts[0, 0], ttl_pts[0, 1],
        c="C0", s=80, zorder=5, marker="o", edgecolors="white", linewidths=1,
    )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    ax.set_title("TTL vs track boundaries (use toolbar to zoom/pan)")
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    plt.tight_layout()

    save_path = args.save if args.save is not None else default_save
    save_path = save_path.resolve()
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {save_path}")

    if not args.no_show:
        print("Opening interactive window. Use toolbar: zoom, pan, save, home.")
        plt.show()
    else:
        plt.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
