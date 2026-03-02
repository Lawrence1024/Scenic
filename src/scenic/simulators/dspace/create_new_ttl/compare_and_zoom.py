#!/usr/bin/env python3
"""
Compare two TTL CSVs and visualize around a specific coordinate.

Usage:
  python src/scenic/simulators/dspace/create_new_ttl/compare_and_zoom.py [--center 614.66 -302.78] [--radius 60] [--save out.png]
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

_CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = _CREATE_NEW_TTL.parent.parent.parent.parent.parent
DEFAULT_CENTERLINE = REPO_ROOT / "assets/ttls/LS_ENU_TTL_CSV/ttl_fellow_test_xodr_all.csv"
DEFAULT_OTHER = _CREATE_NEW_TTL / "temp_aligned_to_centerline.csv"


def load_csv(path: str) -> np.ndarray:
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


def main():
    parser = argparse.ArgumentParser(description="Compare two TTL CSVs with zoom around a point")
    parser.add_argument("centerline", nargs="?", default=str(DEFAULT_CENTERLINE), help="Known centerline CSV")
    parser.add_argument("other", nargs="?", default=str(DEFAULT_OTHER), help="Other CSV to compare")
    parser.add_argument("--center", nargs=2, type=float, default=[614.659946, -302.782016],
                        metavar=("X", "Y"), help="Center of zoom (x y)")
    parser.add_argument("--radius", type=float, default=60.0, help="Zoom radius (m) around center")
    parser.add_argument("--save", type=str, default=None, help="Save figure path")
    parser.add_argument("--no-show", action="store_true", help="Do not show plot")
    args = parser.parse_args()

    cx, cy = args.center[0], args.center[1]
    radius = args.radius

    pts_a = load_csv(args.centerline)
    pts_b = load_csv(args.other)
    if len(pts_a) < 2 or len(pts_b) < 2:
        raise SystemExit("Need at least 2 points in each CSV")

    name_a = Path(args.centerline).stem
    name_b = Path(args.other).stem

    # Window around focus point
    xmin, xmax = cx - radius, cx + radius
    ymin, ymax = cy - radius, cy + radius

    # Mask points inside window (for clean zoom; also keep segments that cross the window)
    def in_window(pts, margin=0):
        x, y = pts[:, 0], pts[:, 1]
        return (x >= xmin - margin) & (x <= xmax + margin) & (y >= ymin - margin) & (y <= ymax + margin)

    # For plotting zoom: take all points, but we'll set axis limits (so lines can extend slightly)
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))

    # Left: full view with zoom box
    ax1 = axes[0]
    ax1.plot(pts_a[:, 0], pts_a[:, 1], "-", color="C0", linewidth=0.8, label=name_a, alpha=0.9)
    ax1.plot(pts_b[:, 0], pts_b[:, 1], "-", color="C1", linewidth=0.8, label=name_b, alpha=0.9)
    rect = plt.Rectangle((xmin, ymin), 2 * radius, 2 * radius, fill=False, edgecolor="black", linewidth=2, linestyle="--")
    ax1.add_patch(rect)
    ax1.scatter([cx], [cy], c="red", s=80, zorder=5, marker="*", label="Focus")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Full view (box = zoom region)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="box")

    # Right: zoomed view around (cx, cy)
    ax2 = axes[1]
    ax2.plot(pts_a[:, 0], pts_a[:, 1], "-", color="C0", linewidth=1.5, label=name_a, alpha=0.9)
    ax2.plot(pts_b[:, 0], pts_b[:, 1], "-", color="C1", linewidth=1.5, label=name_b, alpha=0.9)
    ax2.scatter([cx], [cy], c="red", s=120, zorder=5, marker="*", label=f"Focus ({cx:.2f}, {cy:.2f})")
    ax2.set_xlim(xmin, xmax)
    ax2.set_ylim(ymin, ymax)
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title(f"Zoom around ({cx:.2f}, {cy:.2f})")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect("equal", adjustable="box")

    plt.suptitle(f"Comparison: {name_a} vs {name_b}", fontsize=12, fontweight="bold")
    plt.tight_layout()

    save_path = args.save or _CREATE_NEW_TTL / "compare_zoom_centerline.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {save_path}")
    if not args.no_show:
        plt.show()
    else:
        plt.close()


if __name__ == "__main__":
    main()
