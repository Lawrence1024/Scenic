#!/usr/bin/env python3
"""
Compare two racing line CSV files and visualize differences.

Handles:
- laguna_main_racing_line.csv (x,y only)
- ttl_fellow_test_xodr_all.csv (x,y,z)

Since they may be in different coordinate systems, the script:
1. Reports stats (point count, path length, bounds) for each
2. Plots both in native coordinates (side-by-side)
3. Normalizes both to same scale and overlays for shape comparison
4. Optionally computes pointwise distance when lengths match (after resampling)

Run from Scenic repo root, e.g.:
  python src/scenic/simulators/dspace/create_new_ttl/compare_racing_lines.py assets/ttls/.../file_a.csv assets/ttls/.../file_b.csv --save out.png
"""

import argparse
import csv
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt


def load_racing_line_csv(path: str):
    """Load x,y (and optional z) from a racing line CSV. Returns (N, 2) or (N, 3) array."""
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
                    z = float(row[2])
                    points.append([x, y, z])
                else:
                    points.append([x, y])
            except (ValueError, IndexError):
                continue
    return np.array(points) if points else np.empty((0, 2))


def path_length(pts: np.ndarray) -> float:
    """Total path length (xy only)."""
    if len(pts) < 2:
        return 0.0
    d = np.diff(pts[:, :2], axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def normalize_for_shape(pts: np.ndarray) -> np.ndarray:
    """Center and scale so path fits in ~[-1,1] for shape comparison."""
    xy = pts[:, :2].astype(float)
    center = np.mean(xy, axis=0)
    xy_centered = xy - center
    scale = np.max(np.abs(xy_centered)) or 1.0
    return xy_centered / scale


def main():
    parser = argparse.ArgumentParser(description="Compare two racing line CSVs and visualize differences")
    parser.add_argument("file_a", type=str, help="First CSV (e.g. laguna_main_racing_line.csv)")
    parser.add_argument("file_b", type=str, help="Second CSV (e.g. ttl_fellow_test_xodr_all.csv)")
    parser.add_argument("--save", type=str, default=None, help="Save figure path")
    parser.add_argument("--no-show", action="store_true", help="Do not show plot")
    args = parser.parse_args()

    path_a = Path(args.file_a)
    path_b = Path(args.file_b)
    if not path_a.exists():
        raise SystemExit(f"File not found: {path_a}")
    if not path_b.exists():
        raise SystemExit(f"File not found: {path_b}")

    pts_a = load_racing_line_csv(str(path_a))
    pts_b = load_racing_line_csv(str(path_b))

    if pts_a.size == 0:
        raise SystemExit(f"No points loaded from {path_a}")
    if pts_b.size == 0:
        raise SystemExit(f"No points loaded from {path_b}")

    name_a = path_a.stem
    name_b = path_b.stem

    len_a = path_length(pts_a)
    len_b = path_length(pts_b)
    n_a, n_b = len(pts_a), len(pts_b)

    xa, ya = pts_a[:, 0], pts_a[:, 1]
    xb, yb = pts_b[:, 0], pts_b[:, 1]

    print("=" * 60)
    print("RACING LINE COMPARISON")
    print("=" * 60)
    print(f"\n{name_a}:")
    print(f"  Points: {n_a},  Path length: {len_a:.2f} m")
    print(f"  X range: [{np.min(xa):.2f}, {np.max(xa):.2f}],  Y range: [{np.min(ya):.2f}, {np.max(ya):.2f}]")
    print(f"\n{name_b}:")
    print(f"  Points: {n_b},  Path length: {len_b:.2f} m")
    print(f"  X range: [{np.min(xb):.2f}, {np.max(xb):.2f}],  Y range: [{np.min(yb):.2f}, {np.max(yb):.2f}]")
    print(f"\nDifference in point count: {abs(n_a - n_b)}")
    print(f"Difference in path length: {abs(len_a - len_b):.2f} m")
    print()

    # Normalized (shape-only) overlay
    norm_a = normalize_for_shape(pts_a)
    norm_b = normalize_for_shape(pts_b)

    fig, axes = plt.subplots(2, 2, figsize=(14, 12))

    # 1) Native coords: file A
    ax1 = axes[0, 0]
    ax1.plot(pts_a[:, 0], pts_a[:, 1], "-", color="C0", linewidth=1.2, label=name_a)
    ax1.scatter(pts_a[0, 0], pts_a[0, 1], c="C0", s=40, zorder=5, marker="o")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title(f"{name_a}\n(native coordinates)")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="box")

    # 2) Native coords: file B
    ax2 = axes[0, 1]
    ax2.plot(pts_b[:, 0], pts_b[:, 1], "-", color="C1", linewidth=1.2, label=name_b)
    ax2.scatter(pts_b[0, 0], pts_b[0, 1], c="C1", s=40, zorder=5, marker="o")
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title(f"{name_b}\n(native coordinates)")
    ax2.legend()
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect("equal", adjustable="box")

    # 3) Overlay in native coordinates (same frame — shows real-world lateral/position difference)
    ax3 = axes[1, 0]
    ax3.plot(pts_a[:, 0], pts_a[:, 1], "-", color="C0", linewidth=1.0, label=name_a, alpha=0.9)
    ax3.plot(pts_b[:, 0], pts_b[:, 1], "-", color="C1", linewidth=1.0, label=name_b, alpha=0.9)
    ax3.scatter(pts_a[0, 0], pts_a[0, 1], c="C0", s=50, zorder=5, marker="o")
    ax3.scatter(pts_b[0, 0], pts_b[0, 1], c="C1", s=50, zorder=5, marker="s")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title("Overlay (native coordinates)\n— same frame")
    ax3.legend()
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect("equal", adjustable="box")

    # 4) Normalized overlay (shape-only comparison)
    ax4 = axes[1, 1]
    ax4.plot(norm_a[:, 0], norm_a[:, 1], "-", color="C0", linewidth=1.2, label=name_a, alpha=0.9)
    ax4.plot(norm_b[:, 0], norm_b[:, 1], "-", color="C1", linewidth=1.2, label=name_b, alpha=0.9)
    ax4.scatter(norm_a[0, 0], norm_a[0, 1], c="C0", s=50, zorder=5, marker="o")
    ax4.scatter(norm_b[0, 0], norm_b[0, 1], c="C1", s=50, zorder=5, marker="s")
    ax4.set_xlabel("X (normalized)")
    ax4.set_ylabel("Y (normalized)")
    ax4.set_title("Shape overlay\n(centered & scaled)")
    ax4.legend()
    ax4.grid(True, alpha=0.3)
    ax4.set_aspect("equal", adjustable="box")

    plt.suptitle("Racing line comparison: " + name_a + " vs " + name_b, fontsize=12, fontweight="bold")
    plt.tight_layout()

    if args.save:
        plt.savefig(args.save, dpi=150, bbox_inches="tight")
        print(f"Saved: {args.save}")
    if not args.no_show:
        plt.show()
    else:
        plt.close()


if __name__ == "__main__":
    main()
