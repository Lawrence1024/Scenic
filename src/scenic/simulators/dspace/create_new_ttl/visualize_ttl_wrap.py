#!/usr/bin/env python3
"""
Visualize a TTL racing line CSV and check if the end wraps back to the beginning.

Plots the full (x,y) path, marks first and last points, and draws the gap
between last and first. Run from Scenic repo root:
  python src/scenic/simulators/dspace/create_new_ttl/visualize_ttl_wrap.py
  python src/scenic/simulators/dspace/create_new_ttl/visualize_ttl_wrap.py --csv path/to/ttl.csv
"""

import argparse
import csv
import math
from pathlib import Path

import matplotlib.pyplot as plt
import numpy as np

_REPO_ROOT = Path(__file__).resolve().parent.parent.parent.parent.parent
DEFAULT_CSV = _REPO_ROOT / "assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv"


def load_csv(path: Path) -> np.ndarray:
    """Load x,y,z from CSV. Returns (N, 3) array."""
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
                z = float(row[2]) if len(row) >= 3 else 0.0
                points.append([x, y, z])
            except (ValueError, IndexError):
                continue
    return np.array(points) if points else np.empty((0, 3))


def main():
    ap = argparse.ArgumentParser(description="Visualize TTL and check wrap (end→start).")
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Path to TTL CSV")
    ap.add_argument("--save", type=Path, default=None, help="Save figure to path")
    args = ap.parse_args()

    path = args.csv
    if not path.is_absolute():
        path = _REPO_ROOT / path
    if not path.exists():
        print(f"File not found: {path}")
        return 1

    pts = load_csv(path)
    if len(pts) < 2:
        print("Not enough points in CSV")
        return 1

    first_xy = pts[0, :2]
    last_xy = pts[-1, :2]
    gap_xy = np.sqrt(np.sum((last_xy - first_xy) ** 2))

    print(f"TTL: {path}")
    print(f"Points: {len(pts)}")
    print(f"First (x,y): ({first_xy[0]:.6f}, {first_xy[1]:.6f})")
    print(f"Last  (x,y): ({last_xy[0]:.6f}, {last_xy[1]:.6f})")
    print(f"Gap (last->first): {gap_xy:.6f}")
    if gap_xy < 1.0:
        print("Wrap: YES (end is close to start)")
    else:
        print("Wrap: NO (end does not nicely wrap back to beginning)")

    fig, ax = plt.subplots(1, 1, figsize=(10, 10))
    ax.plot(pts[:, 0], pts[:, 1], "b-", linewidth=0.8, alpha=0.9, label="Racing line")
    ax.plot(first_xy[0], first_xy[1], "go", markersize=12, label="First point", zorder=5)
    ax.plot(last_xy[0], last_xy[1], "ro", markersize=12, label="Last point", zorder=5)
    ax.plot(
        [last_xy[0], first_xy[0]],
        [last_xy[1], first_xy[1]],
        "r--",
        linewidth=2,
        label=f"Gap = {gap_xy:.2f}",
        zorder=4,
    )
    ax.set_xlabel("x")
    ax.set_ylabel("y")
    ax.set_title(f"TTL wrap check: {path.name}")
    ax.legend(loc="best")
    ax.set_aspect("equal")
    ax.grid(True, alpha=0.3)
    plt.tight_layout()

    out_path = args.save or (path.parent / "ttl_wrap_check.png")
    out_path.parent.mkdir(parents=True, exist_ok=True)
    fig.savefig(out_path, dpi=150, bbox_inches="tight")
    print(f"Saved: {out_path}")
    if args.save is None:
        plt.show()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
