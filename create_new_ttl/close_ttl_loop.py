#!/usr/bin/env python3
"""
Close the TTL racing line loop by linearly interpolating points between
the last and first waypoints so the track forms a continuous loop.

New segment lengths on the closing arc match the typical spacing of the
existing track (last segment may not divide perfectly). Run from Scenic repo root:
  python create_new_ttl/close_ttl_loop.py
  python create_new_ttl/close_ttl_loop.py --csv path/to/ttl.csv --output path/to/ttl_closed.csv
"""

import argparse
import csv
from pathlib import Path

import numpy as np

DEFAULT_CSV = Path(__file__).resolve().parent.parent / "assets/ttls/LS_ENU_TTL_CSV/ttl_racing_line_xodr.csv"


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


def segment_lengths(pts: np.ndarray) -> np.ndarray:
    """Length of each segment between consecutive points. Shape (N-1,)."""
    d = np.diff(pts, axis=0)
    return np.sqrt(np.sum(d * d, axis=1))


def main():
    ap = argparse.ArgumentParser(description="Close TTL loop with linear interpolation.")
    ap.add_argument("--csv", type=Path, default=DEFAULT_CSV, help="Input TTL CSV")
    ap.add_argument("--output", "-o", type=Path, default=None, help="Output CSV (default: input stem + _closed.csv)")
    ap.add_argument("--in-place", action="store_true", help="Overwrite input file")
    ap.add_argument("--exact-close", action="store_true", help="Append first point so file ends exactly at start (gap 0); last segment may be shorter")
    args = ap.parse_args()

    repo = Path(__file__).resolve().parent.parent
    path = args.csv if args.csv.is_absolute() else repo / args.csv
    if not path.exists():
        print(f"File not found: {path}")
        return 1

    pts = load_csv(path)
    n = len(pts)
    if n < 2:
        print("Need at least 2 points")
        return 1

    first = pts[0]
    last = pts[-1]
    gap = np.sqrt(np.sum((first - last) ** 2))
    if gap < 1e-6:
        print("Track is already closed (gap < 1e-6). No change.")
        if args.in_place:
            return 0
        out_path = path.parent / (path.stem + "_closed" + path.suffix)
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z"])
            for row in pts:
                w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}"])
        print(f"Wrote copy to {out_path}")
        return 0

    seg_len = segment_lengths(pts)
    typical = float(np.mean(seg_len))
    # Number of new points so that closing segments have length ~ typical
    # gap = (K+1) * spacing  =>  K+1 = gap/typical  =>  K = gap/typical - 1
    k = max(1, int(round(gap / typical) - 1))
    closing_spacing = gap / (k + 1)

    # Linear interpolation from last to first (parameter t from 0 to 1)
    # t=0 -> last, t=1 -> first. Add points at t = 1/(K+1), 2/(K+1), ..., K/(K+1)
    new_pts = []
    for i in range(1, k + 1):
        t = i / (k + 1)
        pt = (1 - t) * last + t * first
        new_pts.append(pt)
    new_pts = np.array(new_pts)

    closed = np.vstack([pts, new_pts])
    if args.exact_close:
        closed = np.vstack([closed, first.reshape(1, -1)])
    n_closed = len(closed)
    gap_after = np.sqrt(np.sum((closed[-1] - closed[0]) ** 2))

    if args.in_place:
        out_path = path
    else:
        out_path = args.output
        if out_path is None:
            out_path = path.parent / (path.stem + "_closed" + path.suffix)
        if not out_path.is_absolute():
            out_path = repo / out_path

    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        for row in closed:
            w.writerow([f"{row[0]:.6f}", f"{row[1]:.6f}", f"{row[2]:.6f}"])

    print(f"Input:  {n} points, gap (last->first) = {gap:.4f}")
    print(f"Typical segment length: {typical:.4f}")
    print(f"Added {k} points on closing arc (segment length ~ {closing_spacing:.4f})")
    print(f"Output: {n_closed} points, remaining gap = {gap_after:.6f}")
    print(f"Wrote: {out_path}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
