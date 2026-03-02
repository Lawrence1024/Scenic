#!/usr/bin/env python3
"""
Analyze TTL quality: distance to inner/outer boundaries and curvature.

Use this to check whether the TTL is too close to a boundary (e.g. "on the
boundary" in section 1) or has poor geometry in a specific segment (e.g. segment 43).
Run from Scenic repo root:
  python create_new_ttl/analyze_ttl_quality.py
  python create_new_ttl/analyze_ttl_quality.py --csv out.csv
  python create_new_ttl/analyze_ttl_quality.py --range 800 950   # waypoint index range (e.g. segment 43)
"""

import argparse
import csv
import math
import sys
from pathlib import Path

REPO_ROOT = Path(__file__).resolve().parent.parent
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import numpy as np

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
    """Extract left/right boundary segments from XODR via Scenic."""
    if not xodr_path.exists():
        return None
    try:
        from tools.compare_ttls import extract_track_boundaries as _extract
        return _extract(str(xodr_path), main_track_road_names=main_track_road_names)
    except Exception as e:
        print(f"[WARN] Could not extract boundaries: {e}")
        return None


def segments_to_points(segments) -> np.ndarray:
    """Flatten list of segments (each list of (x,y)) into (N, 2) array."""
    out = []
    for seg in segments or []:
        if not seg:
            continue
        for pt in seg:
            if len(pt) >= 2:
                out.append([float(pt[0]), float(pt[1])])
    return np.array(out) if out else np.empty((0, 2))


def min_distance_to_set(points: np.ndarray, targets: np.ndarray) -> np.ndarray:
    """For each point in points (Nx2), return min Euclidean distance to any target (Mx2)."""
    if points.size == 0 or targets.size == 0:
        return np.array([])
    # (N, 1, 2) - (1, M, 2) -> (N, M, 2) -> (N, M) norms
    diff = points[:, np.newaxis, :] - targets[np.newaxis, :, :]
    dist = np.linalg.norm(diff, axis=2)
    return np.min(dist, axis=1)


def curvature_at(p_prev: np.ndarray, p_curr: np.ndarray, p_next: np.ndarray) -> float:
    """Curvature (1/m) at p_curr. Returns 0 if segment lengths are too small."""
    a = p_curr - p_prev
    b = p_next - p_curr
    len_a = np.linalg.norm(a)
    len_b = np.linalg.norm(b)
    if len_a < 1e-9 or len_b < 1e-9:
        return 0.0
    angle_a = math.atan2(a[1], a[0])
    angle_b = math.atan2(b[1], b[0])
    turn = angle_b - angle_a
    while turn > math.pi:
        turn -= 2 * math.pi
    while turn < -math.pi:
        turn += 2 * math.pi
    ds = 0.5 * (len_a + len_b)
    if ds < 1e-9:
        return 0.0
    return abs(turn) / ds


def main():
    default_xodr = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    default_ttl = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_racing_line_xodr.csv"

    parser = argparse.ArgumentParser(
        description="Analyze TTL: distance to inner/outer boundaries and curvature."
    )
    parser.add_argument("--xodr", type=Path, default=default_xodr, help="XODR file for boundaries")
    parser.add_argument("--ttl", type=Path, default=default_ttl, help="TTL CSV")
    parser.add_argument("--csv", type=Path, default=None, metavar="PATH", help="Write per-waypoint CSV")
    parser.add_argument("--range", type=int, nargs=2, metavar=("START", "END"),
                        default=None, help="Waypoint index range to summarize (e.g. segment 43)")
    args = parser.parse_args()

    xodr_path = args.xodr.resolve()
    ttl_path = args.ttl.resolve()

    if not xodr_path.exists():
        print(f"[ERROR] XODR not found: {xodr_path}")
        return 1
    if not ttl_path.exists():
        print(f"[ERROR] TTL not found: {ttl_path}")
        return 1

    use_main = "LagunaSeca" in xodr_path.name
    boundaries = extract_track_boundaries(
        xodr_path,
        main_track_road_names=MAIN_TRACK_ROAD_NAMES if use_main else None,
    )
    if not boundaries:
        print("[WARN] No boundaries; distance-to-boundary will be empty.")

    ttl_pts = load_csv(ttl_path)
    if ttl_pts.shape[0] < 2:
        print("[ERROR] TTL has too few points.")
        return 1
    ttl_xy = ttl_pts[:, :2].astype(float)

    # Distances to inner (left) and outer (right) boundaries
    left_pts = segments_to_points(boundaries.get("left_boundary_segments", []) if boundaries else [])
    right_pts = segments_to_points(boundaries.get("right_boundary_segments", []) if boundaries else [])
    dist_left = min_distance_to_set(ttl_xy, left_pts) if left_pts.size else np.full(ttl_xy.shape[0], np.nan)
    dist_right = min_distance_to_set(ttl_xy, right_pts) if right_pts.size else np.full(ttl_xy.shape[0], np.nan)

    # Curvature at each interior point
    curv = np.zeros(ttl_xy.shape[0])
    for i in range(1, ttl_xy.shape[0] - 1):
        curv[i] = curvature_at(ttl_xy[i - 1], ttl_xy[i], ttl_xy[i + 1])
    if ttl_xy.shape[0] > 1:
        curv[0] = curv[1]
        curv[-1] = curv[-2]

    def summary(name, arr, valid=None):
        if valid is None:
            valid = np.isfinite(arr)
        if not np.any(valid):
            print(f"  {name}: (no valid data)")
            return
        a = arr[valid]
        print(f"  {name}: min={np.min(a):.3f} m, mean={np.mean(a):.3f} m, max={np.max(a):.3f} m")

    def summary_curv(name, arr):
        valid = np.isfinite(arr) & (arr >= 0)
        if not np.any(valid):
            print(f"  {name}: (no valid data)")
            return
        a = arr[valid]
        print(f"  {name}: min={np.min(a):.6f} 1/m, mean={np.mean(a):.6f} 1/m, max={np.max(a):.6f} 1/m")

    print("=== TTL quality (full track) ===")
    if left_pts.size and right_pts.size:
        print("Distance to inner boundary (left):")
        summary("dist_inner", dist_left)
        print("Distance to outer boundary (right):")
        summary("dist_outer", dist_right)
        # Closest boundary
        min_dist = np.minimum(dist_left, dist_right)
        print("Distance to nearest boundary:")
        summary("dist_nearest", min_dist)
    print("Curvature (1/m):")
    summary_curv("curvature", curv)

    if args.range is not None:
        start, end = args.range[0], args.range[1]
        start = max(0, min(start, ttl_xy.shape[0]))
        end = max(0, min(end, ttl_xy.shape[0]))
        if start >= end:
            print(f"\n[WARN] Invalid --range {args.range}; skipped range summary.")
        else:
            print(f"\n=== TTL quality (waypoint index {start} .. {end - 1}) ===")
            if left_pts.size and right_pts.size:
                summary("dist_inner", dist_left[start:end])
                summary("dist_outer", dist_right[start:end])
                summary("dist_nearest", np.minimum(dist_left, dist_right)[start:end])
            summary_curv("curvature", curv[start:end])

    if args.csv is not None:
        out_path = args.csv.resolve()
        with open(out_path, "w", newline="") as f:
            w = csv.writer(f)
            w.writerow(["idx", "x", "y", "dist_inner_m", "dist_outer_m", "dist_nearest_m", "curvature_1pm"])
            for i in range(ttl_xy.shape[0]):
                d_left = dist_left[i] if np.isfinite(dist_left[i]) else ""
                d_right = dist_right[i] if np.isfinite(dist_right[i]) else ""
                d_nearest = min(dist_left[i], dist_right[i]) if np.isfinite(dist_left[i]) and np.isfinite(dist_right[i]) else ""
                w.writerow([
                    i, ttl_xy[i, 0], ttl_xy[i, 1],
                    d_left, d_right, d_nearest, curv[i],
                ])
        print(f"\nWrote per-waypoint CSV: {out_path}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
