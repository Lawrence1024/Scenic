#!/usr/bin/env python3
"""
Verify that a TTL CSV (known correct centerline) matches the centerline extracted
from the Laguna Seca XODR file.

If ttl_fellow_test_xodr_all.csv is the ground-truth centerline for the map,
this script checks whether the centerline we get from the XODR (via Scenic's
OpenDRIVE parser) matches it.

Usage (run from Scenic repo root):
  python src/scenic/simulators/dspace/create_new_ttl/verify_ttl_against_xodr.py
  python src/scenic/simulators/dspace/create_new_ttl/verify_ttl_against_xodr.py --ttl path/to/ttl.csv --xodr path/to/LagunaSeca.xodr --save out.png
"""

import argparse
import csv
import sys
from pathlib import Path

import numpy as np
import matplotlib.pyplot as plt

# Script lives at src/scenic/simulators/dspace/create_new_ttl/
_CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = _CREATE_NEW_TTL.parent.parent.parent.parent.parent
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

# Main track only = these XODR road names (exclude Pit Lane and all junction roads)
MAIN_TRACK_ROAD_NAMES = ("The Corkscrew1", "Andretti Hairpin1_3")


def load_ttl_csv(path: str) -> np.ndarray:
    """Load x,y (and optional z) from TTL CSV. Returns (N, 2+) array."""
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


def _point_to_xy(pt):
    """Convert a point from PolylineRegion etc. to (x, y)."""
    if isinstance(pt, (tuple, list)) and len(pt) >= 2:
        return (float(pt[0]), float(pt[1]))
    if hasattr(pt, "x") and hasattr(pt, "y"):
        return (float(pt.x), float(pt.y))
    return None


def extract_xodr_centerline(
    xodr_path: str,
    ref_points: int = 100,
    main_track_only: bool = True,
    use_reference_line: bool = True,
):
    """
    Extract centerline (x, y) points from XODR using Scenic's OpenDRIVE parser.

    - main_track_only: if True, include only roads whose name is in MAIN_TRACK_ROAD_NAMES
      (The Corkscrew1, Andretti Hairpin1_3), excluding pit lane and junction roads.
    - use_reference_line: if True, use road reference line (planView); if False,
      use lane centerlines (reference + lane offset). TTL from (s, t=0) in
      ControlDesk typically corresponds to reference line (t=0).
    """
    if not Path(xodr_path).exists():
        return None
    try:
        from scenic.formats.opendrive import xodr_parser
        from scenic.domains.driving.roads import Network

        network = Network.fromOpenDrive(xodr_path, ref_points=ref_points)
        centerline_points = []

        for road in network.roads:
            road_name = (getattr(road, "name", None) or "").strip()
            if main_track_only and road_name not in MAIN_TRACK_ROAD_NAMES:
                continue
            if use_reference_line:
                # Road centerline in Scenic's OpenDRIVE conversion is the reference line (planView).
                cl = getattr(road, "centerline", None)
                if cl is None:
                    continue
                pts = getattr(cl, "points", None)
                if pts is None and hasattr(cl, "__iter__"):
                    pts = list(cl)
                if not pts:
                    continue
                for p in pts:
                    c = _point_to_xy(p)
                    if c:
                        centerline_points.append(c)
            else:
                # Lane centerlines (reference + lane offset).
                for lane in road.lanes:
                    if not getattr(lane, "centerline", None):
                        continue
                    try:
                        cl = lane.centerline
                        pts = getattr(cl, "points", None) or (list(cl) if hasattr(cl, "__iter__") else [])
                        if not pts:
                            continue
                        step = max(1, len(pts) // 500)
                        for p in pts[::step]:
                            c = _point_to_xy(p)
                            if c:
                                centerline_points.append(c)
                    except Exception:
                        pass
        return np.array(centerline_points) if centerline_points else None
    except Exception as e:
        print(f"[ERROR] Could not extract XODR centerline: {e}")
        import traceback
        traceback.print_exc()
        return None


def path_length(pts: np.ndarray) -> float:
    if len(pts) < 2:
        return 0.0
    d = np.diff(pts[:, :2], axis=0)
    return float(np.sum(np.hypot(d[:, 0], d[:, 1])))


def main():
    default_ttl = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_fellow_test_xodr_all.csv"
    default_xodr = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"

    parser = argparse.ArgumentParser(
        description="Verify TTL (known centerline) matches XODR-extracted centerline"
    )
    parser.add_argument("--ttl", type=str, default=str(default_ttl), help="TTL CSV path")
    parser.add_argument("--xodr", type=str, default=str(default_xodr), help="XODR map path")
    parser.add_argument("--save", type=str, default=None, help="Save figure path")
    parser.add_argument("--no-show", action="store_true", help="Do not show plot")
    parser.add_argument("--ref-points", type=int, default=100, help="Scenic ref_points for XODR (default 100)")
    parser.add_argument("--all-roads", action="store_true", help="Include pit lane (default: main track only)")
    parser.add_argument("--lane-centerline", action="store_true",
                        help="Use lane centerlines instead of reference line (default: reference line, i.e. t=0)")
    parser.add_argument("--compare-both", action="store_true",
                        help="Run both reference-line and lane-centerline and print comparison")
    args = parser.parse_args()

    ttl_path = Path(args.ttl)
    xodr_path = Path(args.xodr)
    if not ttl_path.exists():
        print(f"[ERROR] TTL file not found: {ttl_path}")
        return 1
    if not xodr_path.exists():
        print(f"[ERROR] XODR file not found: {xodr_path}")
        return 1

    main_track_only = not args.all_roads
    use_reference_line = not args.lane_centerline

    print("=" * 70)
    print("TTL vs XODR centerline verification")
    print("=" * 70)
    print(f"\nTTL (ground truth from ControlDesk s,t=0 inverse transform): {ttl_path.name}")
    print(f"XODR map: {xodr_path.name}")
    print(f"Options: main_track_only={main_track_only}, use_reference_line={use_reference_line}\n")

    # Load TTL
    ttl_pts = load_ttl_csv(str(ttl_path))
    if len(ttl_pts) == 0:
        print("[ERROR] No points in TTL CSV")
        return 1
    ttl_xy = ttl_pts[:, :2].astype(float)
    ttl_len = path_length(ttl_pts)
    print(f"TTL:  {len(ttl_pts)} points, path length {ttl_len:.2f} m")
    print(f"      X range [{ttl_xy[:, 0].min():.2f}, {ttl_xy[:, 0].max():.2f}], "
          f"Y range [{ttl_xy[:, 1].min():.2f}, {ttl_xy[:, 1].max():.2f}]")

    # Optionally run both reference-line and lane-centerline and compare
    if args.compare_both:
        print("\n" + "-" * 70)
        print("Comparing REFERENCE LINE vs LANE CENTERLINE (main track only)")
        print("-" * 70)
        for name, use_ref in [("Reference line (planView, t=0)", True), ("Lane centerline (ref + offset)", False)]:
            xodr_pts_v = extract_xodr_centerline(
                str(xodr_path), ref_points=args.ref_points,
                main_track_only=True, use_reference_line=use_ref
            )
            if xodr_pts_v is None or len(xodr_pts_v) == 0:
                print(f"  {name}: no points")
                continue
            xy = xodr_pts_v[:, :2].astype(float) if xodr_pts_v.ndim >= 2 else xodr_pts_v.reshape(-1, 2)
            dists = np.array([np.min(np.linalg.norm(xy - ttl_xy[i], axis=1)) for i in range(len(ttl_xy))])
            print(f"  {name}: {len(xodr_pts_v)} pts, mean={np.mean(dists):.4f} m, max={np.max(dists):.4f} m, "
                  f"%<=0.5m={100*np.sum(dists<=0.5)/len(dists):.1f}%")
        print()

    # Extract XODR centerline
    print("Extracting centerline from XODR (Scenic OpenDRIVE parser)...")
    xodr_pts = extract_xodr_centerline(
        str(xodr_path), ref_points=args.ref_points,
        main_track_only=main_track_only, use_reference_line=use_reference_line
    )
    if xodr_pts is None or len(xodr_pts) == 0:
        print("[ERROR] No centerline points from XODR")
        return 1
    xodr_xy = xodr_pts[:, :2].astype(float) if xodr_pts.ndim >= 2 else xodr_pts
    if xodr_xy.ndim == 1:
        xodr_xy = xodr_xy.reshape(-1, 2)
    xodr_len = path_length(xodr_pts)
    mode = "reference line (main track only)" if (main_track_only and use_reference_line) else (
        "lane centerline (main track only)" if main_track_only else "all roads"
    )
    print(f"XODR ({mode}): {len(xodr_pts)} points, path length {xodr_len:.2f} m")
    print(f"      X range [{xodr_xy[:, 0].min():.2f}, {xodr_xy[:, 0].max():.2f}], "
          f"Y range [{xodr_xy[:, 1].min():.2f}, {xodr_xy[:, 1].max():.2f}]")

    # Compare: for each TTL point, distance to nearest XODR point and that point's index
    from numpy.linalg import norm
    distances = []
    nearest_indices = []
    for i in range(len(ttl_xy)):
        d = norm(xodr_xy - ttl_xy[i], axis=1)
        j = np.argmin(d)
        distances.append(d[j])
        nearest_indices.append(j)
    distances = np.array(distances)
    nearest_indices = np.array(nearest_indices)
    # Systematic offset: TTL - nearest XODR (if constant, suggests transform/calibration issue)
    offsets = ttl_xy - xodr_xy[nearest_indices]
    median_offset = np.median(offsets, axis=0)
    mean_offset = np.mean(offsets, axis=0)
    offset_std = np.std(offsets, axis=0)
    mean_d = float(np.mean(distances))
    max_d = float(np.max(distances))
    median_d = float(np.median(distances))
    within_05 = np.sum(distances <= 0.5) / len(distances) * 100
    within_1 = np.sum(distances <= 1.0) / len(distances) * 100
    within_2 = np.sum(distances <= 2.0) / len(distances) * 100

    print("\n" + "-" * 70)
    print("Alignment (TTL point -> nearest XODR centerline point)")
    print("-" * 70)
    print(f"  Mean distance:   {mean_d:.4f} m")
    print(f"  Median distance: {median_d:.4f} m")
    print(f"  Max distance:    {max_d:.4f} m")
    print(f"  % within 0.5 m:  {within_05:.1f}%")
    print(f"  % within 1.0 m:  {within_1:.1f}%")
    print(f"  % within 2.0 m:  {within_2:.1f}%")
    print(f"  Offset (TTL - XODR): median=({median_offset[0]:.3f}, {median_offset[1]:.3f}) m, "
          f"mean=({mean_offset[0]:.3f}, {mean_offset[1]:.3f}) m, std=({offset_std[0]:.3f}, {offset_std[1]:.3f})")
    if np.linalg.norm(median_offset) > 0.5:
        print("  -> Non-negligible median offset may indicate transform/calibration difference.")

    if mean_d < 0.5 and max_d < 2.0:
        print("\n  Verdict: TTL matches XODR centerline well (same coordinate system / geometry).")
    elif mean_d < 1.5 and max_d < 5.0:
        print("\n  Verdict: TTL is close to XODR centerline but has local differences.")
    else:
        print("\n  Verdict: TTL and XODR centerline differ significantly (different coords or geometry).")
        print("\n  Possible causes of discrepancy:")
        print("    1. t=0 definition: ControlDesk (s,t=0) may be lane center; XODR reference line")
        print("       is planView (often left edge). Try --lane-centerline for lane center.")
        print("    2. Road connectivity: TTL is one continuous loop; XODR main track is two roads")
        print("       (The Corkscrew1, Andretti Hairpin1_3). Junction/transition may not align.")
        print("    3. Parser difference: Scenic uses full OpenDRIVE (paramPoly3); dSPACE may use")
        print("       RD or a different XODR path - geometry can differ slightly.")
        print("    4. Large max error (e.g. ~40 m) often at segment boundaries or junction.")
        worst = np.argsort(distances)[-5:][::-1]
        print("    Worst TTL indices (by distance):", worst.tolist())
        for idx in worst[:3]:
            print(f"      TTL[{idx}] = ({ttl_xy[idx,0]:.2f}, {ttl_xy[idx,1]:.2f}) -> dist = {distances[idx]:.2f} m")

    # Plot
    fig, axes = plt.subplots(1, 2, figsize=(14, 7))
    ax1, ax2 = axes[0], axes[1]

    ax1.plot(ttl_xy[:, 0], ttl_xy[:, 1], "-", color="C0", linewidth=1.2, label="TTL (known centerline)", alpha=0.9)
    ax1.plot(xodr_xy[:, 0], xodr_xy[:, 1], "-", color="C1", linewidth=0.8, label="XODR centerline", alpha=0.9)
    ax1.scatter(ttl_xy[0, 0], ttl_xy[0, 1], c="C0", s=60, zorder=5, marker="o")
    ax1.scatter(xodr_xy[0, 0], xodr_xy[0, 1], c="C1", s=60, zorder=5, marker="s")
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title("Overlay: TTL vs XODR centerline")
    ax1.legend()
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="box")

    ax2.hist(distances, bins=50, color="C2", edgecolor="black", alpha=0.7)
    ax2.axvline(mean_d, color="red", linestyle="--", linewidth=2, label=f"Mean = {mean_d:.3f} m")
    ax2.axvline(max_d, color="darkred", linestyle=":", linewidth=1.5, label=f"Max = {max_d:.3f} m")
    ax2.set_xlabel("Distance (m)")
    ax2.set_ylabel("Count")
    ax2.set_title("TTL -> nearest XODR point distance")
    ax2.legend()
    ax2.grid(True, alpha=0.3)

    plt.suptitle("Does TTL (ttl_fellow_test_xodr_all) match Laguna Seca XODR centerline?", fontsize=11, fontweight="bold")
    plt.tight_layout()

    save_path = args.save
    if save_path is None:
        save_path = _CREATE_NEW_TTL / "ttl_vs_xodr_centerline.png"
    plt.savefig(save_path, dpi=150, bbox_inches="tight")
    print(f"\nSaved: {save_path}")
    if not args.no_show:
        plt.show()
    else:
        plt.close()

    return 0


if __name__ == "__main__":
    sys.exit(main())
