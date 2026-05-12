#!/usr/bin/env python3
"""
Interactive map visualizer: XODR track boundaries + centerline + temp_aligned TTL.

Layers (drawn in order):
  1. Track boundaries from XODR (left/right edges, main track + pit)
  2. ttl_fellow_test_xodr_all.csv (known centerline)
  3. temp_aligned_to_centerline.csv

Use the matplotlib toolbar: zoom (magnifying glass), pan (hand), save, home.
Run from Scenic repo root:
  python src/scenic/simulators/dspace/create_new_ttl/interactive_map_visualizer.py
"""

import argparse
import csv
import sys
from pathlib import Path

# Scenic repo root and path setup
_CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = _CREATE_NEW_TTL.parent.parent.parent.parent.parent
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

import matplotlib.pyplot as plt
import numpy as np


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


# Main track road names in original LagunaSeca.xodr (exclude junctions so we get one track envelope)
MAIN_TRACK_ROAD_NAMES = ("The Corkscrew1", "Andretti Hairpin1_3")


def extract_track_boundaries(xodr_path: Path, main_track_road_names=None):
    """Extract left/right boundary segments from XODR via Scenic. Returns dict or None.
    If main_track_road_names is set, only those roads are included (for a single track envelope).
    """
    if not xodr_path.exists():
        return None
    try:
        from tools.compare_ttls import extract_track_boundaries as _extract
        return _extract(str(xodr_path), main_track_road_names=main_track_road_names)
    except Exception as e:
        print(f"[WARN] Could not extract boundaries: {e}")
        return None


def _plot_segments(ax, segments, color, linewidth=1.0, label=None, alpha=0.9):
    """Plot a list of segments (each segment = list of (x,y) or Nx2 array)."""
    first = True
    for seg in segments:
        if not seg:
            continue
        pts = np.asarray(seg)
        if pts.ndim == 2 and pts.shape[1] >= 2:
            ax.plot(pts[:, 0], pts[:, 1], "-", color=color, linewidth=linewidth, alpha=alpha,
                    label=label if first else None)
            first = False


def main():
    default_xodr = REPO_ROOT / "assets/maps/dSPACE/LGS_v1.xodr"
    default_main_track_xodr = REPO_ROOT / "assets/maps/dSPACE/LGS_v1_MainTrack_FromTTL.xodr"
    centerline_path = REPO_ROOT / "assets/ttls/LS_ENU_TTL_CSV/ttl_fellow_test_xodr_all.csv"
    temp_path = _CREATE_NEW_TTL / "temp_aligned_to_centerline.csv"

    parser = argparse.ArgumentParser(description="Interactive map: XODR boundaries + TTL centerline (use toolbar to zoom/pan)")
    parser.add_argument("--xodr", type=Path, default=None,
                        help=f"XODR file for track boundaries (default: {default_xodr.name})")
    parser.add_argument("--main-track", action="store_true",
                        help="Use main-track-only XODR (LagunaSeca_MainTrack_FromTTL.xodr) and centerline TTL only")
    parser.add_argument("--no-temp", action="store_true", help="Do not draw temp_aligned_to_centerline")
    args = parser.parse_args()

    if args.main_track:
        xodr_path = default_main_track_xodr
        args.no_temp = True
    else:
        xodr_path = args.xodr if args.xodr is not None else default_xodr

    xodr_path = xodr_path.resolve()
    if not xodr_path.exists():
        print(f"[ERROR] XODR not found: {xodr_path}")
        return 1

    print(f"XODR: {xodr_path.name}")
    print("Loading XODR boundaries...")
    # For original LagunaSeca.xodr, restrict to main track roads so we get 2 consistent boundaries (no junction mix)
    use_main_track_only = xodr_path.name == "LagunaSeca.xodr"
    boundaries = extract_track_boundaries(
        xodr_path,
        main_track_road_names=MAIN_TRACK_ROAD_NAMES if use_main_track_only else None,
    )
    if not boundaries:
        print("[WARN] No boundaries; plot will show TTLs only.")

    print("Loading TTLs...")
    centerline_pts = load_csv(centerline_path)
    if len(centerline_pts) < 2:
        print("[ERROR] Centerline CSV has too few points.")
        return 1
    temp_pts = load_csv(temp_path) if not args.no_temp else None
    if not args.no_temp and (temp_pts is None or len(temp_pts) < 2):
        print("[ERROR] temp_aligned CSV has too few points.")
        return 1

    # Interactive figure (toolbar has zoom, pan, save)
    fig, ax = plt.subplots(figsize=(12, 10))

    # 1. Track boundaries (XODR) – 2 lines only: outer left and outer right (no lane-internal edges).
    #    Applies to both the original LagunaSeca.xodr and the main-track-only XODR.
    if boundaries:
        left_segs = boundaries.get("left_boundary_segments", []) or []
        right_segs = boundaries.get("right_boundary_segments", []) or []
        pit_left = boundaries.get("pit_left_boundary_segments", []) or []
        pit_right = boundaries.get("pit_right_boundary_segments", []) or []
        outer_left = [left_segs[0]] if left_segs else []
        outer_right = [right_segs[-1]] if right_segs else []
        _plot_segments(ax, outer_left, color="#333333", linewidth=2.0, label="Track (outer left)", alpha=0.95)
        _plot_segments(ax, outer_right, color="#333333", linewidth=2.0, label="Track (outer right)", alpha=0.95)
        if pit_left or pit_right:
            po_left = [pit_left[0]] if pit_left else []
            po_right = [pit_right[-1]] if pit_right else []
            _plot_segments(ax, po_left, color="#666666", linewidth=1.5, label="Pit (outer left)", alpha=0.8)
            _plot_segments(ax, po_right, color="#666666", linewidth=1.5, alpha=0.8)
    else:
        ax.set_title("(No XODR boundaries loaded)")

    # 2. Known centerline (ttl_fellow_test_xodr_all)
    ax.plot(
        centerline_pts[:, 0], centerline_pts[:, 1],
        "-", color="C0", linewidth=1.8, label="ttl_fellow_test_xodr_all (centerline)", alpha=0.95
    )
    ax.scatter(
        centerline_pts[0, 0], centerline_pts[0, 1],
        c="C0", s=60, zorder=5, marker="o", edgecolors="white", linewidths=1
    )

    # 3. temp_aligned_to_centerline (optional)
    if not args.no_temp and temp_pts is not None:
        ax.plot(
            temp_pts[:, 0], temp_pts[:, 1],
            "-", color="C1", linewidth=1.5, label="temp_aligned_to_centerline", alpha=0.9
        )
        ax.scatter(
            temp_pts[0, 0], temp_pts[0, 1],
            c="C1", s=60, zorder=5, marker="s", edgecolors="white", linewidths=1
        )

    ax.set_xlabel("X (m)")
    ax.set_ylabel("Y (m)")
    title = f"Interactive map: {xodr_path.name} + centerline TTL (use toolbar to zoom/pan)"
    ax.set_title(title)
    ax.legend(loc="upper right", fontsize=9)
    ax.grid(True, alpha=0.3)
    ax.set_aspect("equal", adjustable="box")
    plt.tight_layout()

    print("Opening interactive window. Use toolbar: zoom, pan, save, home.")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
