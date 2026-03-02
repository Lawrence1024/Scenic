#!/usr/bin/env python3
"""
Visualize the combined TTL in 3 panels: pitlane only, main loop only, combined.
Run after combine_and_compare_ttl.py has written ttl_pitlane.csv to assets. From repo root:

  python src/scenic/simulators/dspace/create_new_ttl/visualize_combined_ttl.py
  python src/scenic/simulators/dspace/create_new_ttl/visualize_combined_ttl.py --save comparison.png
"""
import argparse
import csv
import sys
from pathlib import Path

CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = CREATE_NEW_TTL.parent.parent.parent.parent.parent
EXISTING_TTL = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_main_road.csv"
COMBINED_TTL = CREATE_NEW_TTL / "ttl_pitlane_main_pitlane.csv"
PITLANE_TTL = CREATE_NEW_TTL / "temp_pitlane_begin_ttl.csv"
# Closed loop: Pit Lane -> Corkscrew -> Pit Lane (in assets folder)
PIT_CORKSCREW_LOOP_TTL = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_pitlane.csv"

if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))

import matplotlib
matplotlib.use("TkAgg")
import matplotlib.pyplot as plt
import numpy as np


def load_ttl(path: Path) -> np.ndarray:
    """Load x,y,z CSV; return (N, 3) array."""
    points = []
    with path.open("r", newline="") as f:
        r = csv.reader(f)
        for row in r:
            if row and row[0].strip().lower() != "x":
                if len(row) >= 2:
                    try:
                        x, y = float(row[0]), float(row[1])
                        z = float(row[2]) if len(row) >= 3 else 0.0
                        points.append([x, y, z])
                    except ValueError:
                        continue
    return np.array(points) if points else np.empty((0, 3))


def main():
    p = argparse.ArgumentParser(description="Visualize TTLs: 3-panel view or overlay of the two closed loops")
    p.add_argument("--overlap", action="store_true", help="Overlay the two closed-loop TTLs (Andretti+Corkscrew vs Pit Lane+Corkscrew)")
    p.add_argument("--save", type=Path, default=None, metavar="PATH", help="Save figure to file (and still show window)")
    args = p.parse_args()

    if not EXISTING_TTL.exists():
        print(f"[ERROR] Existing TTL not found: {EXISTING_TTL}")
        return 1

    existing = load_ttl(EXISTING_TTL)
    if len(existing) < 2:
        print("[ERROR] Existing TTL has too few points.")
        return 1

    if args.overlap:
        # Overlay: Andretti+Corkscrew loop (existing) vs Pit Lane+Corkscrew loop
        if not PIT_CORKSCREW_LOOP_TTL.exists():
            print(f"[ERROR] {PIT_CORKSCREW_LOOP_TTL.name} not found. Run from repo root: python src/scenic/simulators/dspace/create_new_ttl/combine_and_compare_ttl.py")
            return 1
        pit_loop = load_ttl(PIT_CORKSCREW_LOOP_TTL)
        if len(pit_loop) < 2:
            print("[ERROR] Pit Lane+Corkscrew loop has too few points.")
            return 1
        fig, ax = plt.subplots(figsize=(12, 10))
        ax.plot(existing[:, 0], existing[:, 1], "-", color="C0", linewidth=1.6, alpha=0.9, label="Andretti + Corkscrew (existing)")
        ax.scatter(existing[0, 0], existing[0, 1], c="C0", s=80, zorder=5, marker="o", edgecolors="white", linewidths=1.5)
        ax.plot(pit_loop[:, 0], pit_loop[:, 1], "-", color="C1", linewidth=1.5, alpha=0.9, label="Pit Lane + Corkscrew (new)")
        ax.scatter(pit_loop[0, 0], pit_loop[0, 1], c="C1", s=80, zorder=5, marker="s", edgecolors="white", linewidths=1.5)
        ax.set_xlabel("X (m)")
        ax.set_ylabel("Y (m)")
        ax.set_title("Two closed-loop TTLs overlaid")
        ax.legend(loc="upper right", fontsize=9)
        ax.grid(True, alpha=0.3)
        ax.set_aspect("equal", adjustable="box")
        plt.tight_layout()
        print("Andretti+Corkscrew (existing):", len(existing), "points")
        print("Pit Lane+Corkscrew (new):", len(pit_loop), "points")
        if args.save is not None:
            fig.savefig(args.save, dpi=150, bbox_inches="tight")
            print("Saved:", args.save)
        print("Opening overlay window. Close to exit.")
        plt.show()
        return 0

    # Default: 3-panel view
    if not COMBINED_TTL.exists():
        print(f"[ERROR] Combined TTL not found: {COMBINED_TTL}")
        print("Run from repo root: python src/scenic/simulators/dspace/create_new_ttl/combine_and_compare_ttl.py")
        return 1
    combined = load_ttl(COMBINED_TTL)
    if PITLANE_TTL.exists():
        pitlane = load_ttl(PITLANE_TTL)
    else:
        n_pit = 960
        pitlane = combined[:n_pit] if len(combined) >= n_pit else combined[: len(combined) // 3]
    if len(combined) < 2 or len(pitlane) < 2:
        print("[ERROR] One or more TTLs have too few points.")
        return 1

    fig, (ax1, ax2, ax3) = plt.subplots(1, 3, figsize=(16, 6))

    ax1.plot(pitlane[:, 0], pitlane[:, 1], "-", color="C1", linewidth=1.6, alpha=0.95)
    ax1.scatter(pitlane[0, 0], pitlane[0, 1], c="C1", s=80, zorder=5, marker="s", edgecolors="white", linewidths=1.5)
    ax1.set_xlabel("X (m)")
    ax1.set_ylabel("Y (m)")
    ax1.set_title(f"Pitlane TTL only ({len(pitlane)} points)")
    ax1.grid(True, alpha=0.3)
    ax1.set_aspect("equal", adjustable="box")

    ax2.plot(existing[:, 0], existing[:, 1], "-", color="C0", linewidth=1.6, alpha=0.95)
    ax2.scatter(existing[0, 0], existing[0, 1], c="C0", s=80, zorder=5, marker="o", edgecolors="white", linewidths=1.5)
    ax2.set_xlabel("X (m)")
    ax2.set_ylabel("Y (m)")
    ax2.set_title(f"Main loop TTL only ({len(existing)} points)")
    ax2.grid(True, alpha=0.3)
    ax2.set_aspect("equal", adjustable="box")

    ax3.plot(combined[:, 0], combined[:, 1], "-", color="C2", linewidth=1.4, label="Combined", alpha=0.95)
    ax3.scatter(combined[0, 0], combined[0, 1], c="C2", s=80, zorder=5, marker="^", edgecolors="white", linewidths=1.5, label="Start (pitlane)")
    ax3.set_xlabel("X (m)")
    ax3.set_ylabel("Y (m)")
    ax3.set_title(f"Combined TTL ({len(combined)} points)")
    ax3.legend(loc="upper right", fontsize=8)
    ax3.grid(True, alpha=0.3)
    ax3.set_aspect("equal", adjustable="box")

    plt.tight_layout()
    print("Pitlane TTL:", len(pitlane), "points")
    print("Main loop TTL:", len(existing), "points")
    print("Combined TTL:", len(combined), "points")
    if args.save is not None:
        fig.savefig(args.save, dpi=150, bbox_inches="tight")
        print("Saved:", args.save)
    print("Opening window with 3 panels. Close the window to exit.")
    plt.show()
    return 0


if __name__ == "__main__":
    sys.exit(main())
