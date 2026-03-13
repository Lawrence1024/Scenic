#!/usr/bin/env python3
"""Export TTL centerline CSVs from dSPACE route measurements.

Reads route_st_to_xodr_measurements.csv (R2 = main track, R1 = pitlane + Andretti),
extracts centerline points (t_input_m == 0) in (s_input_m) order, and writes
x,y,z using XODR coordinates so the result can replace ttl_main_road.csv and
ttl_pitlane.csv.

R2 is the main track; R1 is the pit lane (incomplete collection is still used;
overlap with R2 at the end is assumed).

Usage (from repo root):
  python src/scenic/simulators/dspace/create_new_ttl/measurements_to_centerlines.py
  python .../measurements_to_centerlines.py --copy-to-assets
  python .../measurements_to_centerlines.py -o path/to/dir
"""

import argparse
import csv
import shutil
import sys
from pathlib import Path
from typing import List, Tuple

THIS_DIR = Path(__file__).resolve().parent
MEASUREMENTS_DIR = THIS_DIR / "measurements"
MEASUREMENTS_CSV = MEASUREMENTS_DIR / "route_st_to_xodr_measurements.csv"
REPO_ROOT = THIS_DIR.parent.parent.parent.parent
ASSETS_TTL = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"

MAIN_FROM_MEASUREMENTS = "ttl_main_road_from_measurements.csv"
PIT_FROM_MEASUREMENTS = "ttl_pitlane_from_measurements.csv"


def load_centerline(csv_path: Path, route: str) -> List[Tuple[float, float, float]]:
    """Load centerline (t_input_m == 0) for the given route. Returns list of (x, y, z) in s order."""
    rows = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            if row.get("route") != route:
                continue
            try:
                t = float(row["t_input_m"])
                if t != 0.0:
                    continue
                x = float(row["xodr_x_m"])
                y = float(row["xodr_y_m"])
                z = float(row["xodr_z_m"])
                s = float(row["s_input_m"])
                rows.append((s, x, y, z))
            except (KeyError, ValueError):
                continue
    rows.sort(key=lambda r: r[0])
    return [(r[1], r[2], r[3]) for r in rows]


def write_ttl_csv(path: Path, points: List[Tuple[float, float, float]]) -> None:
    """Write a TTL CSV with header x,y,z and one row per point."""
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        for x, y, z in points:
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])
    print(f"  Wrote {len(points)} points -> {path}")


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Export main and pit centerline CSVs from route measurements (R2, R1)."
    )
    parser.add_argument(
        "-o", "--output-dir",
        type=Path,
        default=THIS_DIR,
        help=f"Directory for output CSVs (default: create_new_ttl)",
    )
    parser.add_argument(
        "--copy-to-assets",
        action="store_true",
        help="Copy outputs to assets/ttls/LS_ENU_TTL_CSV as ttl_main_road.csv and ttl_pitlane.csv",
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=MEASUREMENTS_CSV,
        help=f"Path to route_st_to_xodr_measurements.csv",
    )
    args = parser.parse_args()

    if not args.csv.exists():
        print(f"ERROR: Measurements CSV not found: {args.csv}", file=sys.stderr)
        return 1

    print(f"Loading centerlines from {args.csv.name} ...")
    main_pts = load_centerline(args.csv, "R2")
    pit_pts = load_centerline(args.csv, "R1")
    if not main_pts:
        print("ERROR: No R2 (main track) centerline points found.", file=sys.stderr)
        return 1
    if not pit_pts:
        print("ERROR: No R1 (pitlane) centerline points found.", file=sys.stderr)
        return 1

    print(f"  R2 (main): {len(main_pts)} points")
    print(f"  R1 (pit): {len(pit_pts)} points")

    out_dir = args.output_dir.resolve()
    main_path = out_dir / MAIN_FROM_MEASUREMENTS
    pit_path = out_dir / PIT_FROM_MEASUREMENTS
    write_ttl_csv(main_path, main_pts)
    write_ttl_csv(pit_path, pit_pts)

    if args.copy_to_assets:
        if not ASSETS_TTL.exists():
            print(f"ERROR: Assets TTL folder not found: {ASSETS_TTL}", file=sys.stderr)
            return 1
        dest_main = ASSETS_TTL / "ttl_main_road.csv"
        dest_pit = ASSETS_TTL / "ttl_pitlane.csv"
        shutil.copy2(main_path, dest_main)
        shutil.copy2(pit_path, dest_pit)
        print(f"  Copied to {dest_main}")
        print(f"  Copied to {dest_pit}")

    return 0


if __name__ == "__main__":
    sys.exit(main())
