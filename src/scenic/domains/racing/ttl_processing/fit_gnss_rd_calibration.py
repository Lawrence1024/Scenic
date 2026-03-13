#!/usr/bin/env python3
"""
Fit GNSS <-> RD (dSPACE) calibration from route_st_to_xodr_measurements.csv.

Uses the large measurement set (route sweep with RD positions and GPS) to compute
the affine transform (lon, lat) <-> (rd_x_m, rd_y_m) and writes
gps_rd_calibration.json for use by readback and verification.

Usage (from repo root):
  python -m scenic.domains.racing.ttl_processing.fit_gnss_rd_calibration
  python -m scenic.domains.racing.ttl_processing.fit_gnss_rd_calibration --csv path/to/measurements.csv -o path/to/gps_rd_calibration.json
"""

import argparse
import csv
import sys
from pathlib import Path

# Resolve repo root (parent of src)
_THIS_DIR = Path(__file__).resolve().parent
_RACING = _THIS_DIR.parent
_SCENIC_SRC = _RACING.parent.parent.parent
_REPO_ROOT = _SCENIC_SRC.parent

# Default paths
_DEFAULT_MEASUREMENTS = (
    _REPO_ROOT
    / "src"
    / "scenic"
    / "simulators"
    / "dspace"
    / "create_new_ttl"
    / "measurements"
    / "route_st_to_xodr_measurements.csv"
)
_DEFAULT_OUTPUT = (
    _REPO_ROOT
    / "src"
    / "scenic"
    / "simulators"
    / "dspace"
    / "geometry"
    / "gps_rd_calibration.json"
)


def load_measurements_table(csv_path: Path) -> "tuple[list[tuple[float, float, float, float]], list[dict]]":
    """Load (lon_deg, lat_deg, rd_x_m, rd_y_m) from route_st_to_xodr_measurements.csv.
    Returns (rows as list of 4-tuples, raw row dicts for optional use).
    """
    rows = []
    raw = []
    with open(csv_path, newline="", encoding="utf-8") as f:
        reader = csv.DictReader(f)
        for row in reader:
            try:
                lon = float(row.get("gps_longitude_deg", row.get("longitude_deg", "")) or 0)
                lat = float(row.get("gps_latitude_deg", row.get("latitude_deg", "")) or 0)
                rd_x = float(row.get("rd_x_m", row.get("x_rd", "")) or 0)
                rd_y = float(row.get("rd_y_m", row.get("y_rd", "")) or 0)
            except (ValueError, TypeError):
                continue
            rows.append((lon, lat, rd_x, rd_y))
            raw.append(row)
    return rows, raw


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Fit GNSS <-> RD calibration from route measurements and update gps_rd_calibration.json"
    )
    parser.add_argument(
        "--csv",
        type=Path,
        default=_DEFAULT_MEASUREMENTS,
        help=f"Path to route_st_to_xodr_measurements.csv (default: create_new_ttl/measurements)",
    )
    parser.add_argument(
        "-o", "--output",
        type=Path,
        default=_DEFAULT_OUTPUT,
        help=f"Output JSON path (default: simulators/dspace/geometry/gps_rd_calibration.json)",
    )
    parser.add_argument(
        "--max-points",
        type=int,
        default=None,
        help="Subsample to at most N points (default: use all)",
    )
    parser.add_argument(
        "--validate-only",
        action="store_true",
        help="Load existing calibration and print validation against measurements (no fit, no write)",
    )
    args = parser.parse_args()

    csv_path = args.csv if args.csv.is_absolute() else _REPO_ROOT / args.csv
    if not csv_path.exists():
        print(f"[ERROR] Measurements CSV not found: {csv_path}", file=sys.stderr)
        return 1

    if str(_SCENIC_SRC) not in sys.path:
        sys.path.insert(0, str(_SCENIC_SRC))

    from scenic.domains.racing.gnss_transform import (
        fit_transform_from_table,
        load_calibration,
        save_calibration,
        GNSSLocalTransform,
    )
    import numpy as np

    rows, _ = load_measurements_table(csv_path)
    if len(rows) < 3:
        print(f"[ERROR] Need at least 3 rows; got {len(rows)}", file=sys.stderr)
        return 1

    if args.max_points and len(rows) > args.max_points:
        # Uniform subsample
        step = len(rows) / args.max_points
        indices = [int(i * step) for i in range(args.max_points)]
        rows = [rows[i] for i in indices]
        print(f"[INFO] Subsampled to {len(rows)} points")

    table = np.array(rows)  # (N, 4) -> lon, lat, rd_x, rd_y
    print(f"Loaded {len(table)} measurement points from {csv_path.name}")

    if args.validate_only:
        out_path = args.output if args.output.is_absolute() else _REPO_ROOT / args.output
        if not out_path.exists():
            print(f"[ERROR] No existing calibration at {out_path}", file=sys.stderr)
            return 1
        cal = load_calibration(out_path)
        print("Validation (existing calibration):")
    else:
        # Fit: (lon, lat) -> (rd_x, rd_y); table columns are (lon, lat, x_local, y_local)
        cal = fit_transform_from_table(table)
        print("Fitted GNSS -> RD transform (reference + affine)")

    # Validation: residuals and round-trip
    lon, lat = table[:, 0], table[:, 1]
    rd_x, rd_y = table[:, 2], table[:, 3]
    pred_x = np.array([cal.gnss_to_local(lon[i], lat[i])[0] for i in range(len(table))])
    pred_y = np.array([cal.gnss_to_local(lon[i], lat[i])[1] for i in range(len(table))])
    err_x = pred_x - rd_x
    err_y = pred_y - rd_y
    err_m = np.sqrt(err_x**2 + err_y**2)
    print(f"  GNSS -> RD: mean error = {np.mean(err_m):.6f} m, max = {np.max(err_m):.6f} m, std = {np.std(err_m):.6f} m")

    # Round-trip RD -> GNSS -> RD
    back_x = np.array([cal.local_to_gnss(rd_x[i], rd_y[i])[0] for i in range(len(table))])
    back_y = np.array([cal.local_to_gnss(rd_x[i], rd_y[i])[1] for i in range(len(table))])
    pred2_x = np.array([cal.gnss_to_local(back_x[i], back_y[i])[0] for i in range(len(table))])
    pred2_y = np.array([cal.gnss_to_local(back_x[i], back_y[i])[1] for i in range(len(table))])
    rt_err = np.sqrt((pred2_x - rd_x) ** 2 + (pred2_y - rd_y) ** 2)
    print(f"  Round-trip RD->GNSS->RD: mean error = {np.mean(rt_err):.6f} m, max = {np.max(rt_err):.6f} m")

    if args.validate_only:
        return 0

    out_path = args.output if args.output.is_absolute() else _REPO_ROOT / args.output
    out_path.parent.mkdir(parents=True, exist_ok=True)
    save_calibration(cal, out_path)
    print(f"Saved calibration to {out_path}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
