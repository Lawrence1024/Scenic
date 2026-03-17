#!/usr/bin/env python3
"""
Fit GPS <-> dSPACE transform from gps_dspace_table.csv and save calibration.

Run from repo root after a run that produced gps_dspace_table.csv:
  python src/scenic/simulators/dspace/converters/fit_gps_dspace_calibration.py
  python src/scenic/simulators/dspace/converters/fit_gps_dspace_calibration.py --csv src/scenic/simulators/dspace/converters/gps_dspace_table.csv --output src/scenic/simulators/dspace/geometry/gps_dspace_calibration.json
"""

import argparse
import sys
from pathlib import Path

# Script lives in .../dspace/converters/; dspace is parent, geometry is sibling of converters
_CONVERTERS = Path(__file__).resolve().parent
_DSPACE = _CONVERTERS.parent
_REPO_ROOT = _DSPACE.parent.parent.parent.parent

if str(_REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "src"))

from scenic.domains.racing.gnss_transform import (
    fit_transform_from_csv,
    load_calibration,
    save_calibration,
    load_calibration_table_csv,
    GNSSLocalTransform,
)


def main():
    ap = argparse.ArgumentParser(description="Fit GPS<->dSPACE transform from table CSV and save calibration")
    ap.add_argument("--csv", type=Path, default=_CONVERTERS / "gps_dspace_table.csv", help="Input CSV (sim_time, x_dspace, y_dspace, ..., longitude_deg, latitude_deg, ...)")
    ap.add_argument("--output", "-o", type=Path, default=None, help="Output JSON path (default: gps_dspace_calibration.json or gps_rd_calibration.json when --rd)")
    ap.add_argument("--rd", action="store_true", help="Fit to x_rd, y_rd (dSPACE RD) when present; output gps_rd_calibration.json for full round-trip verify")
    args = ap.parse_args()

    csv_path = args.csv if args.csv.is_absolute() else _REPO_ROOT / args.csv
    if not csv_path.exists():
        print(f"[ERROR] CSV not found: {csv_path}")
        return 1

    target = "rd" if args.rd else "xodr"
    if args.rd:
        table_rd = load_calibration_table_csv(csv_path, target="rd")
        if len(table_rd) < 3:
            print("[ERROR] Too few rows with x_rd, y_rd. Run a simulation to collect RD columns, or omit --rd.")
            return 1
    out_path = args.output
    if out_path is None:
        out_path = _DSPACE / "geometry" / ("gps_rd_calibration.json" if args.rd else "gps_dspace_calibration.json")
    else:
        out_path = out_path if out_path.is_absolute() else _REPO_ROOT / out_path
    out_path = Path(out_path)

    print(f"Loading table (target={target})...")
    transform = fit_transform_from_csv(csv_path, target=target)
    save_calibration(transform, out_path)
    print(f"Saved calibration to {out_path}")

    # Quick validation: first and last row round-trip
    table = load_calibration_table_csv(csv_path, target=target)
    if len(table) >= 2:
        for label, i in [("first", 0), ("last", -1)]:
            lon, lat, x, y = table[i]
            x2, y2 = transform.gps_to_dspace(lon, lat)
            lon2, lat2 = transform.dspace_to_gps(x, y)
            err_xy = (x2 - x) ** 2 + (y2 - y) ** 2
            err_ll = (lon2 - lon) ** 2 + (lat2 - lat) ** 2
            print(f"  {label} row: GPS({lon:.6f},{lat:.6f}) -> dSPACE({x2:.3f},{y2:.3f}) [true ({x:.3f},{y:.3f})] xy_err^2={err_xy:.6f}")
            print(f"           dSPACE({x:.3f},{y:.3f}) -> GPS({lon2:.6f},{lat2:.6f}) [true ({lon:.6f},{lat:.6f})] ll_err^2={err_ll:.2e}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
