#!/usr/bin/env python3
"""
Verify round trip: Scenic (XODR) -> place -> read GPS -> GPS->dSPACE (RD) -> dSPACE->Scenic (XODR).
Uses gps_dspace_table.csv: for each row we have (x_dspace, y_dspace) = Scenic XODR, (lon, lat) = GPS,
and optionally (x_rd, y_rd) = dSPACE RD. Compares final Scenic to initial.

Run from repo root:
  python src/scenic/simulators/dspace/converters/verify_gps_round_trip.py
  python src/scenic/simulators/dspace/converters/verify_gps_round_trip.py --csv gps_dspace_table.csv --samples 50
"""

import argparse
import math
import sys
from pathlib import Path

_CONVERTERS = Path(__file__).resolve().parent
_DSPACE = _CONVERTERS.parent
_REPO_ROOT = _DSPACE.parent.parent.parent.parent

if str(_REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(_REPO_ROOT / "src"))

from scenic.domains.racing.gnss_transform import (
    load_calibration,
    load_calibration_table_csv,
    load_gps_table_rows,
)
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_coordinate_transform,
    apply_inverse_coordinate_transform,
)


def _dist(a, b):
    return math.sqrt((a[0] - b[0]) ** 2 + (a[1] - b[1]) ** 2)


def main():
    ap = argparse.ArgumentParser(description="Verify GPS <-> dSPACE <-> Scenic round trip")
    ap.add_argument("--csv", type=Path, default=_REPO_ROOT / "gps_dspace_table.csv", help="Table CSV")
    ap.add_argument("--samples", type=int, default=100, help="Number of sample rows to verify")
    ap.add_argument("--cal-xodr", type=Path, default=_DSPACE / "geometry" / "gps_dspace_calibration.json", help="GPS->XODR (or position) calibration")
    ap.add_argument("--cal-rd", type=Path, default=_DSPACE / "geometry" / "gps_rd_calibration.json", help="GPS->RD calibration (when table has x_rd, y_rd)")
    ap.add_argument("--xodr-rd-transform", type=Path, default=None, help="XODR<->RD transform JSON (default: assets/maps/dSPACE/Laguna_Seca_transform.json)")
    args = ap.parse_args()

    csv_path = args.csv if args.csv.is_absolute() else _REPO_ROOT / args.csv
    if not csv_path.exists():
        print(f"[ERROR] CSV not found: {csv_path}")
        return 1

    rows = load_gps_table_rows(csv_path)
    if not rows:
        print("[ERROR] No valid rows in CSV")
        return 1

    has_rd = "x_rd" in rows[0] and "y_rd" in rows[0] and rows[0].get("x_rd") is not None and rows[0].get("y_rd") is not None
    if has_rd:
        try:
            cal_rd = load_calibration(args.cal_rd if args.cal_rd.is_absolute() else _REPO_ROOT / args.cal_rd)
        except Exception as e:
            print(f"[ERROR] Table has x_rd/y_rd but could not load GPS->RD calibration: {e}")
            print("        Run: python .../fit_gps_dspace_calibration.py --csv <table> --output .../geometry/gps_rd_calibration.json (with CSV that has x_rd, y_rd)")
            return 1
        transform_path = args.xodr_rd_transform or _REPO_ROOT / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        transform_path = transform_path if transform_path.is_absolute() else _REPO_ROOT / transform_path
        if not transform_path.exists():
            print(f"[ERROR] XODR<->RD transform not found: {transform_path}")
            return 1
        xodr_rd = load_transform(str(transform_path))
        print("Round trip: Scenic (XODR) -> RD -> GPS -> RD (from GPS cal) -> Scenic (XODR)")
    else:
        cal_xodr = load_calibration(args.cal_xodr if args.cal_xodr.is_absolute() else _REPO_ROOT / args.cal_xodr)
        print("Table has no x_rd/y_rd: verifying GPS <-> Scenic (x_dspace, y_dspace) round trip only.")
        print("(Full chain through dSPACE RD requires a run that collects x_rd, y_rd.)")

    n = min(args.samples, len(rows))
    step = max(1, len(rows) // n)
    indices = list(range(0, len(rows), step))[:n]

    errs_xy = []
    errs_ll = []
    errs_scenic_back = []  # final Scenic vs initial (full chain)

    for i in indices:
        r = rows[i]
        xodr = (r["x_dspace"], r["y_dspace"])
        lon, lat = r["longitude_deg"], r["latitude_deg"]

        if has_rd:
            rd_table = (r["x_rd"], r["y_rd"])
            rd_from_gps = cal_rd.gnss_to_local(lon, lat)
            xodr_back = apply_inverse_coordinate_transform(xodr_rd, rd_from_gps)
            err_rd = _dist(rd_from_gps, rd_table)
            err_scenic = _dist(xodr_back, xodr)
            errs_xy.append(err_rd)
            errs_scenic_back.append(err_scenic)
        else:
            xy_cal = cal_xodr.gps_to_dspace(lon, lat)
            err_xy = _dist(xy_cal, xodr)
            errs_xy.append(err_xy)
            lon2, lat2 = cal_xodr.local_to_gnss(xodr[0], xodr[1])
            xy2 = cal_xodr.gnss_to_local(lon2, lat2)
            err_back = _dist(xy2, xodr)
            errs_scenic_back.append(err_back)

    def stats(name, vals):
        if not vals:
            return
        m = sum(vals) / len(vals)
        mx = max(vals)
        print(f"  {name}: mean={m:.4f} m  max={mx:.4f} m")

    print(f"\nSampled {len(indices)} rows (step={step}).")
    stats("GPS -> position (cal)", errs_xy)
    stats("Round-trip back to Scenic", errs_scenic_back)

    if has_rd:
        ok = (sum(errs_scenic_back) / len(errs_scenic_back)) < 1.0 and max(errs_scenic_back) < 5.0
    else:
        ok = (sum(errs_xy) / len(errs_xy)) < 1.0 and (sum(errs_scenic_back) / len(errs_scenic_back)) < 1.0
    print("\n[OK] Round trip within tolerance." if ok else "\n[WARN] Some errors large; check calibration or re-fit.")
    return 0 if ok else 0


if __name__ == "__main__":
    sys.exit(main())
