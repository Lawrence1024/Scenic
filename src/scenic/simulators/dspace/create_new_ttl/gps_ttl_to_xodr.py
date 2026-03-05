#!/usr/bin/env python3
"""
Convert GPS-format TTL CSVs (latitude, longitude, altitude) to OpenDRIVE (x, y, z) using
the GNSS->XODR calibration (gps_dspace_calibration.json). Writes copies with _xodr suffix
in the same folder (e.g. ttl_left.csv -> ttl_left_xodr.csv).
"""
import csv
import sys
from pathlib import Path

# Script lives at .../create_new_ttl/; dspace is parent^2
CREATE_NEW_TTL = Path(__file__).resolve().parent
DSPACE = CREATE_NEW_TTL.parent
REPO_ROOT = CREATE_NEW_TTL.parent.parent.parent.parent.parent

# Ensure we can import scenic (package is under src/)
src = REPO_ROOT / "src"
if str(src) not in sys.path:
    sys.path.insert(0, str(src))

from scenic.domains.racing.gnss_transform import load_calibration

TTLS_FOLDER = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"
CALIBRATION_JSON = DSPACE / "geometry" / "gps_dspace_calibration.json"

# Base names (no extension) of the 4 GPS TTLs to convert
GPS_TTL_BASES = ("ttl_left", "ttl_optimal", "ttl_pit", "ttl_right")


def convert_one(cal, in_path: Path, out_path: Path) -> int:
    """Read GPS CSV, convert each (lat, lon, alt) -> (x, y, z), write XODR CSV. Returns row count."""
    rows = []
    with open(in_path, newline="", encoding="utf-8") as f:
        r = csv.DictReader(f)
        if not r.fieldnames:
            return 0
        for row in r:
            try:
                lat = float(row.get("latitude", ""))
                lon = float(row.get("longitude", ""))
                alt = float(row.get("altitude", ""))
            except (ValueError, TypeError):
                continue
            x, y = cal.gnss_to_local(lon, lat)
            rows.append((x, y, alt))
    with open(out_path, "w", newline="", encoding="utf-8") as f:
        w = csv.writer(f)
        w.writerow(("x", "y", "z"))
        w.writerows(rows)
    return len(rows)


def main():
    if not CALIBRATION_JSON.exists():
        print(f"[ERROR] Calibration not found: {CALIBRATION_JSON}")
        return 1
    cal = load_calibration(CALIBRATION_JSON)
    if not TTLS_FOLDER.exists():
        print(f"[ERROR] TTL folder not found: {TTLS_FOLDER}")
        return 1
    for base in GPS_TTL_BASES:
        in_path = TTLS_FOLDER / f"{base}.csv"
        out_path = TTLS_FOLDER / f"{base}_xodr.csv"
        if not in_path.exists():
            print(f"[SKIP] {in_path.name} not found")
            continue
        n = convert_one(cal, in_path, out_path)
        print(f"[OK] {in_path.name} -> {out_path.name} ({n} points)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
