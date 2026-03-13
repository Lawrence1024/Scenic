#!/usr/bin/env python3
"""
Convert GPS-format TTL CSVs (latitude, longitude, altitude) to RD/XODR (x, y, z)
using gps_rd_calibration.json (GNSS↔RD). Overwrites the 4 *_xodr.csv files.

Since the new XODR is generated from RD centerline, RD and XODR coordinates
are the same; this calibration is the correct one for producing _xodr files.

Usage (from repo root):
  python -m scenic.domains.racing.ttl_processing.gps_ttl_to_rd_xodr
"""

import csv
import sys
from pathlib import Path

_THIS_DIR = Path(__file__).resolve().parent
_RACING = _THIS_DIR.parent
_SCENIC_SRC = _RACING.parent.parent.parent
_REPO_ROOT = _SCENIC_SRC.parent

TTLS_FOLDER = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"
CALIBRATION_JSON = (
    _REPO_ROOT
    / "src"
    / "scenic"
    / "simulators"
    / "dspace"
    / "geometry"
    / "gps_rd_calibration.json"
)

GPS_TTL_BASES = ("ttl_left", "ttl_optimal", "ttl_pit", "ttl_right")


def convert_one(cal, in_path: Path, out_path: Path) -> int:
    """Read GPS CSV (latitude, longitude, altitude), convert to (x, y, z) via GNSS->RD, write."""
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


def main() -> int:
    if not CALIBRATION_JSON.exists():
        print(f"[ERROR] Calibration not found: {CALIBRATION_JSON}", file=sys.stderr)
        return 1
    if str(_SCENIC_SRC) not in sys.path:
        sys.path.insert(0, str(_SCENIC_SRC))
    from scenic.domains.racing.gnss_transform import load_calibration

    cal = load_calibration(CALIBRATION_JSON)
    if not TTLS_FOLDER.exists():
        print(f"[ERROR] TTL folder not found: {TTLS_FOLDER}", file=sys.stderr)
        return 1

    for base in GPS_TTL_BASES:
        in_path = TTLS_FOLDER / f"{base}.csv"
        out_path = TTLS_FOLDER / f"{base}_xodr.csv"
        if not in_path.exists():
            print(f"[SKIP] {in_path.name} not found", file=sys.stderr)
            continue
        n = convert_one(cal, in_path, out_path)
        print(f"[OK] {in_path.name} -> {out_path.name} ({n} points)")
    return 0


if __name__ == "__main__":
    sys.exit(main())
