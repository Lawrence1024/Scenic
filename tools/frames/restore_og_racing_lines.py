"""Restore the pre-restructure (OLD) racing-line TTLs into NEW XODR frame.

The OLD ``ttl_*_xodr_og.csv`` files (3 columns: x, y, z) live in dSPACE RD frame
(equivalently OLD XODR frame, since the OLD ``LagunaSeca.xodr`` was auto-generated
to match dSPACE RD). They were empirically verified pre-restructure to keep ego
inside dSPACE's rendered track polygon for the entire F-bank.

This script translates each OLD file by ``(+6.101, +50.761)`` (the inverse of the
calibration in ``LGS_v1_gps_rd_calibration.json``, i.e. ``RD -> NEW XODR``) and
overwrites the corresponding active ``ttl_*_xodr.csv`` file.

Operations performed:
  1. Backup current race_common-derived ``ttl_*_xodr.csv`` to
     ``ttl_*_xodr_racecommon.csv`` (only if backup doesn't already exist).
  2. Read each ``ttl_*_xodr_og.csv``, translate xy by ``(+6.101, +50.761)``, leave z
     untouched, write to ``ttl_*_xodr.csv`` with header ``x,y,z``.
  3. Rename ``ttl_*_xodr_full.csv`` -> ``ttl_*_xodr_full_racecommon.csv`` so the
     loader's auto-pickup doesn't pair OLD-line waypoints with race_common's
     boundary columns (different physical lines -> bound-vs-waypoint index mismatch
     -> wrong corridor cost). The corridor MPC silently falls back to no-barrier
     mode, which is fine because the OLD lines don't need it.

Run from repo root:
    python tools/frames/restore_og_racing_lines.py
"""
from __future__ import annotations

import csv
import json
import shutil
from pathlib import Path
from typing import List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"
CALIB_JSON = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LGS_v1_gps_rd_calibration.json"

# (og source, active target). All four racing lines mapped together so left/right/pit
# stay in sync with optimal.
PAIRS: List[Tuple[str, str]] = [
    ("ttl_optimal_xodr_og.csv", "ttl_optimal_xodr.csv"),
    ("ttl_left_xodr_og.csv",    "ttl_left_xodr.csv"),
    ("ttl_right_xodr_og.csv",   "ttl_right_xodr.csv"),
    ("ttl_pit_xodr_og.csv",     "ttl_pit_xodr.csv"),
]
FULL_FILES = [
    "ttl_optimal_xodr_full.csv",
    "ttl_left_xodr_full.csv",
    "ttl_right_xodr_full.csv",
    "ttl_pit_xodr_full.csv",
]


def load_calibration_translation() -> Tuple[float, float]:
    """Return (tx, ty) for ``RD -> NEW_XODR`` (inverse of the stored XODR->RD)."""
    with open(CALIB_JSON) as f:
        d = json.load(f)
    if d.get("model") != "translation":
        raise ValueError(f"{CALIB_JSON.name} model={d.get('model')!r}; only 'translation' supported")
    t = d["translation_xy_xodr_to_rd"]  # XODR + t = RD
    # OG files are in RD frame. To get NEW XODR: NEW_XODR = RD - t = RD + (-tx, -ty)
    tx, ty = -float(t[0]), -float(t[1])
    return tx, ty


def translate_csv(src: Path, dst: Path, tx: float, ty: float) -> int:
    """Read 3-col ``x,y,z`` CSV, translate xy by ``(tx, ty)``, write to dst. Returns row count."""
    with open(src, "r", newline="") as f:
        rd = csv.reader(f)
        rows_in = list(rd)
    if not rows_in:
        raise ValueError(f"{src} is empty")
    # Detect header: row 0 has 'x' / 'X' as first cell
    header = rows_in[0]
    if header and ("x" in header[0].lower() or "X" in header[0]):
        data = rows_in[1:]
        out_header = header
    else:
        data = rows_in
        out_header = ["x", "y", "z"]

    out_rows: List[List[str]] = []
    n = 0
    for row in data:
        if len(row) < 2:
            continue
        try:
            x = float(row[0]); y = float(row[1])
        except ValueError:
            continue
        z = float(row[2]) if len(row) >= 3 else 0.0
        out_rows.append([f"{x + tx:.12f}", f"{y + ty:.12f}", f"{z:.12f}"])
        n += 1

    with open(dst, "w", newline="") as f:
        w = csv.writer(f)
        w.writerow(out_header)
        w.writerows(out_rows)
    return n


def main() -> int:
    tx, ty = load_calibration_translation()
    print(f"Translation RD -> NEW XODR: ({tx:+.3f}, {ty:+.3f})")
    print(f"  (inverse of {CALIB_JSON.name} translation_xy_xodr_to_rd)")
    print(f"TTL dir: {TTL_DIR}")
    print()

    # 1. Backup current ttl_*_xodr.csv to *_racecommon.csv (only if backup absent)
    print("Step 1: backup current race_common-derived 3-col files")
    for _og, active in PAIRS:
        active_path = TTL_DIR / active
        backup_path = TTL_DIR / active.replace(".csv", "_racecommon.csv")
        if not active_path.is_file():
            print(f"  SKIP {active}: not present")
            continue
        if backup_path.is_file():
            print(f"  SKIP {active}: backup {backup_path.name} already exists")
            continue
        shutil.copy2(active_path, backup_path)
        print(f"  copied {active} -> {backup_path.name}")
    print()

    # 2. Translate _og files into active slots
    print("Step 2: translate OG racing lines and overwrite active TTL CSVs")
    for og, active in PAIRS:
        og_path = TTL_DIR / og
        active_path = TTL_DIR / active
        if not og_path.is_file():
            print(f"  ERROR: {og} not found; skipping")
            continue
        n = translate_csv(og_path, active_path, tx, ty)
        # Show first row for sanity
        with open(active_path, "r") as f:
            rd = csv.reader(f)
            next(rd, None)  # header
            first = next(rd, None)
        print(f"  {og:30s} -> {active:25s}  ({n} rows; first xy = ({first[0][:9]}, {first[1][:9]}))")
    print()

    # 3. Rename *_full.csv -> *_full_racecommon.csv (preserve as reference)
    print("Step 3: rename race_common *_full.csv files to break loader auto-pickup")
    print("        (loader would otherwise pair OLD-line waypoints with race_common bounds)")
    for full in FULL_FILES:
        src = TTL_DIR / full
        dst = TTL_DIR / full.replace("_full.csv", "_full_racecommon.csv")
        if not src.is_file():
            print(f"  SKIP {full}: not present")
            continue
        if dst.is_file():
            print(f"  SKIP {full}: target {dst.name} already exists")
            continue
        src.rename(dst)
        print(f"  renamed {full} -> {dst.name}")
    print()

    print("Done. Next step: re-run F0 with --time 2000 and verify in-bounds visualization.")
    print("To revert: copy back the *_racecommon.csv to ttl_*_xodr.csv and rename")
    print("           ttl_*_xodr_full_racecommon.csv -> ttl_*_xodr_full.csv.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
