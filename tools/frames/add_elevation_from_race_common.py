"""Backfill the z column of our XODR-frame TTL files with real elevation from race_common.

Background
----------
Our ttl_*_xodr.csv files have a z column but it's near-zero (range ~0.06 m across
the entire Laguna Seca lap). The longitudinal MPC has full grade-compensation
infrastructure (mpc_longitudinal.py:399 applies gravity_force = mass * g * sin(grade))
that reads grade from waypoint dz, so feeding it flat z means gravity compensation
is effectively a no-op even though the Corkscrew alone drops 18 m.

race_common ships the real elevation profile: src/external/common/race_metadata/ttls/
LS_ENU_TTL_CSV/ttl_17.csv (the optimal racing line) is in 20-column format with
x,y,z,yaw,curvature_radius,...,lon_vel,... columns. Z range -5.33 to +49.54 m
(54.86 m span) -- this is what the longitudinal MPC needs.

race_common's ENU origin (36.5869133, -121.7559026, 231.9349051) matches LGS_v1.xodr's
<geoReference>, so the (x, y) values are directly compatible with our XODR-xy frame
(sub-meter projection difference). We can nearest-neighbor lookup race_common (x, y)
to find local elevation, then write it as the z column of our TTLs.

This script touches only data assets (no code), is idempotent, and can be re-run if
race_common's TTL changes.

Usage (from repo root):
    python tools/frames/add_elevation_from_race_common.py [--dry-run]

Outputs:
    Rewrites assets/ttls/LS_ENU_TTL_CSV/ttl_optimal_xodr.csv
                                       ttl_left_xodr.csv
                                       ttl_right_xodr.csv
                                       ttl_main_road_xodr.csv
                                       ttl_pitlane_xodr.csv
    (with new z values; x,y unchanged; format remains 3-column x,y,z)
"""
from __future__ import annotations

import argparse
import csv
import math
from pathlib import Path
from typing import List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"

# WSL path (race_common ships in linux-side home). User runs this from Windows / git
# bash, so we need WSL to access it. We'll write a small WSL invocation to read the
# file, or document that it's been copied locally first.
RACE_COMMON_TTL_WSL = (
    "/home/bklfh/ros_ws/race_common/src/external/common/race_metadata/ttls/"
    "LS_ENU_TTL_CSV/ttl_17.csv"
)
# Optional local copy (if user has staged it, skip the WSL hop)
RACE_COMMON_TTL_LOCAL_CANDIDATES = [
    REPO_ROOT / "tools" / "frames" / "data" / "race_common_ttl_17.csv",
    REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "_race_common_ttl_17_full.csv",
]

# Files to update (relative to TTL_DIR)
TTL_FILES_TO_UPDATE = [
    "ttl_optimal_xodr.csv",
    "ttl_left_xodr.csv",
    "ttl_right_xodr.csv",
    "ttl_main_road_xodr.csv",
    "ttl_pitlane_xodr.csv",
]


def read_race_common_ttl(path: Path) -> List[Tuple[float, float, float]]:
    """Parse race_common ttl_17.csv.

    Format: 2 header lines (track metadata), then per-row 20+ columns:
        col 0: X (ENU East, m)
        col 1: Y (ENU North, m)
        col 2: Z (Up, m relative to GPS origin)
        col 3..: yaw, curvature_radius, ..., lon_vel, ..., bounds

    We only need (x, y, z) for elevation backfill.
    """
    pts: List[Tuple[float, float, float]] = []
    with open(path, "r", encoding="utf-8") as f:
        lines = f.readlines()
    for ln in lines[2:]:
        parts = ln.strip().split(",")
        if len(parts) < 3:
            continue
        try:
            x = float(parts[0])
            y = float(parts[1])
            z = float(parts[2])
            pts.append((x, y, z))
        except ValueError:
            continue
    return pts


def find_local_race_common_ttl() -> Path:
    """Try to find a local copy of race_common's ttl_17.csv before falling back to WSL."""
    for cand in RACE_COMMON_TTL_LOCAL_CANDIDATES:
        if cand.exists():
            return cand
    raise FileNotFoundError(
        f"No local copy found. Either:\n"
        f"  1) Run from WSL with the path {RACE_COMMON_TTL_WSL}, OR\n"
        f"  2) Stage a local copy at one of:\n"
        + "\n".join(f"     {p}" for p in RACE_COMMON_TTL_LOCAL_CANDIDATES) +
        f"\n\nTo stage from WSL:\n"
        f"  wsl cp {RACE_COMMON_TTL_WSL} \\\n"
        f"      <repo>/{RACE_COMMON_TTL_LOCAL_CANDIDATES[0].relative_to(REPO_ROOT).as_posix()}"
    )


def nearest_z(x: float, y: float, ref: List[Tuple[float, float, float]]) -> float:
    """Nearest-neighbor lookup. O(N) per query — fine for one-shot tool use."""
    best_d2 = float("inf")
    best_z = 0.0
    for rx, ry, rz in ref:
        dx = rx - x
        dy = ry - y
        d2 = dx * dx + dy * dy
        if d2 < best_d2:
            best_d2 = d2
            best_z = rz
    return best_z


def update_ttl_file(path: Path, ref: List[Tuple[float, float, float]],
                    dry_run: bool = False) -> Tuple[int, float, float, float]:
    """Read x,y,z from path; replace z with nearest race_common z; write back.

    Returns: (n_rows, z_min_new, z_max_new, max_lookup_dist_m).
    """
    if not path.exists():
        return (0, 0.0, 0.0, 0.0)
    rows: List[Tuple[float, float, float]] = []
    with open(path, "r", encoding="utf-8", newline="") as f:
        rdr = csv.DictReader(f)
        for r in rdr:
            try:
                rows.append((float(r["x"]), float(r["y"]), float(r["z"])))
            except (KeyError, ValueError):
                continue

    if not rows:
        return (0, 0.0, 0.0, 0.0)

    new_rows: List[Tuple[float, float, float]] = []
    max_dist = 0.0
    for x, y, _z_old in rows:
        # Track max nearest-neighbor distance for sanity (should be small if our
        # waypoints lie near race_common's racing line).
        best_d2 = float("inf")
        best_z = 0.0
        for rx, ry, rz in ref:
            dx = rx - x
            dy = ry - y
            d2 = dx * dx + dy * dy
            if d2 < best_d2:
                best_d2 = d2
                best_z = rz
        d_m = math.sqrt(best_d2)
        if d_m > max_dist:
            max_dist = d_m
        new_rows.append((x, y, best_z))

    z_vals = [r[2] for r in new_rows]
    z_min, z_max = min(z_vals), max(z_vals)

    if not dry_run:
        with open(path, "w", encoding="utf-8", newline="") as f:
            w = csv.writer(f)
            w.writerow(["x", "y", "z"])
            for x, y, z in new_rows:
                w.writerow([f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])

    return (len(new_rows), z_min, z_max, max_dist)


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--dry-run", action="store_true",
                    help="Don't write; just report what would change")
    ap.add_argument("--race-common-ttl", default=None, type=Path,
                    help="Override path to race_common ttl_17.csv (default: try local copies)")
    args = ap.parse_args()

    src_path = args.race_common_ttl if args.race_common_ttl else find_local_race_common_ttl()
    print(f"Reading race_common reference: {src_path}")
    ref = read_race_common_ttl(src_path)
    if not ref:
        print(f"  ERROR: no rows parsed from {src_path}")
        return 1
    z_ref = [r[2] for r in ref]
    print(f"  {len(ref)} reference points, z range [{min(z_ref):.2f}, {max(z_ref):.2f}] m "
          f"(span {max(z_ref) - min(z_ref):.2f} m)")
    print()

    print(f"Updating TTL files in {TTL_DIR}:")
    for fname in TTL_FILES_TO_UPDATE:
        path = TTL_DIR / fname
        n, zmin, zmax, max_d = update_ttl_file(path, ref, dry_run=args.dry_run)
        if n == 0:
            print(f"  {fname:30s} SKIP (file missing or empty)")
            continue
        action = "[DRY-RUN]" if args.dry_run else "[WROTE]"
        print(f"  {fname:30s} {action} n={n:>5d}  z=[{zmin:6.2f}, {zmax:6.2f}] m  "
              f"max_lookup_dist={max_d:.2f} m")

    print()
    if args.dry_run:
        print("Dry-run only. Re-run without --dry-run to apply.")
    else:
        print("Done. Verify by running F2_tactical -- grade_profile in MPC should now be non-zero.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
