"""Post-process the centerline-calibration drive log into LGS_v1 centerline CSVs.

Input:
    tools/frames/data/lgs_v1_centerline_drive.csv  (from measure_lgs_v1_centerline.scenic)

Outputs (RD-frame x,y,z polylines, format identical to ttl_main_road.csv):
    tools/frames/data/lgs_v1_centerline_main.csv  (R2 d=0 fellow trajectory)
    tools/frames/data/lgs_v1_centerline_pit.csv   (R1 d=0 fellow trajectory)

Reports a 3-way comparison:
    new_empirical (this run, LGS_v1 dSPACE project)
    old_empirical (assets/ttls/LS_ENU_TTL_CSV/ttl_main_road.csv, pre-LGS_v1 vintage)
    xodr_derived  (assets/ttls/LS_ENU_TTL_CSV/ttl_main_road_xodr.csv, Phase A.2)

Decision criteria printed at the end:
    - new_empirical ~= old_empirical (mean < 0.5 m, max < 2 m): old empirical was
      still valid for LGS_v1 -> safe to keep current placement; B6 swap to XODR
      only requires a constant s-offset.
    - new_empirical diverges from old_empirical: refresh empirical files using
      lgs_v1_centerline_*.csv before any further B6 work.
"""
from __future__ import annotations

import argparse
import csv
import math
from collections import defaultdict
from pathlib import Path
from typing import List, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
DATA_DIR = REPO_ROOT / "tools" / "frames" / "data"
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"

DRIVE_CSV = DATA_DIR / "lgs_v1_centerline_drive.csv"
OUT_MAIN = DATA_DIR / "lgs_v1_centerline_main.csv"
OUT_PIT = DATA_DIR / "lgs_v1_centerline_pit.csv"
OLD_MAIN = TTL_DIR / "ttl_main_road.csv"
OLD_PIT = TTL_DIR / "ttl_pitlane.csv"
XODR_MAIN = TTL_DIR / "ttl_main_road_xodr.csv"
XODR_PIT = TTL_DIR / "ttl_pitlane_xodr.csv"

# Calibration translation: RD = XODR + (RD_OFFSET); per project_frame_calibration memory.
XODR_TO_RD = (-6.101, -50.761)


def read_drive_csv(path: Path) -> dict:
    """Group per-step rows by race_number; return {race_number: list of (t, route, d, x, y, yaw, v)}."""
    if not path.exists():
        raise FileNotFoundError(
            f"Drive log not found at {path}. Run the scenario first:\n"
            f"  scenic examples/racing/calibration/measure_lgs_v1_centerline.scenic --2d "
            f"--model scenic.simulators.dspace.racing_model --simulate --time 20000"
        )
    by_id: dict = defaultdict(list)
    with open(path, "r", encoding="utf-8", newline="") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            try:
                t = float(row["sim_t"])
                rn = int(row["race_number"]) if row["race_number"] else -1
                route = row["route"]
                d = float(row["d_setpoint_m"])
                x = float(row["x_rd"])
                y = float(row["y_rd"])
                yaw = float(row["yaw_rad"])
                v = float(row["speed_mps"])
            except (KeyError, ValueError):
                continue
            by_id[rn].append((t, route, d, x, y, yaw, v))
    return by_id


def read_xy_csv(path: Path) -> List[Tuple[float, float]]:
    pts: List[Tuple[float, float]] = []
    if not path.exists():
        return pts
    with open(path, "r", encoding="utf-8", newline="") as f:
        rdr = csv.DictReader(f)
        for row in rdr:
            try:
                pts.append((float(row["x"]), float(row["y"])))
            except (KeyError, ValueError):
                continue
    return pts


def write_xy_csv(path: Path, pts: List[Tuple[float, float]], z: float = 0.0) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        for x, y in pts:
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])


def thin_by_arclength(rows: List[tuple], min_step: float = 0.5) -> List[Tuple[float, float]]:
    """Keep only rows whose (x,y) is at least min_step from the previous kept point."""
    out: List[Tuple[float, float]] = []
    for r in rows:
        x, y = r[3], r[4]
        if not out:
            out.append((x, y))
            continue
        px, py = out[-1]
        if math.hypot(x - px, y - py) >= min_step:
            out.append((x, y))
    return out


def min_dist_polyline(px: float, py: float, line: List[Tuple[float, float]]) -> float:
    if not line or len(line) < 2:
        return float("inf")
    best = float("inf")
    for i in range(len(line) - 1):
        x0, y0 = line[i]
        x1, y1 = line[i + 1]
        dx, dy = x1 - x0, y1 - y0
        ll = dx * dx + dy * dy
        if ll < 1e-18:
            d = math.hypot(px - x0, py - y0)
        else:
            t = max(0.0, min(1.0, ((px - x0) * dx + (py - y0) * dy) / ll))
            d = math.hypot(px - (x0 + t * dx), py - (y0 + t * dy))
        if d < best:
            best = d
    return best


def hausdorff_summary(a: List[Tuple[float, float]], b: List[Tuple[float, float]]) -> dict:
    """One-sided distances from a -> b, sampled at ~5m on `a`."""
    if not a or not b or len(b) < 2:
        return {"n": 0, "mean": float("nan"), "p50": float("nan"),
                "p95": float("nan"), "max": float("nan")}
    sampled: List[Tuple[float, float]] = []
    last = None
    for x, y in a:
        if last is None or math.hypot(x - last[0], y - last[1]) >= 5.0:
            sampled.append((x, y))
            last = (x, y)
    dists = [min_dist_polyline(x, y, b) for (x, y) in sampled]
    dists_sorted = sorted(dists)
    n = len(dists_sorted)

    def pct(p: float) -> float:
        if n == 0:
            return float("nan")
        i = max(0, min(n - 1, int(round(p * (n - 1)))))
        return dists_sorted[i]
    return {
        "n": n,
        "mean": sum(dists_sorted) / max(1, n),
        "p50": pct(0.5),
        "p95": pct(0.95),
        "max": dists_sorted[-1],
    }


def fmt_summary(label: str, s: dict) -> str:
    return (
        f"  {label:<28s} n={s['n']:>4d}  "
        f"mean={s['mean']:6.3f} m  p50={s['p50']:6.3f} m  "
        f"p95={s['p95']:6.3f} m  max={s['max']:6.3f} m"
    )


def translate(line: List[Tuple[float, float]], dx: float, dy: float) -> List[Tuple[float, float]]:
    return [(x + dx, y + dy) for (x, y) in line]


def find_d0_fellow(by_id: dict, route: str, tolerance: float = 0.5) -> int:
    """Return race_number of the d~=0 fellow on the given route, or -1."""
    best_id = -1
    best_d = float("inf")
    for rn, rows in by_id.items():
        if not rows:
            continue
        # mode-d (most common d_setpoint)
        d_vals = [r[2] for r in rows]
        d_med = sorted(d_vals)[len(d_vals) // 2]
        d_route = rows[len(rows) // 2][1]
        if d_route != route:
            continue
        if abs(d_med) < best_d and abs(d_med) < tolerance:
            best_d = abs(d_med)
            best_id = rn
    return best_id


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__,
                                 formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--drive-csv", default=str(DRIVE_CSV), type=Path)
    ap.add_argument("--out-main", default=str(OUT_MAIN), type=Path)
    ap.add_argument("--out-pit", default=str(OUT_PIT), type=Path)
    ap.add_argument("--thin-step", default=0.5, type=float,
                    help="min spacing (m) between consecutive output points")
    ap.add_argument("--skip-warmup-s", default=4.0, type=float,
                    help="drop the first N seconds of each fellow's data")
    args = ap.parse_args()

    print(f"[Calibration] Reading {args.drive_csv}")
    by_id = read_drive_csv(args.drive_csv)
    print(f"  parsed {sum(len(v) for v in by_id.values())} rows from "
          f"{len(by_id)} fellow(s)")

    # Per-fellow summary so the user can sanity-check the run.
    print("\nPer-fellow summary (route, median d_setpoint, n_rows, "
          "first/last sim_t):")
    for rn in sorted(by_id):
        rows = by_id[rn]
        if not rows:
            continue
        rows = [r for r in rows if r[0] >= args.skip_warmup_s]
        if not rows:
            print(f"  rn={rn:>4d}  (all rows in warmup window)")
            continue
        d_vals = sorted(r[2] for r in rows)
        d_med = d_vals[len(d_vals) // 2]
        route_med = rows[len(rows) // 2][1]
        v_med = sorted(r[6] for r in rows)[len(rows) // 2]
        print(f"  rn={rn:>4d}  route={route_med:<5s}  d~={d_med:+.2f} m  "
              f"n={len(rows):>5d}  t=[{rows[0][0]:.1f}, {rows[-1][0]:.1f}] s  "
              f"v_med={v_med:.1f} m/s")

    main_rn = find_d0_fellow(by_id, "Lap")
    pit_rn = find_d0_fellow(by_id, "Pit")
    print(f"\nIdentified d=0 R2 (Lap) fellow: race_number={main_rn}")
    print(f"Identified d=0 R1 (Pit) fellow: race_number={pit_rn}")

    if main_rn < 0:
        print("WARNING: no d~=0 Lap fellow found; main centerline will not be written.")
    else:
        rows = [r for r in by_id[main_rn] if r[0] >= args.skip_warmup_s]
        main_pts = thin_by_arclength(rows, min_step=args.thin_step)
        write_xy_csv(args.out_main, main_pts)
        print(f"  -> wrote {args.out_main} ({len(main_pts)} points)")

    if pit_rn < 0:
        print("WARNING: no d~=0 Pit fellow found; pit centerline will not be written.")
    else:
        rows = [r for r in by_id[pit_rn] if r[0] >= args.skip_warmup_s]
        pit_pts = thin_by_arclength(rows, min_step=args.thin_step)
        write_xy_csv(args.out_pit, pit_pts)
        print(f"  -> wrote {args.out_pit} ({len(pit_pts)} points)")

    print("\n=== 3-way comparison: new (LGS_v1) vs old empirical vs XODR-derived ===")
    print("(Distances are one-sided: how far is each new-line point from the reference?)")

    if main_rn >= 0:
        rows = [r for r in by_id[main_rn] if r[0] >= args.skip_warmup_s]
        new_main = thin_by_arclength(rows, min_step=args.thin_step)
        old_main = read_xy_csv(OLD_MAIN)
        xodr_main_xodr = read_xy_csv(XODR_MAIN)
        xodr_main_rd = translate(xodr_main_xodr, *XODR_TO_RD)
        print(f"\n[Main / R2] new_empirical n={len(new_main)} (RD frame, this run)")
        print(f"           old_empirical n={len(old_main)} (RD frame, mtime pre-LGS_v1)")
        print(f"           xodr_derived  n={len(xodr_main_rd)} (XODR frame, +calibration -> RD)")
        s_old = hausdorff_summary(new_main, old_main)
        s_xodr = hausdorff_summary(new_main, xodr_main_rd)
        print(fmt_summary("new -> old_empirical:", s_old))
        print(fmt_summary("new -> xodr_derived (RD):", s_xodr))

    if pit_rn >= 0:
        rows = [r for r in by_id[pit_rn] if r[0] >= args.skip_warmup_s]
        new_pit = thin_by_arclength(rows, min_step=args.thin_step)
        old_pit = read_xy_csv(OLD_PIT)
        xodr_pit_xodr = read_xy_csv(XODR_PIT)
        xodr_pit_rd = translate(xodr_pit_xodr, *XODR_TO_RD)
        print(f"\n[Pit / R1] new_empirical n={len(new_pit)} (RD frame, this run)")
        print(f"          old_empirical n={len(old_pit)} (RD frame, mtime pre-LGS_v1)")
        print(f"          xodr_derived  n={len(xodr_pit_rd)} (XODR frame, +calibration -> RD)")
        s_old = hausdorff_summary(new_pit, old_pit)
        s_xodr = hausdorff_summary(new_pit, xodr_pit_rd)
        print(fmt_summary("new -> old_empirical:", s_old))
        print(fmt_summary("new -> xodr_derived (RD):", s_xodr))

    print("\nDecision rubric:")
    print("  - mean < 0.5 m AND p95 < 1.5 m AND max < 3 m -> reference is still valid for LGS_v1")
    print("  - else -> reference has drifted; do not B6-swap to it without further work")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
