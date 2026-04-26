"""Check whether ego/fellow trajectories from F0/F2 logs stay inside the race_common
track boundary (LS_ENU canonical frame) AND the new XODR-derived racing line.

Uses the inside.csv / outside.csv geofences pulled from race_common as ground truth.
Both are in the canonical ENU frame (matches LGS_v1.xodr <geoReference>).

Run from repo root:
    python tools/frames/check_ego_in_bounds.py
"""
from __future__ import annotations

import csv
import math
import re
from pathlib import Path
from typing import List, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"


def load_xy(p: Path) -> np.ndarray:
    """Load Easting/Northing or x/y columns; tolerate either header."""
    with open(p, "r", encoding="utf-8") as f:
        rd = csv.reader(f)
        header = next(rd)
        # Find x/y columns by name
        cols = [h.strip().lower() for h in header]
        if "easting" in cols and "northing" in cols:
            ix, iy = cols.index("easting"), cols.index("northing")
        elif "x" in cols and "y" in cols:
            ix, iy = cols.index("x"), cols.index("y")
        else:
            ix, iy = 0, 1
        pts = []
        for row in rd:
            if len(row) < 2:
                continue
            try:
                pts.append((float(row[ix]), float(row[iy])))
            except ValueError:
                continue
    return np.asarray(pts, dtype=float)


def min_dist_to_polyline(p: np.ndarray, poly: np.ndarray) -> float:
    """Min distance from point p (2,) to polyline poly (N, 2) — open polyline."""
    seg_a = poly[:-1]
    seg_b = poly[1:]
    v = seg_b - seg_a
    L2 = (v * v).sum(axis=1)
    L2_safe = np.where(L2 < 1e-12, 1.0, L2)
    u = ((p - seg_a) * v).sum(axis=1) / L2_safe
    u_c = np.clip(u, 0.0, 1.0)
    foot = seg_a + u_c[:, None] * v
    d = np.linalg.norm(p - foot, axis=1)
    d = np.where(L2 < 1e-12, np.linalg.norm(p - seg_a, axis=1), d)
    return float(np.min(d))


def point_in_polygon(p: np.ndarray, poly: np.ndarray) -> bool:
    """Even-odd ray cast for polygon containment. poly is closed (N, 2), no need for repeated last vertex."""
    x, y = float(p[0]), float(p[1])
    n = len(poly)
    inside = False
    j = n - 1
    for i in range(n):
        xi, yi = poly[i]
        xj, yj = poly[j]
        if ((yi > y) != (yj > y)) and (x < (xj - xi) * (y - yi) / (yj - yi + 1e-30) + xi):
            inside = not inside
        j = i
    return inside


def parse_log(path: Path):
    """Extract ego/fellow trajectory snapshots from a UTF-16 log file."""
    raw = path.read_bytes()
    text = raw.decode("utf-16", errors="replace")
    init_pat = re.compile(r"Initialized: starting at \((-?\d+\.\d+),\s*(-?\d+\.\d+)\), heading=(-?\d+\.\d+)deg")
    ego_pat = re.compile(r"\[FollowRacingLineMPC ego\] t=(\d+\.\d+)s Step \d+: pos=\((-?\d+\.\d+),(-?\d+\.\d+)\)")
    fellow_pat = re.compile(r"\[FellowHarness\] t=(\d+\.\d+)s idx=\d+ speed_mps=(-?\d+\.\d+) x=(-?\d+\.\d+) y=(-?\d+\.\d+)")
    init = init_pat.search(text)
    egos = [(float(t), float(x), float(y)) for t, x, y in ego_pat.findall(text)]
    fellows = [(float(t), float(x), float(y), float(sp)) for t, sp, x, y in fellow_pat.findall(text)]
    init_pos = (float(init.group(1)), float(init.group(2))) if init else None
    init_hdg = float(init.group(3)) if init else None
    return init_pos, init_hdg, egos, fellows


def main() -> int:
    t_in = load_xy(TTL_DIR / "track_inside.csv")
    t_out = load_xy(TTL_DIR / "track_outside.csv")
    p_in = load_xy(TTL_DIR / "pit_inside.csv")
    p_out = load_xy(TTL_DIR / "pit_outside.csv")
    optimal = load_xy(TTL_DIR / "ttl_optimal_xodr.csv")

    print(f"Track  inside : {len(t_in)} pts, x in [{t_in[:,0].min():+.1f},{t_in[:,0].max():+.1f}], y in [{t_in[:,1].min():+.1f},{t_in[:,1].max():+.1f}]")
    print(f"Track outside : {len(t_out)} pts, x in [{t_out[:,0].min():+.1f},{t_out[:,0].max():+.1f}], y in [{t_out[:,1].min():+.1f},{t_out[:,1].max():+.1f}]")
    print(f"Pit   inside  : {len(p_in)} pts")
    print(f"Pit   outside : {len(p_out)} pts")
    print(f"Optimal TTL   : {len(optimal)} pts, x in [{optimal[:,0].min():+.1f},{optimal[:,0].max():+.1f}], y in [{optimal[:,1].min():+.1f},{optimal[:,1].max():+.1f}]")
    print()

    for fname in ("F0.log", "F2.log"):
        log_path = REPO_ROOT / fname
        if not log_path.is_file():
            print(f"--- {fname} : not found ---")
            continue
        init_pos, init_hdg, egos, fellows = parse_log(log_path)
        print(f"--- {fname} ---")
        if init_pos:
            p = np.asarray(init_pos)
            d_in = min_dist_to_polyline(p, t_in)
            d_out = min_dist_to_polyline(p, t_out)
            d_opt = min_dist_to_polyline(p, optimal)
            in_track = point_in_polygon(p, t_out) and not point_in_polygon(p, t_in)
            print(f"  Init pos: ({init_pos[0]:+.2f}, {init_pos[1]:+.2f})  hdg={init_hdg}deg")
            print(f"    dist_to_track_inside={d_in:6.2f}m  dist_to_track_outside={d_out:6.2f}m  "
                  f"dist_to_optimal_TTL={d_opt:5.2f}m  in_track={in_track}")
        print(f"  Ego trajectory ({len(egos)} samples):")
        for t, x, y in egos:
            p = np.asarray([x, y])
            d_in = min_dist_to_polyline(p, t_in)
            d_out = min_dist_to_polyline(p, t_out)
            d_opt = min_dist_to_polyline(p, optimal)
            in_track = point_in_polygon(p, t_out) and not point_in_polygon(p, t_in)
            mark = "OK" if in_track else "OUT"
            print(f"    t={t:5.2f}s pos=({x:+8.2f},{y:+8.2f}) "
                  f"d_in={d_in:5.2f}m d_out={d_out:5.2f}m d_opt={d_opt:5.2f}m  [{mark}]")
        if fellows:
            print(f"  Fellow trajectory ({len(fellows)} samples):")
            for t, x, y, sp in fellows[::5]:
                p = np.asarray([x, y])
                d_in = min_dist_to_polyline(p, t_in)
                d_out = min_dist_to_polyline(p, t_out)
                d_opt = min_dist_to_polyline(p, optimal)
                in_track = point_in_polygon(p, t_out) and not point_in_polygon(p, t_in)
                mark = "OK" if in_track else "OUT"
                print(f"    t={t:5.2f}s pos=({x:+8.2f},{y:+8.2f}) speed={sp:5.2f} "
                      f"d_in={d_in:5.2f}m d_out={d_out:5.2f}m d_opt={d_opt:5.2f}m  [{mark}]")
        print()
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
