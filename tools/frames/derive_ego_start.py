"""Derive new-frame ego start positions for the F-bank by arc-length matching on TTLs.

Old F-bank scenarios hardcode two ego starting (x, y) values that were authored against the
old TTL/XODR frame:
    (-78.86454576530903, -112.41203639782893)  # F0-F9, start/finish straight area
    (146.6773, -311.8879)                       # F10-F12, corner-entry section

After the TTL frame change, those exact xy values land ~19.5 m off the new optimal racing line.
This script:
  1. Projects each old (x, y) onto the OLD optimal TTL (ttl_optimal_xodr_og.csv) to get its
     arc-length s on that polyline.
  2. Samples the NEW optimal TTL (ttl_optimal_xodr.csv) at the same s, giving the new-frame
     (x, y) of the same lap-position.

This preserves racing semantics (same point in the lap, just expressed in the new frame)
and produces an auditable mapping for the commit message.

Run from repo root:
    python tools/frames/derive_ego_start.py
"""
from __future__ import annotations

import csv
from pathlib import Path
from typing import List, Tuple

import numpy as np

REPO_ROOT = Path(__file__).resolve().parents[2]
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"

OLD_TTL = TTL_DIR / "ttl_optimal_xodr_og.csv"
NEW_TTL = TTL_DIR / "ttl_optimal_xodr.csv"

# Old hardcoded ego start positions in F-bank scenarios.
OLD_STARTS: List[Tuple[str, Tuple[float, float]]] = [
    ("F0-F9 (start/finish straight)", (-78.86454576530903, -112.41203639782893)),
    ("F10-F12 (corner-entry)",        ( 146.6773,            -311.8879)),
]


def load_xy(csv_path: Path) -> np.ndarray:
    """Load an (x,y,z) TTL CSV; return Nx2 array of xy points (z dropped)."""
    pts: List[Tuple[float, float]] = []
    with open(csv_path, "r", encoding="utf-8") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        if header is None or [h.strip().lower() for h in header[:2]] != ["x", "y"]:
            raise ValueError(f"{csv_path}: expected header 'x,y[,z]'; got {header!r}")
        for row in reader:
            if len(row) < 2:
                continue
            try:
                pts.append((float(row[0]), float(row[1])))
            except ValueError:
                continue
    arr = np.asarray(pts, dtype=float)
    if arr.shape[0] < 2:
        raise ValueError(f"{csv_path}: need >=2 rows, got {arr.shape[0]}")
    return arr


def cumulative_arclength(pts: np.ndarray) -> np.ndarray:
    """Return arc-length s at each polyline vertex (length N, s[0]=0)."""
    seg = np.linalg.norm(np.diff(pts, axis=0), axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return s


def project_point_to_polyline(p: np.ndarray, pts: np.ndarray, s: np.ndarray) -> Tuple[float, float, int]:
    """Project xy point p onto polyline pts; return (s_proj, lateral_signed, segment_index).

    s_proj is the arc-length of the closest foot on the polyline.
    lateral_signed: positive = left of direction of travel, negative = right.
    """
    best_d2 = np.inf
    best_s = 0.0
    best_lat = 0.0
    best_idx = 0
    for i in range(len(pts) - 1):
        a, b = pts[i], pts[i + 1]
        v = b - a
        L2 = float(v @ v)
        if L2 <= 1e-12:
            continue
        u = float((p - a) @ v) / L2
        u_c = max(0.0, min(1.0, u))
        foot = a + u_c * v
        d_vec = p - foot
        d2 = float(d_vec @ d_vec)
        if d2 < best_d2:
            best_d2 = d2
            best_s = s[i] + u_c * float(np.linalg.norm(v))
            # Signed lateral: cross product z-component of (v, p-a) gives sign.
            cross_z = v[0] * (p[1] - a[1]) - v[1] * (p[0] - a[0])
            best_lat = float(np.copysign(np.sqrt(d2), cross_z)) if d2 > 0 else 0.0
            best_idx = i
    return best_s, best_lat, best_idx


def sample_polyline_at_s(pts: np.ndarray, s: np.ndarray, s_target: float) -> Tuple[float, float]:
    """Sample polyline xy at arc-length s_target via linear interpolation between vertices."""
    s_total = float(s[-1])
    s_clamped = max(0.0, min(s_total, float(s_target)))
    # Find the segment containing s_clamped.
    idx = int(np.searchsorted(s, s_clamped, side="right")) - 1
    idx = max(0, min(idx, len(pts) - 2))
    seg_len = s[idx + 1] - s[idx]
    if seg_len <= 1e-12:
        return float(pts[idx, 0]), float(pts[idx, 1])
    u = (s_clamped - s[idx]) / seg_len
    a, b = pts[idx], pts[idx + 1]
    p = a + u * (b - a)
    return float(p[0]), float(p[1])


def main() -> int:
    print(f"Repo root: {REPO_ROOT}")
    print(f"OLD TTL: {OLD_TTL.name}  ({'exists' if OLD_TTL.exists() else 'MISSING'})")
    print(f"NEW TTL: {NEW_TTL.name}  ({'exists' if NEW_TTL.exists() else 'MISSING'})")
    if not OLD_TTL.exists() or not NEW_TTL.exists():
        return 1

    old_pts = load_xy(OLD_TTL)
    new_pts = load_xy(NEW_TTL)
    s_old = cumulative_arclength(old_pts)
    s_new = cumulative_arclength(new_pts)
    print(f"OLD TTL: {len(old_pts)} pts, total arc-length {s_old[-1]:.2f} m")
    print(f"NEW TTL: {len(new_pts)} pts, total arc-length {s_new[-1]:.2f} m")
    if abs(s_old[-1] - s_new[-1]) > 5.0:
        print(f"  WARN: arc-lengths differ by {abs(s_old[-1] - s_new[-1]):.2f} m "
              f"(>5 m). Frame may have changed scale/topology, not just translation.")
    print()

    print("Per-position derivation:")
    print("-" * 78)
    for label, (x_old, y_old) in OLD_STARTS:
        p = np.asarray([x_old, y_old], dtype=float)
        s_proj, lat, seg_idx = project_point_to_polyline(p, old_pts, s_old)
        x_new, y_new = sample_polyline_at_s(new_pts, s_new, s_proj)
        print(f"{label}")
        print(f"  OLD xy:           ({x_old: .6f}, {y_old: .6f})")
        print(f"  Projected on OLD: s = {s_proj:.4f} m, lateral = {lat:+.4f} m  (seg #{seg_idx})")
        print(f"  Sampled NEW @ s:  ({x_new: .6f}, {y_new: .6f})")
        print(f"  Replace `at ({x_old}, {y_old})` -> `at ({x_new}, {y_new})`")
        print()

    print("Note: lateral != 0 means the OLD ego start was offset from the OLD optimal TTL")
    print("      (e.g. on the centerline, not the racing line). The same lateral offset is")
    print("      not preserved here -- we only sample the NEW racing line at the same s.")
    print("      If your scenarios depend on the lateral offset, adjust manually.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
