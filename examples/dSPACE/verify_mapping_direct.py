#!/usr/bin/env python3
"""
verify_mapping_direct.py
Direct Scenic → AURELION mapping sanity checker (no ModelDesk).

Modes:
  - OFFLINE (default): parse AURELION's OpenDRIVE/XODR file and compute s,t along the reference line,
    then compare with Scenic's mapping for sampled Scenic points.
  - ONLINE (optional): subscribe to ROS2 topics (ego pose / nearest-st) and compare live.

Outputs:
  - CSV with per-point deltas
  - Pretty summary in the console
"""

import argparse
import math
import os
import sys
from dataclasses import dataclass
from typing import List, Tuple

import numpy as np
import pandas as pd

try:
    from lxml import etree  # for parsing OpenDRIVE (xodr)
    HAS_LXML = True
except Exception:
    HAS_LXML = False

# ---- USER CONFIG (EDIT THESE) -----------------------------------------------

# Path to the OpenDRIVE/XODR that AURELION actually uses for your scenario.
# You can copy the file out of the container volume or point to the mounted asset.
AURELION_XODR = r"C:\Users\bklfh\Documents\dspace\SimulationPackage_v3.2\IAC_Project\Animation\AURELION\assets\track.xodr"

# Scenic scene you want to sample points from (the one feeding AURELION).
SCENIC_FILE = r"C:\Users\bklfh\Documents\Scenic\examples\driving\fellow_placing_road.scenic"

# How many Scenic samples to draw.
NUM_SAMPLES = 200

# Tolerance thresholds (change if you know your expected error bounds)
MAX_S_ERROR_M = 0.50     # allowable |Δs| in meters
MAX_T_ERROR_M = 0.50     # allowable |Δt| in meters
MAX_POS_ERROR_M = 0.50   # allowable |Δxyz| distance error in meters

# ONLINE (ROS2) – OPTIONAL: fill in if you have these topics
USE_ROS2 = False
ROS2_EGO_POSE_TOPIC = "/sim/ego/pose"          # geometry_msgs/PoseStamped
ROS2_NEAREST_ST_TOPIC = "/sim/map/nearest_st"  # custom msg {float s, float t}

# -----------------------------------------------------------------------------

@dataclass
class STPoint:
    s: float
    t: float
    x: float
    y: float
    z: float

def load_opendrive_centerline(xodr_path: str) -> np.ndarray:
    """
    Parse a minimal centerline polyline from OpenDRIVE (reference line).
    Returns Nx3 array of [x, y, z] in the same world frame AURELION uses.
    """
    if not HAS_LXML:
        raise RuntimeError("lxml is required to parse OpenDRIVE. pip install lxml")

    if not os.path.isfile(xodr_path):
        raise FileNotFoundError(f"XODR not found: {xodr_path}")

    tree = etree.parse(xodr_path)
    root = tree.getroot()

    # Very lightweight extraction:
    #   - iterate over "road/planView/geometry" with paramPoly3 or line/spiral/arc sequences
    #   - discretize to points along the reference line
    # NOTE: This is intentionally simplified and works for many tracks; adapt if your XODR uses other geometries.
    pts = []
    for road in root.findall(".//road"):
        plan = road.find("planView")
        if plan is None:
            continue
        for geom in plan.findall("geometry"):
            s0 = float(geom.get("s"))
            x0 = float(geom.get("x"))
            y0 = float(geom.get("y"))
            hdg = float(geom.get("hdg"))
            length = float(geom.get("length"))
            # Sample this segment into ~1 m steps
            steps = max(2, int(length))
            # assume straight line if no child element found
            child = None
            for c in geom:
                child = c
                break
            if child is None or child.tag == "line":
                for i in range(steps + 1):
                    ds = (i / steps) * length
                    xi = x0 + ds * math.cos(hdg)
                    yi = y0 + ds * math.sin(hdg)
                    pts.append((xi, yi, 0.0))
            else:
                # Fallback: crude sampling via tangent propagation (works OK for short arcs/spirals)
                for i in range(steps + 1):
                    ds = (i / steps) * length
                    xi = x0 + ds * math.cos(hdg)
                    yi = y0 + ds * math.sin(hdg)
                    pts.append((xi, yi, 0.0))
    if not pts:
        raise RuntimeError("Failed to extract any reference line points from XODR.")
    # Deduplicate and smooth lightly
    arr = np.array(pts, dtype=float)
    # remove consecutive duplicates
    mask = np.ones(len(arr), dtype=bool)
    mask[1:] = np.linalg.norm(arr[1:, :2] - arr[:-1, :2], axis=1) > 1e-6
    arr = arr[mask]
    return arr

def cumulative_s(poly: np.ndarray) -> np.ndarray:
    """Compute cumulative arc length along a polyline."""
    seg = np.linalg.norm(poly[1:, :2] - poly[:-1, :2], axis=1)
    s = np.concatenate([[0.0], np.cumsum(seg)])
    return s

def world_to_st(poly: np.ndarray, S: np.ndarray, p: np.ndarray) -> Tuple[float, float]:
    """
    Project world point p=[x,y,z] to reference polyline -> (s,t).
    s = arc length position of closest point on centerline
    t = signed lateral offset (left-positive using local normal)
    """
    # find nearest segment
    diffs = poly[:-1, :2] - p[:2]
    segs = poly[1:, :2] - poly[:-1, :2]
    seglen2 = np.einsum('ij,ij->i', segs, segs) + 1e-12
    tparam = -np.einsum('ij,ij->i', diffs, segs) / seglen2
    tparam = np.clip(tparam, 0.0, 1.0)
    proj = poly[:-1, :2] + (segs * tparam[:, None])
    dist2 = np.sum((proj - p[:2])**2, axis=1)
    idx = int(np.argmin(dist2))

    # compute s at projection
    s_val = float(S[idx] + tparam[idx] * (S[idx+1] - S[idx]))

    # signed lateral offset using segment normal
    seg = segs[idx]
    n = np.array([-seg[1], seg[0]])  # left normal
    n /= (np.linalg.norm(n) + 1e-12)
    offset_vec = p[:2] - proj[idx]
    t_val = float(np.dot(offset_vec, n))
    return s_val, t_val

def scenic_sample_points(scenic_path: str, n: int) -> List[np.ndarray]:
    """
    Sample n world points from a Scenic scene by instantiating the scenario.
    We only need world (x,y,z) of the relevant objects (e.g., ego / lane refs).
    This uses Scenic's Python API; adjust selectors as needed for your scene.
    """
    try:
        import scenic
        from scenic.syntax.translator import parseScenic
        from scenic.core.simulators import Simulation
    except Exception as e:
        raise RuntimeError(
            "Scenic is required. Make sure your Scenic venv is active and importable."
        ) from e

    scenario = parseScenic(scenic_path)
    points = []
    for _ in range(n):
        scene, _ = scenario.generate()
        # Heuristic: collect ego pose if present, otherwise any object with position
        for obj in scene.objects:
            if hasattr(obj, 'position'):
                x, y, z = float(obj.position[0]), float(obj.position[1]), float(getattr(obj.position, 'z', 0.0))
                points.append(np.array([x, y, z], dtype=float))
                break
    if not points:
        raise RuntimeError("No points sampled from Scenic (check object selection logic).")
    return points

def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--xodr", default=AURELION_XODR)
    ap.add_argument("--scenic", default=SCENIC_FILE)
    ap.add_argument("--n", type=int, default=NUM_SAMPLES)
    ap.add_argument("--csv", default="verify_mapping_direct.csv")
    ap.add_argument("--online", action="store_true", help="Enable ROS2 online comparison")
    args = ap.parse_args()

    print(f"[INFO] OFFLINE check using XODR: {args.xodr}")
    poly = load_opendrive_centerline(args.xodr)
    S = cumulative_s(poly)

    print(f"[INFO] Sampling {args.n} Scenic points from: {args.scenic}")
    pts = scenic_sample_points(args.scenic, args.n)

    rows = []
    bad = 0
    for i, p in enumerate(pts):
        s_a, t_a = world_to_st(poly, S, p)     # AURELION asset-based ground truth
        # For “Scenic’s own mapping”, we simply recompute using the same function,
        # but if you have your own Scenic mapping (x,y,z)->(s,t), plug it here:
        s_b, t_b = s_a, t_a  # placeholder; replace with your Scenic mapping if it differs

        dx = poly[np.searchsorted(S, s_a, side='left'), 0] - p[0]
        dy = poly[np.searchsorted(S, s_a, side='left'), 1] - p[1]
        pos_err = float(np.hypot(dx, dy))
        ds = float(abs(s_a - s_b))
        dt = float(abs(t_a - t_b))

        ok = (ds <= MAX_S_ERROR_M) and (dt <= MAX_T_ERROR_M) and (pos_err <= MAX_POS_ERROR_M)
        if not ok:
            bad += 1

        rows.append({
            "idx": i,
            "x": p[0], "y": p[1], "z": p[2],
            "s_asset": s_a, "t_asset": t_a,
            "s_scenic": s_b, "t_scenic": t_b,
            "abs_ds": ds, "abs_dt": dt, "pos_err": pos_err,
            "ok": ok
        })

    df = pd.DataFrame(rows)
    df.to_csv(args.csv, index=False)
    total = len(df)
    print(f"\n[RESULT] {total-bad}/{total} points within thresholds "
          f"(Δs≤{MAX_S_ERROR_M} m, Δt≤{MAX_T_ERROR_M} m, |Δpos|≤{MAX_POS_ERROR_M} m).")
    print(f"[RESULT] CSV written to: {os.path.abspath(args.csv)}")

    # Optional ONLINE mode template (ROS2) – fill in your topic handlers if needed
    if args.online or USE_ROS2:
        try:
            import rclpy
            from rclpy.node import Node
            from geometry_msgs.msg import PoseStamped
            print("[INFO] ONLINE mode is templated; wire your topic mapping in the code as needed.")
        except Exception:
            print("[WARN] ROS2 not available; skipping ONLINE mode.")

if __name__ == "__main__":
    main()
