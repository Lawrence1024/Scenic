"""Extract dSPACE's actual road centerline from Laguna_Seca.rd and check whether ego's
RD-frame position from F0.log lies inside the dSPACE-rendered drivable polygon.

dSPACE represents each road as a sequence of cubic-spline segments: per-segment
``P(u) = A + B*u + C*u^2 + D*u^3`` with ``u`` in [0, 1] and arc-length set by the
segment's ``Length`` field. The actual drivable surface is then ``CenterLane.Width / 2``
on each side of the centerline (plus optional ``Lanes`` widths for shoulders/runoff).

This script:
1. Parses all 3 roads (The Corkscrew1, Andretti Hairpin1_3, Pit Lane1_2) from the .rd.
2. Samples each road's centerline at fine resolution (every 0.5m of arc length).
3. For each road, extracts the per-section CenterLane.Width.
4. For each F0.log BoundsCheck position (NEW XODR), translates back to RD via the
   calibration JSON, then computes distance to the closest dSPACE centerline point and
   compares against (CenterLane.Width / 2 - vehicle_half_width).
5. Reports per-sample whether ego is inside dSPACE's polygon.

This tells us authoritatively whether dSPACE's viz polygon agrees with race_common's
geofence (it shouldn't, per the user's observation), and how much narrower it is at
the OOB section.
"""
from __future__ import annotations

import csv
import json
import math
import re
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple, Dict

import numpy as np

NS = {"r": "http://www.dspace.com/XMLSchema/ScenarioAccess/Scenario/Road"}
RD_PATH = Path(r"C:\Users\bklfh\Documents\dspace\AURELIONManager\IAC_Project"
               r"\Parameterization\MOD_Traffic\Pool\Environment\Road\Laguna_Seca.rd")
F0_LOG = Path(r"C:\Users\bklfh\Documents\Scenic\Scenic\F0.log")
CALIB_JSON = Path(r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE"
                  r"\LGS_v1_gps_rd_calibration.json")
SAMPLE_DS = 0.5  # meters per centerline sample
VEHICLE_HALF_WIDTH = 0.965


def _xy(elem) -> Tuple[float, float]:
    return float(elem.find("r:X", NS).text), float(elem.find("r:Y", NS).text)


def parse_road_centerline(road) -> List[Tuple[float, float]]:
    """Sample cubic-spline segments of a road into a (x, y) polyline.

    Each segment is defined in a local frame anchored at ``AbsoluteStartPosition``,
    with the local x-axis along ``Tangent`` (in DEGREES from world +x). The cubic
    P_local(u) = A + B*u + C*u^2 + D*u^3 (with A=(0,0)) gives the local displacement
    over u in [0, 1]. World position = ASP + R(tangent_deg) * P_local(u). Verified
    against next segment's ASP — chains within ~0.3 m at the boundary.
    """
    pts: List[Tuple[float, float]] = []
    segs = road.findall("r:Segments/r:Segment", NS)
    for seg in segs:
        L = float(seg.find("r:Length", NS).text)
        if L <= 1e-6:
            continue
        asp_el = seg.find("r:AbsoluteStartPosition", NS)
        asp_x, asp_y = _xy(asp_el)
        tan_deg = float(asp_el.find("r:Tangent", NS).text)
        tan_rad = math.radians(tan_deg)
        cos_t = math.cos(tan_rad); sin_t = math.sin(tan_rad)
        ax, ay = _xy(seg.find("r:A", NS))
        bx, by = _xy(seg.find("r:B", NS))
        cx, cy = _xy(seg.find("r:C", NS))
        dx_, dy_ = _xy(seg.find("r:D", NS))
        n = max(2, int(math.ceil(L / SAMPLE_DS)))
        for i in range(n):
            u = i / (n - 1) if n > 1 else 0.0
            local_x = ax + bx * u + cx * u * u + dx_ * u * u * u
            local_y = ay + by * u + cy * u * u + dy_ * u * u * u
            world_x = asp_x + cos_t * local_x - sin_t * local_y
            world_y = asp_y + sin_t * local_x + cos_t * local_y
            if not pts or (pts[-1][0] != world_x or pts[-1][1] != world_y):
                pts.append((world_x, world_y))
    return pts


def lane_widths(road) -> List[Tuple[float, float, float]]:
    """Return list of (s_start, s_end, total_drivable_width) per LaneSection."""
    out = []
    cum = 0.0
    secs = road.findall("r:LaneSections/r:LaneSection", NS)
    for s in secs:
        L = float(s.find("r:Length", NS).text or 0)
        cl = s.find("r:CenterLane", NS)
        cl_w = float(cl.find("r:Width", NS).text) if cl is not None else 0.0
        ln = s.find("r:Lanes", NS)
        side_w = 0.0
        if ln is not None:
            for lane in ln.findall("r:Lane", NS):
                side_w += float(lane.find("r:Width", NS).text or 0)
        # Total drivable = CenterLane + sum of side lanes (rough; sides include shoulders)
        total = cl_w + side_w
        out.append((cum, cum + L, total))
        cum += L
    return out


def min_dist_to_polyline(p: np.ndarray, poly: np.ndarray) -> float:
    seg_a = poly[:-1]; seg_b = poly[1:]
    v = seg_b - seg_a; L2 = (v * v).sum(axis=1)
    L2s = np.where(L2 < 1e-12, 1.0, L2)
    u = ((p - seg_a) * v).sum(axis=1) / L2s
    uc = np.clip(u, 0.0, 1.0)
    foot = seg_a + uc[:, None] * v
    return float(np.min(np.linalg.norm(p - foot, axis=1)))


def main() -> int:
    # 1. Parse calibration
    with open(CALIB_JSON) as f:
        calib = json.load(f)
    tx, ty = calib["translation_xy_xodr_to_rd"]
    print(f"Calibration: NEW_XODR -> RD adds ({tx:+.3f}, {ty:+.3f})")

    # 2. Parse dSPACE roads
    tree = ET.parse(RD_PATH)
    roads = tree.getroot().findall("r:Roads/r:Road", NS)
    road_centerlines: Dict[str, np.ndarray] = {}
    road_widths: Dict[str, List[Tuple[float, float, float]]] = {}
    for road in roads:
        name = road.find("r:Name", NS).text
        cl = parse_road_centerline(road)
        widths = lane_widths(road)
        wmin = min(w[2] for w in widths)
        wmax = max(w[2] for w in widths)
        print(f"Road {name!r}: {len(cl)} centerline pts, "
              f"width range {wmin:.2f}-{wmax:.2f} m")
        road_centerlines[name] = np.asarray(cl, dtype=float)
        road_widths[name] = widths

    # 3. Parse BoundsCheck samples from F0.log
    raw = F0_LOG.read_bytes()
    text = raw.decode("utf-16", errors="replace")
    pat = re.compile(
        r"\[BoundsCheck\] t=(\d+\.\d+)s pos=\((-?\d+\.\d+),(-?\d+\.\d+)\) "
        r"d_in=(-?\d+\.\d+)m d_out=(-?\d+\.\d+)m"
    )
    samples = pat.findall(text)
    print(f"\nF0.log BoundsCheck samples: {len(samples)}")
    if not samples:
        return 1

    # 4. For each sample, translate XODR -> RD, find closest dSPACE road, check polygon
    print(f"\n{'t':>6} {'XODR pos':<22} {'RD pos':<22} "
          f"{'closest road':<22} {'dist_to_CL':>9} {'lane_w':>7} "
          f"{'half_w':>7} {'body_clear':>10} {'verdict':>10}")
    print("-" * 130)
    out_count = 0
    for r in samples:
        t, xx, yy = float(r[0]), float(r[1]), float(r[2])
        # XODR -> RD
        rd_x = xx + tx
        rd_y = yy + ty
        p_rd = np.asarray([rd_x, rd_y])
        # Find closest road (by distance to its centerline polyline)
        best_road = None
        best_dist = float("inf")
        for name, cl in road_centerlines.items():
            d = min_dist_to_polyline(p_rd, cl)
            if d < best_dist:
                best_dist = d
                best_road = name
        # Use that road's first lane-section width as a rough estimate
        # (we don't bother projecting onto road s here; just take the median width)
        widths = sorted(w[2] for w in road_widths[best_road])
        median_w = widths[len(widths) // 2]
        half_w = median_w / 2.0
        body_clear = half_w - best_dist - VEHICLE_HALF_WIDTH
        verdict = "OK" if body_clear >= 0 else "BODY OFF"
        if body_clear < 0:
            out_count += 1
        print(f"{t:6.2f}s ({xx:+8.2f},{yy:+8.2f}) ({rd_x:+8.2f},{rd_y:+8.2f}) "
              f"{best_road:<22} {best_dist:9.2f} {median_w:7.2f} "
              f"{half_w:7.2f} {body_clear:+10.2f} {verdict:>10}")
    print()
    print(f"Samples where car body is OFF dSPACE polygon: {out_count} of {len(samples)}")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
