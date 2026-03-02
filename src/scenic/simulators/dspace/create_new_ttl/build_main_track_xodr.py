#!/usr/bin/env python3
"""
Build a new OpenDRIVE file for the main track only, using the TTL centerline
(ttl_fellow_test_xodr_all.csv) as the reference line and lane widths from the
existing Laguna Seca XODR. Pit lane is ignored.

Output: one road, one lane section, left/right lanes with widths sampled from
the source XODR at the closest reference-line position for each TTL point.

Run from Scenic repo root:
  python src/scenic/simulators/dspace/create_new_ttl/build_main_track_xodr.py [--output path] [--xodr path] [--ttl path]
"""

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

# Scenic repo root and path setup
_CREATE_NEW_TTL = Path(__file__).resolve().parent
REPO_ROOT = _CREATE_NEW_TTL.parent.parent.parent.parent.parent
if str(REPO_ROOT / "src") not in sys.path:
    sys.path.insert(0, str(REPO_ROOT / "src"))
if str(REPO_ROOT) not in sys.path:
    sys.path.insert(0, str(REPO_ROOT))

# Main track road names in LagunaSeca.xodr (exclude pit and junctions)
MAIN_TRACK_ROAD_NAMES = ("The Corkscrew1", "Andretti Hairpin1_3")


def load_ttl(path: Path) -> List[Tuple[float, float]]:
    """Load (x, y) from TTL CSV. Returns list of (x, y)."""
    points = []
    with open(path, newline="") as f:
        reader = csv.reader(f)
        header = next(reader, None)
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                points.append((float(row[0]), float(row[1])))
            except (ValueError, IndexError):
                continue
    return points


def _sample_main_track_ref(xodr_path: Path, ds: float = 0.5) -> List[Tuple[float, float, float, str]]:
    """Sample reference line (planView) for each main-track road. Returns (x, y, s, road_name)."""
    root = ET.parse(xodr_path).getroot()
    ns = "" if not root.tag.startswith("{") else root.tag.split("}")[0] + "}"
    from scenic.simulators.dspace.geometry.xodr_parser import _road_local_ref

    ref_points = []
    for road in root.findall(f"{ns}road"):
        rn = (road.get("name") or "").strip()
        if rn not in MAIN_TRACK_ROAD_NAMES:
            continue
        road_id = road.get("id")
        if not road_id:
            continue
        pts, length, _ = _road_local_ref(root, road_id, ns, step=ds)
        for s_global, x, y, z, h in pts:
            s_local = min(s_global, length)
            ref_points.append((float(x), float(y), s_local, rn))
    return ref_points


def sample_ref_and_get_widths(
    xodr_path: Path,
    ttl_points: List[Tuple[float, float]],
) -> Tuple[List[float], List[float], List[float]]:
    """
    For each TTL point, find closest point on main-track reference lines,
    get that road's s, then left/right width at s.
    Returns (s_new_list, w_left_list, w_right_list) where s_new is cumulative
    distance along TTL (0 to total length).
    """
    from tools.verify_road_edges_overlap import get_xodr_lane_widths_at_s

    # Build combined ref points: (x, y, s, road_name)
    ref_points = _sample_main_track_ref(xodr_path, ds=0.5)
    for rx, ry, rs, rname in ref_points[:3]:  # ensure we have (x,y,s,road_name)
        pass

    if not ref_points:
        raise RuntimeError("No reference points from main track roads")

    s_new_list = []
    w_left_list = []
    w_right_list = []
    cum_s = 0.0
    for i in range(len(ttl_points)):
        x, y = ttl_points[i]
        # Closest ref point
        best_dist = float("inf")
        best_s = 0.0
        best_road = MAIN_TRACK_ROAD_NAMES[0]
        for rx, ry, rs, rname in ref_points:
            d = math.hypot(x - rx, y - ry)
            if d < best_dist:
                best_dist = d
                best_s = rs
                best_road = rname
        w_left, w_right = get_xodr_lane_widths_at_s(str(xodr_path), best_road, best_s)
        # If source has only one side (e.g. right-only lane), split total width for symmetric track
        total = w_left + w_right
        if total > 0 and w_left <= 0:
            w_left = total / 2.0
            w_right = total / 2.0
        elif total > 0 and w_right <= 0:
            w_left = total / 2.0
            w_right = total / 2.0
        s_new_list.append(cum_s)
        w_left_list.append(w_left)
        w_right_list.append(w_right)
        if i + 1 < len(ttl_points):
            cum_s += math.hypot(
                ttl_points[i + 1][0] - x,
                ttl_points[i + 1][1] - y,
            )
    return s_new_list, w_left_list, w_right_list


def build_xodr(
    ttl_points: List[Tuple[float, float]],
    s_list: List[float],
    w_left: List[float],
    w_right: List[float],
    road_name: str = "MainTrack_FromTTL",
    width_sample_step: int = 25,
) -> ET.Element:
    """
    Build OpenDRIVE root element with one road: planView from TTL line segments,
    one lane section with left/right lanes and width records.
    """
    ns = ""
    root = ET.Element("OpenDRIVE")
    header = ET.SubElement(
        root, "header",
        revMajor="1", revMinor="6", name="MainTrack_TTL", date="", vendor="build_main_track_xodr",
        north="0", south="0", east="0", west="0",
    )
    road_len = s_list[-1] if s_list else 0.0
    if len(ttl_points) > 1:
        road_len += math.hypot(
            ttl_points[-1][0] - ttl_points[-2][0],
            ttl_points[-1][1] - ttl_points[-2][1],
        )
    road = ET.SubElement(root, "road", name=road_name, length=f"{road_len:.6f}", id="1", junction="-1")
    ET.SubElement(road, "type", s="0", type="rural")
    plan_view = ET.SubElement(road, "planView")
    cum_s = 0.0
    for i in range(len(ttl_points) - 1):
        x0, y0 = ttl_points[i]
        x1, y1 = ttl_points[i + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        hdg = math.atan2(y1 - y0, x1 - x0)
        geom = ET.SubElement(
            plan_view,
            "geometry",
            s=f"{cum_s:.6f}",
            x=f"{x0:.6f}",
            y=f"{y0:.6f}",
            hdg=f"{hdg:.6f}",
            length=f"{length:.6f}",
        )
        ET.SubElement(geom, "line")
        cum_s += length
    lanes_el = ET.SubElement(road, "lanes")
    lane_section = ET.SubElement(lanes_el, "laneSection", s="0")
    center = ET.SubElement(lane_section, "center")
    ET.SubElement(center, "lane", id="0", type="none", level="false")
    left_el = ET.SubElement(lane_section, "left")
    lane_left = ET.SubElement(left_el, "lane", id="1", type="driving", level="false")
    ET.SubElement(lane_left, "roadMark", sOffset="0", type="solid", weight="standard", color="standard", width="0.13")
    right_el = ET.SubElement(lane_section, "right")
    lane_right = ET.SubElement(right_el, "lane", id="-1", type="driving", level="false")
    ET.SubElement(lane_right, "roadMark", sOffset="0", type="solid", weight="standard", color="standard", width="0.13")
    # Width records: subsample to avoid huge XML (every width_sample_step points, plus 0 and end)
    n = len(s_list)
    indices = [0]
    for j in range(width_sample_step, n - 1, width_sample_step):
        indices.append(j)
    if n > 1:
        indices.append(n - 1)
    for idx in indices:
        s_val = s_list[idx]
        ET.SubElement(
            lane_left,
            "width",
            sOffset=f"{s_val:.6f}",
            a=f"{max(0.1, w_left[idx]):.3f}",
            b="0",
            c="0",
            d="0",
        )
        ET.SubElement(
            lane_right,
            "width",
            sOffset=f"{s_val:.6f}",
            a=f"{max(0.1, w_right[idx]):.3f}",
            b="0",
            c="0",
            d="0",
        )
    return root


def main():
    parser = argparse.ArgumentParser(description="Build main-track-only OpenDRIVE from TTL centerline and XODR widths")
    default_xodr = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    default_ttl = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_fellow_test_xodr_all.csv"
    default_out = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LagunaSeca_MainTrack_FromTTL.xodr"
    parser.add_argument("--xodr", type=Path, default=default_xodr, help="Source XODR (for widths)")
    parser.add_argument("--ttl", type=Path, default=default_ttl, help="TTL centerline CSV (x,y,z)")
    parser.add_argument("--output", "-o", type=Path, default=default_out, help="Output XODR path")
    parser.add_argument("--width-step", type=int, default=25, help="Sample width every N points (default 25)")
    args = parser.parse_args()

    if not args.xodr.exists():
        print(f"ERROR: XODR not found: {args.xodr}")
        return 1
    if not args.ttl.exists():
        print(f"ERROR: TTL not found: {args.ttl}")
        return 1

    print("Loading TTL centerline...")
    ttl_points = load_ttl(args.ttl)
    if len(ttl_points) < 2:
        print("ERROR: TTL has fewer than 2 points")
        return 1
    print(f"  Loaded {len(ttl_points)} points")

    print("Projecting TTL to main track and sampling widths...")
    s_list, w_left, w_right = sample_ref_and_get_widths(args.xodr, ttl_points)
    total_len = s_list[-1]
    if len(ttl_points) > 1:
        total_len += math.hypot(
            ttl_points[-1][0] - ttl_points[-2][0],
            ttl_points[-1][1] - ttl_points[-2][1],
        )
    print(f"  Road length: {total_len:.1f} m")
    print(f"  Width range left: {min(w_left):.2f}–{max(w_left):.2f} m, right: {min(w_right):.2f}–{max(w_right):.2f} m")

    print("Building OpenDRIVE...")
    root = build_xodr(
        ttl_points,
        s_list,
        w_left,
        w_right,
        road_name="MainTrack_FromTTL",
        width_sample_step=args.width_step,
    )
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    args.output.parent.mkdir(parents=True, exist_ok=True)
    tree.write(
        args.output,
        encoding="utf-8",
        xml_declaration=True,
        default_namespace=None,
        method="xml",
    )
    print(f"Wrote: {args.output}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
