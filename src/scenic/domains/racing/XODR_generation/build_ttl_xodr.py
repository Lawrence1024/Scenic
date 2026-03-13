#!/usr/bin/env python3
"""
Build an OpenDRIVE file from TTL centerline CSVs (main + pit) with fixed lane widths.

Creates three roads connected by predecessor/successor links (no junction elements):
- Road 1 (main arc A): pit entry → pit exit along main centerline (5.5 m each side).
- Road 2 (pit): pit centerline trimmed in overlap so main dominates (3 m each side).
- Road 3 (main arc B): pit exit → pit entry along main centerline (5.5 m each side).

The full main loop (Andretti, Corkscrew, etc.) is road 1 + road 3. In overlap regions
(Corkscrew, Andretti) pit points within OVERLAP_MAIN_WINS_M of main are trimmed so
main track width applies.

Run from Scenic repo root:
  python -m scenic.domains.racing.XODR_generation.build_ttl_xodr [options]
  python src/scenic/domains/racing/XODR_generation/build_ttl_xodr.py --main path --pit path -o out.xodr
"""

import argparse
import csv
import math
import sys
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple

_PKG_DIR = Path(__file__).resolve().parent
_REPO_ROOT = _PKG_DIR.parent.parent.parent.parent.parent

# Default paths (relative to repo root)
_DEFAULT_MAIN_TTL = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_main_road.csv"
_DEFAULT_PIT_TTL = _REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV" / "ttl_pitlane.csv"
_DEFAULT_OUTPUT = _PKG_DIR / "generated" / "track_from_ttl.xodr"

# Lane widths (meters each side of centerline)
MAIN_LANE_WIDTH = 5.5
PIT_LANE_WIDTH = 3.0

# Where pit centerline is within this distance of main, trim pit so main dominates (same idea as track_regions).
OVERLAP_MAIN_WINS_M = 5.0


def load_ttl_csv(path: Path) -> List[Tuple[float, float]]:
    """Load (x, y) from a TTL CSV. Expects header row; uses first two numeric columns as x, y."""
    points = []
    with open(path, newline="", encoding="utf-8") as f:
        reader = csv.reader(f)
        next(reader, None)  # skip header
        for row in reader:
            if not row or len(row) < 2:
                continue
            try:
                x, y = float(row[0]), float(row[1])
                points.append((x, y))
            except (ValueError, IndexError):
                continue
    return points


def _closest_point_index(poly: List[Tuple[float, float]], point: Tuple[float, float]) -> int:
    """Return index into poly of the point closest to (x, y)."""
    px, py = point
    best_i = 0
    best_d2 = float("inf")
    for i, (x, y) in enumerate(poly):
        d2 = (x - px) ** 2 + (y - py) ** 2
        if d2 < best_d2:
            best_d2 = d2
            best_i = i
    return best_i


def _distance_point_to_polyline(
    point: Tuple[float, float],
    poly: List[Tuple[float, float]],
) -> float:
    """Minimum distance from point to any segment of the polyline."""
    px, py = point
    best_d = float("inf")
    for i in range(len(poly) - 1):
        x0, y0 = poly[i]
        x1, y1 = poly[i + 1]
        # Vector along segment, and from segment start to point
        dx, dy = x1 - x0, y1 - y0
        qx, qy = px - x0, py - y0
        seg_len_sq = dx * dx + dy * dy
        if seg_len_sq < 1e-18:
            d = math.hypot(qx, qy)
        else:
            t = max(0.0, min(1.0, (qx * dx + qy * dy) / seg_len_sq))
            proj_x = x0 + t * dx
            proj_y = y0 + t * dy
            d = math.hypot(px - proj_x, py - proj_y)
        if d < best_d:
            best_d = d
    return best_d


def _trim_pit_overlap(
    pit_pts: List[Tuple[float, float]],
    main_pts: List[Tuple[float, float]],
    threshold_m: float,
) -> List[Tuple[float, float]]:
    """Trim pit points that are within threshold_m of main centerline so main dominates in overlap."""
    if not pit_pts or not main_pts or threshold_m <= 0:
        return pit_pts
    out = []
    for p in pit_pts:
        if _distance_point_to_polyline(p, main_pts) >= threshold_m:
            out.append(p)
    # Keep at least two points so the road is valid; if we trimmed everything, return original
    if len(out) < 2:
        return pit_pts
    return out


def _polyline_length(pts: List[Tuple[float, float]]) -> float:
    """Cumulative length along polyline."""
    return sum(
        math.hypot(pts[i + 1][0] - pts[i][0], pts[i + 1][1] - pts[i][1])
        for i in range(len(pts) - 1)
    )


def _build_plan_view(
    parent: ET.Element,
    points: List[Tuple[float, float]],
) -> float:
    """Append planView geometry (line segments) for the given polyline. Returns total length."""
    cum_s = 0.0
    for i in range(len(points) - 1):
        x0, y0 = points[i]
        x1, y1 = points[i + 1]
        length = math.hypot(x1 - x0, y1 - y0)
        if length < 1e-9:
            continue
        hdg = math.atan2(y1 - y0, x1 - x0)
        geom = ET.SubElement(
            parent,
            "geometry",
            s=f"{cum_s:.6f}",
            x=f"{x0:.6f}",
            y=f"{y0:.6f}",
            hdg=f"{hdg:.6f}",
            length=f"{length:.6f}",
        )
        ET.SubElement(geom, "line")
        cum_s += length
    return cum_s


def _add_lane_section_constant_width(
    lanes_el: ET.Element,
    width_left: float,
    width_right: float,
    road_length: float,
) -> None:
    """Add one lane section with center, left lane (id=1), right lane (id=-1), constant width."""
    lane_section = ET.SubElement(lanes_el, "laneSection", s="0")
    center = ET.SubElement(lane_section, "center")
    ET.SubElement(center, "lane", id="0", type="none", level="false")
    left_el = ET.SubElement(lane_section, "left")
    lane_left = ET.SubElement(left_el, "lane", id="1", type="driving", level="false")
    ET.SubElement(
        lane_left, "roadMark",
        sOffset="0", type="solid", weight="standard", color="standard", width="0.13",
    )
    ET.SubElement(
        lane_left, "width",
        sOffset="0.000000", a=f"{max(0.1, width_left):.3f}", b="0", c="0", d="0",
    )
    ET.SubElement(
        lane_left, "width",
        sOffset=f"{road_length:.6f}", a=f"{max(0.1, width_left):.3f}", b="0", c="0", d="0",
    )
    right_el = ET.SubElement(lane_section, "right")
    lane_right = ET.SubElement(right_el, "lane", id="-1", type="driving", level="false")
    ET.SubElement(
        lane_right, "roadMark",
        sOffset="0", type="solid", weight="standard", color="standard", width="0.13",
    )
    ET.SubElement(
        lane_right, "width",
        sOffset="0.000000", a=f"{max(0.1, width_right):.3f}", b="0", c="0", d="0",
    )
    ET.SubElement(
        lane_right, "width",
        sOffset=f"{road_length:.6f}", a=f"{max(0.1, width_right):.3f}", b="0", c="0", d="0",
    )


def _build_road(
    root: ET.Element,
    road_id: str,
    name: str,
    points: List[Tuple[float, float]],
    width_left: float,
    width_right: float,
    pred_road_id: str,
    pred_contact: str,
    succ_road_id: str,
    succ_contact: str,
) -> float:
    """Append one road (planView + lanes + link). Returns road length."""
    road = ET.SubElement(root, "road", name=name, id=road_id, junction="-1")
    ET.SubElement(road, "type", s="0", type="rural")
    link = ET.SubElement(road, "link")
    ET.SubElement(
        link, "predecessor",
        elementType="road", elementId=pred_road_id, contactPoint=pred_contact,
    )
    ET.SubElement(
        link, "successor",
        elementType="road", elementId=succ_road_id, contactPoint=succ_contact,
    )
    plan_view = ET.SubElement(road, "planView")
    length = _build_plan_view(plan_view, points)
    road.set("length", f"{length:.6f}")
    lanes_el = ET.SubElement(road, "lanes")
    _add_lane_section_constant_width(lanes_el, width_left, width_right, length)
    return length


def build_connected_ttl_xodr(
    main_ttl_path: Path,
    pit_ttl_path: Path,
    output_path: Path,
    main_width: float = MAIN_LANE_WIDTH,
    pit_width: float = PIT_LANE_WIDTH,
    overlap_main_wins_m: float = OVERLAP_MAIN_WINS_M,
) -> Path:
    """
    Build an OpenDRIVE file with full main loop (two arcs) and pit road, connected (no junctions).

    - Road 1 (MainTrack arc A): pit entry → pit exit along main centerline (5.5 m).
    - Road 2 (PitTrack): pit centerline trimmed where within overlap_main_wins_m of main (3 m).
    - Road 3 (MainTrack arc B): pit exit → pit entry along main centerline (5.5 m).

    So the full main loop (including Andretti Hairpin) is road 1 + road 3. In overlap
    (e.g. Corkscrew) pit is trimmed so main track width dominates.

    Returns the path where the file was written.
    """
    main_pts = load_ttl_csv(main_ttl_path)
    pit_pts = load_ttl_csv(pit_ttl_path)
    if len(main_pts) < 2:
        raise ValueError(f"Main TTL has fewer than 2 points: {main_ttl_path}")
    if len(pit_pts) < 2:
        raise ValueError(f"Pit TTL has fewer than 2 points: {pit_ttl_path}")

    # Pit exit = index on main closest to pit start; pit entry = index on main closest to pit end
    exit_idx = _closest_point_index(main_pts, pit_pts[0])
    entry_idx = _closest_point_index(main_pts, pit_pts[-1])

    # Main split into two arcs so we have two connection points (exit and entry) and full loop is present
    # Arc A: entry → exit (main_pts[entry_idx:] + main_pts[:exit_idx+1])
    # Arc B: exit → entry (main_pts[exit_idx:] + main_pts[:entry_idx+1])
    main_arc_a = main_pts[entry_idx:] + main_pts[: exit_idx + 1]
    main_arc_b = main_pts[exit_idx:] + main_pts[: entry_idx + 1]

    # Trim pit where it overlaps main so main track width dominates
    pit_trimmed = _trim_pit_overlap(pit_pts, main_pts, overlap_main_wins_m)
    if len(pit_trimmed) < 2:
        pit_trimmed = pit_pts  # fallback if trim removed too much

    root = ET.Element("OpenDRIVE")
    ET.SubElement(
        root, "header",
        revMajor="1", revMinor="6", name="Track_From_TTL", date="", vendor="Scenic_XODR_generation",
        north="0", south="0", east="0", west="0",
    )

    # Cycle: road 1 (main arc A) → road 2 (pit) → road 3 (main arc B) → road 1
    # Road 1: main arc entry→exit; pred=3, succ=2
    _build_road(
        root,
        road_id="1",
        name="MainTrack_A",
        points=main_arc_a,
        width_left=main_width,
        width_right=main_width,
        pred_road_id="3",
        pred_contact="end",
        succ_road_id="2",
        succ_contact="start",
    )
    # Road 2: pit (trimmed); pred=1, succ=3
    _build_road(
        root,
        road_id="2",
        name="PitTrack",
        points=pit_trimmed,
        width_left=pit_width,
        width_right=pit_width,
        pred_road_id="1",
        pred_contact="end",
        succ_road_id="3",
        succ_contact="start",
    )
    # Road 3: main arc exit→entry (includes Andretti etc.); pred=2, succ=1
    _build_road(
        root,
        road_id="3",
        name="MainTrack_B",
        points=main_arc_b,
        width_left=main_width,
        width_right=main_width,
        pred_road_id="2",
        pred_contact="end",
        succ_road_id="1",
        succ_contact="start",
    )

    output_path.parent.mkdir(parents=True, exist_ok=True)
    tree = ET.ElementTree(root)
    ET.indent(tree, space="  ")
    tree.write(
        output_path,
        encoding="utf-8",
        xml_declaration=True,
        default_namespace=None,
        method="xml",
    )
    return output_path


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Build OpenDRIVE from TTL centerlines (main + pit, connected, fixed widths)",
    )
    parser.add_argument(
        "--main", "-m",
        type=Path,
        default=_DEFAULT_MAIN_TTL,
        help=f"Main track TTL CSV (default: {_DEFAULT_MAIN_TTL.name})",
    )
    parser.add_argument(
        "--pit", "-p",
        type=Path,
        default=_DEFAULT_PIT_TTL,
        help=f"Pit lane TTL CSV (default: {_DEFAULT_PIT_TTL.name})",
    )
    parser.add_argument(
        "--output", "-o",
        type=Path,
        default=_DEFAULT_OUTPUT,
        help=f"Output XODR path (default: generated/track_from_ttl.xodr in this package)",
    )
    parser.add_argument(
        "--main-width",
        type=float,
        default=MAIN_LANE_WIDTH,
        help=f"Lane width each side for main track in m (default: {MAIN_LANE_WIDTH})",
    )
    parser.add_argument(
        "--pit-width",
        type=float,
        default=PIT_LANE_WIDTH,
        help=f"Lane width each side for pit track in m (default: {PIT_LANE_WIDTH})",
    )
    parser.add_argument(
        "--overlap-main-wins",
        type=float,
        default=OVERLAP_MAIN_WINS_M,
        help=f"Trim pit points within this distance (m) of main so main dominates (default: {OVERLAP_MAIN_WINS_M})",
    )
    args = parser.parse_args()

    if not args.main.exists():
        print(f"ERROR: Main TTL not found: {args.main}", file=sys.stderr)
        return 1
    if not args.pit.exists():
        print(f"ERROR: Pit TTL not found: {args.pit}", file=sys.stderr)
        return 1

    print("Loading TTL centerlines...")
    out = build_connected_ttl_xodr(
        args.main,
        args.pit,
        args.output,
        main_width=args.main_width,
        pit_width=args.pit_width,
        overlap_main_wins_m=args.overlap_main_wins,
    )
    print(f"Wrote: {out}")
    return 0


if __name__ == "__main__":
    sys.exit(main())
