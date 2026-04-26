"""Derive track centerline CSVs from an XODR map's <planView> geometry.

The placement-time (s, t) projection in src/scenic/simulators/dspace/modeldesk/placement.py
loads two centerline CSVs:
    ttl_main_road.csv  -- main lap centerline
    ttl_pitlane.csv    -- pit lane centerline

Historically these were measured empirically by driving in dSPACE. This script replaces
that workflow by sampling the XODR road reference line directly. Output CSVs match the
existing format (header `x,y,z`).

Currently supports XODR <planView> primitives: <line/>, <arc curvature="..."/>.
Other primitives (<spiral>, <paramPoly3>, <poly3>) raise NotImplementedError so partial
data is never silently produced.

Run from repo root:
    python tools/frames/derive_centerline_from_xodr.py [--xodr PATH] [--ds METERS]

Default XODR: assets/maps/dSPACE/LGS_v1.xodr.
Default sample step: 0.5 m.

Output:
    assets/ttls/LS_ENU_TTL_CSV/ttl_main_road_xodr.csv  (Road 1 + Road 2 composed in loop order)
    assets/ttls/LS_ENU_TTL_CSV/ttl_pitlane_xodr.csv    (Pit Lane)

Both written in XODR-xy frame. If A.1 verification finds XODR != dSPACE RD, a follow-up
step composes a GPS->RD calibration before placement.py consumes them.
"""
from __future__ import annotations

import argparse
import csv
import math
import xml.etree.ElementTree as ET
from dataclasses import dataclass
from pathlib import Path
from typing import Dict, List, Optional, Tuple

REPO_ROOT = Path(__file__).resolve().parents[2]
DEFAULT_XODR = REPO_ROOT / "assets" / "maps" / "dSPACE" / "LGS_v1.xodr"
TTL_DIR = REPO_ROOT / "assets" / "ttls" / "LS_ENU_TTL_CSV"


@dataclass
class GeomSeg:
    s0: float        # start arc-length along road
    x0: float        # start x in XODR frame
    y0: float        # start y in XODR frame
    hdg: float       # start heading (rad)
    length: float    # segment length (m)
    kind: str        # 'line', 'arc', or 'spiral'
    curvature: float = 0.0     # rad/m, signed (left positive); for arc, constant kappa
    curv_start: float = 0.0    # rad/m at s=s0; for spiral
    curv_end: float = 0.0      # rad/m at s=s0+length; for spiral


@dataclass
class Road:
    id: str
    name: str
    length: float
    geoms: List[GeomSeg]
    successor_id: Optional[str] = None
    successor_contact: Optional[str] = None  # 'start' or 'end'
    predecessor_id: Optional[str] = None
    predecessor_contact: Optional[str] = None


def parse_xodr_roads(xodr_path: Path) -> Dict[str, Road]:
    """Parse <road> elements with planView geometry and link topology."""
    tree = ET.parse(xodr_path)
    root = tree.getroot()
    roads: Dict[str, Road] = {}
    for road_el in root.findall("road"):
        rid = road_el.attrib["id"]
        name = road_el.attrib.get("name", "")
        length = float(road_el.attrib["length"])

        geoms: List[GeomSeg] = []
        plan_view = road_el.find("planView")
        if plan_view is None:
            continue
        for g in plan_view.findall("geometry"):
            s0 = float(g.attrib["s"])
            x0 = float(g.attrib["x"])
            y0 = float(g.attrib["y"])
            hdg = float(g.attrib["hdg"])
            seg_len = float(g.attrib["length"])
            line = g.find("line")
            arc = g.find("arc")
            spiral = g.find("spiral")
            poly3 = g.find("poly3")
            param_poly3 = g.find("paramPoly3")
            if line is not None:
                geoms.append(GeomSeg(s0, x0, y0, hdg, seg_len, "line", 0.0))
            elif arc is not None:
                k = float(arc.attrib["curvature"])
                geoms.append(GeomSeg(s0, x0, y0, hdg, seg_len, "arc", k))
            elif spiral is not None:
                k0 = float(spiral.attrib["curvStart"])
                k1 = float(spiral.attrib["curvEnd"])
                geoms.append(GeomSeg(s0, x0, y0, hdg, seg_len, "spiral",
                                     curv_start=k0, curv_end=k1))
            elif poly3 is not None or param_poly3 is not None:
                kinds = []
                if poly3 is not None:
                    kinds.append("poly3")
                if param_poly3 is not None:
                    kinds.append("paramPoly3")
                raise NotImplementedError(
                    f"Road {rid!r} has unsupported geometry primitive(s): {kinds}. "
                    "Add support before using this script on this XODR."
                )
            else:
                raise NotImplementedError(
                    f"Road {rid!r} geometry at s={s0} has no recognized primitive."
                )

        link = road_el.find("link")
        succ_id = succ_contact = pred_id = pred_contact = None
        if link is not None:
            s_el = link.find("successor")
            p_el = link.find("predecessor")
            if s_el is not None and s_el.attrib.get("elementType") == "road":
                succ_id = s_el.attrib.get("elementId")
                succ_contact = s_el.attrib.get("contactPoint")
            if p_el is not None and p_el.attrib.get("elementType") == "road":
                pred_id = p_el.attrib.get("elementId")
                pred_contact = p_el.attrib.get("contactPoint")

        roads[rid] = Road(
            id=rid,
            name=name,
            length=length,
            geoms=geoms,
            successor_id=succ_id,
            successor_contact=succ_contact,
            predecessor_id=pred_id,
            predecessor_contact=pred_contact,
        )
    return roads


def sample_segment(seg: GeomSeg, ds: float) -> List[Tuple[float, float, float]]:
    """Sample (x, y, s_local) along a single planView segment with step ds (m).

    s_local is the arc-length from the start of the segment, NOT cumulative on the road.
    Includes the start point (s_local=0) but excludes the end (caller composes).
    """
    pts: List[Tuple[float, float, float]] = []
    n = max(1, int(math.ceil(seg.length / ds)))
    if seg.kind == "line":
        for i in range(n):
            s_loc = i * (seg.length / n)
            x = seg.x0 + s_loc * math.cos(seg.hdg)
            y = seg.y0 + s_loc * math.sin(seg.hdg)
            pts.append((x, y, s_loc))
    elif seg.kind == "arc":
        k = seg.curvature
        if abs(k) < 1e-12:
            # Degenerate "arc" with zero curvature - treat as line.
            for i in range(n):
                s_loc = i * (seg.length / n)
                x = seg.x0 + s_loc * math.cos(seg.hdg)
                y = seg.y0 + s_loc * math.sin(seg.hdg)
                pts.append((x, y, s_loc))
        else:
            inv_k = 1.0 / k
            for i in range(n):
                s_loc = i * (seg.length / n)
                psi = seg.hdg + k * s_loc
                x = seg.x0 + inv_k * (math.sin(psi) - math.sin(seg.hdg))
                y = seg.y0 + inv_k * (math.cos(seg.hdg) - math.cos(psi))
                pts.append((x, y, s_loc))
    elif seg.kind == "spiral":
        # Clothoid: kappa(s) = k0 + (k1 - k0) * s / L
        # heading:  psi(s) = hdg + k0*s + (k1 - k0)*s^2 / (2L)
        # position: numerical integration of (cos psi, sin psi).
        k0 = seg.curv_start
        k1 = seg.curv_end
        L = seg.length
        # Sub-step finer than ds for accuracy on the integration.
        sub_steps = max(n, 32) * 4
        ds_sub = L / sub_steps
        x = seg.x0
        y = seg.y0
        s_acc = 0.0
        # Record samples at the requested cadence (every L/n).
        target_interval = L / n if n > 0 else L
        next_target = 0.0
        # Always emit start point.
        pts.append((x, y, 0.0))
        next_target += target_interval
        psi = seg.hdg
        for _ in range(sub_steps):
            s_mid = s_acc + 0.5 * ds_sub
            psi_mid = seg.hdg + k0 * s_mid + (k1 - k0) * s_mid * s_mid / (2.0 * L)
            x += math.cos(psi_mid) * ds_sub
            y += math.sin(psi_mid) * ds_sub
            s_acc += ds_sub
            psi = seg.hdg + k0 * s_acc + (k1 - k0) * s_acc * s_acc / (2.0 * L)
            # Emit a sample if we've crossed the next target arc-length.
            while s_acc + 1e-9 >= next_target and next_target < L - 1e-9:
                pts.append((x, y, s_acc))
                next_target += target_interval
    else:
        raise AssertionError(f"unhandled segment kind {seg.kind!r}")
    return pts


def sample_road(road: Road, ds: float) -> List[Tuple[float, float, float]]:
    """Sample (x, y, s_road) along all planView segments of a road, in order, plus end point."""
    pts: List[Tuple[float, float, float]] = []
    for seg in road.geoms:
        seg_pts = sample_segment(seg, ds)
        for x, y, s_loc in seg_pts:
            pts.append((x, y, seg.s0 + s_loc))
    # Append the final endpoint of the last segment (always missed by the i<n loop).
    last = road.geoms[-1]
    x_end, y_end = _segment_endpoint(last)
    pts.append((x_end, y_end, last.s0 + last.length))
    return pts


def _segment_endpoint(seg: GeomSeg) -> Tuple[float, float]:
    """Closed-form (or integrated) endpoint of a planView segment."""
    if seg.kind == "line":
        x_end = seg.x0 + seg.length * math.cos(seg.hdg)
        y_end = seg.y0 + seg.length * math.sin(seg.hdg)
        return x_end, y_end
    if seg.kind == "arc":
        k = seg.curvature
        if abs(k) < 1e-12:
            x_end = seg.x0 + seg.length * math.cos(seg.hdg)
            y_end = seg.y0 + seg.length * math.sin(seg.hdg)
            return x_end, y_end
        inv_k = 1.0 / k
        psi = seg.hdg + k * seg.length
        x_end = seg.x0 + inv_k * (math.sin(psi) - math.sin(seg.hdg))
        y_end = seg.y0 + inv_k * (math.cos(seg.hdg) - math.cos(psi))
        return x_end, y_end
    if seg.kind == "spiral":
        # Integrate (cos psi, sin psi) ds over [0, L].
        k0 = seg.curv_start
        k1 = seg.curv_end
        L = seg.length
        sub_steps = max(64, int(L * 200))
        ds_sub = L / sub_steps
        x = seg.x0
        y = seg.y0
        s_acc = 0.0
        for _ in range(sub_steps):
            s_mid = s_acc + 0.5 * ds_sub
            psi_mid = seg.hdg + k0 * s_mid + (k1 - k0) * s_mid * s_mid / (2.0 * L)
            x += math.cos(psi_mid) * ds_sub
            y += math.sin(psi_mid) * ds_sub
            s_acc += ds_sub
        return x, y
    raise AssertionError(f"unhandled segment kind {seg.kind!r}")


def compose_lap(roads: Dict[str, Road], main_id: str, ds: float) -> List[Tuple[float, float, float]]:
    """Walk main road's planView and append its successor (and any further chained roads).

    Returns lap-cumulative arc-length points. Stops when we'd revisit main_id.
    """
    pts: List[Tuple[float, float, float]] = []
    visited = set()
    cur_id = main_id
    cum = 0.0
    while cur_id is not None and cur_id not in visited:
        visited.add(cur_id)
        road = roads[cur_id]
        road_pts = sample_road(road, ds=ds)
        # Drop the duplicate start point on subsequent roads (it equals the previous end).
        start_off = 1 if pts else 0
        for x, y, s_road in road_pts[start_off:]:
            pts.append((x, y, cum + s_road))
        cum += road.length
        nxt = road.successor_id
        if nxt is None or nxt == main_id:
            break
        cur_id = nxt
    return pts


def write_csv(out_path: Path, pts: List[Tuple[float, float, float]], z: float = 0.0) -> None:
    """Write (x, y, z) CSV with z constant (centerline elevation -- placement only uses xy)."""
    out_path.parent.mkdir(parents=True, exist_ok=True)
    with open(out_path, "w", encoding="utf-8", newline="") as f:
        w = csv.writer(f)
        w.writerow(["x", "y", "z"])
        for x, y, _s in pts:
            w.writerow([f"{x:.6f}", f"{y:.6f}", f"{z:.6f}"])


def main() -> int:
    ap = argparse.ArgumentParser(description=__doc__, formatter_class=argparse.RawDescriptionHelpFormatter)
    ap.add_argument("--xodr", default=str(DEFAULT_XODR), type=Path,
                    help="path to XODR (default: %(default)s)")
    ap.add_argument("--ds", default=0.5, type=float,
                    help="centerline sample step in meters (default: %(default)s)")
    ap.add_argument("--main-id", default="1",
                    help="XODR road id for main racing road (default: %(default)s)")
    ap.add_argument("--pit-id", default="0",
                    help="XODR road id for pit lane (default: %(default)s)")
    ap.add_argument("--out-main", default=str(TTL_DIR / "ttl_main_road_xodr.csv"), type=Path)
    ap.add_argument("--out-pit",  default=str(TTL_DIR / "ttl_pitlane_xodr.csv"),   type=Path)
    args = ap.parse_args()

    ds = float(args.ds)

    print(f"XODR: {args.xodr}")
    if not args.xodr.exists():
        print(f"  ERROR: XODR not found")
        return 1

    roads = parse_xodr_roads(args.xodr)
    print(f"Parsed {len(roads)} road(s):")
    for rid, road in roads.items():
        succ = f" -> succ:{road.successor_id}({road.successor_contact})" if road.successor_id else ""
        pred = f" pred:{road.predecessor_id}({road.predecessor_contact})" if road.predecessor_id else ""
        print(f"  id={rid:>2}  name={road.name!r:>20}  length={road.length:8.2f} m  geoms={len(road.geoms)}{succ}{pred}")

    if args.main_id not in roads:
        print(f"  ERROR: main road id {args.main_id!r} not found")
        return 1
    if args.pit_id not in roads:
        print(f"  ERROR: pit road id {args.pit_id!r} not found")
        return 1

    print()
    print(f"Composing main lap from road id={args.main_id} (and successors)...")
    main_pts = compose_lap(roads, args.main_id, ds=ds)
    print(f"  {len(main_pts)} points, total arc-length {main_pts[-1][2]:.2f} m")
    write_csv(args.out_main, main_pts)
    print(f"  wrote {args.out_main}")

    print()
    print(f"Sampling pit road id={args.pit_id}...")
    pit_road = roads[args.pit_id]
    pit_pts_raw = sample_road(pit_road, ds=ds)
    pit_pts = [(x, y, s) for (x, y, s) in pit_pts_raw]
    print(f"  {len(pit_pts)} points, total arc-length {pit_pts[-1][2]:.2f} m")
    write_csv(args.out_pit, pit_pts)
    print(f"  wrote {args.out_pit}")

    print()
    print("Sanity: first/last sample of each output:")
    print(f"  main first: ({main_pts[0][0]:.3f}, {main_pts[0][1]:.3f})")
    print(f"  main last:  ({main_pts[-1][0]:.3f}, {main_pts[-1][1]:.3f})")
    print(f"  pit  first: ({pit_pts[0][0]:.3f}, {pit_pts[0][1]:.3f})")
    print(f"  pit  last:  ({pit_pts[-1][0]:.3f}, {pit_pts[-1][1]:.3f})")
    print()
    print("Next step (Phase A.2 verification):")
    print("  Run a Scenic scene placing ego at e.g. seq.StartPosition = 100 on the main route,")
    print("  AdditionalLateralOffset = 0. Read back RD-xy after warmup. The RD-xy should be")
    print("  within ~0.5 m of the sample point on the new centerline at s = 100.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
