#!/usr/bin/env python3
"""
Investigate if there's a non-identity transformation between RD and XODR maps.

This script investigates:
1. Whether RD defines centerlines vs reference lines
2. Whether there's a transformation between RD and XODR
3. Whether this transformation is segment-dependent
"""

import sys
import math
import xml.etree.ElementTree as ET
from pathlib import Path
from typing import List, Tuple, Dict, Optional
from bisect import bisect_right
import numpy as np

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.utils import MAIN_ROAD_NAMES


def _poly(a, b, c, d, u):
    """Evaluate cubic polynomial."""
    return a + b*u + c*u*u + d*u*u*u


def _piecewise_poly_at(records, s):
    """Evaluate piecewise polynomial at s."""
    if not records:
        return 0.0
    s0s = [r[0] for r in records]
    i = bisect_right(s0s, s) - 1
    if i < 0:
        i = 0
    s0, a, b, c, d = records[i]
    return _poly(a, b, c, d, s - s0)


def road_width_and_center_t(xodr_path: str, road_name: str, s: float) -> Tuple[float, float]:
    """Get width and t_center from XODR (from temp3.md)."""
    root = ET.parse(xodr_path).getroot()
    ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0] + '}'
    
    road = None
    for r in root.findall(f".//{ns}road"):
        if (r.get("name") or "") == road_name:
            road = r
            break
    if road is None:
        raise ValueError(f'Road "{road_name}" not found.')
    
    length = float(road.get("length", "0"))
    s = max(0.0, min(length, s))
    
    lanes = road.find(f'{ns}lanes')
    if lanes is None:
        O = 0.0
    else:
        lane_offsets = []
        for lo in lanes.findall(f'{ns}laneOffset'):
            lane_offsets.append((
                float(lo.get("s", "0")),
                float(lo.get("a", "0")),
                float(lo.get("b", "0")),
                float(lo.get("c", "0")),
                float(lo.get("d", "0")),
            ))
        lane_offsets.sort(key=lambda x: x[0])
        O = _piecewise_poly_at(lane_offsets, s)
    
    if lanes is None:
        return (0.0, 0.0)
    
    sections = [(float(ls.get("s", "0")), ls) for ls in lanes.findall(f'{ns}laneSection')]
    if not sections:
        return (0.0, 0.0)
    
    sections.sort(key=lambda x: x[0])
    s_starts = [x[0] for x in sections]
    idx = bisect_right(s_starts, s) - 1
    if idx < 0:
        idx = 0
    s0, ls = sections[idx]
    ds = s - s0
    
    def lane_width_at(lane_elem, ds):
        wrecs = []
        for w in lane_elem.findall(f'{ns}width'):
            wrecs.append((
                float(w.get("sOffset", "0")),
                float(w.get("a", "0")),
                float(w.get("b", "0")),
                float(w.get("c", "0")),
                float(w.get("d", "0")),
            ))
        wrecs.sort(key=lambda x: x[0])
        if not wrecs:
            return 0.0
        w_s0s = [x[0] for x in wrecs]
        j = bisect_right(w_s0s, ds) - 1
        if j < 0:
            j = 0
        sOff, a, b, c, d = wrecs[j]
        return _poly(a, b, c, d, ds - sOff)
    
    def side_sum(side_elem):
        if side_elem is None:
            return 0.0
        total = 0.0
        for lane in side_elem.findall(f'{ns}lane'):
            if lane.get("id") == "0":
                continue
            if lane.get("type") != "driving":
                continue
            total += lane_width_at(lane, ds)
        return total
    
    L = side_sum(ls.find(f'{ns}left'))
    R = side_sum(ls.find(f'{ns}right'))
    driving_width = L + R
    
    t_center = O + (L - R) / 2.0
    return driving_width, t_center


def sample_rd_reference_line(rd_path: str, road_name: str, ds: float = 0.5) -> List[Tuple[float, float, float, float]]:
    """Sample RD reference line with headings."""
    root = ET.parse(rd_path).getroot()
    ns = '' if root.tag.startswith('{') else root.tag.split('}')[0] + '}'
    
    roads = root.find(f'.//{ns}Roads')
    if roads is None:
        roads = root.find('.//{*}Roads')
    if roads is None:
        raise ValueError(f"No Roads element found in RD file")
    
    road = None
    for r in roads.findall(f'./{ns}Road'):
        name_elem = r.find(f'./{ns}Name')
        if name_elem is None:
            name_elem = r.find('./{*}Name')
        if (name_elem.text if name_elem is not None and name_elem.text else "").strip() == road_name:
            road = r
            break
    
    if road is None:
        for r in roads.findall('./{*}Road'):
            name_elem = r.find('./{*}Name')
            if (name_elem.text if name_elem is not None and name_elem.text else "").strip() == road_name:
                road = r
                break
    
    if road is None:
        raise ValueError(f"Road not found: {road_name}")
    
    pts = []
    s_cum = 0.0
    prev_xy = None
    
    segs = road.find(f'./{ns}Segments')
    if segs is None:
        segs = road.find('./{*}Segments')
    if segs is None:
        return pts
    
    seg_list = segs.findall(f'./{ns}Segment')
    if not seg_list:
        seg_list = segs.findall('./{*}Segment')
    
    def _txt(node, default="0"):
        if node is None or node.text is None:
            return default
        return node.text.strip() or default
    
    def _pt(seg, tag, ns=''):
        n = seg.find(f'./{ns}{tag}')
        if n is None:
            n = seg.find(f'./{{*}}{tag}')
        if n is None:
            return (0.0, 0.0)
        x_elem = n.find(f'./{ns}X')
        if x_elem is None:
            x_elem = n.find('./{*}X')
        y_elem = n.find(f'./{ns}Y')
        if y_elem is None:
            y_elem = n.find('./{*}Y')
        return (float(_txt(x_elem)), float(_txt(y_elem)))
    
    for seg in seg_list:
        seg_type = None
        for k, v in seg.attrib.items():
            if k.endswith('}type'):
                seg_type = v
                break
        if seg_type is None:
            seg_type = seg.attrib.get('type', '')
        
        if seg_type != "Spline":
            continue
        
        asp = seg.find(f'./{ns}AbsoluteStartPosition')
        if asp is None:
            asp = seg.find('./{*}AbsoluteStartPosition')
        if asp is None:
            continue
        
        X0 = float(_txt(asp.find(f'./{ns}X') or asp.find('./{*}X')))
        Y0 = float(_txt(asp.find(f'./{ns}Y') or asp.find('./{*}Y')))
        theta = math.radians(float(_txt(asp.find(f'./{ns}Tangent') or asp.find('./{*}Tangent'))))
        L = float(_txt(seg.find(f'./{ns}Length') or seg.find('./{*}Length')))
        
        Ax, Ay = _pt(seg, "A", ns)
        Bx, By = _pt(seg, "B", ns)
        Cx, Cy = _pt(seg, "C", ns)
        Dx, Dy = _pt(seg, "D", ns)
        
        n = max(2, int(math.ceil(L / ds)) + 1)
        for i in range(n):
            s_local = min(L, i * ds)
            t = 0.0 if L <= 0 else (s_local / L)
            
            xL = Ax + Bx*t + Cx*(t*t) + Dx*(t*t*t)
            yL = Ay + By*t + Cy*(t*t) + Dy*(t*t*t)
            
            dxL_dt = Bx + 2*Cx*t + 3*Dx*(t*t)
            dyL_dt = By + 2*Cy*t + 3*Dy*(t*t)
            hdg_local = math.atan2(dyL_dt, dxL_dt)
            
            xW = X0 + xL*math.cos(theta) - yL*math.sin(theta)
            yW = Y0 + xL*math.sin(theta) + yL*math.cos(theta)
            hdg_world = hdg_local + theta
            
            if prev_xy is None:
                pts.append((xW, yW, s_cum, hdg_world, seg))  # Include segment for analysis
            else:
                ds_actual = math.hypot(xW - prev_xy[0], yW - prev_xy[1])
                s_cum += ds_actual
                pts.append((xW, yW, s_cum, hdg_world, seg))
            prev_xy = (xW, yW)
    
    return pts


def sample_xodr_reference_line(xodr_path: str, road_name: str, ds: float = 0.5) -> List[Tuple[float, float, float, float]]:
    """Sample XODR reference line with headings."""
    import xml.etree.ElementTree as ET
    
    root = ET.parse(xodr_path).getroot()
    ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0] + '}'
    
    road = None
    for r in root.findall(f'{ns}road'):
        if r.get('name', '') == road_name:
            road = r
            break
    
    if road is None:
        raise ValueError(f"Road not found in XODR: {road_name}")
    
    road_id = road.get('id')
    from scenic.simulators.dspace.geometry.xodr_parser import _road_local_ref
    
    pts, length, _ = _road_local_ref(root, road_id, ns, step=ds, apply_lane_offset=False)
    
    result = []
    for s_global, x, y, z, h in pts:
        s_local = s_global if s_global <= length else length
        result.append((x, y, s_local, h))
    
    return result


def compute_transformation_analysis(rd_path: str, xodr_path: str, road_name: str):
    """Analyze if there's a transformation between RD and XODR."""
    print(f"\n{'='*80}")
    print(f"TRANSFORMATION ANALYSIS: {road_name}")
    print(f"{'='*80}")
    
    # Sample reference lines
    print(f"\n  Step 1: Sampling reference lines...")
    rd_ref = sample_rd_reference_line(rd_path, road_name, ds=0.5)
    xodr_ref = sample_xodr_reference_line(xodr_path, road_name, ds=0.5)
    
    print(f"    RD points: {len(rd_ref)}")
    print(f"    XODR points: {len(xodr_ref)}")
    
    # Check if reference lines match (identity transformation)
    print(f"\n  Step 2: Checking if reference lines match (identity transformation)...")
    ref_errors = []
    for rd_x, rd_y, rd_s, rd_hdg, rd_seg in rd_ref:
        min_dist = float('inf')
        for xodr_x, xodr_y, xodr_s, xodr_hdg in xodr_ref:
            dist = math.hypot(rd_x - xodr_x, rd_y - xodr_y)
            if dist < min_dist:
                min_dist = dist
        ref_errors.append(min_dist)
    
    ref_errors = np.array(ref_errors)
    print(f"    Mean error: {np.mean(ref_errors):.2e} m")
    print(f"    Max error:  {np.max(ref_errors):.2e} m")
    
    if np.max(ref_errors) < 1e-10:
        print(f"    [IDENTITY] Reference lines match - no transformation needed")
    else:
        print(f"    [NON-IDENTITY] Reference lines differ - transformation may exist")
    
    # Check if RD reference line matches XODR centerline (with t_center offset)
    print(f"\n  Step 3: Checking if RD reference line = XODR centerline (with t_center)...")
    
    # Compute XODR centerline
    xodr_centerline = []
    xodr_t_centers = []
    for x, y, s, hdg in xodr_ref:
        width, t_center = road_width_and_center_t(xodr_path, road_name, s)
        xodr_t_centers.append(t_center)
        
        # Offset reference line by t_center
        nx = -math.sin(hdg)  # Left normal
        ny = math.cos(hdg)
        x_center = x + t_center * nx
        y_center = y + t_center * ny
        xodr_centerline.append((x_center, y_center, s))
    
    # Compare RD reference line to XODR centerline
    centerline_errors = []
    for rd_x, rd_y, rd_s, rd_hdg, rd_seg in rd_ref:
        min_dist = float('inf')
        for xodr_x, xodr_y, xodr_s in xodr_centerline:
            dist = math.hypot(rd_x - xodr_x, rd_y - xodr_y)
            if dist < min_dist:
                min_dist = dist
        centerline_errors.append(min_dist)
    
    centerline_errors = np.array(centerline_errors)
    print(f"    Mean error: {np.mean(centerline_errors):.2e} m")
    print(f"    Max error:  {np.max(centerline_errors):.2e} m")
    print(f"    Average t_center: {np.mean(xodr_t_centers):.2f} m")
    print(f"    t_center range: {np.min(xodr_t_centers):.2f} m - {np.max(xodr_t_centers):.2f} m")
    
    if np.max(centerline_errors) < np.max(ref_errors):
        print(f"    [HYPOTHESIS] RD reference line might be XODR centerline!")
        print(f"      (RD ref matches XODR centerline better than XODR ref)")
    elif np.max(centerline_errors) < 0.01:
        print(f"    [POSSIBLE] RD reference line could be XODR centerline")
    else:
        print(f"    [UNLIKELY] RD reference line is not XODR centerline")
    
    # Check segment-dependent transformation
    print(f"\n  Step 4: Checking for segment-dependent transformation...")
    
    # Group RD points by segment
    segment_errors = {}
    for i, (rd_x, rd_y, rd_s, rd_hdg, rd_seg) in enumerate(rd_ref):
        seg_id = id(rd_seg)  # Use segment object ID as identifier
        if seg_id not in segment_errors:
            segment_errors[seg_id] = []
        
        # Compare to XODR reference line
        min_dist_ref = float('inf')
        for xodr_x, xodr_y, xodr_s, xodr_hdg in xodr_ref:
            dist = math.hypot(rd_x - xodr_x, rd_y - xodr_y)
            if dist < min_dist_ref:
                min_dist_ref = dist
        
        # Compare to XODR centerline
        min_dist_center = float('inf')
        for xodr_x, xodr_y, xodr_s in xodr_centerline:
            dist = math.hypot(rd_x - xodr_x, rd_y - xodr_y)
            if dist < min_dist_center:
                min_dist_center = dist
        
        segment_errors[seg_id].append({
            'ref_error': min_dist_ref,
            'center_error': min_dist_center,
            's': rd_s
        })
    
    print(f"    Found {len(segment_errors)} segments")
    for seg_id, errors in segment_errors.items():
        ref_errors_seg = [e['ref_error'] for e in errors]
        center_errors_seg = [e['center_error'] for e in errors]
        
        mean_ref = np.mean(ref_errors_seg)
        mean_center = np.mean(center_errors_seg)
        max_ref = np.max(ref_errors_seg)
        max_center = np.max(center_errors_seg)
        
        s_range = (min(e['s'] for e in errors), max(e['s'] for e in errors))
        
        print(f"\n    Segment (s={s_range[0]:.1f} to {s_range[1]:.1f}):")
        print(f"      Ref line error:  mean={mean_ref:.2e} m, max={max_ref:.2e} m")
        print(f"      Centerline error: mean={mean_center:.2e} m, max={max_center:.2e} m")
        
        if mean_center < mean_ref * 0.1:
            print(f"      [HYPOTHESIS] This segment: RD ref ≈ XODR centerline")
        elif mean_ref < mean_center * 0.1:
            print(f"      [HYPOTHESIS] This segment: RD ref ≈ XODR ref (identity)")
        else:
            print(f"      [UNCLEAR] This segment: Neither clearly matches")
    
    # Summary
    print(f"\n  {'='*80}")
    print(f"  SUMMARY")
    print(f"  {'='*80}")
    
    if np.max(ref_errors) < 1e-10:
        print(f"  [CONCLUSION] Identity transformation: RD ref = XODR ref")
        print(f"    - No transformation needed")
        print(f"    - RD reference line matches XODR reference line")
    elif np.max(centerline_errors) < np.max(ref_errors) * 0.1:
        print(f"  [CONCLUSION] Non-identity transformation: RD ref = XODR centerline")
        print(f"    - Transformation: XODR_ref + t_center = RD_ref")
        print(f"    - RD stores centerline, XODR stores reference line")
        print(f"    - t_center varies by road/segment")
    else:
        print(f"  [CONCLUSION] Complex transformation or mismatch")
        print(f"    - Neither identity nor simple t_center offset explains differences")
        print(f"    - May need more complex transformation")


def main():
    """Main function."""
    print("="*80)
    print("RD/XODR TRANSFORMATION INVESTIGATION")
    print("="*80)
    print("\nThis script investigates:")
    print("  1. Is there a transformation between RD and XODR?")
    print("  2. Is it identity (RD ref = XODR ref)?")
    print("  3. Or is RD ref = XODR centerline (with t_center offset)?")
    print("  4. Is the transformation segment-dependent?")
    print("="*80)
    
    # Find map files
    script_dir = Path(__file__).parent
    project_root = script_dir.parent
    rd_path = project_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    xodr_path = project_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    
    if not rd_path.exists():
        rd_path = Path("assets/maps/dSPACE/Laguna_Seca.rd")
    if not xodr_path.exists():
        xodr_path = Path("assets/maps/dSPACE/LagunaSeca.xodr")
    
    if not rd_path.exists():
        rd_path = Path("Scenic/assets/maps/dSPACE/Laguna_Seca.rd")
    if not xodr_path.exists():
        xodr_path = Path("Scenic/assets/maps/dSPACE/LagunaSeca.xodr")
    
    if not rd_path.exists():
        print(f"\nERROR: RD file not found: {rd_path}")
        return 1
    
    if not xodr_path.exists():
        print(f"\nERROR: XODR file not found: {xodr_path}")
        return 1
    
    print(f"\nRD file:   {rd_path}")
    print(f"XODR file: {xodr_path}")
    
    # Analyze each road
    for road_name in MAIN_ROAD_NAMES:
        try:
            compute_transformation_analysis(str(rd_path), str(xodr_path), road_name)
        except Exception as e:
            print(f"\n  [ERROR] analyzing {road_name}: {e}")
            import traceback
            traceback.print_exc()
    
    return 0


if __name__ == "__main__":
    sys.exit(main())



