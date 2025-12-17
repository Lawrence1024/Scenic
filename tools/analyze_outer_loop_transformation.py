#!/usr/bin/env python3
"""
Focused analysis of outer loop transformation (The Corkscrew1 and Andretti Hairpin1_3).

This script:
1. Focuses only on the outer loop roads (excludes Pit Lane)
2. Uses s-coordinate matching instead of closest-point matching
3. Compares RD reference line vs XODR reference line
4. Compares RD reference line vs XODR centerline (with t_center)
5. Analyzes if transformation is segment-dependent
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

# Focus on outer loop only
OUTER_LOOP_ROADS = ['The Corkscrew1', 'Andretti Hairpin1_3']


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
    """Get width and t_center from XODR."""
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
                pts.append((xW, yW, s_cum, hdg_world))
            else:
                ds_actual = math.hypot(xW - prev_xy[0], yW - prev_xy[1])
                s_cum += ds_actual
                pts.append((xW, yW, s_cum, hdg_world))
            prev_xy = (xW, yW)
    
    return pts


def sample_xodr_reference_line(xodr_path: str, road_name: str, ds: float = 0.5) -> List[Tuple[float, float, float, float]]:
    """Sample XODR reference line with headings."""
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


def find_point_at_s(points: List[Tuple[float, float, float, float]], s_target: float) -> Tuple[float, float, float]:
    """Find point at target s-coordinate using linear interpolation.
    
    Returns (x, y, heading) or None if s_target is out of range.
    """
    if not points:
        return None
    
    # Find surrounding points
    for i in range(len(points) - 1):
        s1 = points[i][2]
        s2 = points[i + 1][2]
        
        if s1 <= s_target <= s2:
            # Linear interpolation
            if s2 == s1:
                return (points[i][0], points[i][1], points[i][3])
            
            t = (s_target - s1) / (s2 - s1)
            x = points[i][0] + t * (points[i + 1][0] - points[i][0])
            y = points[i][1] + t * (points[i + 1][1] - points[i][1])
            hdg = points[i][3] + t * (points[i + 1][3] - points[i][3])
            return (x, y, hdg)
    
    # If s_target is beyond the last point, return last point
    if s_target >= points[-1][2]:
        return (points[-1][0], points[-1][1], points[-1][3])
    
    # If s_target is before first point, return first point
    return (points[0][0], points[0][1], points[0][3])


def analyze_outer_loop(rd_path: str, xodr_path: str, road_name: str):
    """Analyze transformation for outer loop road using s-coordinate matching."""
    print(f"\n{'='*80}")
    print(f"OUTER LOOP ANALYSIS: {road_name}")
    print(f"{'='*80}")
    
    # Sample reference lines
    print(f"\n  Step 1: Sampling reference lines...")
    rd_ref = sample_rd_reference_line(rd_path, road_name, ds=0.5)
    xodr_ref = sample_xodr_reference_line(xodr_path, road_name, ds=0.5)
    
    print(f"    RD points: {len(rd_ref)}, length={rd_ref[-1][2]:.2f}m")
    print(f"    XODR points: {len(xodr_ref)}, length={xodr_ref[-1][2]:.2f}m")
    
    # Find common s-range
    rd_length = rd_ref[-1][2]
    xodr_length = xodr_ref[-1][2]
    max_s = min(rd_length, xodr_length)
    
    print(f"    Common length: {max_s:.2f}m")
    
    # Sample at common s-coordinates
    print(f"\n  Step 2: Comparing at common s-coordinates...")
    num_samples = 200
    s_samples = np.linspace(0, max_s, num_samples)
    
    ref_errors = []
    centerline_errors = []
    t_centers = []
    
    for s in s_samples:
        # Get RD point at this s
        rd_pt = find_point_at_s(rd_ref, s)
        if rd_pt is None:
            continue
        
        # Get XODR reference point at this s
        xodr_pt = find_point_at_s(xodr_ref, s)
        if xodr_pt is None:
            continue
        
        # Compare reference lines
        ref_error = math.hypot(rd_pt[0] - xodr_pt[0], rd_pt[1] - xodr_pt[1])
        ref_errors.append(ref_error)
        
        # Get t_center and compute XODR centerline
        width, t_center = road_width_and_center_t(xodr_path, road_name, s)
        t_centers.append(t_center)
        
        # Compute centerline point
        hdg = xodr_pt[2]
        nx = -math.sin(hdg)  # Left normal
        ny = math.cos(hdg)
        x_center = xodr_pt[0] + t_center * nx
        y_center = xodr_pt[1] + t_center * ny
        
        # Compare to RD reference line
        centerline_error = math.hypot(rd_pt[0] - x_center, rd_pt[1] - y_center)
        centerline_errors.append(centerline_error)
    
    ref_errors = np.array(ref_errors)
    centerline_errors = np.array(centerline_errors)
    t_centers = np.array(t_centers)
    
    print(f"\n  Step 3: Results...")
    print(f"\n    Reference Line Comparison (RD ref vs XODR ref):")
    print(f"      Mean error: {np.mean(ref_errors):.2e} m")
    print(f"      Max error:  {np.max(ref_errors):.2e} m")
    print(f"      Median:    {np.median(ref_errors):.2e} m")
    print(f"      Std dev:   {np.std(ref_errors):.2e} m")
    
    print(f"\n    Centerline Comparison (RD ref vs XODR centerline):")
    print(f"      Mean error: {np.mean(centerline_errors):.2e} m")
    print(f"      Max error:  {np.max(centerline_errors):.2e} m")
    print(f"      Median:    {np.median(centerline_errors):.2e} m")
    print(f"      Std dev:   {np.std(centerline_errors):.2e} m")
    
    print(f"\n    t_center Statistics:")
    print(f"      Mean: {np.mean(t_centers):.2f} m")
    print(f"      Range: {np.min(t_centers):.2f} m - {np.max(t_centers):.2f} m")
    print(f"      Std dev: {np.std(t_centers):.2f} m")
    
    # Analysis
    print(f"\n  Step 4: Analysis...")
    
    ref_mean = np.mean(ref_errors)
    centerline_mean = np.mean(centerline_errors)
    ref_max = np.max(ref_errors)
    centerline_max = np.max(centerline_errors)
    
    if ref_max < 1e-10:
        print(f"    [IDENTITY] RD reference line = XODR reference line")
        print(f"      - No transformation needed")
        print(f"      - Tracks match at reference line level")
    elif centerline_max < ref_max * 0.1 and centerline_max < 0.01:
        print(f"    [NON-IDENTITY] RD reference line = XODR centerline")
        print(f"      - Transformation: RD_ref = XODR_ref + t_center(s)")
        print(f"      - Average t_center: {np.mean(t_centers):.2f} m")
        print(f"      - This suggests RD stores centerlines, XODR stores reference lines")
    elif ref_mean < 0.01 and centerline_mean < 0.01:
        print(f"    [BOTH MATCH] Both reference line and centerline match well")
        print(f"      - This suggests tracks are essentially identical")
        print(f"      - Small differences may be due to numerical precision")
    elif ref_mean < centerline_mean * 0.5:
        print(f"    [PREFER REF] Reference lines match better than centerlines")
        print(f"      - Suggests identity transformation (RD ref = XODR ref)")
        print(f"      - But centerline differences suggest width/offset issues")
    elif centerline_mean < ref_mean * 0.5:
        print(f"    [PREFER CENTERLINE] Centerlines match better than reference lines")
        print(f"      - Suggests non-identity transformation")
        print(f"      - RD may store centerlines while XODR stores reference lines")
    else:
        print(f"    [UNCLEAR] Neither clearly matches")
        print(f"      - May indicate coordinate system mismatch")
        print(f"      - Or different geometry definitions")
    
    # Segment analysis (if t_center varies significantly)
    if np.std(t_centers) > 0.1:
        print(f"\n  Step 5: Segment-dependent analysis...")
        print(f"    t_center varies significantly (std={np.std(t_centers):.2f}m)")
        print(f"    This suggests transformation may be segment-dependent")
        
        # Find regions where t_center changes
        t_center_changes = []
        for i in range(1, len(t_centers)):
            if abs(t_centers[i] - t_centers[i-1]) > 0.05:  # Significant change
                s = s_samples[i]
                t_center_changes.append((s, t_centers[i-1], t_centers[i]))
        
        if t_center_changes:
            print(f"    Found {len(t_center_changes)} significant t_center changes:")
            for s, t_old, t_new in t_center_changes[:5]:  # Show first 5
                print(f"      At s={s:.1f}m: {t_old:.2f}m -> {t_new:.2f}m")
    
    return {
        'road_name': road_name,
        'ref_mean': ref_mean,
        'ref_max': ref_max,
        'centerline_mean': centerline_mean,
        'centerline_max': centerline_max,
        't_center_mean': np.mean(t_centers),
        't_center_std': np.std(t_centers),
    }


def main():
    """Main function."""
    print("="*80)
    print("OUTER LOOP TRANSFORMATION ANALYSIS")
    print("="*80)
    print("\nFocusing on outer loop roads only:")
    print("  - The Corkscrew1")
    print("  - Andretti Hairpin1_3")
    print("\nUsing s-coordinate matching for accurate comparison")
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
    
    # Analyze outer loop roads
    all_stats = []
    for road_name in OUTER_LOOP_ROADS:
        try:
            stats = analyze_outer_loop(str(rd_path), str(xodr_path), road_name)
            all_stats.append(stats)
        except Exception as e:
            print(f"\n  [ERROR] analyzing {road_name}: {e}")
            import traceback
            traceback.print_exc()
    
    # Summary
    print(f"\n{'='*80}")
    print("SUMMARY")
    print(f"{'='*80}")
    
    if all_stats:
        for stats in all_stats:
            print(f"\n  {stats['road_name']}:")
            print(f"    Ref line error:     mean={stats['ref_mean']:.2e}m, max={stats['ref_max']:.2e}m")
            print(f"    Centerline error:   mean={stats['centerline_mean']:.2e}m, max={stats['centerline_max']:.2e}m")
            print(f"    t_center:           mean={stats['t_center_mean']:.2f}m, std={stats['t_center_std']:.2f}m")
        
        overall_ref_max = max(s['ref_max'] for s in all_stats)
        overall_centerline_max = max(s['centerline_max'] for s in all_stats)
        
        print(f"\n  Overall:")
        print(f"    Max ref line error:     {overall_ref_max:.2e} m")
        print(f"    Max centerline error:   {overall_centerline_max:.2e} m")
        
        if overall_ref_max < 1e-10:
            print(f"\n  [CONCLUSION] Identity transformation confirmed")
            print(f"    RD reference lines = XODR reference lines")
        elif overall_centerline_max < overall_ref_max * 0.1:
            print(f"\n  [CONCLUSION] Non-identity transformation")
            print(f"    RD reference lines = XODR centerlines")
            print(f"    Transformation: RD_ref = XODR_ref + t_center(s)")
        else:
            print(f"\n  [CONCLUSION] Complex relationship")
            print(f"    Neither simple identity nor centerline transformation")
            print(f"    May require further investigation")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())



