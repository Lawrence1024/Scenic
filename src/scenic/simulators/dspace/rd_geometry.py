"""
RD File Geometry Parser for Direct Use in Scenic
This bypasses the XODR conversion and uses Aurelion's native coordinate system.
"""

import math
import xml.etree.ElementTree as ET
from typing import Tuple, List, Optional

NS = {'r': 'http://www.dspace.com/XMLSchema/ScenarioAccess/Scenario/Road'}


def _coeff2d(seg, tag: str) -> Tuple[float, float]:
    """Extract 2D coefficient from RD segment."""
    el = seg.find(f'r:{tag}', NS)
    return float(el.find('r:X', NS).text), float(el.find('r:Y', NS).text)


def parse_rd_geometry(rd_path: str, step: float = 1.0) -> List[dict]:
    """Parse RD file and return road geometries in Aurelion's native coordinate system.
    
    Args:
        rd_path: Path to .rd file
        step: Sampling step size in meters (smaller = more accurate)
        
    Returns:
        List of road dictionaries with 'sec_points' containing (x, y, s) tuples
    """
    root = ET.parse(rd_path).getroot()
    roads_elem = root.find('r:Roads', NS)
    if roads_elem is None:
        raise RuntimeError("No <Roads> in RD file")
    
    all_roads = []
    
    for road_idx, road in enumerate(list(roads_elem)):
        segs = road.find('r:Segments', NS)
        if segs is None:
            continue
        
        s_cum = 0.0
        sec_points = []
        
        for seg in list(segs):
            sp = seg.find('r:AbsoluteStartPosition', NS)
            if sp is None:
                continue
            
            # Get segment parameters
            x0 = float(sp.find('r:X', NS).text)
            y0 = float(sp.find('r:Y', NS).text)
            th0 = math.radians(float(sp.find('r:Tangent', NS).text))
            L = float(seg.find('r:Length', NS).text)
            
            # Get cubic polynomial coefficients
            Ax, Ay = _coeff2d(seg, 'A')
            Bx, By = _coeff2d(seg, 'B')
            Cx, Cy = _coeff2d(seg, 'C')
            Dx, Dy = _coeff2d(seg, 'D')
            
            cos0, sin0 = math.cos(th0), math.sin(th0)
            
            def to_world(px, py):
                """Transform local coordinates to world coordinates."""
                return (x0 + px*cos0 - py*sin0, y0 + px*sin0 + py*cos0)
            
            # Sample the segment
            n = max(2, int(math.ceil(L / max(step, 0.1))))
            prev_xy = None
            
            for i in range(n + 1):
                u = i / n
                # Evaluate cubic polynomial
                px = Ax + Bx*u + Cx*u*u + Dx*u*u*u
                py = Ay + By*u + Cy*u*u + Dy*u*u*u
                xw, yw = to_world(px, py)
                
                if prev_xy is None:
                    sec_points.append((xw, yw, s_cum))
                else:
                    ds = math.hypot(xw - prev_xy[0], yw - prev_xy[1])
                    s_cum += ds
                    sec_points.append((xw, yw, s_cum))
                prev_xy = (xw, yw)
        
        if sec_points:
            # Deduplicate tiny steps
            clean_points = [sec_points[0]]
            for pt in sec_points[1:]:
                if pt[2] - clean_points[-1][2] > 1e-6:
                    clean_points.append(pt)
            
            all_roads.append({
                'id': road_idx,
                'sec_points': [clean_points],  # Wrap in list for compatibility with project_world_to_st
                'total_length': s_cum
            })
    
    return all_roads


def build_rd_road_index(rd_path: str, step: float = 1.0) -> dict:
    """Build road index from RD file compatible with project_world_to_st function.
    
    Returns a dict with 'roads' key containing road data with 'sec_points' lists.
    This uses Aurelion's native RD coordinate system instead of converted XODR.
    
    Args:
        rd_path: Path to .rd file
        step: Sampling step size (smaller = more accurate, default 1.0m recommended)
        
    Returns:
        Road index dict compatible with existing projection functions
    """
    roads = parse_rd_geometry(rd_path, step=step)
    
    # Find the main/longest road (usually the racing circuit)
    if not roads:
        raise RuntimeError("No roads found in RD file")
    
    main_road = max(roads, key=lambda r: r['total_length'])
    
    # Return in format expected by project_world_to_st
    return {
        'roads': {
            'main_road': main_road
        }
    }


def project_world_to_st_rd(rd_index: dict, pos: Tuple[float, float]) -> Tuple[float, float]:
    """Project world (x,y) onto RD road geometry to get (s,t).
    
    This function uses the NATIVE RD coordinate system, ensuring perfect alignment
    with Aurelion's internal representation.
    
    Args:
        rd_index: Road index from build_rd_road_index()
        pos: (x, y) world coordinates
        
    Returns:
        (s, t) road coordinates in Aurelion's native system
    """
    # Import the existing projection function
    from . import utils as dutils
    
    # Use the existing projection algorithm, but with RD geometry
    return dutils.project_world_to_st(rd_index, pos)


if __name__ == "__main__":
    # Test the RD geometry parser
    rd_path = '../../../assets/maps/dSPACE/Laguna_Seca.rd'
    
    print("Testing RD Geometry Parser")
    print("=" * 80)
    
    # Build index with fine sampling
    rd_index = build_rd_road_index(rd_path, step=0.5)
    
    main_road = rd_index['roads']['main_road']
    print(f"Main road length: {main_road['total_length']:.2f}m")
    print(f"Sample points: {len(main_road['sec_points'][0])}")
    
    # Test projection
    test_points = [
        (-101.92, -457.52),  # Should be near s=0
        (-109.05, -412.06),  # Should be near s=50
        (200.60, -826.06),   # Should be near s=1000
    ]
    
    print("\nTest Projections:")
    for x, y in test_points:
        s, t = project_world_to_st_rd(rd_index, (x, y))
        print(f"  ({x:7.2f}, {y:7.2f}) -> s={s:7.2f}m, t={t:5.2f}m")

