"""XODR file parser for dSPACE geometry."""

import math
import xml.etree.ElementTree as ET


def _sample_line(x0,y0,hdg,length,step=2.0):
    n = max(2, int(math.ceil(length/max(step,0.5))))
    for i in range(n+1):
        u=i/n; sL=u*length
        yield (sL,
               x0 + sL*math.cos(hdg),
               y0 + sL*math.sin(hdg),
               0.0,
               hdg)


def _sample_arc(x0,y0,hdg,length,kappa,step=2.0):
    n = max(2, int(math.ceil(length/max(step,0.5))))
    for i in range(n+1):
        u=i/n; sL=u*length
        if abs(kappa)<1e-12:
            x = x0 + sL*math.cos(hdg)
            y = y0 + sL*math.sin(hdg)
            th = hdg
        else:
            R=1.0/kappa
            cx = x0 - R*math.sin(hdg)
            cy = y0 + R*math.cos(hdg)
            th = hdg + kappa*sL
            x = cx + R*math.sin(th)
            y = cy + R*math.cos(th)
        yield (sL,x,y,0.0,th)

def _road_local_ref(root, road_id, ns, step=2.0):
    road = root.find(f'{ns}road[@id="{road_id}"]')
    if road is None: return [], 0.0, None
    pv = road.find(f'{ns}planView')
    if pv is None: return [], 0.0, None
    pts = []
    for g in pv.findall(f'{ns}geometry'):
        s0=float(g.get('s','0')); x0=float(g.get('x','0')); y0=float(g.get('y','0'))
        hdg=float(g.get('hdg','0')); L=float(g.get('length','0'))
        line=g.find(f'{ns}line'); arc=g.find(f'{ns}arc')
        if line is not None:
            seg = _sample_line(x0,y0,hdg,L,step)
        elif arc is not None:
            k=float(arc.get('curvature','0'))
            seg = _sample_arc(x0,y0,hdg,L,k,step)
        else:
            seg = _sample_line(x0,y0,hdg,L,step=1.0)
        for sL,x,y,z,h in seg:
            pts.append((s0+sL,x,y,z,h))
    length = float(road.get('length','0'))
    succ = road.find(f'{ns}link/{ns}successor')
    succ_id = succ.get('elementId') if succ is not None and succ.get('elementType')=='road' else None
    return pts, length, succ_id


def build_xodr_sec_points(xodr_path, step=2.0):
    """Build a road index from XODR file with independent road s-coordinate systems.
    
    Returns a dict with 'roads' key containing road data with 'sec_points' lists.
    Each road gets its own independent s-coordinate system starting from 0.
    This fixes the issue where multiple cars on pit lane get the same s-coordinate.
    """
    try:
        import xml.etree.ElementTree as ET
        
        root = ET.parse(xodr_path).getroot()
        ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0]+'}'
        
        # Get all roads
        road_elements = root.findall(f'{ns}road')
        if not road_elements:
            raise RuntimeError("No <road> elements in XODR")
        
        road_index = {'roads': {}}
        
        # Process each road independently - FILTER TO MAIN ROADS ONLY
        from .utils import MAIN_ROAD_NAMES
        main_road_names = MAIN_ROAD_NAMES
        
        for road_elem in road_elements:
            road_id = road_elem.get('id')
            road_name = road_elem.get('name', f'Road_{road_id}')
            road_length = float(road_elem.get('length', '0'))
            
            # Only process main roads, skip junction roads
            if road_name not in main_road_names:
                continue
                
            if road_length <= 0:
                continue
            
            # Build reference line for this specific road
            pts, L, _ = _road_local_ref(root, road_id, ns, step)
            if not pts:
                continue
            
            # Convert to independent s-coordinate system (starting from 0)
            sec_points_list = []
            for s_local, x, y, z, h in pts:
                # Use local s-coordinate (0 to road_length) instead of cumulative
                sec_points_list.append((x, y, s_local))
            
            # Store road data with independent s-coordinate system
            road_data = {
                'id': road_id,
                'name': road_name,
                'length': road_length,
                'sec_points': [sec_points_list]
            }
            
            # Use road name as key for easier identification
            road_index['roads'][road_name] = road_data
            
            print(f"  Built independent road: {road_name} (ID: {road_id}, Length: {road_length:.1f}m, Points: {len(sec_points_list)})")
        
        print(f"Built {len(road_index['roads'])} independent roads with separate s-coordinate systems")
        return road_index
        
    except Exception as e:
        print(f"Error building XODR road index: {e}")
        return {'roads': {}}

