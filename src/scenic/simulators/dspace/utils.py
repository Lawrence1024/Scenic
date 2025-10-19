# utils.py — absolute (s,t) placement helpers for ModelDesk

import math, xml.etree.ElementTree as ET
from typing import Tuple
import bisect

# ---- collection/segment helpers (same patterns as your originals) ----

def _count_any(coll):
    try:
        return int(getattr(coll, "Count", len(coll)))
    except Exception:
        return 0

def clear_collection(coll):
    n = _count_any(coll)
    for i in reversed(range(n)):
        for m in ("Remove", "Delete", "RemoveAt"):
            if hasattr(coll, m):
                try:
                    getattr(coll, m)(i)
                    break
                except Exception:
                    pass

def ensure_two_segments(sequence):
    segs = sequence.Segments
    while _count_any(segs) < 2:
        if hasattr(segs, "Add"):
            segs.Add()
        else:
            raise RuntimeError("Segments.Add() missing; please pre-create 2 segs in UI.")
    return segs

def activate_type(typed_obj, element_name: str) -> bool:
    try:
        typed_obj.Activate(element_name); return True
    except Exception:
        pass
    avail = getattr(typed_obj, "AvailableElements", None)
    if avail:
        for el in avail:
            if str(el).lower() == element_name.lower():
                typed_obj.Activate(el); return True
    return False

def set_activity_constant(typed_obj, value: float):
    # typed_obj → ActiveElement → SourceType → ActiveElement → Constant
    tgt = typed_obj.ActiveElement
    tgt = tgt.SourceType
    tgt = tgt.ActiveElement
    tgt.Constant = float(value)

def make_endless_transition(segs):
    try:
        conds = segs[1].Transition.Conditions
        for i in reversed(range(_count_any(conds))):
            try: conds.Remove(i)
            except Exception: pass
        conds.Add("Endless")
    except Exception:
        pass

def make_endless_transition_segment(segment):
    """Set up endless transition for a single segment."""
    try:
        tr = segment.Transition
        if hasattr(tr, 'Conditions'):
            conds = tr.Conditions
            # Clear existing conditions
            while conds.Count > 0:
                try:
                    conds.Remove(0)
                except:
                    break
            # Add Endless condition
            if hasattr(conds, 'Add'):
                conds.Add("Endless")
    except Exception as e:
        print(f"    Warning: Could not set up endless transition: {e}")

def find_road_id_for_position(road_index, x, y):
    """Find which road ID a position projects onto.
    
    Args:
        road_index: Road index from build_xodr_sec_points or build_rd_road_index
        x, y: World coordinates
        
    Returns:
        Road ID or None if not found
    """
    try:
        if not road_index:
            return None
            
        roads_obj = road_index.get('roads', {})
        if not roads_obj:
            return None
        
        best_road_id = None
        min_distance = float('inf')
        
        # Check each road to find the closest projection
        for road_name, road_data in roads_obj.items():
            sec_list = road_data.get('sec_points', [])
            if not sec_list:
                continue
                
            for pts in sec_list:
                if not pts or len(pts) < 2:
                    continue
                    
                for i in range(len(pts) - 1):
                    x0, y0, s0 = pts[i]
                    x1, y1, s1 = pts[i+1]
                    vx, vy = x1 - x0, y1 - y0
                    seg_len2 = vx*vx + vy*vy
                    if seg_len2 <= 1e-12:
                        continue
                        
                    wx, wy = x - x0, y - y0
                    u = (wx*vx + wy*vy) / seg_len2
                    u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                    qx = x0 + u*vx
                    qy = y0 + u*vy
                    dx, dy = x - qx, y - qy
                    dist2 = dx*dx + dy*dy
                    
                    if dist2 < min_distance:
                        min_distance = dist2
                        best_road_id = road_data.get('id')
        
        return best_road_id
        
    except Exception as e:
        print(f"    [RoadID] Error finding road ID: {e}")
        return None

# ---- Segment configuration: ABSOLUTE pose on seg0 ----

def configure_seg0_absolute_pose(segs, *, s: float, t: float):
    # Longitudinal: "Position" (absolute along reference line)
    lt0 = segs[0].Activity.LongitudinalType
    if not activate_type(lt0, "Position"):
        # some setups name it "DistancePosition" or similar; try fallback
        if not activate_type(lt0, "DistanceMeter"):
            raise RuntimeError("seg0.LongitudinalType 'Position' not available.")
    set_activity_constant(lt0, s)

    # Lateral: "Deviation" with DependencyType "Absolute" (not Relative)
    lat0 = segs[0].Activity.LateralType
    activate_type(lat0, "Deviation")
    dep = getattr(lat0.ActiveElement, "DependencyType", None)
    if dep is not None:
        # Prefer Absolute; fall back to 'Road' if that's how your MD version names it
        if not activate_type(dep, "Absolute"):
            activate_type(dep, "Road")
    set_activity_constant(lat0, t)

def configure_seg1_motion(segs, *, v: float, t: float):
    # Longitudinal: Velocity (or Speed)
    lt1 = segs[1].Activity.LongitudinalType
    if not activate_type(lt1, "Velocity"):
        activate_type(lt1, "Speed")
    set_activity_constant(lt1, v)

    # Lateral: Continue (don’t change lane)
    lat1 = segs[1].Activity.LateralType
    activate_type(lat1, "Continue")
    # If you want to pin absolute t again here, uncomment:
    # activate_type(lat1, "Deviation")
    # dep = lat1.ActiveElement.DependencyType
    # activate_type(dep, "Absolute")
    # set_activity_constant(lat1, t)

# ---- OpenDRIVE: planView sampling into (x,y,s) polyline(s) ----

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
            y = cy - R*math.cos(th)
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


# ---- Constants ----

# Main road names for Laguna Seca (consistent across XODR and RD paths)
MAIN_ROAD_NAMES = ['The Corkscrew1', 'Pit Lane1_2', 'Andretti Hairpin1_3']

# ---- Projection: (x,y) → (s,t) on index or Scenic road_map ----

# Note: Calibration functions below are legacy and not used in main projection path
# The main project_world_to_st() function uses direct OpenDRIVE s-coordinates
# which provides better accuracy and eliminates clustering issues

def calibrate_s_coordinates():
    """Create calibration mapping from Scenic coordinates to ModelDesk and Aurelion coordinates.
    
    Based on the recorded data from Data.csv:
    Scenic (x,y,z) → ModelDesk (s,t) → Aurelion (x,y,z)
    
    The data shows that ModelDesk s-coordinates are the primary positioning signal,
    while t-coordinates are small lateral offsets.
    
    NOTE: This function is legacy and not used in the main projection path.
    The current implementation uses direct OpenDRIVE s-coordinates for better accuracy.
    """
    # Recorded calibration points: (scenic_x, scenic_y, scenic_z, modeldesk_s, modeldesk_t, aurelion_x, aurelion_y, aurelion_z)
    calibration_points = [
        (-106.456824, -339.457701, 0.0, 1200.0, 0.0, 4.861786, -278.744141, 0.428908),
        (555.676315, -200.333758, 0.0, 648.8248, 6.693, -161.855133, -342.384369, 10.03586),
        (552.493095, -746.136087, 0.0, 830.892, 4.7479, -137.576584, -519.907654, 1.896162),
        (595.17769, -367.843827, 0.0, 711.59, -2.766, -160.898987, -405.132385, 6.904655),
        (128.195399, -307.05382, 0.0, 1128.808, -2.155189, -64.871284, -266.965302, 0.33966),
        (105.074621, -301.809824, 0.0, 1136.1196, 1.9677646, -57.639919, -266.063995, 0.325254),
        (198.335681, -495.033772, 0.0, 1060.8257, -1.2345, -103.311455, -317.357361, 0.474136),
    ]
    return calibration_points


def project_world_to_st(index_or_map, pos: Tuple[float, float], xodr_file: str = None):
    """Project world (x,y) onto nearest ref segment; return (s, t) calibrated for ModelDesk.
    
    This function:
    1. Projects Scenic coordinates onto the OpenDRIVE road network
    2. Uses robust mapping that works across the full track length
    3. Provides full track coverage using the road geometry
    """
    px, py = float(pos[0]), float(pos[1])

    roads_obj = None
    if isinstance(index_or_map, dict) and 'roads' in index_or_map:
        roads_obj = index_or_map['roads']
    else:
        roads_obj = getattr(index_or_map, 'roads', None)

    if not roads_obj:
        # Fallback to simple mapping if no road network available
        return 0.0, 0.0

    # Project onto OpenDRIVE road network - IMPROVED ALGORITHM
    # Collect all projections first, then find the truly closest one
    all_projections = []  # List of (dist2, s_proj, t_signed, road_id, road_name)
    
    it = roads_obj.values() if isinstance(roads_obj, dict) else roads_obj
    for road in it:
        sec_list = road.get('sec_points') if isinstance(road, dict) else getattr(road, 'sec_points', [])
        if not sec_list:
            continue
        for pts in sec_list:
            if not pts or len(pts) < 2:
                continue
            for i in range(len(pts) - 1):
                x0, y0, s0 = pts[i]
                x1, y1, s1 = pts[i+1]
                vx, vy = x1 - x0, y1 - y0
                seg_len2 = vx*vx + vy*vy
                if seg_len2 <= 1e-12:
                    continue
                wx, wy = px - x0, py - y0
                u = (wx*vx + wy*vy) / seg_len2
                u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                qx = x0 + u*vx
                qy = y0 + u*vy
                dx, dy = px - qx, py - qy
                dist2 = dx*dx + dy*dy

                seg_len = seg_len2 ** 0.5
                # Calculate normal vector for t-coordinate
                nx_left, ny_left = -vy/seg_len, vx/seg_len  # left normal
                
                # Calculate t-coordinate using geometric projection with scaling
                # Scale down t-coordinates to match expected road width (typically 3-4m lanes)
                raw_t = dx*nx_left + dy*ny_left
                t_signed = raw_t * 0.3  # Scale factor to match calibration data
                s_proj = s0 + u*(s1 - s0)
                
                # Get road ID and name
                road_id = road.get('id') if isinstance(road, dict) else getattr(road, 'id', None)
                road_name = road.get('name') if isinstance(road, dict) else getattr(road, 'name', f'Road_{road_id}')
                
                # Store all projections
                all_projections.append((dist2, s_proj, t_signed, road_id, road_name))
    
    # Find the truly closest projection
    if not all_projections:
        # Fallback to simple mapping if projection fails
        return 0.0, 0.0
    
    # Sort by distance and take the closest
    all_projections.sort(key=lambda x: x[0])
    best = all_projections[0]
    

    if best is None:
        # Fallback to simple mapping if projection fails
        return 0.0, 0.0
    
    # Get raw s-coordinate from OpenDRIVE
    raw_s = float(best[1])
    t_val = float(best[2])
    road_id = best[3]
    
    # DIRECT APPROACH: Use OpenDRIVE s directly for full track coverage
    # This provides the full 0-2484.6m range without calibration capping
    return raw_s, t_val





def build_refline(path, step=2.0):
    tree = ET.parse(path); root = tree.getroot()
    ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0]+'}'
    pts = []
    for geom in root.findall(f'{ns}road/{ns}planView/{ns}geometry'):
        s0 = float(geom.get('s')); x0=float(geom.get('x')); y0=float(geom.get('y'))
        hdg=float(geom.get('hdg')); length=float(geom.get('length'))
        line = geom.find(f'{ns}line'); arc = geom.find(f'{ns}arc')
        n = max(2, int(math.ceil(length/step)))
        for i in range(n+1):
            u = i/n; s = s0 + u*length
            if line is not None:
                x = x0 + u*length*math.cos(hdg)
                y = y0 + u*length*math.sin(hdg)
                h = hdg
            elif arc is not None:
                kappa = float(arc.get('curvature'))
                if abs(kappa)<1e-12:
                    x = x0 + u*length*math.cos(hdg)
                    y = y0 + u*length*math.sin(hdg)
                    h = hdg
                else:
                    R = 1/kappa
                    cx = x0 - R*math.sin(hdg)
                    cy = y0 + R*math.cos(hdg)
                    th = hdg + kappa*u*length
                    x = cx + R*math.sin(th)
                    y = cy - R*math.cos(th)
                    h = th
            else:
                x,y,h = x0,y0,hdg
            pts.append((s,x,y,0.0,h))  # z=0 unless you parse elevation
    return pts

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


def st_to_world(refline, s, t=0.0):
    S=[r[0] for r in refline]
    i = bisect.bisect_left(S, s)
    if i<=0: i=1
    if i>=len(S): i=len(S)-1
    s0,x0,y0,z0,h0 = refline[i-1]
    s1,x1,y1,z1,h1 = refline[i]
    u = (s - s0)/(s1 - s0) if s1>s0 else 0.0
    xr = x0 + u*(x1-x0)
    yr = y0 + u*(y1-y0)
    zr = z0 + u*(z1-z0)
    hr = h0 + u*(h1-h0)
    nx, ny = -math.sin(hr), math.cos(hr)  # left normal
    return xr + t*nx, yr + t*ny, zr

if __name__ == "__main__":
    # Test the new independent road system
    road_index = build_xodr_sec_points('../../assets/maps/dSPACE/LagunaSeca.xodr', step=2.0)
    print(f"Built road index with {len(road_index['roads'])} roads")
    for road_name, road_data in road_index['roads'].items():
        print(f"  {road_name}: {road_data['length']:.1f}m, {len(road_data['sec_points'][0])} points")
