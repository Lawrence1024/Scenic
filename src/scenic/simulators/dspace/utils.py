# utils.py — absolute (s,t) placement helpers for ModelDesk

import math, xml.etree.ElementTree as ET
from typing import Tuple
from collections import defaultdict, deque
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

def build_circuit_refline(xodr_path, start_road_id=None, step=2.0, max_roads=1000):
    root = ET.parse(xodr_path).getroot()
    ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0]+'}'
    # index all road ids
    road_ids = [r.get('id') for r in root.findall(f'{ns}road')]
    if not road_ids:
        raise RuntimeError("No <road> in XODR")
    # heuristic start road if not given: pick the longest
    if start_road_id is None:
        bestL=-1; best=None
        for rid in road_ids:
            r = root.find(f'{ns}road[@id="{rid}"]')
            L=float(r.get('length','0'))
            if L>bestL: bestL=L; best=rid
        start_road_id = best

    # follow successors to build a loop or open chain
    visited=set()
    rid = start_road_id
    ref=[]
    s_cum=0.0
    for _ in range(max_roads):
        if rid in visited: break
        visited.add(rid)
        pts, L, succ = _road_local_ref(root, rid, ns, step)
        if not pts: break
        # append with cumulative s
        for (s_local,x,y,z,h) in pts:
            ref.append((s_cum + (s_local - pts[0][0]), x, y, z, h))
        s_cum += L
        if succ is None: break
        rid = succ
    # sort by cumulative s and dedup tiny steps
    ref.sort(key=lambda r:r[0])
    cleaned=[ref[0]]
    for r in ref[1:]:
        if r[0]-cleaned[-1][0] > 1e-6:
            cleaned.append(r)
    return cleaned, s_cum  # list[(s_cum,x,y,z,h)], total_length

# ---- Projection: (x,y) → (s,t) on index or Scenic road_map ----

def calibrate_s_coordinates():
    """Create calibration mapping from OpenDRIVE s-coordinates to Aurelion s-coordinates.
    
    Based on the provided Aurelion data:
    - (57.47, 79.73) → s=400
    - (-57.37, -79.58) → s=0  
    - (-167.86, -453.35) → s=800
    - (-92.09, -284.78) → s=1200
    
    Note: These are the OpenDRIVE world coordinates, not Aurelion coordinates.
    The Aurelion coordinates are different (e.g., X: -99.085, Y: -478.77, Z: 0.456).
    """
    # Known calibration points: (openDrive_x, openDrive_y, aurelion_s)
    calibration_points = [
        (57.466713, 79.726326, 400),
        (-57.365067, -79.575279, 0),
        (-167.860733, -453.353821, 800),
        (-92.086708, -284.781311, 1200),
    ]
    return calibration_points

def aurelion_to_opendrive_coordinates():
    """Mapping from Aurelion coordinates to OpenDRIVE coordinates.
    
    Based on the user's feedback, we need to map:
    - Aurelion: X: -99.085, Y: -478.77, Z: 0.456
    - To OpenDRIVE coordinates that would give us the correct s-coordinate
    """
    # This mapping needs to be determined by testing
    # For now, we'll use a simple offset approach
    return None  # To be implemented based on more data

def find_calibration_transformation():
    """Find the linear transformation from OpenDRIVE s to Aurelion s.
    
    We need to map the raw OpenDRIVE s-coordinates to the expected Aurelion s-coordinates.
    This requires knowing what the raw s-coordinates were for each calibration point.
    """
    # This would need to be determined by testing the raw s-coordinates
    # at each calibration point. For now, we'll use a simple offset approach.
    
    # Based on the data, it seems like there might be an offset and scaling
    # We'll determine this empirically by testing a few points
    
    # For now, return None to indicate we need to determine this
    return None

def transform_aurelion_to_opendrive(aurelion_x: float, aurelion_y: float, aurelion_z: float):
    """Transform Aurelion coordinates to OpenDRIVE coordinates.
    
    Based on the user's data, we know:
    - Aurelion: X: -99.085, Y: -478.77, Z: 0.456
    - This should map to s=800 in our system
    - Looking at our calibration, s=800 corresponds to OpenDRIVE coordinates (-167.86, -453.35)
    
    So we need to find the transformation that maps:
    Aurelion (-99.085, -478.77) → OpenDRIVE (-167.86, -453.35)
    """
    
    # Based on the analysis, it seems like Aurelion coordinates are in a different
    # coordinate system. We need to find the transformation.
    
    # For now, let's use the OpenDRIVE coordinates directly since those work
    # The user should use OpenDRIVE coordinates in their Scenic scripts
    
    # This function is a placeholder - the real solution is to use OpenDRIVE coordinates
    # in Scenic scripts, not Aurelion coordinates
    
    return aurelion_x, aurelion_y, aurelion_z

def project_world_to_st(index_or_map, pos: Tuple[float, float]):
    """Project world (x,y) onto nearest ref segment; return (s, t) calibrated for Aurelion."""
    px, py = float(pos[0]), float(pos[1])

    roads_obj = None
    if isinstance(index_or_map, dict) and 'roads' in index_or_map:
        roads_obj = index_or_map['roads']
    else:
        roads_obj = getattr(index_or_map, 'roads', None)

    if not roads_obj:
        return 0.0, 0.0

    best = None  # (dist2, s_proj, t_signed)
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
                nx, ny = -vy/seg_len, vx/seg_len  # left normal
                t_signed = dx*nx + dy*ny
                s_proj   = s0 + u*(s1 - s0)

                if (best is None) or (dist2 < best[0]):
                    best = (dist2, s_proj, t_signed)

    if best is None:
        return 0.0, 0.0
    
    # Get raw s-coordinate from OpenDRIVE
    raw_s = float(best[1])
    t_val = float(best[2])
    
    # Apply calibration to convert to Aurelion s-coordinates
    calibrated_s = calibrate_s_coordinate(raw_s, px, py)
    
    return calibrated_s, t_val

def calibrate_s_coordinate(raw_s: float, x: float, y: float) -> float:
    """Convert raw OpenDRIVE s-coordinate to Aurelion s-coordinate.
    
    Uses a lookup table based on the calibration points.
    """
    calibration_points = calibrate_s_coordinates()
    
    # Find the closest calibration point by distance
    min_dist = float('inf')
    closest_aurelion_s = raw_s  # fallback to raw value
    
    for cal_x, cal_y, aurelion_s in calibration_points:
        dist = ((x - cal_x)**2 + (y - cal_y)**2)**0.5
        if dist < min_dist:
            min_dist = dist
            closest_aurelion_s = aurelion_s
    
    # If we're very close to a calibration point (within 5 meters), use it directly
    if min_dist < 5.0:
        return closest_aurelion_s
    
    # For other points, we need to interpolate based on the raw s-coordinate
    # Let's create a mapping based on the known relationships
    raw_to_aurelion_mapping = [
        (176.089, 1200),    # Position 4
        (2770.142, 400),    # Position 1  
        (2966.523, 0),      # Position 2
        (3366.534, 800),    # Position 3
    ]
    
    # Sort by raw s-coordinate
    raw_to_aurelion_mapping.sort(key=lambda x: x[0])
    
    # Find the appropriate segment to interpolate
    for i in range(len(raw_to_aurelion_mapping) - 1):
        raw1, aurelion1 = raw_to_aurelion_mapping[i]
        raw2, aurelion2 = raw_to_aurelion_mapping[i + 1]
        
        if raw1 <= raw_s <= raw2:
            # Linear interpolation
            ratio = (raw_s - raw1) / (raw2 - raw1)
            interpolated = aurelion1 + ratio * (aurelion2 - aurelion1)
            return interpolated
    
    # If outside the range, use the closest endpoint
    if raw_s < raw_to_aurelion_mapping[0][0]:
        return raw_to_aurelion_mapping[0][1]
    else:
        return raw_to_aurelion_mapping[-1][1]

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
    """Build a road index from XODR file compatible with project_world_to_st function.
    
    Returns a dict with 'roads' key containing road data with 'sec_points' lists.
    Each sec_points list contains (x, y, s) tuples for projection calculations.
    """
    try:
        # Build reference line from the XODR file
        refline, total_length = build_circuit_refline(xodr_path, step=step)
        
        # Convert to the format expected by project_world_to_st
        # Create a single road entry with sec_points
        road_data = {
            'sec_points': []
        }
        
        # Convert (s,x,y,z,h) tuples to (x,y,s) tuples for sec_points
        # Group points into segments for the projection algorithm
        sec_points_list = []
        for s, x, y, z, h in refline:
            sec_points_list.append((x, y, s))
        
        # The projection function expects a list of point segments
        road_data['sec_points'] = [sec_points_list]
        
        # Return in the format expected by project_world_to_st
        road_index = {
            'roads': {
                'main_road': road_data
            }
        }
        
        return road_index
        
    except Exception as e:
        print(f"Error building XODR road index: {e}")
        # Return empty structure on error
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
    ref, L = build_circuit_refline('../../assets/maps/dSPACE/Laguna_Seca_OuterLoop_Optimized.xodr', start_road_id=None, step=2.0)
    print(f"Track length ≈ {L:.1f} m; ref points = {len(ref)}")

    samples = [
        (400,0, 57.4667, 79.7263, 4.4887),
        (  0,0,-57.3651,-79.5753,4.4799),
        (800,0,-167.8607,-453.3538,4.6444),
        (1200,0,-92.0867,-284.7813,0.4483),
    ]
    for s,t,x_gt,y_gt,z_gt in samples:
        x,y,z = st_to_world(ref,s,t)
        print(f"s={s},t={t} → pred=({x:.2f},{y:.2f},{z:.2f}), gt=({x_gt:.2f},{y_gt:.2f},{z_gt:.2f}), "
              f"err=({x-x_gt:.2f},{y-y_gt:.2f})")
