# rd_to_xodr.py
# Read a dSPACE RoadNetwork (.rd) file → sample a refline → write CSV and OpenDRIVE (.xodr)
# Usage:
#   python rd_to_xodr.py --rd Laguna_Seca.rd --xodr LS_converted.xodr --csv LS_refline.csv
#
# Notes:
# - The .rd is treated as authoritative geometry. We simply resample it (step meters)
#   and export the result to XODR with <line/> geometries.
# - The XODR is exported as a single closed <road> with self links and a minimal laneSection.

import math
import argparse
import xml.etree.ElementTree as ET
from typing import Tuple, List
import numpy as np
import csv
import sys

NS = {'r': 'http://www.dspace.com/XMLSchema/ScenarioAccess/Scenario/Road'}

# ---------- RD → refline (s, x, y, heading) ----------

def _coeff2d(seg, tag: str) -> Tuple[float, float]:
    el = seg.find(f'r:{tag}', NS)
    return float(el.find('r:X', NS).text), float(el.find('r:Y', NS).text)

def rd_to_reflines(path: str, step: float = 2.0) -> list:
    """Return a list of ref arrays, one per RD <Road>: each is (N,4) [s,x,y,heading]."""
    root = ET.parse(path).getroot()
    roads = root.find('r:Roads', NS)
    if roads is None:
        raise RuntimeError("No <Roads> in RD file")

    reflist = []
    for road in list(roads):
        segs = road.find('r:Segments', NS)
        if segs is None:
            continue

        s_cum = 0.0
        ref = []
        for seg in list(segs):
            sp = seg.find('r:AbsoluteStartPosition', NS)
            if sp is None:
                continue
            x0 = float(sp.find('r:X', NS).text)
            y0 = float(sp.find('r:Y', NS).text)
            th0 = math.radians(float(sp.find('r:Tangent', NS).text))
            L  = float(seg.find('r:Length', NS).text)

            Ax, Ay = _coeff2d(seg, 'A')
            Bx, By = _coeff2d(seg, 'B')
            Cx, Cy = _coeff2d(seg, 'C')
            Dx, Dy = _coeff2d(seg, 'D')

            cos0, sin0 = math.cos(th0), math.sin(th0)
            def to_world(px, py):
                return (x0 + px*cos0 - py*sin0, y0 + px*sin0 + py*cos0)

            n = max(2, int(math.ceil(L / max(step, 0.5))))
            prev_xy = None
            for i in range(n + 1):
                u = i / n
                px = Ax + Bx*u + Cx*u*u + Dx*u*u*u
                py = Ay + By*u + Cy*u*u + Dy*u*u*u
                xw, yw = to_world(px, py)
                tx = Bx + 2*Cx*u + 3*Dx*u*u
                ty = By + 2*Cy*u + 3*Dy*u*u
                th = math.atan2(ty, tx) + th0

                if prev_xy is None:
                    ref.append((s_cum, xw, yw, th))
                else:
                    ds = math.hypot(xw - prev_xy[0], yw - prev_xy[1])
                    s_cum += ds
                    ref.append((s_cum, xw, yw, th))
                prev_xy = (xw, yw)

        if ref:
            # dedup tiny steps
            clean = [ref[0]]
            for r in ref[1:]:
                if r[0] - clean[-1][0] > 1e-6:
                    clean.append(r)
            reflist.append(np.array(clean))
    if not reflist:
        raise RuntimeError("No segments sampled from RD file.")
    return reflist



# ---------- Write CSV for sanity ----------

def write_refline_csv(ref: np.ndarray, csv_path: str) -> None:
    with open(csv_path, 'w', newline='') as f:
        w = csv.writer(f)
        w.writerow(['s_m', 'x_m', 'y_m', 'heading_rad'])
        for s, x, y, h in ref:
            w.writerow([f'{s:.6f}', f'{x:.6f}', f'{y:.6f}', f'{h:.12f}'])


# ---------- Racing Circuit Layout Functions ----------

def find_closest_points_on_tracks(lap_ref: np.ndarray, pit_ref: np.ndarray, num_candidates: int = 10):
    """Find potential diverge/converge points between lap and pit lanes."""
    lap_xy = lap_ref[:, 1:3]  # x,y coordinates
    pit_xy = pit_ref[:, 1:3]  # x,y coordinates
    
    # Find all close point pairs
    distances = []
    for i, lap_pt in enumerate(lap_xy):
        for j, pit_pt in enumerate(pit_xy):
            dist = np.linalg.norm(lap_pt - pit_pt)
            distances.append((dist, i, j, lap_pt, pit_pt))
    
    # Sort by distance and return top candidates
    distances.sort(key=lambda x: x[0])
    return distances[:num_candidates]

def create_junction_connections(lap_ref: np.ndarray, pit_ref: np.ndarray, diverge_point, converge_point):
    """Create connecting roads between lap and pit lanes at diverge/converge points."""
    # Extract junction points
    lap_div_idx, pit_div_idx = diverge_point[1], diverge_point[2]
    lap_conv_idx, pit_conv_idx = converge_point[1], converge_point[2]
    
    # Create short connecting segments
    # Diverge: lap -> pit
    div_connector = np.array([
        lap_ref[lap_div_idx, 1:3],   # lap point
        pit_ref[pit_div_idx, 1:3]    # pit point
    ])
    
    # Converge: pit -> lap  
    conv_connector = np.array([
        pit_ref[pit_conv_idx, 1:3],  # pit point
        lap_ref[lap_conv_idx, 1:3]   # lap point
    ])
    
    return div_connector, conv_connector, (lap_div_idx, pit_div_idx), (lap_conv_idx, pit_conv_idx)

def create_junction_xml(od, junction_id: int, connection_roads: list):
    """Create OpenDRIVE junction XML element."""
    def elem(tag, **attrs):
        e = ET.Element(tag)
        for k, v in attrs.items():
            e.set(k, str(v))
        return e
    
    junction = elem('junction', id=str(junction_id), name=f'Junction_{junction_id}')
    
    for conn_id, (incoming_road, connecting_road, contact_point) in enumerate(connection_roads):
        connection = elem('connection', id=str(conn_id), 
                         incomingRoad=str(incoming_road),
                         connectingRoad=str(connecting_road),
                         contactPoint=contact_point)
        
        # Add lane links (simplified - connect lane -1 to lane -1)
        lane_link = elem('laneLink')
        lane_link.set('from', '-1')
        lane_link.set('to', '-1')
        connection.append(lane_link)
        junction.append(connection)
    
    od.append(junction)
    return junction

# ---------- Refline → XODR ----------

def refline_to_polyline(ref: np.ndarray, ds: float = 2.0) -> np.ndarray:
    """
    Resample the refline to approximately uniform spacing (for XODR line segments).
    Returns Nx2 array of XY points (closed if input path is closed).
    """
    S = ref[:, 0]
    X = ref[:, 1]
    Y = ref[:, 2]
    L = S[-1]
    n = max(2, int(round(L / max(ds, 0.2))))
    Su = np.linspace(0.0, L, n + 1)  # include L
    Xu = np.interp(Su, S, X)
    Yu = np.interp(Su, S, Y)
    # ensure strictly closed (last point = first)
    Xu[-1], Yu[-1] = Xu[0], Yu[0]
    return np.column_stack([Xu, Yu])

def write_xodr_from_polyline(XY: np.ndarray, out_path: str, lane_width: float = 3.7) -> None:
    """
    Emit a minimal OpenDRIVE file:
      - one closed <road id="0">
      - planView with <geometry><line/></geometry> pieces along XY
      - self predecessor/successor links
      - minimal laneSection with one lane per side (constant width)
    """
    # drop zero-length edges, compute segs
    d = np.sqrt(np.sum(np.diff(XY, axis=0)**2, axis=1))
    keep = d > 1e-9
    XY = XY[np.r_[True, keep]]
    d = np.sqrt(np.sum(np.diff(XY, axis=0)**2, axis=1))
    hdg = np.arctan2(np.diff(XY[:, 1]), np.diff(XY[:, 0]))
    sCum = np.r_[0.0, np.cumsum(d)]
    road_len = float(sCum[-1])

    # XML build
    def elem(tag, **attrs):
        e = ET.Element(tag)
        for k, v in attrs.items():
            e.set(k, str(v))
        return e

    od = elem('OpenDRIVE')
    header = elem('header', revMajor='1', revMinor='6', name='', version='1.6',
                  date='', north='0', south='0', east='0', west='0')
    od.append(header)

    road = elem('road', name='From_RD', length=f'{road_len:.6f}', id='0', junction='0')
    od.append(road)

    # closed-loop links
    link = elem('link')
    pred = elem('predecessor', elementType='road', elementId='0', contactPoint='end')
    succ = elem('successor',   elementType='road', elementId='0', contactPoint='start')
    link.append(pred); link.append(succ)
    road.append(link)

    rtype = elem('type', s='0', type='rural')
    road.append(rtype)

    pv = elem('planView')
    road.append(pv)

    s0 = 0.0
    for i in range(len(d)):  # one geometry per edge
        g = elem('geometry',
                 s=f'{s0:.6f}',
                 x=f'{XY[i,0]:.6f}',
                 y=f'{XY[i,1]:.6f}',
                 hdg=f'{hdg[i]:.12f}',
                 length=f'{d[i]:.6f}')
        line = elem('line')
        g.append(line)
        pv.append(g)
        s0 += float(d[i])

    lanes = elem('lanes')
    road.append(lanes)

    laneOffset = elem('laneOffset', s='0', a='0', b='0', c='0', d='0')
    lanes.append(laneOffset)

    ls = elem('laneSection', s='0')
    lanes.append(ls)

    center = elem('center'); ls.append(center)
    lane0 = elem('lane', id='0', type='none', level='false'); center.append(lane0)
    rm0 = elem('roadMark', sOffset='0', type='solid', weight='standard', color='standard', width='0.13')
    lane0.append(rm0)

    left = elem('left'); ls.append(left)
    l1 = elem('lane', id='1', type='driving', level='false'); left.append(l1)
    
    # Add lane links for left lane (self-loop)
    l1_link = elem('link')
    l1_pred = elem('predecessor', id='-1')
    l1_succ = elem('successor', id='-1')
    l1_link.append(l1_pred); l1_link.append(l1_succ)
    l1.append(l1_link)
    
    rml = elem('roadMark', sOffset='0', type='broken', weight='standard', color='standard', width='0.13', laneChange='both')
    l1.append(rml)
    w = elem('width', sOffset='0', a=f'{lane_width:.3f}', b='0', c='0', d='0'); l1.append(w)

    right = elem('right'); ls.append(right)
    r1 = elem('lane', id='-1', type='driving', level='false'); right.append(r1)
    
    # Add lane links for right lane (self-loop)
    r1_link = elem('link')
    r1_pred = elem('predecessor', id='1')
    r1_succ = elem('successor', id='1')
    r1_link.append(r1_pred); r1_link.append(r1_succ)
    r1.append(r1_link)
    
    rmr = elem('roadMark', sOffset='0', type='broken', weight='standard', color='standard', width='0.13', laneChange='both')
    r1.append(rmr)
    w2 = elem('width', sOffset='0', a=f'{lane_width:.3f}', b='0', c='0', d='0'); r1.append(w2)

    # write file
    tree = ET.ElementTree(od)
    ET.indent(tree, space="  ", level=0)  # Python 3.9+
    tree.write(out_path, encoding='utf-8', xml_declaration=True)


def resample_xy(ref: np.ndarray, ds: float, closed: bool = True) -> np.ndarray:
    S, X, Y = ref[:,0], ref[:,1], ref[:,2]
    L = S[-1]
    n = max(2, int(round(L / max(ds, 0.2))))
    Su = np.linspace(0.0, L, n + 1)
    Xu = np.interp(Su, S, X)
    Yu = np.interp(Su, S, Y)
    
    # Only force closure for closed roads
    if closed:
        Xu[-1], Yu[-1] = Xu[0], Yu[0]
    
    return np.column_stack([Xu, Yu])

def append_road_xml(od, road_id: int, XY: np.ndarray, name="Road", closed=True, lane_width=3.7, 
                   predecessor=None, successor=None, junction_id="-1"):
    def elem(tag, **attrs):
        e = ET.Element(tag); [e.set(k,str(v)) for k,v in attrs.items()]; return e
    d = np.sqrt(np.sum(np.diff(XY, axis=0)**2, axis=1))
    keep = d > 1e-9
    XY = XY[np.r_[True, keep]]
    d = np.sqrt(np.sum(np.diff(XY, axis=0)**2, axis=1))
    hdg = np.arctan2(np.diff(XY[:,1]), np.diff(XY[:,0]))
    sCum = np.r_[0.0, np.cumsum(d)]
    road_len = float(sCum[-1])

    road = elem('road', name=name, length=f'{road_len:.6f}', id=str(road_id), junction=str(junction_id))
    od.append(road)

    # Handle links based on parameters
    link = elem('link')
    has_links = False
    
    if predecessor is not None:
        if isinstance(predecessor, dict):
            pred = elem('predecessor', **predecessor)
        else:
            pred = elem('predecessor', elementType='road', elementId=str(predecessor), contactPoint='end')
        link.append(pred)
        has_links = True
        
    if successor is not None:
        if isinstance(successor, dict):
            succ = elem('successor', **successor)
        else:
            succ = elem('successor', elementType='road', elementId=str(successor), contactPoint='start')
        link.append(succ)
        has_links = True
        
    if closed and predecessor is None and successor is None:
        # Self-loop for closed roads
        pred = elem('predecessor', elementType='road', elementId=str(road_id), contactPoint='end')
        succ = elem('successor',   elementType='road', elementId=str(road_id), contactPoint='start')
        link.append(pred); link.append(succ)
        has_links = True
    
    if has_links:
        road.append(link)

    rtype = elem('type', s='0', type='rural'); road.append(rtype)
    pv = elem('planView'); road.append(pv)

    s0 = 0.0
    for i in range(len(d)):
        g = elem('geometry',
                 s=f'{s0:.6f}', x=f'{XY[i,0]:.6f}', y=f'{XY[i,1]:.6f}',
                 hdg=f'{hdg[i]:.12f}', length=f'{d[i]:.6f}')
        g.append(elem('line')); pv.append(g); s0 += float(d[i])

    lanes = elem('lanes'); road.append(lanes)
    lanes.append(elem('laneOffset', s='0', a='0', b='0', c='0', d='0'))
    ls = elem('laneSection', s='0'); lanes.append(ls)
    center = elem('center'); ls.append(center)
    lane0 = elem('lane', id='0', type='none', level='false'); center.append(lane0)
    lane0.append(elem('roadMark', sOffset='0', type='solid', weight='standard', color='standard', width='0.13'))
    left = elem('left'); ls.append(left)
    l1 = elem('lane', id='1', type='driving', level='false'); left.append(l1)
    
    # Add proper lane links for left lane
    l1_link = elem('link')
    if predecessor is not None:
        l1_pred = elem('predecessor', id='-1')
        l1_link.append(l1_pred)
    if successor is not None:
        l1_succ = elem('successor', id='-1')
        l1_link.append(l1_succ)
    if closed and predecessor is None and successor is None:
        l1_pred = elem('predecessor', id='-1')
        l1_succ = elem('successor', id='-1')
        l1_link.append(l1_pred); l1_link.append(l1_succ)
    l1.append(l1_link)
    
    l1.append(elem('roadMark', sOffset='0', type='broken', weight='standard', color='standard', width='0.13', laneChange='both'))
    l1.append(elem('width', sOffset='0', a=f'{lane_width:.3f}', b='0', c='0', d='0'))
    right = elem('right'); ls.append(right)
    r1 = elem('lane', id='-1', type='driving', level='false'); right.append(r1)
    
    # Add proper lane links for right lane
    r1_link = elem('link')
    if predecessor is not None:
        r1_pred = elem('predecessor', id='1')
        r1_link.append(r1_pred)
    if successor is not None:
        r1_succ = elem('successor', id='1')
        r1_link.append(r1_succ)
    if closed and predecessor is None and successor is None:
        r1_pred = elem('predecessor', id='1')
        r1_succ = elem('successor', id='1')
        r1_link.append(r1_pred); r1_link.append(r1_succ)
    r1.append(r1_link)
    
    r1.append(elem('roadMark', sOffset='0', type='broken', weight='standard', color='standard', width='0.13', laneChange='both'))
    r1.append(elem('width', sOffset='0', a=f'{lane_width:.3f}', b='0', c='0', d='0'))
    return road

def closest_points(A: np.ndarray, B: np.ndarray) -> tuple:
    # brute-force nearest pair (good enough for 2–5k points)
    i_min=j_min=0; dmin=1e18
    for i in range(len(A)):
        d = np.sum((B - A[i])**2, axis=1)
        j = int(np.argmin(d)); v = float(d[j])
        if v < dmin: dmin=v; i_min=i; j_min=j
    return i_min, j_min, math.sqrt(dmin)

def make_connector(a_xy: np.ndarray, b_xy: np.ndarray) -> np.ndarray:
    # simple 2-point straight connector (could densify if you like)
    return np.vstack([a_xy, b_xy])

def identify_track_sections(refs: list) -> list:
    """Identify and categorize track sections based on Laguna Seca characteristics."""
    track_sections = []
    lengths = [r[-1,0] for r in refs]
    
    for i, (ref, length) in enumerate(zip(refs, lengths)):
        # Analyze track characteristics
        x_coords = ref[:, 1]
        y_coords = ref[:, 2]
        x_range = np.max(x_coords) - np.min(x_coords)
        y_range = np.max(y_coords) - np.min(y_coords)
        
        # Determine section type based on geometry and length
        if length > 2000:  # Long main section
            section_name = "The Corkscrew"
            section_type = "main_track"
        elif i == 1:  # Track Section 2 is the pit lane based on the RD file structure
            section_name = "Pit Lane"
            section_type = "pit_lane"
        elif i == 2:  # Track Section 3 is the main track section
            section_name = "Track Section 3"
            section_type = "track_section"
        else:
            section_name = f"Track Section {i+1}"
            section_type = "track_section"
            
        track_sections.append({
            'id': i,
            'ref': ref,
            'length': length,
            'name': section_name,
            'type': section_type,
            'x_range': x_range,
            'y_range': y_range
        })
    
    return track_sections

def generate_comprehensive_mode(od, track_sections, args):
    """Generate comprehensive mode with all roads and junctions."""
    print("\n=== COMPREHENSIVE MODE ===")
    print("Generating full Laguna Seca circuit with all roads and junctions...")
    
    # Use the original complex logic for comprehensive mode
    def distance_2d(p1, p2):
        return np.sqrt((p1[0] - p2[0])**2 + (p1[1] - p2[1])**2)
    
    # Find optimal connection order
    corkscrew = next(s for s in track_sections if s['name'] == 'The Corkscrew')
    remaining = [s for s in track_sections if s['name'] != 'The Corkscrew']
    
    ordered_sections = [corkscrew]
    current_end = (corkscrew['ref'][-1, 1], corkscrew['ref'][-1, 2])
    
    while remaining:
        best_idx = 0
        best_dist = float('inf')
        
        for i, section in enumerate(remaining):
            start = (section['ref'][0, 1], section['ref'][0, 2])
            dist = distance_2d(current_end, start)
            if dist < best_dist:
                best_dist = dist
                best_idx = i
        
        next_section = remaining.pop(best_idx)
        ordered_sections.append(next_section)
        current_end = (next_section['ref'][-1, 1], next_section['ref'][-1, 2])
        print(f"  Connection: {ordered_sections[-2]['name']} → {next_section['name']} ({best_dist:.1f}m gap)")
    
    # Find junction clusters
    def find_junction_clusters(sections, threshold=100.0):
        endpoints = []
        for i, section in enumerate(sections):
            ref = section['ref']
            start = (ref[0, 1], ref[0, 2], i, 'start')
            end = (ref[-1, 1], ref[-1, 2], i, 'end')
            endpoints.append(start)
            endpoints.append(end)
        
        clusters = []
        used_points = set()
        
        for i, (x1, y1, road1, type1) in enumerate(endpoints):
            if i in used_points:
                continue
            
            cluster = [(x1, y1, road1, type1)]
            used_points.add(i)
            
            for j, (x2, y2, road2, type2) in enumerate(endpoints):
                if j in used_points or j == i:
                    continue
                
                dist = distance_2d((x1, y1), (x2, y2))
                if dist <= threshold:
                    cluster.append((x2, y2, road2, type2))
                    used_points.add(j)
            
            if len(cluster) > 1:
                clusters.append(cluster)
        
        return clusters
    
    junction_clusters = find_junction_clusters(ordered_sections, threshold=100.0)
    
    print(f"\nIdentified {len(junction_clusters)} junction points")
    for i, cluster in enumerate(junction_clusters):
        center_x = np.mean([x for x, y, road, ptype in cluster])
        center_y = np.mean([y for x, y, road, ptype in cluster])
        print(f"  Junction {i+1}: {len(cluster)} roads at ({center_x:.1f}, {center_y:.1f})")
    
    # Create roads with junction connections
    road_id = 0
    for i, section in enumerate(ordered_sections):
        XY = resample_xy(section['ref'], ds=args.step, closed=False)
        
        # Determine junction connections
        road_start = (section['ref'][0, 1], section['ref'][0, 2])
        road_end = (section['ref'][-1, 1], section['ref'][-1, 2])
        
        start_junction = None
        end_junction = None
        
        for j, cluster in enumerate(junction_clusters):
            for x, y, cluster_road_id, point_type in cluster:
                if cluster_road_id == i:
                    if point_type == 'start':
                        start_junction = j + 1
                    elif point_type == 'end':
                        end_junction = j + 1
        
        predecessor = {'elementType': 'junction', 'elementId': str(start_junction), 'contactPoint': 'end'} if start_junction else None
        successor = {'elementType': 'junction', 'elementId': str(end_junction), 'contactPoint': 'start'} if end_junction else None
        
        append_road_xml(od, road_id=road_id, XY=XY, name=section['name'],
                       closed=False, lane_width=args.lanewidth,
                       predecessor=predecessor, successor=successor)
        road_id += 1
    
    # Create junctions
    for junction_id, cluster in enumerate(junction_clusters, 1):
        center_x = np.mean([x for x, y, road, ptype in cluster])
        center_y = np.mean([y for x, y, road, ptype in cluster])
        
        junction_connections = []
        for x, y, cluster_road_id, point_type in cluster:
            if distance_2d((x, y), (center_x, center_y)) > 1.0:
                connector_xy = np.array([[x, y], [center_x, center_y]])
                connector_name = f"Junction_{junction_id}_Connector_{cluster_road_id}_{point_type}"
                
                append_road_xml(od, road_id=road_id, XY=connector_xy, name=connector_name,
                               closed=False, lane_width=args.lanewidth, junction_id=str(junction_id))
                
                contact_point = 'start' if point_type == 'end' else 'end'
                junction_connections.append((cluster_road_id, road_id, contact_point))
                road_id += 1
        
        if junction_connections:
            create_junction_xml(od, junction_id, junction_connections)
    
    total_length = sum(s['length'] for s in ordered_sections)
    print(f"\nGenerated comprehensive Laguna Seca circuit:")
    print(f"  - Main track roads: {len(ordered_sections)}")
    print(f"  - Junction connectors: {road_id - len(ordered_sections)}")
    print(f"  - Total junctions: {len(junction_clusters)}")
    print(f"  - Total main track length: {total_length:.1f}m")

def generate_simple_circuit_mode(od, track_sections, args, mode):
    """Generate outer loop or pit lane mode using shared logic."""
    mode_name = "outer loop" if mode == "outer_loop" else "pit lane"
    print(f"\n=== {mode.upper().replace('_', ' ')} MODE ===")
    print(f"Generating {mode_name}...")
    
    # Helper function to get road endpoints
    def get_road_endpoints(section):
        ref = section['ref']
        start_x, start_y = ref[0, 1], ref[0, 2]  # First point
        end_x, end_y = ref[-1, 1], ref[-1, 2]    # Last point
        return (start_x, start_y), (end_x, end_y)
    
    # Find roads based on mode
    corkscrew = next((s for s in track_sections if s['name'] == 'The Corkscrew'), None)
    
    if mode == "outer_loop":
        # Find main track roads (exclude pit lane)
        main_tracks = [s for s in track_sections if s['type'] != 'pit_lane']
        if len(main_tracks) < 2:
            raise ValueError("Need at least 2 main track sections for outer loop")
        second_road = next((s for s in main_tracks if s['name'] == 'Track Section 3'), None)
        if not second_road:
            raise ValueError("Could not find Track Section 3")
    else:  # pit_lane mode
        second_road = next((s for s in track_sections if s['type'] == 'pit_lane'), None)
        if not second_road:
            raise ValueError("Could not find Pit Lane section")
    
    if not corkscrew:
        raise ValueError("Could not find The Corkscrew")
    
    print(f"  Found: {corkscrew['name']} ({corkscrew['length']:.1f}m)")
    print(f"  Found: {second_road['name']} ({second_road['length']:.1f}m)")
    
    # Get endpoints
    corkscrew_start, corkscrew_end = get_road_endpoints(corkscrew)
    second_start, second_end = get_road_endpoints(second_road)
    
    print(f"  Corkscrew: start={corkscrew_start}, end={corkscrew_end}")
    print(f"  {second_road['name']}: start={second_start}, end={second_end}")
    
    # Determine connection points
    if mode == "outer_loop":
        # For outer loop: connect road endpoints directly
        entry_point = corkscrew_end
        exit_point = corkscrew_start
        entry_target = second_start
        exit_target = second_end
    else:  # pit_lane mode
        # For pit lane: find optimal entry point on corkscrew
        corkscrew_ref = corkscrew['ref']
        best_entry_idx = 0
        min_entry_dist = float('inf')
        for i in range(len(corkscrew_ref)):
            point = (corkscrew_ref[i, 1], corkscrew_ref[i, 2])
            dist = np.sqrt((point[0] - second_start[0])**2 + (point[1] - second_start[1])**2)
            if dist < min_entry_dist:
                min_entry_dist = dist
                best_entry_idx = i
        entry_point = (corkscrew_ref[best_entry_idx, 1], corkscrew_ref[best_entry_idx, 2])
        exit_point = corkscrew_start  # Use start for exit (closest to pit end)
        entry_target = second_start
        exit_target = second_end
        print(f"  Pit entry point on corkscrew: {entry_point} (index {best_entry_idx})")
        print(f"  Pit exit point on corkscrew: {exit_point}")
    
    # Calculate connector distances
    dist_entry = np.sqrt((entry_point[0] - entry_target[0])**2 + (entry_point[1] - entry_target[1])**2)
    dist_exit = np.sqrt((exit_target[0] - exit_point[0])**2 + (exit_target[1] - exit_point[1])**2)
    print(f"  Connector Entry distance: {dist_entry:.1f}m")
    print(f"  Connector Exit distance: {dist_exit:.1f}m")
    
    # Create roads with connections
    road_id = 0
    
    # Road 0: The Corkscrew
    XY0 = resample_xy(corkscrew['ref'], ds=args.step, closed=False)
    append_road_xml(od, road_id=0, XY=XY0, name="The Corkscrew", closed=False, lane_width=args.lanewidth,
                   predecessor={'elementType': 'road', 'elementId': '3', 'contactPoint': 'end'},
                   successor={'elementType': 'road', 'elementId': '2', 'contactPoint': 'start'})
    
    # Road 1: Second Road (Track Section 3 or Pit Lane)
    XY1 = resample_xy(second_road['ref'], ds=args.step, closed=False)
    append_road_xml(od, road_id=1, XY=XY1, name=second_road['name'], closed=False, lane_width=args.lanewidth,
                   predecessor={'elementType': 'road', 'elementId': '2', 'contactPoint': 'end'},
                   successor={'elementType': 'road', 'elementId': '3', 'contactPoint': 'start'})
    
    # Road 2: Entry Connector
    connector2_xy = np.array([entry_point, entry_target])
    entry_name = f"{mode.replace('_', '_').title()}_Entry_Connector" if mode == "pit_lane" else "Connector_0_to_1"
    append_road_xml(od, road_id=2, XY=connector2_xy, name=entry_name, closed=False, lane_width=args.lanewidth,
                   predecessor={'elementType': 'road', 'elementId': '0', 'contactPoint': 'end'},
                   successor={'elementType': 'road', 'elementId': '1', 'contactPoint': 'start'})
    
    # Road 3: Exit Connector
    connector3_xy = np.array([exit_target, exit_point])
    exit_name = f"{mode.replace('_', '_').title()}_Exit_Connector" if mode == "pit_lane" else "Connector_1_to_0"
    append_road_xml(od, road_id=3, XY=connector3_xy, name=exit_name, closed=False, lane_width=args.lanewidth,
                   predecessor={'elementType': 'road', 'elementId': '1', 'contactPoint': 'end'},
                   successor={'elementType': 'road', 'elementId': '0', 'contactPoint': 'start'})
    
    total_length = corkscrew['length'] + second_road['length']
    print(f"\nGenerated {mode_name}:")
    print(f"  - Road 0: The Corkscrew ({corkscrew['length']:.1f}m)")
    print(f"  - Road 1: {second_road['name']} ({second_road['length']:.1f}m)")
    print(f"  - Road 2: {entry_name} ({dist_entry:.1f}m)")
    print(f"  - Road 3: {exit_name} ({dist_exit:.1f}m)")
    print(f"  - Total track length: {total_length:.1f}m")
    print(f"  - Circuit: 0 → 2 → 1 → 3 → 0")



# ---------- CLI ----------

def main():
    ap = argparse.ArgumentParser(description="RD → Laguna Seca XODR Generator")
    ap.add_argument('--rd', required=True, help='Input RD file path')
    ap.add_argument('--xodr', required=True, help='Output XODR file path')
    ap.add_argument('--csv', default=None, help='Output CSV file prefix (optional)')
    ap.add_argument('--step', type=float, default=2.0, help='Resampling step size in meters')
    ap.add_argument('--lanewidth', type=float, default=3.7, help='Lane width in meters')
    
    # Mode selection - mutually exclusive
    mode_group = ap.add_mutually_exclusive_group(required=True)
    mode_group.add_argument('--comprehensive', action='store_true',
                           help='Generate full comprehensive map with all roads and junctions')
    mode_group.add_argument('--outer-loop', action='store_true',
                           help='Generate outer loop (main track only, no pit lane)')
    mode_group.add_argument('--pit-lane', action='store_true',
                           help='Generate corkscrew connected to pit lane mode')
    
    # Optional single-track support
    ap.add_argument('--single-track', action='store_true',
                    help='Generate only one track section')
    ap.add_argument('--main-road', type=int, default=None,
                    help='Index of road to use as main track (for single-track mode)')
    
    args = ap.parse_args()

    # 1) RD → multiple ref lines
    refs = rd_to_reflines(args.rd, step=args.step)
    print(f"Found {len(refs)} RD roads.")
    
    # Identify track sections based on Laguna Seca circuit characteristics
    track_sections = identify_track_sections(refs)
    
    # Print information about each road
    for section in track_sections:
        print(f"  Road {section['id']}: {section['name']} - {section['length']:.1f}m, range=({section['x_range']:.1f}, {section['y_range']:.1f})")

    # 2) Generate CSV files for all sections if requested
    if args.csv:
        base = args.csv.rsplit('.',1)[0]
        for section in track_sections:
            filename = f"{base}_{section['name'].replace(' ', '_')}.csv"
            write_refline_csv(section['ref'], filename)
            print(f"Wrote CSV: {filename}")

    # 3) Build XODR based on selected mode
    od = ET.Element('OpenDRIVE')
    header = ET.Element('header')
    
    # Set header name based on mode
    if args.comprehensive:
        header_name = 'Laguna_Seca_Comprehensive'
    elif args.outer_loop:
        header_name = 'Laguna_Seca_OuterLoop'
    elif args.pit_lane:
        header_name = 'Laguna_Seca_PitLane'
    else:
        header_name = 'Laguna_Seca_SingleTrack'
    
    for k,v in dict(revMajor='1', revMinor='6', name=header_name, version='1.6',
                    date='', north='0', south='0', east='0', west='0').items():
        header.set(k,v)
    od.append(header)

    # 4) Generate XODR based on selected mode
    if args.comprehensive:
        generate_comprehensive_mode(od, track_sections, args)
    elif args.outer_loop:
        generate_simple_circuit_mode(od, track_sections, args, "outer_loop")
    elif args.pit_lane:
        generate_simple_circuit_mode(od, track_sections, args, "pit_lane")
    elif args.single_track and args.main_road is not None:
        # Single-track mode
        main_idx = args.main_road
        if main_idx < 0 or main_idx >= len(refs):
            raise ValueError(f"--main-road {main_idx} out of range [0,{len(refs)-1}]")
        
        section = track_sections[main_idx]
        XY = resample_xy(section['ref'], ds=args.step, closed=True)
        append_road_xml(od, road_id=0, XY=XY, name=section['name'], 
                       closed=True, lane_width=args.lanewidth)
        
        print(f"Generated single track: {section['name']} ({section['length']:.1f}m)")
    else:
        raise ValueError("No mode specified. Use --comprehensive, --outer-loop, or --pit-lane")

    # 5) Write XODR
    tree = ET.ElementTree(od)
    ET.indent(tree, space="  ", level=0)
    tree.write(args.xodr, encoding='utf-8', xml_declaration=True)
    
    print(f"\n✅ Successfully wrote XODR file: {args.xodr}")


if __name__ == '__main__':
    try:
        main()
    except Exception as e:
        print(f"[ERROR] {e}", file=sys.stderr)
        sys.exit(1)
