#!/usr/bin/env python3
"""
Analyze XODR file to check if road reference line (planView) matches lane centerline.

This script:
1. Extracts road reference line (planView geometry)
2. Extracts lane centerline (reference line + lane offset)
3. Compares them to see if there's an offset
"""

import sys
import math
import xml.etree.ElementTree as ET
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))


def parse_xodr_lanes(xodr_path):
    """Parse XODR file to extract lane information."""
    root = ET.parse(xodr_path).getroot()
    ns = '' if not root.tag.startswith('{') else root.tag.split('}')[0]+'}'
    
    roads_info = []
    
    for road in root.findall(f'{ns}road'):
        road_id = road.get('id')
        road_name = road.get('name', f'Road_{road_id}')
        
        # Get planView (reference line)
        plan_view = road.find(f'{ns}planView')
        ref_points = []
        if plan_view is not None:
            for geom in plan_view.findall(f'{ns}geometry'):
                s0 = float(geom.get('s', '0'))
                x0 = float(geom.get('x', '0'))
                y0 = float(geom.get('y', '0'))
                hdg = float(geom.get('hdg', '0'))
                length = float(geom.get('length', '0'))
                
                # Sample points along this geometry
                line = geom.find(f'{ns}line')
                arc = geom.find(f'{ns}arc')
                
                n_samples = max(2, int(math.ceil(length / 2.0)))
                for i in range(n_samples + 1):
                    u = i / n_samples
                    s_local = u * length
                    
                    if line is not None:
                        x = x0 + s_local * math.cos(hdg)
                        y = y0 + s_local * math.sin(hdg)
                    elif arc is not None:
                        kappa = float(arc.get('curvature', '0'))
                        if abs(kappa) < 1e-12:
                            x = x0 + s_local * math.cos(hdg)
                            y = y0 + s_local * math.sin(hdg)
                        else:
                            R = 1.0 / kappa
                            cx = x0 - R * math.sin(hdg)
                            cy = y0 + R * math.cos(hdg)
                            th = hdg + kappa * s_local
                            x = cx + R * math.sin(th)
                            y = cy + R * math.cos(th)
                    else:
                        x = x0 + s_local * math.cos(hdg)
                        y = y0 + s_local * math.sin(hdg)
                    
                    ref_points.append((s0 + s_local, x, y))
        
        # Get lane sections
        lanes_elem = road.find(f'{ns}lanes')
        lane_sections = []
        
        if lanes_elem is not None:
            # Get lane offset (if any)
            lane_offsets = []
            for offset_elem in lanes_elem.findall(f'{ns}laneOffset'):
                s = float(offset_elem.get('s', '0'))
                a = float(offset_elem.get('a', '0'))
                b = float(offset_elem.get('b', '0'))
                c = float(offset_elem.get('c', '0'))
                d = float(offset_elem.get('d', '0'))
                lane_offsets.append((s, a, b, c, d))
            
            for lane_section in lanes_elem.findall(f'{ns}laneSection'):
                s_section = float(lane_section.get('s', '0'))
                
                # Get center lane (id=0)
                center = lane_section.find(f'{ns}center')
                center_lane = None
                if center is not None:
                    center_lane = center.find(f'{ns}lane[@id="0"]')
                
                # Get left lanes (positive IDs)
                left_lanes = []
                left = lane_section.find(f'{ns}left')
                if left is not None:
                    for lane in left.findall(f'{ns}lane'):
                        lane_id = int(lane.get('id', '0'))
                        if lane_id > 0:
                            left_lanes.append(lane)
                
                # Get right lanes (negative IDs)
                right_lanes = []
                right = lane_section.find(f'{ns}right')
                if right is not None:
                    for lane in right.findall(f'{ns}lane'):
                        lane_id = int(lane.get('id', '0'))
                        if lane_id < 0:
                            right_lanes.append(lane)
                
                lane_sections.append({
                    's': s_section,
                    'center_lane': center_lane,
                    'left_lanes': left_lanes,
                    'right_lanes': right_lanes
                })
        
        roads_info.append({
            'id': road_id,
            'name': road_name,
            'ref_points': ref_points,
            'lane_offsets': lane_offsets,
            'lane_sections': lane_sections
        })
    
    return roads_info


def analyze_lane_centerline(road_info):
    """Analyze if reference line matches lane centerline."""
    print(f"\nRoad: {road_info['name']} (ID: {road_info['id']})")
    print("-" * 80)
    
    # Check lane offsets
    if road_info['lane_offsets']:
        print(f"Lane offsets found: {len(road_info['lane_offsets'])}")
        for s, a, b, c, d in road_info['lane_offsets']:
            print(f"  At s={s:.2f}m: offset = {a:.6f} + {b:.6f}*s + {c:.6f}*s² + {d:.6f}*s³")
            if abs(a) > 1e-6 or abs(b) > 1e-6 or abs(c) > 1e-6 or abs(d) > 1e-6:
                print(f"    [WARNING] NON-ZERO OFFSET: Reference line is NOT at lane centerline!")
    else:
        print("No lane offsets found (reference line should be at centerline)")
    
    # Check lane sections
    print(f"\nLane sections: {len(road_info['lane_sections'])}")
    for i, section in enumerate(road_info['lane_sections']):
        print(f"\n  Section {i+1} at s={section['s']:.2f}m:")
        
        # Check center lane
        if section['center_lane'] is not None:
            print(f"    Center lane (id=0): EXISTS")
            # Check if center lane has width (should be 0 or very small)
            width_elems = section['center_lane'].findall('.//{*}width')
            if width_elems:
                for w in width_elems:
                    a = float(w.get('a', '0'))
                    if abs(a) > 1e-6:
                        print(f"      ⚠️  Center lane has width {a:.6f}m - reference line may be offset!")
        else:
            print(f"    Center lane (id=0): MISSING")
        
        # Check left lanes
        print(f"    Left lanes: {len(section['left_lanes'])}")
        for lane in section['left_lanes']:
            lane_id = int(lane.get('id', '0'))
            width_elems = lane.findall('.//{*}width')
            widths = [float(w.get('a', '0')) for w in width_elems]
            if widths:
                print(f"      Lane {lane_id}: width={widths[0]:.3f}m")
        
        # Check right lanes
        print(f"    Right lanes: {len(section['right_lanes'])}")
        for lane in section['right_lanes']:
            lane_id = int(lane.get('id', '0'))
            width_elems = lane.findall('.//{*}width')
            widths = [float(w.get('a', '0')) for w in width_elems]
            if widths:
                print(f"      Lane {lane_id}: width={widths[0]:.3f}m")


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Analyze XODR file to check reference line vs lane centerline"
    )
    parser.add_argument(
        '--xodr-path',
        type=str,
        default='assets/maps/dSPACE/LagunaSeca.xodr',
        help='Path to XODR file'
    )
    
    args = parser.parse_args()
    
    xodr_path = Path(args.xodr_path)
    if not xodr_path.exists():
        xodr_path = Path(__file__).parent.parent / args.xodr_path
    
    if not xodr_path.exists():
        print(f"ERROR: XODR file not found: {xodr_path}")
        return 1
    
    print("=" * 80)
    print("XODR Lane Centerline Analysis")
    print("=" * 80)
    print(f"File: {xodr_path}")
    
    roads_info = parse_xodr_lanes(str(xodr_path))
    
    print(f"\nFound {len(roads_info)} roads")
    
    # Filter to main roads only
    from scenic.simulators.dspace.geometry.utils import MAIN_ROAD_NAMES
    main_roads = [r for r in roads_info if r['name'] in MAIN_ROAD_NAMES]
    
    print(f"Main roads: {len(main_roads)}")
    
    for road_info in main_roads:
        analyze_lane_centerline(road_info)
    
    print("\n" + "=" * 80)
    print("CONCLUSION")
    print("=" * 80)
    print("If lane offsets are non-zero or center lane has width, the reference line")
    print("(planView) is NOT the lane centerline. Waypoints extracted from planView")
    print("will be offset from the true lane centerline.")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

