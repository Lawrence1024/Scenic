#!/usr/bin/env python3
"""
Test whether the coordinates in fellow_placing_fixed.scenic are actually on-road in Scenic.

This script will:
1. Load the Scenic scenario
2. Check if each coordinate is on the road network
3. Compare with the t-coordinates we're getting
"""

import sys
from pathlib import Path

# Add Scenic to path if needed
scenic_path = Path(__file__).parent.parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace import utils as dutils


def test_on_road_validation():
    """Test if coordinates are on-road in Scenic space."""
    
    # Read coordinates from the current fellow_placing_fixed.scenic file
    scenic_file = Path(__file__).parent / "fellow_placing_fixed.scenic"
    test_cases = []
    
    with open(scenic_file, 'r') as f:
        for line in f:
            line = line.strip()
            if line.startswith('fellow') and 'at (' in line:
                try:
                    # Extract coordinates from lines like:
                    # fellow1 = new Car at (-106.456824, -339.457701, 0.000000)
                    start = line.find('at (') + 4
                    end = line.find(')', start)
                    coord_str = line[start:end]
                    
                    # Parse x, y, z coordinates
                    parts = coord_str.split(',')
                    x = float(parts[0].strip())
                    y = float(parts[1].strip())
                    z = float(parts[2].strip())
                    
                    # Extract fellow name
                    fellow_name = line.split('=')[0].strip()
                    
                    test_cases.append((x, y, fellow_name))
                except (ValueError, IndexError) as e:
                    print(f"Warning: Could not parse line: {line}")
                    print(f"Error: {e}")
    
    # Load the OpenDRIVE file and build the road index
    xodr_file = Path(__file__).parent.parent.parent / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    road_index = dutils.build_xodr_sec_points(str(xodr_file))
    
    print("Testing if coordinates are on-road in Scenic space")
    print("=" * 80)
    
    for scenic_x, scenic_y, description in test_cases:
        # Get the projection result
        projected_s, projected_t = dutils.project_world_to_st(road_index, (scenic_x, scenic_y))
        
        # Calculate the actual distance to the centerline
        actual_distance = calculate_distance_to_centerline(road_index, scenic_x, scenic_y)
        
        print(f"\n{description}:")
        print(f"  Scenic coords: ({scenic_x:.3f}, {scenic_y:.3f})")
        print(f"  Projected: s={projected_s:.3f}, t={projected_t:.3f}")
        print(f"  Distance to centerline: {actual_distance:.3f}m")
        
        # Determine if this is on-road or off-road
        if actual_distance < 0.1:
            print(f"  ✅ ON ROAD (distance < 0.1m)")
        elif actual_distance < 1.0:
            print(f"  ⚠️  Very close to road (distance < 1.0m)")
        elif actual_distance < 5.0:
            print(f"  ⚠️  Close to road (distance < 5.0m)")
        else:
            print(f"  ❌ OFF ROAD (distance = {actual_distance:.3f}m)")
        
        # Check if the t-coordinate makes sense
        if abs(projected_t) > 10.0:
            print(f"  ⚠️  Large t-coordinate ({projected_t:.3f}) suggests off-road positioning")
        elif abs(projected_t) > 5.0:
            print(f"  ⚠️  Moderate t-coordinate ({projected_t:.3f}) suggests edge of road")
        else:
            print(f"  ✅ Reasonable t-coordinate ({projected_t:.3f})")


def calculate_distance_to_centerline(road_index, px, py):
    """Calculate the actual distance from a point to the centerline."""
    px, py = float(px), float(py)
    
    roads_obj = road_index['roads']
    min_distance = float('inf')
    
    for road in roads_obj.values():
        sec_list = road.get('sec_points', [])
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
                dist = (dx*dx + dy*dy)**0.5
                
                if dist < min_distance:
                    min_distance = dist
    
    return min_distance


def analyze_road_width():
    """Analyze the typical road width to understand reasonable t-coordinate ranges."""
    
    print("\n" + "=" * 80)
    print("Analyzing road width and reasonable t-coordinate ranges...")
    print("=" * 80)
    
    # Load the centerline
    xodr_file = Path(__file__).parent.parent.parent / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    refline, total_length = dutils.build_circuit_refline(str(xodr_file))
    
    print(f"Centerline has {len(refline)} points, total length: {total_length:.1f}m")
    
    # Estimate road width from the OpenDRIVE file
    # This is a rough estimate - we'd need to parse lane widths for accuracy
    estimated_road_width = 3.5  # Typical lane width
    estimated_total_width = 7.0  # Typical two-lane road
    
    print(f"\nEstimated road characteristics:")
    print(f"  Typical lane width: {estimated_road_width:.1f}m")
    print(f"  Typical total road width: {estimated_total_width:.1f}m")
    print(f"  Reasonable t-coordinate range: ±{estimated_total_width/2:.1f}m")
    
    print(f"\nT-coordinate interpretation:")
    print(f"  t = 0.0: On centerline")
    print(f"  t > 0: Right side of centerline")
    print(f"  t < 0: Left side of centerline")
    print(f"  |t| > {estimated_total_width/2:.1f}: Likely off-road")


if __name__ == "__main__":
    test_on_road_validation()
    analyze_road_width()
