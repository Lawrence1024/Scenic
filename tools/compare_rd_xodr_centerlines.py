#!/usr/bin/env python3
"""
Compare RD and XODR centerline waypoints to prove/disprove coordinate mapping.

This script:
1. Extracts centerline waypoints from RD file
2. Extracts centerline waypoints from XODR file
3. Compares them at matching locations
4. Reports differences and statistics
"""

import sys
import math
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.xodr_parser import build_xodr_sec_points
from scenic.simulators.dspace.geometry.route_projection import (
    ROUTE_ROAD_SEQUENCES,
    ROUTE_TRANSITION_POINTS
)


def extract_rd_waypoints(rd_path: str, route_name='R2', step=0.2):
    """Extract waypoints from RD centerline for a route."""
    rd_index = build_rd_road_index(rd_path, step=step)
    
    route_sequence = ROUTE_ROAD_SEQUENCES.get(route_name, [])
    transition_point = ROUTE_TRANSITION_POINTS.get(route_name, 0.0)
    
    if not route_sequence:
        print(f"ERROR: Route {route_name} not found")
        return []
    
    roads = rd_index.get('roads', {})
    waypoints = []
    
    for road_idx, road_name in enumerate(route_sequence):
        road_data = roads.get(road_name)
        if not road_data:
            continue
        
        sec_points = road_data.get('sec_points', [[]])
        if not sec_points or not sec_points[0]:
            continue
        
        points = sec_points[0]  # List of (x, y, s) tuples
        
        if road_idx == 0:
            for x, y, s in points:
                if s <= transition_point:
                    waypoints.append((x, y, s))
                else:
                    break
        else:
            start_idx = 0
            if waypoints:
                last_wp = waypoints[-1]
                first_pt = points[0]
                dist = math.sqrt((last_wp[0] - first_pt[0])**2 + (last_wp[1] - first_pt[1])**2)
                if dist < 0.1:
                    start_idx = 1
            
            for i in range(start_idx, len(points)):
                x, y, s = points[i]
                waypoints.append((x, y, s))
    
    return waypoints


def extract_xodr_waypoints(xodr_path: str, route_name='R2', step=0.2):
    """Extract waypoints from XODR centerline for a route."""
    xodr_index = build_xodr_sec_points(xodr_path, step=step)
    
    route_sequence = ROUTE_ROAD_SEQUENCES.get(route_name, [])
    transition_point = ROUTE_TRANSITION_POINTS.get(route_name, 0.0)
    
    if not route_sequence:
        print(f"ERROR: Route {route_name} not found")
        return []
    
    roads = xodr_index.get('roads', {})
    waypoints = []
    
    for road_idx, road_name in enumerate(route_sequence):
        road_data = roads.get(road_name)
        if not road_data:
            continue
        
        sec_points = road_data.get('sec_points', [[]])
        if not sec_points or not sec_points[0]:
            continue
        
        points = sec_points[0]  # List of (x, y, s) tuples
        
        if road_idx == 0:
            for x, y, s in points:
                if s <= transition_point:
                    waypoints.append((x, y, s))
                else:
                    break
        else:
            start_idx = 0
            if waypoints:
                last_wp = waypoints[-1]
                first_pt = points[0]
                dist = math.sqrt((last_wp[0] - first_pt[0])**2 + (last_wp[1] - first_pt[1])**2)
                if dist < 0.1:
                    start_idx = 1
            
            for i in range(start_idx, len(points)):
                x, y, s = points[i]
                waypoints.append((x, y, s))
    
    return waypoints


def find_closest_point(target_pt, point_list):
    """Find closest point in point_list to target_pt."""
    min_dist = float('inf')
    closest_idx = -1
    
    for i, pt in enumerate(point_list):
        dx = target_pt[0] - pt[0]
        dy = target_pt[1] - pt[1]
        dist = math.sqrt(dx*dx + dy*dy)
        if dist < min_dist:
            min_dist = dist
            closest_idx = i
    
    return closest_idx, min_dist


def compare_waypoints(rd_waypoints, xodr_waypoints):
    """Compare RD and XODR waypoints and report differences."""
    if not rd_waypoints or not xodr_waypoints:
        print("ERROR: Empty waypoint lists")
        return
    
    print("="*80)
    print("RD vs XODR Centerline Comparison")
    print("="*80)
    print(f"RD waypoints: {len(rd_waypoints)}")
    print(f"XODR waypoints: {len(xodr_waypoints)}")
    print()
    
    # Method 1: Compare by index (assuming same order) - only valid if same density
    print("Method 1: Direct Index Comparison")
    print("-"*80)
    if len(rd_waypoints) == len(xodr_waypoints):
        index_diffs = []
        for i in range(len(rd_waypoints)):
            rd_pt = rd_waypoints[i]
            xodr_pt = xodr_waypoints[i]
            dx = rd_pt[0] - xodr_pt[0]
            dy = rd_pt[1] - xodr_pt[1]
            dist = math.sqrt(dx*dx + dy*dy)
            index_diffs.append(dist)
        
        if index_diffs:
            print(f"  Mean distance: {sum(index_diffs)/len(index_diffs):.6f} m")
            print(f"  Max distance: {max(index_diffs):.6f} m")
            print(f"  Min distance: {min(index_diffs):.6f} m")
            print(f"  Std deviation: {math.sqrt(sum((d - sum(index_diffs)/len(index_diffs))**2 for d in index_diffs) / len(index_diffs)):.6f} m")
    else:
        print(f"  SKIPPED: Different waypoint counts (RD: {len(rd_waypoints)}, XODR: {len(xodr_waypoints)})")
        print(f"  Direct index comparison is invalid - using closest point matching instead")
        index_diffs = []
    
    # Method 2: Find closest point for each RD waypoint
    print()
    print("Method 2: Closest Point Matching (RD -> XODR)")
    print("-"*80)
    closest_diffs = []
    max_diff_idx = -1
    max_diff_dist = 0.0
    
    for i, rd_pt in enumerate(rd_waypoints):
        closest_idx, dist = find_closest_point(rd_pt, xodr_waypoints)
        closest_diffs.append(dist)
        if dist > max_diff_dist:
            max_diff_dist = dist
            max_diff_idx = i
    
    if closest_diffs:
        print(f"  Mean distance: {sum(closest_diffs)/len(closest_diffs):.6f} m")
        print(f"  Max distance: {max(closest_diffs):.6f} m (at RD index {max_diff_idx})")
        print(f"  Min distance: {min(closest_diffs):.6f} m")
        print(f"  Std deviation: {math.sqrt(sum((d - sum(closest_diffs)/len(closest_diffs))**2 for d in closest_diffs) / len(closest_diffs)):.6f} m")
    
    # Method 3: Find closest point for each XODR waypoint
    print()
    print("Method 3: Closest Point Matching (XODR -> RD)")
    print("-"*80)
    reverse_diffs = []
    max_reverse_idx = -1
    max_reverse_dist = 0.0
    
    for i, xodr_pt in enumerate(xodr_waypoints):
        closest_idx, dist = find_closest_point(xodr_pt, rd_waypoints)
        reverse_diffs.append(dist)
        if dist > max_reverse_dist:
            max_reverse_dist = dist
            max_reverse_idx = i
    
    if reverse_diffs:
        print(f"  Mean distance: {sum(reverse_diffs)/len(reverse_diffs):.6f} m")
        print(f"  Max distance: {max(reverse_diffs):.6f} m (at XODR index {max_reverse_idx})")
        print(f"  Min distance: {min(reverse_diffs):.6f} m")
        print(f"  Std deviation: {math.sqrt(sum((d - sum(reverse_diffs)/len(reverse_diffs))**2 for d in reverse_diffs) / len(reverse_diffs)):.6f} m")
    
    # Sample comparisons - find closest matches
    print()
    print("Sample Point Comparisons (RD -> closest XODR)")
    print("-"*80)
    # Sample some RD waypoints and find their closest XODR matches
    sample_indices = [0, len(rd_waypoints)//4, len(rd_waypoints)//2, 3*len(rd_waypoints)//4, len(rd_waypoints)-1]
    sample_indices = [i for i in sample_indices if i < len(rd_waypoints)]
    
    for idx in sample_indices:
        rd_pt = rd_waypoints[idx]
        closest_idx, dist = find_closest_point(rd_pt, xodr_waypoints)
        xodr_pt = xodr_waypoints[closest_idx]
        dx = rd_pt[0] - xodr_pt[0]
        dy = rd_pt[1] - xodr_pt[1]
        print(f"  RD index {idx} -> XODR index {closest_idx}:")
        print(f"    RD:   ({rd_pt[0]:12.6f}, {rd_pt[1]:12.6f})")
        print(f"    XODR: ({xodr_pt[0]:12.6f}, {xodr_pt[1]:12.6f})")
        print(f"    Diff: ({dx:12.6f}, {dy:12.6f}) = {dist:.6f} m")
        print()
    
    # Statistical analysis using closest point matching (more accurate)
    print()
    print("Statistical Analysis (using closest point matching)")
    print("-"*80)
    if closest_diffs:
        threshold = 0.01  # 1 cm
        exact_matches = sum(1 for d in closest_diffs if d < threshold)
        print(f"  RD points within {threshold*1000:.1f}mm of XODR: {exact_matches}/{len(closest_diffs)} ({100*exact_matches/len(closest_diffs):.1f}%)")
        
        threshold = 0.1  # 10 cm
        close_matches = sum(1 for d in closest_diffs if d < threshold)
        print(f"  RD points within {threshold*100:.0f}cm of XODR: {close_matches}/{len(closest_diffs)} ({100*close_matches/len(closest_diffs):.1f}%)")
        
        threshold = 1.0  # 1 m
        reasonable_matches = sum(1 for d in closest_diffs if d < threshold)
        print(f"  RD points within {threshold:.1f}m of XODR: {reasonable_matches}/{len(closest_diffs)} ({100*reasonable_matches/len(closest_diffs):.1f}%)")
        
        threshold = 5.0  # 5 m
        large_matches = sum(1 for d in closest_diffs if d < threshold)
        print(f"  RD points within {threshold:.1f}m of XODR: {large_matches}/{len(closest_diffs)} ({100*large_matches/len(closest_diffs):.1f}%)")
    
    # Conclusion - use closest point matching results (more accurate)
    print()
    print("="*80)
    print("CONCLUSION")
    print("="*80)
    if closest_diffs:
        mean_diff = sum(closest_diffs) / len(closest_diffs)
        max_diff = max(closest_diffs)
        
        if mean_diff < 0.01 and max_diff < 0.1:
            print("[PROVEN] RD and XODR centerlines map to the SAME coordinates")
            print(f"   Mean difference: {mean_diff*1000:.2f}mm (negligible)")
            print(f"   Max difference: {max_diff*1000:.2f}mm (within tolerance)")
        elif mean_diff < 0.1 and max_diff < 1.0:
            print("[PARTIAL MATCH] RD and XODR centerlines are CLOSE but not identical")
            print(f"   Mean difference: {mean_diff*100:.2f}cm")
            print(f"   Max difference: {max_diff*100:.2f}cm")
            print("   Possible causes:")
            print("     - Different sampling densities")
            print("     - Spline interpolation vs direct geometry")
            print("     - Numerical precision differences")
        else:
            print("[DISPROVEN] RD and XODR centerlines do NOT map to the same coordinates")
            print(f"   Mean difference: {mean_diff:.3f}m")
            print(f"   Max difference: {max_diff:.3f}m")
            print("   Possible causes:")
            print("     - Different coordinate systems (despite identity transform)")
            print("     - Different centerline definitions")
            print("     - Spline approximation errors in RD")
            print("     - Geometry calculation differences")


def main():
    """Main function."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Compare RD and XODR centerline waypoints"
    )
    parser.add_argument(
        '--rd-path',
        type=str,
        default='assets/maps/dSPACE/Laguna_Seca.rd',
        help='Path to RD file'
    )
    parser.add_argument(
        '--xodr-path',
        type=str,
        default='assets/maps/dSPACE/LagunaSeca.xodr',
        help='Path to XODR file'
    )
    parser.add_argument(
        '--route',
        type=str,
        default='R2',
        help='Route name (R2 for lap, R1 for pit)'
    )
    parser.add_argument(
        '--step',
        type=float,
        default=0.2,
        help='Waypoint spacing in meters'
    )
    
    args = parser.parse_args()
    
    # Convert to Path objects
    rd_path = Path(args.rd_path)
    xodr_path = Path(args.xodr_path)
    
    # Try relative to script location
    if not rd_path.exists():
        rd_path = Path(__file__).parent.parent / args.rd_path
    if not xodr_path.exists():
        xodr_path = Path(__file__).parent.parent / args.xodr_path
    
    if not rd_path.exists():
        print(f"ERROR: RD file not found: {rd_path}")
        return 1
    
    if not xodr_path.exists():
        print(f"ERROR: XODR file not found: {xodr_path}")
        return 1
    
    print("Loading RD waypoints...")
    rd_waypoints = extract_rd_waypoints(str(rd_path), route_name=args.route, step=args.step)
    print(f"  Loaded {len(rd_waypoints)} waypoints")
    print()
    
    print("Loading XODR waypoints...")
    xodr_waypoints = extract_xodr_waypoints(str(xodr_path), route_name=args.route, step=args.step)
    print(f"  Loaded {len(xodr_waypoints)} waypoints")
    print()
    
    if not rd_waypoints or not xodr_waypoints:
        print("ERROR: Failed to extract waypoints")
        return 1
    
    compare_waypoints(rd_waypoints, xodr_waypoints)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

