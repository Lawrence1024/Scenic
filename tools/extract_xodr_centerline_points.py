#!/usr/bin/env python3
"""
Extract centerline points from OpenDRIVE XODR file (what Scenic sees).

This script extracts 10 evenly-spaced centerline points from the main road
in the XODR file, showing the (x, y, s) coordinates in XODR coordinate system.
"""

import sys
import os
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

def extract_centerline_points(xodr_path: str, num_points: int = 10):
    """Extract N evenly-spaced centerline points from XODR file.
    
    Args:
        xodr_path: Path to XODR file
        num_points: Number of points to extract
        
    Returns:
        List of (x, y, s) tuples in XODR coordinate system
    """
    from scenic.simulators.dspace.geometry.xodr_parser import build_xodr_sec_points
    
    print(f"Loading XODR file: {xodr_path}")
    road_index = build_xodr_sec_points(xodr_path, step=2.0)
    
    if not road_index or 'roads' not in road_index:
        print("ERROR: Failed to parse XODR file")
        return []
    
    roads = road_index['roads']
    if not roads:
        print("ERROR: No roads found in XODR file")
        return []
    
    # Get the longest road (main track)
    main_road = max(roads.values(), key=lambda r: r.get('length', 0))
    road_name = main_road.get('name', 'Unknown')
    road_length = main_road.get('length', 0)
    
    print(f"\nMain road: {road_name}")
    print(f"Road length: {road_length:.2f} m")
    
    # Get all centerline points
    sec_points = main_road.get('sec_points', [])
    if not sec_points or not sec_points[0]:
        print("ERROR: No centerline points found")
        return []
    
    all_points = sec_points[0]  # List of (x, y, s) tuples
    print(f"Total centerline points: {len(all_points)}")
    
    # Sample N evenly-spaced points
    if len(all_points) < num_points:
        print(f"WARNING: Only {len(all_points)} points available, returning all")
        return all_points
    
    # Calculate step size to get evenly-spaced points
    step = len(all_points) / (num_points - 1)
    sampled_points = []
    
    for i in range(num_points):
        idx = int(i * step)
        if idx >= len(all_points):
            idx = len(all_points) - 1
        x, y, s = all_points[idx]
        sampled_points.append((x, y, s))
    
    return sampled_points


def main():
    """Main entry point."""
    import argparse
    
    parser = argparse.ArgumentParser(
        description="Extract centerline points from OpenDRIVE XODR file"
    )
    parser.add_argument(
        'xodr_path',
        type=str,
        help='Path to XODR file'
    )
    parser.add_argument(
        '-n', '--num-points',
        type=int,
        default=10,
        help='Number of points to extract (default: 10)'
    )
    
    args = parser.parse_args()
    
    if not os.path.exists(args.xodr_path):
        print(f"ERROR: File not found: {args.xodr_path}")
        return 1
    
    points = extract_centerline_points(args.xodr_path, args.num_points)
    
    if not points:
        print("ERROR: Failed to extract points")
        return 1
    
    print(f"\n{'='*80}")
    print(f"Extracted {len(points)} centerline points (XODR coordinates):")
    print(f"{'='*80}")
    print(f"{'Index':<8} {'X (m)':<15} {'Y (m)':<15} {'s (m)':<15}")
    print(f"{'-'*80}")
    
    for i, (x, y, s) in enumerate(points):
        print(f"{i+1:<8} {x:<15.6f} {y:<15.6f} {s:<15.6f}")
    
    print(f"\n{'='*80}")
    print("Python list format:")
    print(f"{'='*80}")
    print("points = [")
    for i, (x, y, s) in enumerate(points):
        comma = "," if i < len(points) - 1 else ""
        print(f"    ({x:.6f}, {y:.6f}, {s:.6f}){comma}")
    print("]")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

