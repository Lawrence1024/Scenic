#!/usr/bin/env python3
"""
Verify if a coordinate is actually on-road in Scenic's drivable region.
"""

import sys
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.domains.driving.roads import Network
from scenic.core.vectors import Vector

def verify_coordinate(x, y, z=0.0):
    """Verify if a coordinate is on-road."""
    scenic_root = Path(__file__).parent.parent
    xodr_path = scenic_root / "assets" / "maps" / "dSPACE" / "LagunaSeca.xodr"
    
    print(f"Verifying coordinate: ({x:.6f}, {y:.6f}, {z:.6f})")
    print("=" * 80)
    
    # Load network
    print("\n1. Loading XODR network...")
    network = Network.fromOpenDrive(str(xodr_path), ref_points=50)
    
    # Check drivable region
    print("\n2. Checking drivable region...")
    point = Vector(x, y, z)
    
    try:
        is_on_road = network.drivableRegion.containsPoint(point)
        print(f"   containsPoint({point}): {is_on_road}")
    except Exception as e:
        print(f"   ERROR checking containsPoint: {e}")
        is_on_road = False
    
    # Check distance to drivable region
    try:
        distance = network.drivableRegion.distanceTo(point)
        print(f"   distanceTo({point}): {distance:.6f}m")
    except Exception as e:
        print(f"   ERROR checking distanceTo: {e}")
        distance = None
    
    # Check if in road region
    try:
        is_in_road = network.roadRegion.containsPoint(point)
        print(f"   roadRegion.containsPoint: {is_in_road}")
    except Exception as e:
        print(f"   ERROR checking roadRegion: {e}")
        is_in_road = False
    
    # Check if in lane region
    try:
        is_in_lane = network.laneRegion.containsPoint(point)
        print(f"   laneRegion.containsPoint: {is_in_lane}")
    except Exception as e:
        print(f"   ERROR checking laneRegion: {e}")
        is_in_lane = False
    
    # Try with small buffer
    print("\n3. Checking with small buffer...")
    for buffer in [0.1, 0.5, 1.0, 2.0]:
        try:
            buffered_point = Vector(x, y, z)
            # Check if any nearby point is on-road
            test_x = x + buffer
            test_point = Vector(test_x, y, z)
            if network.drivableRegion.containsPoint(test_point):
                print(f"   Point ({test_x:.6f}, {y:.6f}) IS on-road (offset {buffer}m)")
                break
        except Exception as e:
            pass
    
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    print(f"Coordinate ({x:.6f}, {y:.6f}, {z:.6f}):")
    print(f"  On drivable region: {is_on_road}")
    if distance is not None:
        print(f"  Distance to drivable region: {distance:.6f}m")
    print(f"  On road region: {is_in_road}")
    print(f"  On lane region: {is_in_lane}")
    
    return is_on_road

if __name__ == "__main__":
    # Test the coordinate that was rejected
    x, y, z = 70.567889, 108.874718, 0.0
    verify_coordinate(x, y, z)

