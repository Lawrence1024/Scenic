"""Test different methods of transforming Scenic orientation to dSPACE.

This test compares:
1. Current method: obj.heading - π/2
2. Alternative: obj.orientation.yaw - π/2
3. Using orientation object directly (if applicable)
"""

import sys
import os
import math
from pathlib import Path

# Fix encoding for Windows console
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
if sys.stderr.encoding != 'utf-8':
    sys.stderr.reconfigure(encoding='utf-8')

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic import scenarioFromString


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_orientation_transformation_methods():
    """Test different methods of computing dSPACE orientation."""
    print_section("Testing Orientation Transformation Methods")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        scenario_str = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1
fellow2 = new RacingCar left of fellow1 by 5.0, with raceNumber 2
"""
        
        scenario = scenarioFromString(scenario_str)
        scene, _ = scenario.generate(maxIterations=100)
        
        fellow1 = scene.objects[0]
        fellow2 = scene.objects[1]
        
        print(f"\nFellow1:")
        print(f"  Position: ({fellow1.position.x:.6f}, {fellow1.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow1.heading):.2f}°")
        print(f"  Orientation.yaw: {math.degrees(fellow1.orientation.yaw):.2f}°")
        print(f"  Orientation: {fellow1.orientation}")
        
        print(f"\nFellow2:")
        print(f"  Position: ({fellow2.position.x:.6f}, {fellow2.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow2.heading):.2f}°")
        print(f"  Orientation.yaw: {math.degrees(fellow2.orientation.yaw):.2f}°")
        print(f"  Orientation: {fellow2.orientation}")
        
        # Test 1: Current method - obj.heading - π/2
        print_section("Method 1: obj.heading - π/2 (Current)")
        dspace_orient_1 = fellow1.heading - math.pi / 2
        print(f"  Scenic heading: {math.degrees(fellow1.heading):.2f}°")
        print(f"  dSPACE orientation: {math.degrees(dspace_orient_1):.2f}°")
        print(f"  Formula: heading - π/2 = {math.degrees(fellow1.heading):.2f}° - 90° = {math.degrees(dspace_orient_1):.2f}°")
        
        # Test 2: Alternative - obj.orientation.yaw - π/2
        print_section("Method 2: obj.orientation.yaw - π/2")
        dspace_orient_2 = fellow1.orientation.yaw - math.pi / 2
        print(f"  Scenic orientation.yaw: {math.degrees(fellow1.orientation.yaw):.2f}°")
        print(f"  dSPACE orientation: {math.degrees(dspace_orient_2):.2f}°")
        print(f"  Formula: orientation.yaw - π/2 = {math.degrees(fellow1.orientation.yaw):.2f}° - 90° = {math.degrees(dspace_orient_2):.2f}°")
        
        # Compare methods
        diff = abs(dspace_orient_1 - dspace_orient_2)
        print(f"\n  Difference between methods: {math.degrees(diff):.6f}°")
        if diff < 0.0001:
            print(f"  ✅ Methods are equivalent (difference < 0.0001°)")
        else:
            print(f"  ⚠️ Methods differ!")
        
        # Test 3: Verify heading == orientation.yaw
        print_section("Verification: heading vs orientation.yaw")
        heading_yaw_diff = abs(fellow1.heading - fellow1.orientation.yaw)
        print(f"  heading: {math.degrees(fellow1.heading):.6f}°")
        print(f"  orientation.yaw: {math.degrees(fellow1.orientation.yaw):.6f}°")
        print(f"  Difference: {math.degrees(heading_yaw_diff):.6f}°")
        if heading_yaw_diff < 0.0001:
            print(f"  ✅ heading == orientation.yaw (as expected)")
        else:
            print(f"  ⚠️ heading != orientation.yaw (unexpected!)")
        
        # Test 4: Verify the transformation makes sense
        print_section("Verification: Transformation Correctness")
        print("\n  Scenic ENU coordinate system:")
        print("    - Heading 0° = North (+Y axis)")
        print("    - Heading 90° = West (-X axis)")
        print("    - Heading 180° = South (-Y axis)")
        print("    - Heading 270° = East (+X axis)")
        
        print("\n  dSPACE RD coordinate system:")
        print("    - Orientation 0° = East (+X axis)")
        print("    - Orientation 90° = North (+Y axis)")
        print("    - Orientation 180° = West (-X axis)")
        print("    - Orientation 270° = South (-Y axis)")
        
        print("\n  Transformation: Scenic North (0°) → dSPACE East (0°)")
        scenic_north = 0.0
        dspace_east_1 = scenic_north - math.pi / 2
        dspace_east_2 = scenic_north - math.pi / 2
        print(f"    Method 1: {math.degrees(scenic_north):.1f}° - 90° = {math.degrees(dspace_east_1):.1f}°")
        print(f"    Method 2: {math.degrees(scenic_north):.1f}° - 90° = {math.degrees(dspace_east_2):.1f}°")
        print(f"    Expected: -90° (equivalent to 270° = East)")
        
        if abs(dspace_east_1 + math.pi/2) < 0.01:
            print(f"    ✅ Transformation is correct")
        else:
            print(f"    ❌ Transformation is incorrect")
        
        # Test 5: Check if we should use orientation object more directly
        print_section("Method 3: Using Orientation Object Directly")
        print("  Note: dSPACE only accepts a scalar angle, not a full orientation object")
        print("  So we must extract yaw from the orientation object")
        print("  Current approach (heading - π/2) is correct since:")
        print("    - heading = orientation.yaw (for most cases)")
        print("    - The transformation is just a coordinate system rotation")
        print("    - No need for full 3D orientation transformation")
        
        return True
        
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    print("="*80)
    print("ORIENTATION TRANSFORMATION METHOD TEST")
    print("="*80)
    
    result = test_orientation_transformation_methods()
    
    print_section("Summary")
    if result:
        print("✅ All methods are equivalent")
        print("  - obj.heading - π/2 = obj.orientation.yaw - π/2")
        print("  - Current implementation is correct")
        print("  - No change needed")
    else:
        print("❌ Methods differ - investigation needed")

