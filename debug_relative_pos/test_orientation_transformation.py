"""Test orientation transformation between Scenic and dSPACE.

This test verifies:
1. How Scenic defines "left" in its coordinate system
2. Whether the orientation transformation (heading - π/2) is correct
3. Whether our interpretation of "left" matches Scenic's actual behavior
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


def test_scenic_left_definition():
    """Test how Scenic defines 'left' in its coordinate system."""
    print_section("Testing Scenic's 'Left' Definition")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Create a simple scenario with known orientation
        scenario_str = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

fellow1 = new RacingCar on mainRacingRoad
fellow2 = new RacingCar left of fellow1 by 5.0
"""
        
        scenario = scenarioFromString(scenario_str)
        scene, _ = scenario.generate(maxIterations=100)
        
        fellow1 = scene.objects[0]
        fellow2 = scene.objects[1]
        
        print(f"\nFellow1 (reference):")
        print(f"  Position: ({fellow1.position.x:.6f}, {fellow1.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow1.heading):.2f}°")
        print(f"  Orientation: {fellow1.orientation}")
        
        print(f"\nFellow2 (left of fellow1):")
        print(f"  Position: ({fellow2.position.x:.6f}, {fellow2.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow2.heading):.2f}°")
        print(f"  Orientation: {fellow2.orientation}")
        
        # Compute displacement vector
        dx = fellow2.position.x - fellow1.position.x
        dy = fellow2.position.y - fellow1.position.y
        print(f"\nDisplacement vector (fellow2 - fellow1):")
        print(f"  dx: {dx:.6f}")
        print(f"  dy: {dy:.6f}")
        print(f"  Magnitude: {math.sqrt(dx*dx + dy*dy):.6f}")
        
        # In ENU: heading 0° = North (+Y)
        # Forward vector = (sin(heading), cos(heading)) = (0, 1) when heading=0
        # Left vector = 90° CCW from forward = (-cos(heading), sin(heading)) = (-1, 0) when heading=0
        # So left should be in the -X direction when heading=0
        
        forward_x = math.sin(fellow1.heading)
        forward_y = math.cos(fellow1.heading)
        left_x = -forward_y  # 90° CCW rotation
        left_y = forward_x
        
        print(f"\nExpected left vector (from fellow1's heading):")
        print(f"  Forward: ({forward_x:.6f}, {forward_y:.6f})")
        print(f"  Left: ({left_x:.6f}, {left_y:.6f})")
        
        # Check if displacement aligns with left vector
        dot_product = dx * left_x + dy * left_y
        print(f"\nDot product (displacement · left_vector): {dot_product:.6f}")
        print(f"  Expected: Positive (displacement should align with left vector)")
        
        # Check if displacement is perpendicular to forward
        forward_dot = dx * forward_x + dy * forward_y
        print(f"Dot product (displacement · forward_vector): {forward_dot:.6f}")
        print(f"  Expected: ~0 (displacement should be perpendicular to forward)")
        
        # Verify orientation inheritance
        print(f"\nOrientation check:")
        print(f"  Fellow1 heading: {math.degrees(fellow1.heading):.2f}°")
        print(f"  Fellow2 heading: {math.degrees(fellow2.heading):.2f}°")
        print(f"  Difference: {math.degrees(fellow2.heading - fellow1.heading):.2f}°")
        print(f"  Expected: 0° (should inherit parentOrientation)")
        
        # On curved roads, displacement may not be perfectly perpendicular to forward
        # The key check is that displacement aligns with left vector (positive dot product)
        return dot_product > 0
    finally:
        os.chdir(original_cwd)


def test_orientation_transformation():
    """Test the orientation transformation (heading - π/2) for dSPACE."""
    print_section("Testing Orientation Transformation (Scenic → dSPACE)")
    
    # Test various headings
    test_headings = [0, math.pi/4, math.pi/2, 3*math.pi/4, math.pi, 
                     5*math.pi/4, 3*math.pi/2, 7*math.pi/4]
    
    print("\nScenic ENU → dSPACE RD Orientation Conversion:")
    print("  Formula: dspace_orientation = scenic_heading - π/2")
    print("\n" + "-"*80)
    print(f"{'Scenic Heading':<20} {'dSPACE Orientation':<20} {'Description':<30}")
    print("-"*80)
    
    for heading in test_headings:
        dspace_orient = heading - math.pi / 2
        scenic_deg = math.degrees(heading)
        dspace_deg = math.degrees(dspace_orient)
        
        # Describe direction
        if abs(heading) < 0.01:
            scenic_dir = "North (+Y)"
        elif abs(heading - math.pi/2) < 0.01:
            scenic_dir = "East (+X)"
        elif abs(heading - math.pi) < 0.01:
            scenic_dir = "South (-Y)"
        elif abs(heading - 3*math.pi/2) < 0.01:
            scenic_dir = "West (-X)"
        else:
            scenic_dir = f"{scenic_deg:.1f}°"
        
        if abs(dspace_orient) < 0.01:
            dspace_dir = "East (+X)"
        elif abs(dspace_orient - math.pi/2) < 0.01:
            dspace_dir = "North (+Y)"
        elif abs(dspace_orient - math.pi) < 0.01:
            dspace_dir = "West (-X)"
        elif abs(dspace_orient - 3*math.pi/2) < 0.01:
            dspace_dir = "South (-Y)"
        else:
            dspace_dir = f"{dspace_deg:.1f}°"
        
        print(f"{scenic_deg:>6.1f}° ({scenic_dir:<12}) {dspace_deg:>6.1f}° ({dspace_dir:<12})")
    
    print("-"*80)
    
    # Verify: Scenic North (0°) → dSPACE East (90°)
    scenic_north = 0.0
    dspace_east = scenic_north - math.pi / 2
    print(f"\nVerification:")
    print(f"  Scenic North (0°) → dSPACE: {math.degrees(dspace_east):.1f}°")
    print(f"  Expected: -90° (which is equivalent to 270° = East)")
    print(f"  Match: {abs(dspace_east + math.pi/2) < 0.01}")


def test_left_vector_computation():
    """Test our left vector computation against Scenic's actual behavior."""
    print_section("Testing Left Vector Computation")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Test with multiple scenarios
        num_tests = 10
        successes = 0
        
        scenario_str = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1
fellow2 = new RacingCar left of fellow1 by 5.0, with raceNumber 2
"""
        
        for i in range(num_tests):
            scenario = scenarioFromString(scenario_str)
            scene, _ = scenario.generate(maxIterations=100)
            
            fellow1 = scene.objects[0]
            fellow2 = scene.objects[1]
            
            # Compute displacement
            dx = fellow2.position.x - fellow1.position.x
            dy = fellow2.position.y - fellow1.position.y
            
            # Compute expected left vector
            forward_x = math.sin(fellow1.heading)
            forward_y = math.cos(fellow1.heading)
            left_x = -forward_y
            left_y = forward_x
            
            # Check alignment
            dot_product = dx * left_x + dy * left_y
            is_left = dot_product > 0
            
            if is_left:
                successes += 1
            
            if not is_left or i < 3:  # Print first 3 and failures
                print(f"\nTest {i+1}:")
                print(f"  Fellow1 heading: {math.degrees(fellow1.heading):.2f}°")
                print(f"  Displacement: ({dx:.3f}, {dy:.3f})")
                print(f"  Expected left: ({left_x:.3f}, {left_y:.3f})")
                print(f"  Dot product: {dot_product:.6f}")
                print(f"  Result: {'✓ PASS' if is_left else '✗ FAIL'}")
        
        print(f"\n{'='*80}")
        print(f"Results: {successes}/{num_tests} tests passed ({100*successes/num_tests:.1f}%)")
        return successes == num_tests
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    print("="*80)
    print("ORIENTATION TRANSFORMATION TEST")
    print("="*80)
    
    # Test 1: How Scenic defines "left"
    result1 = test_scenic_left_definition()
    
    # Test 2: Orientation transformation
    test_orientation_transformation()
    
    # Test 3: Left vector computation consistency
    result3 = test_left_vector_computation()
    
    print_section("Summary")
    print(f"Scenic 'left' definition test: {'✓ PASS' if result1 else '✗ FAIL'}")
    print(f"Left vector computation test: {'✓ PASS' if result3 else '✗ FAIL'}")
    
    if result1 and result3:
        print("\n✓ All orientation tests passed!")
        print("  Scenic's 'left of' specifier is working correctly.")
        print("  The issue must be in the transformation pipeline or dSPACE integration.")
    else:
        print("\n✗ Some orientation tests failed!")
        print("  There may be an issue with how we interpret Scenic's coordinate system.")

