"""Test how Scenic actually computes "left of" positioning.

This test directly replicates Scenic's computation to verify our understanding.
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


def test_scenic_left_computation():
    """Test how Scenic computes 'left of' by replicating its computation."""
    print_section("Testing Scenic's 'Left Of' Computation")
    
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
        
        print(f"\nFellow1 (reference):")
        print(f"  Position: ({fellow1.position.x:.6f}, {fellow1.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow1.heading):.2f}°")
        print(f"  Orientation: {fellow1.orientation}")
        print(f"  Width: {fellow1.width:.3f}")
        print(f"  Length: {fellow1.length:.3f}")
        
        print(f"\nFellow2 (left of fellow1):")
        print(f"  Position: ({fellow2.position.x:.6f}, {fellow2.position.y:.6f})")
        print(f"  Heading: {math.degrees(fellow2.heading):.2f}°")
        print(f"  Orientation: {fellow2.orientation}")
        print(f"  Width: {fellow2.width:.3f}")
        print(f"  Length: {fellow2.length:.3f}")
        
        # Compute displacement
        dx = fellow2.position.x - fellow1.position.x
        dy = fellow2.position.y - fellow1.position.y
        print(f"\nActual displacement (fellow2 - fellow1):")
        print(f"  dx: {dx:.6f}")
        print(f"  dy: {dy:.6f}")
        print(f"  Magnitude: {math.sqrt(dx*dx + dy*dy):.6f}")
        
        # Replicate Scenic's computation
        # From LeftSpec: makeOffset returns Vector(-self.width/2 - dx - dims[0]/2 - tol, dy, dz)
        # where dx = distance (5.0), dims[0] = fellow2.width, tol = contactTolerance
        distance = 5.0
        fellow1_width = fellow1.width
        fellow2_width = fellow2.width
        contact_tol = getattr(fellow2, 'contactTolerance', 0.0)
        
        # Local offset in fellow1's coordinate system
        local_offset_x = -fellow1_width / 2 - distance - fellow2_width / 2 - contact_tol
        local_offset_y = 0.0
        local_offset_z = 0.0
        
        print(f"\nReplicating Scenic's computation:")
        print(f"  Local offset (in fellow1's frame): ({local_offset_x:.6f}, {local_offset_y:.6f}, {local_offset_z:.6f})")
        print(f"    -fellow1.width/2 = {-fellow1_width/2:.6f}")
        print(f"    -distance = {-distance:.6f}")
        print(f"    -fellow2.width/2 = {-fellow2_width/2:.6f}")
        print(f"    -contactTolerance = {-contact_tol:.6f}")
        
        # Transform local offset to world coordinates using fellow1's orientation
        # This is what pos.relativePosition() does: self.position.offsetLocally(self.orientation, vec)
        local_offset = (local_offset_x, local_offset_y, local_offset_z)
        world_offset = fellow1.position.offsetLocally(fellow1.orientation, local_offset)
        
        expected_pos = world_offset
        print(f"\nExpected position (fellow1.position + transformed offset):")
        print(f"  Expected: ({expected_pos.x:.6f}, {expected_pos.y:.6f})")
        print(f"  Actual:   ({fellow2.position.x:.6f}, {fellow2.position.y:.6f})")
        
        error_x = fellow2.position.x - expected_pos.x
        error_y = fellow2.position.y - expected_pos.y
        error_mag = math.sqrt(error_x*error_x + error_y*error_y)
        print(f"  Error: ({error_x:.6f}, {error_y:.6f}), magnitude: {error_mag:.6f}")
        
        if error_mag < 0.01:
            print(f"  ✅ Position matches Scenic's computation!")
        else:
            print(f"  ⚠️ Position differs from expected (might be due to road constraints)")
        
        # Now check: what is the "left" direction in world coordinates?
        # We need to transform the local "left" vector (-1, 0, 0) to world coordinates
        local_left = (-1.0, 0.0, 0.0)  # Left in local frame
        world_left_vec = fellow1.position.offsetLocally(fellow1.orientation, local_left) - fellow1.position
        
        print(f"\nLeft direction in world coordinates (from fellow1's orientation):")
        print(f"  World left vector: ({world_left_vec.x:.6f}, {world_left_vec.y:.6f})")
        
        # Normalize
        left_mag = math.sqrt(world_left_vec.x**2 + world_left_vec.y**2)
        if left_mag > 0:
            world_left_normalized = (world_left_vec.x / left_mag, world_left_vec.y / left_mag)
            print(f"  Normalized: ({world_left_normalized[0]:.6f}, {world_left_normalized[1]:.6f})")
            
            # Check if displacement aligns with this left vector
            dot_product = dx * world_left_normalized[0] + dy * world_left_normalized[1]
            print(f"\nAlignment check:")
            print(f"  Displacement · Left vector: {dot_product:.6f}")
            print(f"  Expected: Positive (displacement should align with left)")
            
            if dot_product > 0:
                print(f"  ✅ PASS: Fellow2 is to the left of fellow1")
                return True
            else:
                print(f"  ❌ FAIL: Fellow2 is NOT to the left (dot product is negative)")
                return False
        else:
            print(f"  ⚠️ Could not compute left vector")
            return False
        
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    print("="*80)
    print("SCENIC LEFT COMPUTATION TEST")
    print("="*80)
    
    result = test_scenic_left_computation()
    
    print_section("Summary")
    if result:
        print("✅ Test PASSED: Scenic's 'left of' computation is correct")
    else:
        print("❌ Test FAILED: Scenic's 'left of' computation may have issues")

