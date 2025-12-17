#!/usr/bin/env python3
"""
Test consistency of "left of" positioning in Scenic coordinate space.

This test:
1. Generates multiple scenarios with "left of" positioning
2. Verifies that in Scenic's coordinate space, the "left of" vehicle is actually to the left
3. Checks consistency across multiple runs (should ALWAYS work)

The goal is to verify that Scenic's coordinate system is consistent - when Scenic says "left of",
it should actually place the vehicle to the left in Scenic's coordinate space.
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


def compute_left_vector(heading):
    """Compute left vector in Scenic's ENU coordinate system.
    
    In ENU: 
    - Heading 0° = North (+Y)
    - Left = 90° CCW from forward
    - Forward vector: (sin(heading), cos(heading))
    - Left vector: (-cos(heading), sin(heading))
    """
    forward_x = math.sin(heading)
    forward_y = math.cos(heading)
    left_x = -forward_y  # 90° CCW rotation
    left_y = forward_x
    return (left_x, left_y)


def test_left_of_consistency(num_tests=20):
    """Test consistency of 'left of' positioning in Scenic coordinate space."""
    print_section(f"Testing 'Left Of' Consistency ({num_tests} scenarios)")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Reference vehicle
fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1

# Vehicle placed left of fellow1
fellow2 = new RacingCar left of fellow1 by 3.0, with raceNumber 2
"""
        
        results = []
        success_count = 0
        failure_count = 0
        
        for test_num in range(1, num_tests + 1):
            try:
                # Generate scenario
                scenario = scenarioFromString(scenario_code)
                scene, iterations = scenario.generate(maxIterations=10)
                
                if len(scene.objects) < 2:
                    print(f"   Test {test_num}: [ERROR] Expected at least 2 objects")
                    failure_count += 1
                    continue
                
                fellow1 = scene.objects[0]
                fellow2 = scene.objects[1]
                
                # Get positions in Scenic coordinate space (XODR)
                # These are the same values that createObjectInSimulator accesses
                pos1 = (float(fellow1.position.x), float(fellow1.position.y))
                pos2 = (float(fellow2.position.x), float(fellow2.position.y))
                
                # Get orientations (same as createObjectInSimulator accesses)
                heading1 = float(fellow1.heading) if hasattr(fellow1, 'heading') else 0.0
                heading2 = float(fellow2.heading) if hasattr(fellow2, 'heading') else 0.0
                
                # Compute displacement vector
                dx = pos2[0] - pos1[0]
                dy = pos2[1] - pos1[1]
                distance = math.sqrt(dx*dx + dy*dy)
                
                # Compute left vector from fellow1's orientation
                # This is how "left" is defined in Scenic's ENU coordinate system
                left_x, left_y = compute_left_vector(heading1)
                
                # Project displacement onto left vector
                # Positive projection = left, negative = right
                left_component = dx * left_x + dy * left_y
                
                # Check if fellow2 is actually to the left
                is_left = left_component > 0
                
                result = {
                    'test_num': test_num,
                    'pos1': pos1,
                    'pos2': pos2,
                    'heading1': math.degrees(heading1),
                    'heading2': math.degrees(heading2),
                    'distance': distance,
                    'left_component': left_component,
                    'is_left': is_left,
                    'success': is_left
                }
                
                results.append(result)
                
                if is_left:
                    success_count += 1
                    status = "✅"
                else:
                    failure_count += 1
                    status = "❌"
                
                if test_num <= 5 or not is_left:  # Print first 5 and all failures
                    print(f"   Test {test_num}: {status} "
                          f"pos1=({pos1[0]:.2f}, {pos1[1]:.2f}) "
                          f"pos2=({pos2[0]:.2f}, {pos2[1]:.2f}) "
                          f"heading1={math.degrees(heading1):.1f}° "
                          f"left_component={left_component:.3f}m "
                          f"distance={distance:.2f}m")
                
            except Exception as e:
                print(f"   Test {test_num}: [ERROR] {e}")
                failure_count += 1
                import traceback
                traceback.print_exc()
        
        # Summary
        print_section("Summary")
        print(f"   Total tests: {num_tests}")
        print(f"   Success: {success_count} ({100*success_count/num_tests:.1f}%)")
        print(f"   Failure: {failure_count} ({100*failure_count/num_tests:.1f}%)")
        
        if failure_count > 0:
            print(f"\n   [WARNING] {failure_count} tests failed!")
            print(f"   'Left of' positioning is NOT consistent in Scenic coordinate space")
            
            # Analyze failures
            failures = [r for r in results if not r['success']]
            if failures:
                print(f"\n   Failure analysis:")
                for r in failures[:5]:  # Show first 5 failures
                    print(f"      Test {r['test_num']}: left_component={r['left_component']:.3f}m "
                          f"(expected > 0, got {r['left_component']:.3f})")
        else:
            print(f"\n   [OK] All tests passed!")
            print(f"   'Left of' positioning is consistent in Scenic coordinate space")
        
        # Statistics
        if results:
            left_components = [r['left_component'] for r in results]
            avg_left = sum(left_components) / len(left_components)
            min_left = min(left_components)
            max_left = max(left_components)
            
            print(f"\n   Statistics:")
            print(f"      Average left component: {avg_left:.3f}m")
            print(f"      Min left component: {min_left:.3f}m")
            print(f"      Max left component: {max_left:.3f}m")
        
        return failure_count == 0
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_left_of_consistency(num_tests=20)
    sys.exit(0 if success else 1)
