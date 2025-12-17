"""Full pipeline test using actual simulator to trace "left of" positioning.

This test creates a simulator instance and traces through:
1. Scenic's "left of" computation (XODR)
2. XODR → RD transformation
3. Route detection and assignment
4. RD → (s,t) projection
5. T-coordinate sign and magnitude
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


def test_full_pipeline(num_tests=10):
    """Test "left of" positioning through the full simulator pipeline."""
    print_section(f"Full Pipeline Test ({num_tests} scenarios)")
    
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

fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1
fellow2 = new RacingCar left of fellow1 by 5.0, with raceNumber 2
"""
        
        results = []
        step1_failures = 0  # Scenic XODR - left relationship
        step2_failures = 0  # XODR → RD - relationship preserved
        step3_failures = 0  # Route assignment - same route
        step4_failures = 0  # T-coordinate - correct sign
        
        for test_num in range(1, num_tests + 1):
            try:
                scenario = scenarioFromString(scenario_code)
                scene, _ = scenario.generate(maxIterations=10)
                
                if len(scene.objects) < 2:
                    continue
                
                fellow1 = scene.objects[0]
                fellow2 = scene.objects[1]
                
                # Step 1: Verify Scenic's "left of" in XODR coordinates
                pos1_xodr = (float(fellow1.position.x), float(fellow1.position.y))
                pos2_xodr = (float(fellow2.position.x), float(fellow2.position.y))
                
                dx_xodr = pos2_xodr[0] - pos1_xodr[0]
                dy_xodr = pos2_xodr[1] - pos1_xodr[1]
                
                # Compute left vector using Scenic's actual method
                local_left = (-1.0, 0.0, 0.0)
                world_left_vec = fellow1.position.offsetLocally(fellow1.orientation, local_left) - fellow1.position
                left_mag = math.sqrt(world_left_vec.x**2 + world_left_vec.y**2)
                world_left_normalized = None
                left_component_xodr = 0
                if left_mag > 0:
                    world_left_normalized = (world_left_vec.x / left_mag, world_left_vec.y / left_mag)
                    left_component_xodr = dx_xodr * world_left_normalized[0] + dy_xodr * world_left_normalized[1]
                    step1_ok = left_component_xodr > 0
                else:
                    step1_ok = False
                
                if not step1_ok:
                    step1_failures += 1
                
                # Step 2: Transform XODR → RD and verify relationship preserved
                step2_ok = None
                left_component_rd = None
                pos1_rd = None
                pos2_rd = None
                
                # Skip Step 2 if Step 1 failed (no valid left vector)
                if world_left_normalized is not None:
                    try:
                        from scenic.simulators.dspace.geometry.coordinate_transform import (
                            load_transform, apply_coordinate_transform
                        )
                        import numpy as np
                        
                        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
                        if transform_path.exists():
                            coordinate_transform = load_transform(str(transform_path))
                            
                            pos1_rd = apply_coordinate_transform(coordinate_transform, pos1_xodr)
                            pos2_rd = apply_coordinate_transform(coordinate_transform, pos2_xodr)
                            
                            # Convert to Python floats in case they're numpy scalars
                            pos1_rd = (float(pos1_rd[0]), float(pos1_rd[1]))
                            pos2_rd = (float(pos2_rd[0]), float(pos2_rd[1]))
                            
                            dx_rd = pos2_rd[0] - pos1_rd[0]
                            dy_rd = pos2_rd[1] - pos1_rd[1]
                            
                            # Transform the left vector to RD coordinates
                            # If affine transformation, we need to apply the rotation part
                            if coordinate_transform['type'] == 'affine':
                                # Transform the left vector using the affine matrix
                                A = np.array(coordinate_transform['matrix'])
                                left_vec_xodr = np.array([world_left_normalized[0], world_left_normalized[1]])
                                left_vec_rd = A @ left_vec_xodr
                                left_vec_rd_mag = np.linalg.norm(left_vec_rd)
                                if left_vec_rd_mag > 0:
                                    left_vec_rd_normalized = left_vec_rd / left_vec_rd_mag
                                    # Convert to Python tuple
                                    left_vec_rd_normalized = (float(left_vec_rd_normalized[0]), float(left_vec_rd_normalized[1]))
                                else:
                                    left_vec_rd_normalized = (float(left_vec_rd[0]), float(left_vec_rd[1]))
                            else:
                                # Translation only - left vector doesn't change
                                left_vec_rd_normalized = world_left_normalized
                            
                            # Check if left relationship is preserved in RD
                            left_component_rd = dx_rd * left_vec_rd_normalized[0] + dy_rd * left_vec_rd_normalized[1]
                            step2_ok = left_component_rd > 0
                            
                            if not step2_ok:
                                step2_failures += 1
                    except Exception as e:
                        step2_ok = None
                        pos1_rd = None
                        pos2_rd = None
                        left_component_rd = None
                
                # Step 3 & 4: Create simulator and check route + t-coordinate
                step3_ok = None
                step4_ok = None
                route1 = None
                route2 = None
                t1 = None
                t2 = None
                
                try:
                    from scenic.simulators.dspace.simulator import DSpaceSimulator
                    
                    # Create simulator instance (without actually connecting to ModelDesk)
                    sim = DSpaceSimulator()
                    sim._initialize_from_scene(scene)
                    
                    # Get route and t-coordinate for fellow1
                    if hasattr(fellow1, 'position') and pos1_rd:
                        try:
                            position_xy = pos1_rd
                            track_segment1 = sim.detectTrackSegment(position_xy)
                            if track_segment1:
                                route1 = sim.assignRoute(fellow1, track_segment1)
                            
                            # Project to (s,t)
                            if sim._road_index and route1:
                                from scenic.simulators.dspace.geometry.route_projection import (
                                    project_world_to_st_route_specific
                                )
                                s1, t1 = project_world_to_st_route_specific(
                                    sim._road_index,
                                    pos1_rd,
                                    route_preference=route1
                                )
                        except Exception as e:
                            pass
                    
                    # Get route and t-coordinate for fellow2
                    if hasattr(fellow2, 'position') and pos2_rd:
                        try:
                            position_xy = pos2_rd
                            track_segment2 = sim.detectTrackSegment(position_xy)
                            if track_segment2:
                                route2 = sim.assignRoute(fellow2, track_segment2)
                            
                            # Project to (s,t)
                            if sim._road_index and route2:
                                from scenic.simulators.dspace.geometry.route_projection import (
                                    project_world_to_st_route_specific
                                )
                                s2, t2 = project_world_to_st_route_specific(
                                    sim._road_index,
                                    pos2_rd,
                                    route_preference=route2
                                )
                        except Exception as e:
                            pass
                    
                    # Check if same route
                    if route1 and route2:
                        step3_ok = (route1 == route2)
                        if not step3_ok:
                            step3_failures += 1
                        
                        # Check t-coordinate if same route
                        if step3_ok and t1 is not None and t2 is not None:
                            # Fellow2 (left of fellow1) should have more positive t
                            step4_ok = (t2 > t1)
                            if not step4_ok:
                                step4_failures += 1
                
                except Exception as e:
                    # Simulator setup failed - skip steps 3 and 4
                    pass
                
                result = {
                    'test_num': test_num,
                    'step1_ok': step1_ok,
                    'step2_ok': step2_ok,
                    'step3_ok': step3_ok,
                    'step4_ok': step4_ok,
                    'left_component_xodr': left_component_xodr,
                    'left_component_rd': left_component_rd,
                    'route1': route1,
                    'route2': route2,
                    't1': t1,
                    't2': t2,
                    'pos1_xodr': pos1_xodr,
                    'pos2_xodr': pos2_xodr,
                }
                results.append(result)
                
            except Exception as e:
                print(f"   Test {test_num}: [ERROR] {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # Print summary
        print_section("Pipeline Test Results")
        print(f"Total tests: {num_tests}")
        
        print(f"\nStep 1 (Scenic XODR - 'left of' correct):")
        passed = num_tests - step1_failures
        print(f"  Passed: {passed}/{num_tests} ({100*passed/num_tests:.1f}%)")
        print(f"  Failed: {step1_failures}/{num_tests}")
        
        step2_count = sum(1 for r in results if r['step2_ok'] is not None)
        if step2_count > 0:
            step2_passed = sum(1 for r in results if r['step2_ok'] is True)
            print(f"\nStep 2 (XODR → RD transformation):")
            print(f"  Passed: {step2_passed}/{step2_count} ({100*step2_passed/step2_count:.1f}%)")
            print(f"  Failed: {step2_count - step2_passed}/{step2_count}")
        
        step3_count = sum(1 for r in results if r['step3_ok'] is not None)
        if step3_count > 0:
            step3_passed = sum(1 for r in results if r['step3_ok'] is True)
            print(f"\nStep 3 (Route assignment - same route):")
            print(f"  Passed: {step3_passed}/{step3_count} ({100*step3_passed/step3_count:.1f}%)")
            print(f"  Failed: {step3_count - step3_passed}/{step3_count}")
        
        step4_count = sum(1 for r in results if r['step4_ok'] is not None)
        if step4_count > 0:
            step4_passed = sum(1 for r in results if r['step4_ok'] is True)
            print(f"\nStep 4 (T-coordinate - t2 > t1):")
            print(f"  Passed: {step4_passed}/{step4_count} ({100*step4_passed/step4_count:.1f}%)")
            print(f"  Failed: {step4_count - step4_passed}/{step4_count}")
        
        # Print detailed results for failures
        failures = [r for r in results if not r['step1_ok'] or 
                   (r['step2_ok'] is not None and not r['step2_ok']) or
                   (r['step3_ok'] is not None and not r['step3_ok']) or
                   (r['step4_ok'] is not None and not r['step4_ok'])]
        
        if failures:
            print_section("Detailed Failure Analysis")
            for result in failures[:5]:  # Show first 5 failures
                print(f"\nTest {result['test_num']}:")
                print(f"  Step 1 (XODR): {'✅' if result['step1_ok'] else '❌'} (left_component: {result['left_component_xodr']:.6f})")
                if result['step2_ok'] is not None:
                    left_comp_rd_str = f"{result['left_component_rd']:.6f}" if result['left_component_rd'] is not None else "N/A"
                    print(f"  Step 2 (RD): {'✅' if result['step2_ok'] else '❌'} (left_component: {left_comp_rd_str})")
                if result['step3_ok'] is not None:
                    print(f"  Step 3 (Route): {'✅' if result['step3_ok'] else '❌'} (Route1: {result['route1']}, Route2: {result['route2']})")
                if result['step4_ok'] is not None:
                    print(f"  Step 4 (T-coord): {'✅' if result['step4_ok'] else '❌'} (t1: {result['t1']:.6f}, t2: {result['t2']:.6f})")
                print(f"  Position 1 (XODR): {result['pos1_xodr']}")
                print(f"  Position 2 (XODR): {result['pos2_xodr']}")
        
        return {
            'step1_failures': step1_failures,
            'step2_failures': step2_failures,
            'step3_failures': step3_failures,
            'step4_failures': step4_failures,
            'results': results,
        }
        
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    print("="*80)
    print("FULL PIPELINE TEST - LEFT OF POSITIONING")
    print("="*80)
    
    test_results = test_full_pipeline(10)
    results = test_results['results']
    
    print_section("Final Summary")
    
    # Calculate from results list for consistency
    step1_total = len(results)
    step1_passed = sum(1 for r in results if r['step1_ok'])
    if step1_passed == step1_total:
        print("✅ Step 1 (Scenic XODR): All tests passed")
    else:
        print(f"❌ Step 1 (Scenic XODR): {step1_total - step1_passed}/{step1_total} failures")
        print("   Root cause: Scenic's 'left of' specifier issue")
    
    step2_total = sum(1 for r in results if r['step2_ok'] is not None)
    if step2_total > 0:
        step2_passed = sum(1 for r in results if r['step2_ok'] is True)
        if step2_passed == step2_total:
            print("✅ Step 2 (XODR → RD): All tests passed")
        else:
            print(f"❌ Step 2 (XODR → RD): {step2_total - step2_passed}/{step2_total} failures")
            print("   Root cause: Coordinate transformation issue")
    else:
        print("⚠️ Step 2 (XODR → RD): No tests completed (skipped due to Step 1 failures)")
    
    step3_total = sum(1 for r in results if r['step3_ok'] is not None)
    if step3_total > 0:
        step3_passed = sum(1 for r in results if r['step3_ok'] is True)
        if step3_passed == step3_total:
            print("✅ Step 3 (Route assignment): All tests passed")
        else:
            print(f"❌ Step 3 (Route assignment): {step3_total - step3_passed}/{step3_total} failures")
            print("   Root cause: Route detection assigns different routes to vehicles placed relative to each other")
    else:
        print("⚠️ Step 3 (Route assignment): No tests completed (simulator setup issues)")
    
    step4_total = sum(1 for r in results if r['step4_ok'] is not None)
    if step4_total > 0:
        step4_passed = sum(1 for r in results if r['step4_ok'] is True)
        if step4_passed == step4_total:
            print("✅ Step 4 (T-coordinate): All tests passed")
        else:
            print(f"❌ Step 4 (T-coordinate): {step4_total - step4_passed}/{step4_total} failures")
            print("   Root cause: T-coordinate sign or projection issue")
    else:
        print("⚠️ Step 4 (T-coordinate): No tests completed (simulator setup or route assignment issues)")

