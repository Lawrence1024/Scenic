#!/usr/bin/env python3
"""
Trace through the transformation pipeline to find where relative positioning is lost.

This test:
1. Generates a scenario with "left of" positioning
2. Traces through each transformation step:
   - Step 1: Scenic XODR coordinates (original)
   - Step 2: XODR → RD transformation
   - Step 3: RD → (s,t) projection
   - Step 4: Verify left/right relationship at each step
3. Identifies where the relative position relationship is lost
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
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
from scenic.simulators.dspace.geometry import utils as geom_utils


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def compute_left_vector(heading):
    """Compute left vector in Scenic's ENU coordinate system."""
    forward_x = math.sin(heading)
    forward_y = math.cos(heading)
    left_x = -forward_y
    left_y = forward_x
    return (left_x, left_y)


def check_left_relationship(pos1, pos2, heading1, step_name):
    """Check if pos2 is to the left of pos1 in Scenic coordinate space."""
    dx = pos2[0] - pos1[0]
    dy = pos2[1] - pos1[1]
    
    left_x, left_y = compute_left_vector(heading1)
    left_component = dx * left_x + dy * left_y
    
    is_left = left_component > 0
    
    return is_left, left_component


def test_transformation_pipeline(num_tests=10):
    """Trace through transformation pipeline to find where relative position is lost."""
    print_section(f"Tracing Transformation Pipeline ({num_tests} scenarios)")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Load coordinate transform and road index
        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        coordinate_transform = None
        if transform_path.exists():
            try:
                coordinate_transform = load_transform(str(transform_path))
                print(f"   [OK] Loaded coordinate transform")
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
                print(f"   [OK] Built road index")
            except Exception as e:
                print(f"   [WARNING] Could not build road index: {e}")
        
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
        
        for test_num in range(1, num_tests + 1):
            try:
                print_section(f"Test {test_num}")
                
                # Generate scenario
                scenario = scenarioFromString(scenario_code)
                scene, iterations = scenario.generate(maxIterations=10)
                
                if len(scene.objects) < 2:
                    print(f"   [ERROR] Expected at least 2 objects")
                    continue
                
                fellow1 = scene.objects[0]
                fellow2 = scene.objects[1]
                
                # Step 1: Scenic XODR coordinates (original)
                print(f"\n   Step 1: Scenic XODR Coordinates (Original)")
                xodr1 = (float(fellow1.position.x), float(fellow1.position.y))
                xodr2 = (float(fellow2.position.x), float(fellow2.position.y))
                heading1 = float(fellow1.heading) if hasattr(fellow1, 'heading') else 0.0
                heading2 = float(fellow2.heading) if hasattr(fellow2, 'heading') else 0.0
                
                print(f"      Fellow1: ({xodr1[0]:.6f}, {xodr1[1]:.6f}), heading={math.degrees(heading1):.1f}°")
                print(f"      Fellow2: ({xodr2[0]:.6f}, {xodr2[1]:.6f}), heading={math.degrees(heading2):.1f}°")
                
                is_left_xodr, left_comp_xodr = check_left_relationship(xodr1, xodr2, heading1, "XODR")
                print(f"      Left component: {left_comp_xodr:.6f}m")
                print(f"      Is left: {is_left_xodr} {'✅' if is_left_xodr else '❌'}")
                
                if not is_left_xodr:
                    print(f"      [ERROR] Fellow2 is NOT to the left in Scenic XODR coordinates!")
                    print(f"      This indicates a bug in Scenic's 'left of' specifier")
                    results.append({'test_num': test_num, 'failed_at': 'XODR', 'left_comp': left_comp_xodr})
                    continue
                
                # Step 2: XODR → RD transformation
                print(f"\n   Step 2: XODR → RD Transformation")
                if coordinate_transform:
                    rd1 = apply_coordinate_transform(coordinate_transform, xodr1)
                    rd2 = apply_coordinate_transform(coordinate_transform, xodr2)
                    print(f"      Fellow1: ({rd1[0]:.6f}, {rd1[1]:.6f})")
                    print(f"      Fellow2: ({rd2[0]:.6f}, {rd2[1]:.6f})")
                    
                    # Check if left relationship is preserved after coordinate transform
                    # Note: Coordinate transform is affine, so it should preserve relative relationships
                    # But we need to check if heading is still valid in RD space
                    is_left_rd, left_comp_rd = check_left_relationship(rd1, rd2, heading1, "RD")
                    print(f"      Left component: {left_comp_rd:.6f}m")
                    print(f"      Is left: {is_left_rd} {'✅' if is_left_rd else '❌'}")
                    
                    if not is_left_rd:
                        print(f"      [ERROR] Left relationship LOST after XODR → RD transformation!")
                        results.append({'test_num': test_num, 'failed_at': 'RD', 'left_comp': left_comp_rd})
                        continue
                else:
                    rd1 = xodr1
                    rd2 = xodr2
                    print(f"      [NOTE] No coordinate transform, using XODR as RD")
                    is_left_rd = is_left_xodr
                    left_comp_rd = left_comp_xodr
                
                # Step 3: RD → (s,t) projection
                print(f"\n   Step 3: RD → (s,t) Projection")
                
                # Detect route
                params = scene.params if hasattr(scene, 'params') else {}
                track_segment1 = detect_track_segment(rd1, road_index, params, geom_utils)
                track_segment2 = detect_track_segment(rd2, road_index, params, geom_utils)
                route_pref1 = assign_route_for_segment(track_segment1) if track_segment1 else 'Lap'
                route_pref2 = assign_route_for_segment(track_segment2) if track_segment2 else 'Lap'
                
                print(f"      Fellow1 route: {route_pref1} (segment: {track_segment1})")
                print(f"      Fellow2 route: {route_pref2} (segment: {track_segment2})")
                
                if route_pref1 != route_pref2:
                    print(f"      [WARNING] Vehicles on different routes! Cannot compare t-coordinates directly")
                    results.append({'test_num': test_num, 'failed_at': 'Route', 'route1': route_pref1, 'route2': route_pref2})
                    continue
                
                # Project to (s,t)
                s1, t1 = project_world_to_st_route_specific(
                    road_index, rd1, route_preference=route_pref1
                )
                s2, t2 = project_world_to_st_route_specific(
                    road_index, rd2, route_preference=route_pref2
                )
                
                print(f"      Fellow1: (s={s1:.2f}, t={t1:.6f})")
                print(f"      Fellow2: (s={s2:.2f}, t={t2:.6f})")
                print(f"      T difference (t2 - t1): {t2 - t1:.6f}")
                
                # Check t-coordinate relationship
                # If fellow2 is left of fellow1, t2 should be MORE NEGATIVE than t1
                # (because positive t = left, but we're comparing relative to fellow1's position)
                # Actually, wait - let's think about this:
                # - If both are on the same route, and fellow2 is to the left of fellow1
                # - The t-coordinate is relative to the road centerline
                # - If fellow1 has t1, and fellow2 is left of fellow1, then fellow2 should have a more positive t
                # - But wait, that's not right either...
                
                # Let's check: if fellow2 is left of fellow1 in world space, what should the t relationship be?
                # The t-coordinate is the lateral offset from the road centerline
                # If fellow2 is left of fellow1, and both are on the same road:
                # - If fellow1 is on centerline (t1=0), fellow2 should have t2 > 0 (left of centerline)
                # - If fellow1 is left of centerline (t1 > 0), fellow2 should have t2 > t1 (further left)
                # - If fellow1 is right of centerline (t1 < 0), fellow2 could have t2 > t1 (less negative, moving toward left)
                
                # The key insight: if fellow2 is left of fellow1, then t2 should be MORE POSITIVE than t1
                # (assuming they're on the same route and similar s positions)
                
                expected_t_relationship = t2 > t1  # Fellow2 should have more positive t (more left)
                t_diff = t2 - t1
                
                print(f"      Expected: t2 > t1 (fellow2 more left) = {expected_t_relationship}")
                print(f"      Actual: t2 - t1 = {t_diff:.6f}")
                
                if not expected_t_relationship:
                    print(f"      [ERROR] T-coordinate relationship is INVERTED!")
                    print(f"      Fellow2 is left of fellow1 in world space, but t2 < t1")
                    results.append({'test_num': test_num, 'failed_at': 'T-coordinate', 't1': t1, 't2': t2, 'diff': t_diff})
                else:
                    print(f"      [OK] T-coordinate relationship is correct")
                    results.append({'test_num': test_num, 'failed_at': None, 't1': t1, 't2': t2, 'diff': t_diff})
                
            except Exception as e:
                print(f"   [ERROR] Test {test_num} failed: {e}")
                import traceback
                traceback.print_exc()
                results.append({'test_num': test_num, 'failed_at': 'Exception', 'error': str(e)})
        
        # Summary
        print_section("Summary")
        
        total = len(results)
        failed_xodr = sum(1 for r in results if r.get('failed_at') == 'XODR')
        failed_rd = sum(1 for r in results if r.get('failed_at') == 'RD')
        failed_route = sum(1 for r in results if r.get('failed_at') == 'Route')
        failed_t = sum(1 for r in results if r.get('failed_at') == 'T-coordinate')
        success = sum(1 for r in results if r.get('failed_at') is None)
        
        print(f"   Total tests: {total}")
        print(f"   ✅ Success: {success} ({100*success/total:.1f}%)")
        print(f"   ❌ Failed at XODR: {failed_xodr} (Scenic bug)")
        print(f"   ❌ Failed at RD: {failed_rd} (Coordinate transform issue)")
        print(f"   ⚠️  Failed at Route: {failed_route} (Different routes)")
        print(f"   ❌ Failed at T-coordinate: {failed_t} (T-coordinate sign issue)")
        
        if failed_t > 0:
            print(f"\n   [CRITICAL] T-coordinate relationship is INVERTED in {failed_t} cases")
            print(f"   This is where the relative position is lost!")
            print(f"\n   Failed cases:")
            for r in results:
                if r.get('failed_at') == 'T-coordinate':
                    print(f"      Test {r['test_num']}: t1={r['t1']:.6f}, t2={r['t2']:.6f}, diff={r['diff']:.6f}")
        
        return failed_t == 0 and failed_xodr == 0 and failed_rd == 0
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_transformation_pipeline(num_tests=10)
    sys.exit(0 if success else 1)

