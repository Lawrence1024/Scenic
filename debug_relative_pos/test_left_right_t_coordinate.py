"""Extensive test of left/right positioning and t-coordinate relationship in ModelDesk.

Tests that:
- "left of" results in more negative t-coordinate
- "right of" results in more positive t-coordinate

This test verifies the t-coordinate sign convention by testing many scenarios.
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


def test_left_right_t_coordinates(num_tests=50):
    """Test that left/right positioning is correctly reflected in t-coordinates."""
    print_section(f"Left/Right T-Coordinate Test ({num_tests} scenarios each)")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Test scenarios
        left_scenario = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1
fellow2 = new RacingCar left of fellow1 by 5.0, with raceNumber 2
"""
        
        right_scenario = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1
fellow2 = new RacingCar right of fellow1 by 5.0, with raceNumber 2
"""
        
        left_results = []
        right_results = []
        
        # Build road index once (used for all tests)
        from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path))
                print(f"Road index built successfully")
            except Exception as e:
                print(f"Warning: Failed to build road index: {e}")
        
        print("\nTesting 'left of' positioning...")
        left_valid = 0
        
        for test_num in range(1, num_tests + 1):
            try:
                scenario = scenarioFromString(left_scenario)
                scene, _ = scenario.generate(maxIterations=10)
                
                if len(scene.objects) < 2:
                    continue
                
                fellow1 = scene.objects[0]
                fellow2 = scene.objects[1]
                
                # Get positions
                pos1_xodr = (float(fellow1.position.x), float(fellow1.position.y))
                pos2_xodr = (float(fellow2.position.x), float(fellow2.position.y))
                
                # Transform to RD
                try:
                    from scenic.simulators.dspace.geometry.coordinate_transform import (
                        load_transform, apply_coordinate_transform
                    )
                    
                    transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
                    if not transform_path.exists():
                        continue
                    
                    coordinate_transform = load_transform(str(transform_path))
                    pos1_rd = apply_coordinate_transform(coordinate_transform, pos1_xodr)
                    pos2_rd = apply_coordinate_transform(coordinate_transform, pos2_xodr)
                    pos1_rd = (float(pos1_rd[0]), float(pos1_rd[1]))
                    pos2_rd = (float(pos2_rd[0]), float(pos2_rd[1]))
                except Exception as e:
                    continue
                
                # Get t-coordinates using consistent projection method
                # Use basic projection and verify both vehicles are on the same road
                try:
                    if road_index is None:
                        continue
                    
                    from scenic.simulators.dspace.geometry.projection import (
                        project_world_to_st, find_road_id_for_position
                    )
                    
                    # Get road IDs and t-coordinates using basic projection (consistent for both)
                    road_id1 = find_road_id_for_position(road_index, pos1_rd[0], pos1_rd[1])
                    road_id2 = find_road_id_for_position(road_index, pos2_rd[0], pos2_rd[1])
                    s1, t1 = project_world_to_st(road_index, pos1_rd)
                    s2, t2 = project_world_to_st(road_index, pos2_rd)
                    
                    # Only compare if both are on the same road (t-coordinates are comparable)
                    if road_id1 is not None and road_id2 is not None and road_id1 == road_id2 and t1 is not None and t2 is not None:
                        left_valid += 1
                        t_diff = t2 - t1
                        left_results.append({
                            'test_num': test_num,
                            'road_id': road_id1,
                            't1': t1,
                            't2': t2,
                            't_diff': t_diff,
                            'pos1_rd': pos1_rd,
                            'pos2_rd': pos2_rd,
                        })
                        
                except Exception:
                    pass
                    
            except Exception:
                continue
        
        print(f"\nTesting 'right of' positioning...")
        right_valid = 0
        
        for test_num in range(1, num_tests + 1):
            try:
                scenario = scenarioFromString(right_scenario)
                scene, _ = scenario.generate(maxIterations=10)
                
                if len(scene.objects) < 2:
                    continue
                
                fellow1 = scene.objects[0]
                fellow2 = scene.objects[1]
                
                # Get positions
                pos1_xodr = (float(fellow1.position.x), float(fellow1.position.y))
                pos2_xodr = (float(fellow2.position.x), float(fellow2.position.y))
                
                # Transform to RD
                try:
                    from scenic.simulators.dspace.geometry.coordinate_transform import (
                        load_transform, apply_coordinate_transform
                    )
                    
                    transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
                    if not transform_path.exists():
                        continue
                    
                    coordinate_transform = load_transform(str(transform_path))
                    pos1_rd = apply_coordinate_transform(coordinate_transform, pos1_xodr)
                    pos2_rd = apply_coordinate_transform(coordinate_transform, pos2_xodr)
                    pos1_rd = (float(pos1_rd[0]), float(pos1_rd[1]))
                    pos2_rd = (float(pos2_rd[0]), float(pos2_rd[1]))
                except Exception:
                    continue
                
                # Get t-coordinates using consistent projection method
                # Use basic projection and verify both vehicles are on the same road
                try:
                    if road_index is None:
                        continue
                    
                    from scenic.simulators.dspace.geometry.projection import (
                        project_world_to_st, find_road_id_for_position
                    )
                    
                    # Get road IDs and t-coordinates using basic projection (consistent for both)
                    road_id1 = find_road_id_for_position(road_index, pos1_rd[0], pos1_rd[1])
                    road_id2 = find_road_id_for_position(road_index, pos2_rd[0], pos2_rd[1])
                    s1, t1 = project_world_to_st(road_index, pos1_rd)
                    s2, t2 = project_world_to_st(road_index, pos2_rd)
                    
                    # Only compare if both are on the same road (t-coordinates are comparable)
                    if road_id1 is not None and road_id2 is not None and road_id1 == road_id2 and t1 is not None and t2 is not None:
                        right_valid += 1
                        t_diff = t2 - t1
                        right_results.append({
                            'test_num': test_num,
                            'road_id': road_id1,
                            't1': t1,
                            't2': t2,
                            't_diff': t_diff,
                            'pos1_rd': pos1_rd,
                            'pos2_rd': pos2_rd,
                        })
                        
                except Exception:
                    pass
                    
            except Exception:
                continue
        
        # Analyze results
        print_section("Results Analysis")
        
        print(f"\n'Left of' tests:")
        print(f"  Valid tests (same road, both t-coordinates available): {left_valid}/{num_tests}")
        if left_results:
            left_t_diffs = [r['t_diff'] for r in left_results]
            avg_left_diff = sum(left_t_diffs) / len(left_t_diffs)
            min_left_diff = min(left_t_diffs)
            max_left_diff = max(left_t_diffs)
            left_negative = sum(1 for d in left_t_diffs if d < 0)
            left_positive = sum(1 for d in left_t_diffs if d > 0)
            
            print(f"  T-difference (t2 - t1):")
            print(f"    Average: {avg_left_diff:.6f}")
            print(f"    Min: {min_left_diff:.6f}")
            print(f"    Max: {max_left_diff:.6f}")
            print(f"    Negative (t2 < t1): {left_negative}/{len(left_results)} ({100*left_negative/len(left_results):.1f}%)")
            print(f"    Positive (t2 > t1): {left_positive}/{len(left_results)} ({100*left_positive/len(left_results):.1f}%)")
            
            if avg_left_diff < 0:
                print(f"  ✅ 'Left of' → More negative t (t2 < t1) - CONFIRMED")
            else:
                print(f"  ❌ 'Left of' → More positive t (t2 > t1) - INCONSISTENT WITH EXPECTATION")
        else:
            print(f"  ⚠️ No valid results")
        
        print(f"\n'Right of' tests:")
        print(f"  Valid tests (same road, both t-coordinates available): {right_valid}/{num_tests}")
        if right_results:
            right_t_diffs = [r['t_diff'] for r in right_results]
            avg_right_diff = sum(right_t_diffs) / len(right_t_diffs)
            min_right_diff = min(right_t_diffs)
            max_right_diff = max(right_t_diffs)
            right_negative = sum(1 for d in right_t_diffs if d < 0)
            right_positive = sum(1 for d in right_t_diffs if d > 0)
            
            print(f"  T-difference (t2 - t1):")
            print(f"    Average: {avg_right_diff:.6f}")
            print(f"    Min: {min_right_diff:.6f}")
            print(f"    Max: {max_right_diff:.6f}")
            print(f"    Negative (t2 < t1): {right_negative}/{len(right_results)} ({100*right_negative/len(right_results):.1f}%)")
            print(f"    Positive (t2 > t1): {right_positive}/{len(right_results)} ({100*right_positive/len(right_results):.1f}%)")
            
            if avg_right_diff > 0:
                print(f"  ✅ 'Right of' → More positive t (t2 > t1) - CONFIRMED")
            else:
                print(f"  ❌ 'Right of' → More negative t (t2 < t1) - INCONSISTENT WITH EXPECTATION")
        else:
            print(f"  ⚠️ No valid results")
        
        # Summary
        print_section("Summary")
        
        if left_results and right_results:
            left_avg = sum(r['t_diff'] for r in left_results) / len(left_results)
            right_avg = sum(r['t_diff'] for r in right_results) / len(right_results)
            
            if left_avg < 0 and right_avg > 0:
                print("✅ T-COORDINATE CONVENTION CONFIRMED:")
                print("   - 'Left of' → More negative t (t2 < t1)")
                print("   - 'Right of' → More positive t (t2 > t1)")
            elif left_avg > 0 and right_avg < 0:
                print("❌ T-COORDINATE CONVENTION IS INVERTED:")
                print("   - 'Left of' → More positive t (t2 > t1)")
                print("   - 'Right of' → More negative t (t2 < t1)")
                print("   - This is the OPPOSITE of expected behavior")
            else:
                print("⚠️ T-COORDINATE CONVENTION UNCLEAR:")
                print(f"   - 'Left of' average t-diff: {left_avg:.6f}")
                print(f"   - 'Right of' average t-diff: {right_avg:.6f}")
        else:
            print("⚠️ Insufficient data to draw conclusions")
        
        # Show sample results
        if left_results:
            print_section("Sample 'Left of' Results (first 5)")
            for r in left_results[:5]:
                print(f"\nTest {r['test_num']} (Road ID: {r['road_id']}):")
                print(f"  t1: {r['t1']:.6f}")
                print(f"  t2: {r['t2']:.6f}")
                print(f"  t2 - t1: {r['t_diff']:.6f}")
        
        if right_results:
            print_section("Sample 'Right of' Results (first 5)")
            for r in right_results[:5]:
                print(f"\nTest {r['test_num']} (Road ID: {r['road_id']}):")
                print(f"  t1: {r['t1']:.6f}")
                print(f"  t2: {r['t2']:.6f}")
                print(f"  t2 - t1: {r['t_diff']:.6f}")
        
        return {
            'left_results': left_results,
            'right_results': right_results,
        }
        
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    print("="*80)
    print("LEFT/RIGHT T-COORDINATE VERIFICATION TEST")
    print("="*80)
    print("\nThis test verifies that:")
    print("  - 'Left of' positioning results in MORE NEGATIVE t-coordinate")
    print("  - 'Right of' positioning results in MORE POSITIVE t-coordinate")
    print("\nRunning extensive tests (50 scenarios each)...")
    
    results = test_left_right_t_coordinates(50)

