#!/usr/bin/env python3
"""
Test the contains() method for each region to verify point-in-region queries.

This script:
1. Tests points on pit lane → pitLaneRoad.contains() = True, mainRacingRoad.contains() = False
2. Tests points on main circuit → pitLaneRoad.contains() = False, mainRacingRoad.contains() = True
3. Tests points on either → road.contains() = True
"""

import sys
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


def test_region_contains():
    """Test contains() method for each region."""
    print("=" * 80)
    print("Testing Region Contains() Method")
    print("=" * 80)
    
    # Change to Scenic root directory so localPath works correctly
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    try:
        import os
        os.chdir(scenic_root)
        
        # Create a minimal racing scenario
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model
"""
        
        scenario = scenarioFromString(scenario_code)
        scene, _ = scenario.generate(maxIterations=1)
        
        # Access regions from scenario namespace
        road = scenario._namespace.get('road')
        pitLaneRoad = scenario._namespace.get('pitLaneRoad')
        mainRacingRoad = scenario._namespace.get('mainRacingRoad')
        
        if road is None or pitLaneRoad is None or mainRacingRoad is None:
            print("   [ERROR] Could not access regions from scenario namespace")
            print("   [INFO] This test requires direct access to region objects")
            print("   [INFO] Skipping detailed contains() tests")
            return False
        
        print("\n[1] Sampling test points from each region...")
        
        # Sample points from pitLaneRoad
        print("\n   Sampling points from pitLaneRoad...")
        pit_points = []
        for _ in range(10):
            try:
                pit_point = pitLaneRoad.uniformPointInner()
                pit_points.append(pit_point)
            except Exception:
                pass
        
        print(f"   Sampled {len(pit_points)} points from pitLaneRoad")
        
        # Sample points from mainRacingRoad
        print("\n   Sampling points from mainRacingRoad...")
        main_points = []
        for _ in range(10):
            try:
                main_point = mainRacingRoad.uniformPointInner()
                main_points.append(main_point)
            except Exception:
                pass
        
        print(f"   Sampled {len(main_points)} points from mainRacingRoad")
        
        # Sample points from road
        print("\n   Sampling points from road...")
        road_points = []
        for _ in range(20):
            try:
                road_point = road.uniformPointInner()
                road_points.append(road_point)
            except Exception:
                pass
        
        print(f"   Sampled {len(road_points)} points from road")
        
        # Test contains() for pit points
        print("\n[2] Testing contains() for points from pitLaneRoad...")
        print("-" * 80)
        
        pit_in_pit = 0
        pit_in_main = 0
        pit_in_road = 0
        
        for pit_point in pit_points:
            in_pit = pitLaneRoad.containsPoint(pit_point)
            in_main = mainRacingRoad.containsPoint(pit_point)
            in_road = road.containsPoint(pit_point)
            
            if in_pit:
                pit_in_pit += 1
            if in_main:
                pit_in_main += 1
            if in_road:
                pit_in_road += 1
        
        print(f"   Points in pitLaneRoad: {pit_in_pit}/{len(pit_points)}")
        print(f"   Points in mainRacingRoad: {pit_in_main}/{len(pit_points)}")
        print(f"   Points in road: {pit_in_road}/{len(pit_points)}")
        
        if pit_in_pit == len(pit_points) and pit_in_road == len(pit_points):
            print("   [OK] All pit points are correctly contained")
        else:
            print("   [WARNING] Some pit points not correctly contained")
        
        if pit_in_main > 0:
            print("   [WARNING] Some pit points are in mainRacingRoad (overlap detected)")
        else:
            print("   [OK] No pit points in mainRacingRoad (mutually exclusive)")
        
        # Test contains() for main points
        print("\n[3] Testing contains() for points from mainRacingRoad...")
        print("-" * 80)
        
        main_in_pit = 0
        main_in_main = 0
        main_in_road = 0
        
        for main_point in main_points:
            in_pit = pitLaneRoad.containsPoint(main_point)
            in_main = mainRacingRoad.containsPoint(main_point)
            in_road = road.containsPoint(main_point)
            
            if in_pit:
                main_in_pit += 1
            if in_main:
                main_in_main += 1
            if in_road:
                main_in_road += 1
        
        print(f"   Points in pitLaneRoad: {main_in_pit}/{len(main_points)}")
        print(f"   Points in mainRacingRoad: {main_in_main}/{len(main_points)}")
        print(f"   Points in road: {main_in_road}/{len(main_points)}")
        
        if main_in_main == len(main_points) and main_in_road == len(main_points):
            print("   [OK] All main points are correctly contained")
        else:
            print("   [WARNING] Some main points not correctly contained")
        
        if main_in_pit > 0:
            print("   [WARNING] Some main points are in pitLaneRoad (overlap detected)")
        else:
            print("   [OK] No main points in pitLaneRoad (mutually exclusive)")
        
        # Test contains() for road points
        print("\n[4] Testing contains() for points from road...")
        print("-" * 80)
        
        road_in_pit = 0
        road_in_main = 0
        road_in_road = 0
        road_in_neither = 0
        
        for road_point in road_points:
            in_pit = pitLaneRoad.containsPoint(road_point)
            in_main = mainRacingRoad.containsPoint(road_point)
            in_road = road.containsPoint(road_point)
            
            if in_pit:
                road_in_pit += 1
            if in_main:
                road_in_main += 1
            if in_road:
                road_in_road += 1
            if not in_pit and not in_main:
                road_in_neither += 1
        
        print(f"   Points in pitLaneRoad: {road_in_pit}/{len(road_points)}")
        print(f"   Points in mainRacingRoad: {road_in_main}/{len(road_points)}")
        print(f"   Points in road: {road_in_road}/{len(road_points)}")
        print(f"   Points in neither: {road_in_neither}/{len(road_points)}")
        
        if road_in_road == len(road_points):
            print("   [OK] All road points are in road region")
        else:
            print("   [WARNING] Some road points not in road region")
        
        if road_in_neither > len(road_points) * 0.1:  # Allow 10% tolerance
            print("   [WARNING] Some road points are in neither pitLaneRoad nor mainRacingRoad")
        else:
            print("   [OK] Most road points are in pitLaneRoad or mainRacingRoad")
        
        # Summary
        print("\n" + "=" * 80)
        print("Test Summary")
        print("=" * 80)
        
        all_ok = (
            pit_in_pit == len(pit_points) and
            pit_in_road == len(pit_points) and
            pit_in_main == 0 and
            main_in_main == len(main_points) and
            main_in_road == len(main_points) and
            main_in_pit == 0 and
            road_in_road == len(road_points) and
            road_in_neither <= len(road_points) * 0.1
        )
        
        if all_ok:
            print("[OK] All contains() tests passed")
        else:
            print("[WARNING] Some contains() tests had issues")
            print("\nNote: Some warnings may be acceptable due to:")
            print("  - Boundary points shared between regions")
            print("  - Sampling limitations")
            print("  - Floating point precision")
        
        return all_ok
        
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Restore original working directory
        import os
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_region_contains()
    sys.exit(0 if success else 1)
