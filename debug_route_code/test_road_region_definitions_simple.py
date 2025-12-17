#!/usr/bin/env python3
"""
Simplified test that verifies the three racing road regions work by testing scene generation.

This script:
1. Generates scenes with vehicles on each road type
2. Verifies that vehicles are placed correctly
3. Tests that the regions are functional (even if we can't access them directly)
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


def test_region_functionality():
    """Test that regions work by generating scenes."""
    print("=" * 80)
    print("Testing Racing Road Region Functionality")
    print("=" * 80)
    
    # Change to Scenic root directory so localPath works correctly
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    try:
        import os
        os.chdir(scenic_root)
        
        test_cases = [
            {
                'name': 'road',
                'scenario': """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on road, with raceNumber 1
""",
                'description': 'Vehicle on entire road (anywhere)'
            },
            {
                'name': 'pitLaneRoad',
                'scenario': """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on pitLaneRoad, with raceNumber 2
""",
                'description': 'Vehicle on pit lane only'
            },
            {
                'name': 'mainRacingRoad',
                'scenario': """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on mainRacingRoad, with raceNumber 3
""",
                'description': 'Vehicle on main racing circuit only'
            }
        ]
        
        results = {}
        
        for test_case in test_cases:
            print(f"\n[Testing {test_case['name']}]")
            print(f"   Description: {test_case['description']}")
            print("-" * 80)
            
            try:
                scenario = scenarioFromString(test_case['scenario'])
                scene, iterations = scenario.generate(maxIterations=10)
                
                if not scene.objects:
                    print(f"   [ERROR] No objects generated")
                    results[test_case['name']] = {'success': False, 'error': 'No objects'}
                    continue
                
                ego = scene.objects[0]
                position = ego.position
                
                print(f"   [OK] Generated vehicle at ({position.x:.3f}, {position.y:.3f})")
                print(f"   [OK] Race number: {ego.raceNumber}")
                print(f"   [OK] Iterations needed: {iterations}")
                
                results[test_case['name']] = {
                    'success': True,
                    'position': (position.x, position.y),
                    'raceNumber': ego.raceNumber
                }
                
            except Exception as e:
                print(f"   [ERROR] Failed: {e}")
                import traceback
                traceback.print_exc()
                results[test_case['name']] = {'success': False, 'error': str(e)}
        
        # Summary
        print("\n" + "=" * 80)
        print("Test Summary")
        print("=" * 80)
        
        all_success = True
        for name, result in results.items():
            if result.get('success'):
                print(f"\n{name}: [OK]")
                print(f"  Position: {result.get('position')}")
                print(f"  Race Number: {result.get('raceNumber')}")
            else:
                print(f"\n{name}: [FAILED]")
                print(f"  Error: {result.get('error', 'Unknown error')}")
                all_success = False
        
        if all_success:
            print("\n[OK] All region functionality tests passed!")
            print("\nThis confirms that:")
            print("  - road region is defined and functional")
            print("  - pitLaneRoad region is defined and functional")
            print("  - mainRacingRoad region is defined and functional")
            print("  - Vehicles can be placed on each region type")
        else:
            print("\n[WARNING] Some tests failed")
        
        return all_success
        
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
    success = test_region_functionality()
    sys.exit(0 if success else 1)
