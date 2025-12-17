#!/usr/bin/env python3
"""
Test that vehicles placed on different road types get assigned correct routes in dSPACE.

This script:
1. Creates vehicles on pitLaneRoad → should get Route R1 (Pit)
2. Creates vehicles on mainRacingRoad → should get Route R2 (Lap)
3. Creates vehicles on road → should get route based on actual position (auto-detected)
4. Verifies route assignments in ModelDesk
"""

import sys
from pathlib import Path
import time

# Fix encoding for Windows console
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
if sys.stderr.encoding != 'utf-8':
    sys.stderr.reconfigure(encoding='utf-8')

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.utils import legacy as dutils


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("=" * 80)
    print("Connecting to ModelDesk...")
    print("=" * 80)
    
    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    
    if proj is None:
        raise RuntimeError("Open a ModelDesk project first.")
    
    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("Activate an experiment in ModelDesk.")
    
    ts = exp.TrafficScenario
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")
    
    print("[OK] Connected to ModelDesk")
    print(f"   Project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
    print(f"   Experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
    print(f"   TrafficScenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
    
    return app, proj, exp, ts


def get_route_name(seq):
    """Get the active route name for a sequence."""
    try:
        route_sel = seq.Route if hasattr(seq, 'Route') else seq.RouteSelection
        if hasattr(route_sel, 'ActiveElement') and hasattr(route_sel.ActiveElement, 'Name'):
            return route_sel.ActiveElement.Name
        return None
    except Exception:
        return None


def clear_fellows(ts):
    """Clear all existing fellows."""
    try:
        fellows = ts.Fellows
        count = fellows.Count if hasattr(fellows, 'Count') else len(list(fellows))
        for i in range(count - 1, -1, -1):
            try:
                fellows.Remove(i)
            except Exception:
                pass
        print(f"   Cleared {count} existing fellows")
    except Exception as e:
        print(f"   [WARNING] Could not clear fellows: {e}")


def add_fellow_to_modeldesk(ts, fellow_idx, scenic_x, scenic_y, expected_route, transform, road_index):
    """Add a fellow vehicle to ModelDesk and verify route assignment."""
    print(f"\n   Adding Fellow_{fellow_idx + 1} at ({scenic_x:.3f}, {scenic_y:.3f})...")
    
    # Transform XODR -> RD
    if transform:
        from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform
        rd_x, rd_y = apply_coordinate_transform(transform, (scenic_x, scenic_y))
        print(f"      XODR ({scenic_x:.3f}, {scenic_y:.3f}) -> RD ({rd_x:.3f}, {rd_y:.3f})")
    else:
        rd_x, rd_y = scenic_x, scenic_y
    
    # Project RD (x,y) -> (s,t)
    if road_index:
        s_val, t_val = dutils.project_world_to_st(road_index, (rd_x, rd_y))
        print(f"      RD ({rd_x:.3f}, {rd_y:.3f}) -> (s={s_val:.1f}, t={t_val:.3f})")
    else:
        s_val, t_val = 0.0, 0.0
    
    # Create fellow
    try:
        fellow = ts.Fellows.Add()
        seq = fellow.Sequences.Item(0)
        segs = seq.Segments
        
        # Ensure we have at least 2 segments
        from scenic.simulators.dspace.geometry.utils import ensure_two_segments
        ensure_two_segments(seq)
        segs = seq.Segments
        
        # Set position on segment 0
        from scenic.simulators.dspace.geometry.utils import configure_seg0_absolute_pose
        configure_seg0_absolute_pose(segs, s=s_val, t=t_val)
        
        # Set route (let the simulator assign based on position)
        route_sel = seq.Route if hasattr(seq, 'Route') else seq.RouteSelection
        
        # Try to detect route from position
        params = {
            'pitLaneRoadIds': ['1'],
            'mainRacingRoadIds': ['0', '2'],
        }
        
        from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
        from scenic.simulators.dspace.geometry import utils as geom_utils
        
        segment = detect_track_segment((rd_x, rd_y), road_index, params, geom_utils)
        route_pref = assign_route_for_segment(segment) if segment else expected_route
        
        # Map route preference to route name
        route_name_map = {
            'Pit': 'R1',
            'Lap': 'R2',
        }
        route_name = route_name_map.get(route_pref, expected_route)
        
        # Try to activate route
        try:
            route_sel.Activate(route_name)
            print(f"      Route set to: {route_name} (from segment: {segment})")
        except Exception:
            # Try alternatives
            try:
                if route_pref == 'Pit':
                    route_sel.Activate('R1')
                    route_name = 'R1'
                else:
                    route_sel.Activate('R2')
                    route_name = 'R2'
                print(f"      Route set to: {route_name} (fallback)")
            except Exception:
                print(f"      [WARNING] Could not set route, using default")
        
        # Verify route
        actual_route = get_route_name(seq)
        print(f"      Actual route: {actual_route}")
        
        if actual_route:
            # Check if route matches expected
            route_matches = (
                (expected_route == 'R1' and ('R1' in actual_route or 'Pit' in actual_route)) or
                (expected_route == 'R2' and ('R2' in actual_route or 'Lap' in actual_route))
            )
            
            if route_matches:
                print(f"      [OK] Route matches expected ({expected_route})")
                return True, actual_route
            else:
                print(f"      [WARNING] Route '{actual_route}' does not match expected '{expected_route}'")
                return False, actual_route
        else:
            print(f"      [WARNING] Could not determine route")
            return False, None
            
    except Exception as e:
        print(f"      [ERROR] Failed to add fellow: {e}")
        import traceback
        traceback.print_exc()
        return False, None


def test_route_assignment():
    """Test route assignment for different road types."""
    print("=" * 80)
    print("Testing Road Type Route Assignment")
    print("=" * 80)
    
    # Connect to ModelDesk
    app, proj, exp, ts = connect_to_modeldesk()
    
    # Load coordinate transform and road index
    print("\n" + "=" * 80)
    print("Loading Coordinate Transform and Road Index...")
    print("=" * 80)
    
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    transform = None
    if transform_path.exists():
        try:
            from scenic.simulators.dspace.geometry.coordinate_transform import load_transform
            transform = load_transform(str(transform_path))
            print(f"   [OK] Loaded transform")
        except Exception as e:
            print(f"   [WARNING] Could not load transform: {e}")
    
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = None
    if rd_path.exists():
        try:
            from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
            road_index = build_rd_road_index(str(rd_path), step=0.5)
            print(f"   [OK] Built road index")
        except Exception as e:
            print(f"   [WARNING] Could not build road index: {e}")
    
    # Clear existing fellows
    print("\n" + "=" * 80)
    print("Clearing Existing Fellows...")
    print("=" * 80)
    clear_fellows(ts)
    
    # Create test scenarios
    print("\n" + "=" * 80)
    print("Creating Test Vehicles...")
    print("=" * 80)
    
    from scenic import scenarioFromString
    
    test_cases = [
        {
            'name': 'pitLaneRoad',
            'scenario': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on pitLaneRoad, with raceNumber 1
""",
            'expected_route': 'R1'
        },
        {
            'name': 'mainRacingRoad',
            'scenario': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on mainRacingRoad, with raceNumber 2
""",
            'expected_route': 'R2'
        },
        {
            'name': 'road',
            'scenario': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

ego = new RacingCar on road, with raceNumber 3
""",
            'expected_route': None  # Auto-detect
        }
    ]
    
    results = {}
    
    for test_case in test_cases:
        print(f"\n[Testing {test_case['name']}]")
        print("-" * 80)
        
        try:
            scenario = scenarioFromString(test_case['scenario'])
            scene, _ = scenario.generate(maxIterations=10)
            
            if not scene.objects:
                print(f"   [ERROR] No objects generated")
                results[test_case['name']] = None
                continue
            
            ego = scene.objects[0]
            position = ego.position
            
            print(f"   Generated position: ({position.x:.3f}, {position.y:.3f})")
            
            # Determine expected route
            expected_route = test_case['expected_route']
            if expected_route is None:
                # Auto-detect based on position
                if transform:
                    from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform
                    rd_pos = apply_coordinate_transform(transform, (position.x, position.y))
                else:
                    rd_pos = (position.x, position.y)
                
                params = {
                    'pitLaneRoadIds': ['1'],
                    'mainRacingRoadIds': ['0', '2'],
                }
                
                from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
                from scenic.simulators.dspace.geometry import utils as geom_utils
                
                segment = detect_track_segment(rd_pos, road_index, params, geom_utils)
                route_pref = assign_route_for_segment(segment) if segment else 'Lap'
                expected_route = 'R1' if route_pref == 'Pit' else 'R2'
                print(f"   Auto-detected expected route: {expected_route} (from segment: {segment})")
            
            # Add fellow to ModelDesk
            success, actual_route = add_fellow_to_modeldesk(
                ts, len(results), position.x, position.y, expected_route, transform, road_index
            )
            
            results[test_case['name']] = {
                'position': (position.x, position.y),
                'expected_route': expected_route,
                'actual_route': actual_route,
                'success': success
            }
            
        except Exception as e:
            print(f"   [ERROR] Failed: {e}")
            import traceback
            traceback.print_exc()
            results[test_case['name']] = None
    
    # Save scenario
    print("\n" + "=" * 80)
    print("Saving Scenario...")
    print("=" * 80)
    try:
        ts.Save()
        print("   [OK] Scenario saved")
    except Exception as e:
        print(f"   [WARNING] Could not save scenario: {e}")
    
    # Summary
    print("\n" + "=" * 80)
    print("Test Summary")
    print("=" * 80)
    
    for name, result in results.items():
        if result:
            print(f"\n{name}:")
            print(f"  Position: {result['position']}")
            print(f"  Expected route: {result['expected_route']}")
            print(f"  Actual route: {result['actual_route']}")
            print(f"  Status: {'[OK]' if result['success'] else '[FAILED]'}")
        else:
            print(f"\n{name}: [FAILED]")
    
    return all(r is not None and r.get('success', False) for r in results.values() if r is not None)


if __name__ == "__main__":
    try:
        success = test_route_assignment()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
