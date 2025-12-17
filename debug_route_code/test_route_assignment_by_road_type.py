#!/usr/bin/env python3
"""
Test route assignment based on road type sampling.

This script tests if:
1. Vehicles sampled from pitLaneRoad get assigned R1 (Pit) route in ModelDesk
2. Vehicles sampled from mainRacingRoad get assigned R2 (Lap) route in ModelDesk

Note: We know t-coordinate has issues, but we still want to verify route assignment works correctly.
"""

import sys
import os
import time
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

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import find_road_id_for_position
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
from scenic.simulators.dspace.geometry import utils as geom_utils
from scenic.simulators.dspace.utils import legacy as dutils
from scenic import scenarioFromString


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def get_route_from_sequence(seq):
    """Get the route name from a sequence object.
    
    Returns:
        Route name (e.g., 'R1', 'R2', 'Pit', 'Lap') or None if not found
    """
    try:
        route_sel = seq.Route if hasattr(seq, 'Route') else seq.RouteSelection
        if hasattr(route_sel, 'ActiveElement'):
            active = route_sel.ActiveElement
            if hasattr(active, 'Name'):
                return active.Name
        if hasattr(route_sel, 'Active'):
            return route_sel.Active
        # Try to get from AvailableElements
        if hasattr(route_sel, 'AvailableElements'):
            for elem in route_sel.AvailableElements:
                if hasattr(elem, 'Active') and elem.Active:
                    return elem.Name if hasattr(elem, 'Name') else str(elem)
    except Exception as e:
        pass
    return None


def test_route_assignment_for_road_type(road_type_name, expected_route, num_samples=5):
    """Test route assignment for a specific road type.
    
    Args:
        road_type_name: 'pitLaneRoad' or 'mainRacingRoad'
        expected_route: Expected route name ('R1' for pit, 'R2' for lap)
        num_samples: Number of vehicles to sample and test
        
    Returns:
        dict with test results
    """
    print_section(f"Testing Route Assignment for {road_type_name}")
    
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
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
            except Exception as e:
                print(f"   [ERROR] Could not build road index: {e}")
                return None
        else:
            print(f"   [ERROR] RD file not found: {rd_path}")
            return None
        
        # Connect to ModelDesk
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        
        if proj is None:
            print("   [ERROR] Open a ModelDesk project first")
            return None
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("   [ERROR] Activate an experiment in ModelDesk")
            return None
        
        ts = exp.TrafficScenario
        if ts is None:
            print("   [ERROR] Active experiment has no TrafficScenario")
            return None
        
        print(f"   [OK] Connected to ModelDesk")
        
        # Generate vehicles from Scenic
        print(f"\n   Generating {num_samples} vehicles on {road_type_name}...")
        
        scenario_code = f"""
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on {road_type_name}
ego = new RacingCar on {road_type_name}, with raceNumber 1
"""
        
        results = []
        
        for i in range(num_samples):
            print(f"\n   Sample {i+1}/{num_samples}:")
            
            try:
                scenario = scenarioFromString(scenario_code)
                scene, iterations = scenario.generate(maxIterations=10)
                
                if not scene.objects:
                    print(f"      [ERROR] No objects generated")
                    continue
                
                ego = scene.objects[0]
                scenic_xodr = (float(ego.position.x), float(ego.position.y), 0.0)
                print(f"      Generated at XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
                
                # Extract road IDs from scenario params
                pit_lane_road_ids = scene.params.get('pitLaneRoadIds', [])
                main_racing_road_ids = scene.params.get('mainRacingRoadIds', [])
                
                # Transform to RD for segment detection
                if coordinate_transform:
                    rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
                else:
                    rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
                
                print(f"      RD coordinate: ({rd_x:.6f}, {rd_y:.6f})")
                
                # Detect track segment
                params = {
                    'pitLaneRoadIds': [str(rid) for rid in pit_lane_road_ids] if pit_lane_road_ids else [],
                    'mainRacingRoadIds': [str(rid) for rid in main_racing_road_ids] if main_racing_road_ids else [],
                }
                
                track_segment = detect_track_segment((rd_x, rd_y), road_index, params, geom_utils)
                route_pref = assign_route_for_segment(track_segment) if track_segment else None
                
                print(f"      Detected segment: {track_segment}")
                print(f"      Assigned route preference: {route_pref}")
                
                # Project to (s,t)
                if route_pref:
                    s_val, t_val = project_world_to_st_route_specific(
                        road_index,
                        (rd_x, rd_y),
                        route_preference=route_pref
                    )
                else:
                    from scenic.simulators.dspace.geometry.projection import project_world_to_st
                    s_val, t_val = project_world_to_st(road_index, (rd_x, rd_y))
                    route_pref = 'Lap'  # Default
                
                print(f"      Projected (s,t): ({s_val:.1f}, {t_val:.3f})")
                
                # Create fellow in ModelDesk
                try:
                    dutils.clear_collection(ts.Fellows)
                except:
                    pass
                
                fellow = ts.Fellows.Add()
                fellow.Name = f"Test_{road_type_name}_{i+1}"
                
                seqs = fellow.Sequences
                dutils.clear_collection(seqs)
                S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
                segs = dutils.ensure_two_segments(S1)
                
                # Set (s,t)
                dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
                
                # Set route - force based on track_segment
                route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
                route_sel.UseExternal = False
                if hasattr(route_sel, 'Direction'):
                    route_sel.Direction = 0
                
                # Map route preference to ModelDesk route
                route_name_map = {'Pit': 'R1', 'Lap': 'R2'}
                if track_segment == 'pitLane':
                    modeldesk_route = 'R1'
                elif track_segment == 'mainRacing':
                    modeldesk_route = 'R2'
                else:
                    modeldesk_route = route_name_map.get(route_pref, 'R2')
                
                route_sel.Activate(modeldesk_route)
                print(f"      Set route in ModelDesk: {modeldesk_route}")
                
                # Configure segment 1
                try:
                    dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
                    dutils.make_endless_transition(segs)
                except Exception as e:
                    pass
                
                # Read back route from ModelDesk
                actual_route = get_route_from_sequence(S1)
                print(f"      Readback route from ModelDesk: {actual_route}")
                
                # Verify
                route_correct = (actual_route == expected_route or 
                                (expected_route == 'R1' and actual_route in ['R1', 'Pit']) or
                                (expected_route == 'R2' and actual_route in ['R2', 'Lap']))
                
                if route_correct:
                    print(f"      [OK] Route assignment correct: {actual_route} (expected {expected_route})")
                else:
                    print(f"      [ERROR] Route assignment incorrect: {actual_route} (expected {expected_route})")
                
                results.append({
                    'sample': i + 1,
                    'scenic_xodr': scenic_xodr,
                    'rd_coord': (rd_x, rd_y),
                    'track_segment': track_segment,
                    'route_pref': route_pref,
                    'modeldesk_route': modeldesk_route,
                    'actual_route': actual_route,
                    'route_correct': route_correct,
                    's': s_val,
                    't': t_val
                })
                
            except Exception as e:
                print(f"      [ERROR] Test failed: {e}")
                import traceback
                traceback.print_exc()
                continue
        
        # Summary
        print_section(f"Summary for {road_type_name}")
        
        if not results:
            print("   [ERROR] No successful tests")
            return None
        
        correct_count = sum(1 for r in results if r['route_correct'])
        total_count = len(results)
        success_rate = (correct_count / total_count * 100) if total_count > 0 else 0
        
        print(f"\n   Results:")
        print(f"      Total samples: {total_count}")
        print(f"      Correct route assignments: {correct_count}")
        print(f"      Success rate: {success_rate:.1f}%")
        
        print(f"\n   Detailed Results:")
        print(f"   {'Sample':<8} {'Segment':<12} {'Route Pref':<12} {'ModelDesk':<12} {'Actual':<12} {'Status':<10}")
        print(f"   {'-'*8} {'-'*12} {'-'*12} {'-'*12} {'-'*12} {'-'*10}")
        
        for r in results:
            status = "✅ OK" if r['route_correct'] else "❌ ERROR"
            print(f"   {r['sample']:<8} {r['track_segment'] or 'None':<12} {r['route_pref'] or 'None':<12} {r['modeldesk_route']:<12} {r['actual_route'] or 'None':<12} {status}")
        
        if success_rate == 100:
            print(f"\n   ✅ SUCCESS: All samples correctly assigned to {expected_route}")
        elif success_rate >= 80:
            print(f"\n   ⚠️  PARTIAL: Most samples correctly assigned, but some issues")
        else:
            print(f"\n   ❌ FAILURE: Route assignment not working correctly")
        
        return {
            'road_type': road_type_name,
            'expected_route': expected_route,
            'total_samples': total_count,
            'correct_count': correct_count,
            'success_rate': success_rate,
            'results': results
        }
        
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return None
    finally:
        os.chdir(original_cwd)


def test_route_assignment_by_road_type():
    """Test route assignment for both road types."""
    print_section("Route Assignment Test by Road Type")
    
    print("\nThis test verifies that:")
    print("  1. Vehicles sampled from pitLaneRoad get assigned R1 (Pit) route")
    print("  2. Vehicles sampled from mainRacingRoad get assigned R2 (Lap) route")
    print("\nNote: We know t-coordinate has issues, but route assignment should still work.")
    
    # Test pitLaneRoad -> R1
    pit_results = test_route_assignment_for_road_type('pitLaneRoad', 'R1', num_samples=5)
    
    # Test mainRacingRoad -> R2
    main_results = test_route_assignment_for_road_type('mainRacingRoad', 'R2', num_samples=5)
    
    # Overall summary
    print_section("Overall Summary")
    
    if pit_results and main_results:
        pit_success = pit_results['success_rate']
        main_success = main_results['success_rate']
        
        print(f"\n   pitLaneRoad -> R1: {pit_success:.1f}% success rate")
        print(f"   mainRacingRoad -> R2: {main_success:.1f}% success rate")
        
        if pit_success == 100 and main_success == 100:
            print(f"\n   ✅ SUCCESS: Route assignment works correctly for both road types")
        elif pit_success >= 80 and main_success >= 80:
            print(f"\n   ⚠️  PARTIAL: Route assignment mostly works, but some issues")
        else:
            print(f"\n   ❌ FAILURE: Route assignment has significant issues")
    else:
        print(f"\n   [ERROR] Could not complete all tests")


if __name__ == "__main__":
    try:
        test_route_assignment_by_road_type()
        sys.exit(0)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
