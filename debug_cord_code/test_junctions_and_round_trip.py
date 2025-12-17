"""Test script to verify junction positions and refine round-trip coordinate transformation.

This script tests:
1. Junction positions and their route s-coordinates
2. Fine-grained transitions around the 100m offset
3. Complete round-trip: XODR -> RD -> route s -> ModelDesk -> ControlDesk RD -> XODR

Goal: Achieve <1m error in round-trip transformation so Scenic operates in only one coordinate system.
"""

import sys
import os
import time
import math
import pythoncom
from win32com.client import Dispatch

# Add Scenic to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform, apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
from scenic.simulators.dspace.geometry.route_projection import find_road_name_by_id
from scenic.simulators.dspace.utils import legacy as dutils


# Junction coordinates from ModelDesk UI (RD coordinates)
JUNCTIONS = {
    'Junction': (183.45, 28.33, 0.00),  # Near route start points
    'Junction_1': (-97.09, -481.29, 0.00),  # Near transition to The Corkscrew1
}

# Road lengths (from previous testing)
ROAD_LENGTHS = {
    'Pit Lane1_2': 883.4,
    'Andretti Hairpin1_3': 988.0,
    'The Corkscrew1': 2484.6,
}


def test_junction_positions(road_index, fellow, cd, ts):
    """Test placing fellows at junction coordinates and read back route s-coordinates."""
    print("\n" + "="*80)
    print("Testing Junction Positions")
    print("="*80)
    
    results = {}
    
    for junction_name, (rd_x, rd_y, rd_z) in JUNCTIONS.items():
        print(f"\nTesting {junction_name} at RD ({rd_x:.2f}, {rd_y:.2f})...")
        
        # Project RD coordinate to (s, t) on each route
        for route_name in ['R1', 'R2']:
            print(f"  Testing on {route_name}...")
            
            # Get sequence
            sequences = fellow.Sequences
            if sequences.Count == 0:
                seq = sequences.Add()
            else:
                try:
                    seq = sequences.Item(1)
                except:
                    seq = sequences.Item(0)
            
            # Set route
            route_sel = seq.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0
            route_sel.Activate(route_name)
            
            # Ensure 2 segments
            segs = dutils.ensure_two_segments(seq)
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
            
            # Project RD to (s, t) - use route-specific projection if available
            try:
                from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
                route_pref = 'Pit' if route_name == 'R1' else 'Lap'
                s_val, t_val = project_world_to_st_route_specific(
                    road_index, (rd_x, rd_y), route_preference=route_pref
                )
            except:
                # Fallback to regular projection
                s_val, t_val = project_world_to_st(road_index, rd_x, rd_y)
            
            print(f"    Projected to route s={s_val:.2f}, t={t_val:.3f}")
            
            # Place fellow at this (s, t)
            dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
            
            # Save, download, reset, start
            ts.Save()
            ts.Download()
            time.sleep(0.5)
            
            app = Dispatch("ModelDesk.Application")
            exp = app.ActiveProject.ActiveExperiment
            mc = exp.ManeuverControl
            try:
                mc.Stop()
            except:
                pass
            time.sleep(0.2)
            mc.Reset()
            time.sleep(0.2)
            mc.Start(False)
            time.sleep(2.0)
            
            # Step simulation
            for i in range(20):
                cd.advance_simulation_step()
                time.sleep(0.1)
            time.sleep(1.0)
            
            # Read back position
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            readback_rd_x = float(x_arr[0])
            readback_rd_y = float(y_arr[0])
            
            # Calculate error
            error = math.sqrt((readback_rd_x - rd_x)**2 + (readback_rd_y - rd_y)**2)
            
            # Identify which road
            road_id = find_road_id_for_position(road_index, readback_rd_x, readback_rd_y)
            road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
            
            print(f"    Readback RD: ({readback_rd_x:.2f}, {readback_rd_y:.2f})")
            print(f"    Error: {error:.2f}m")
            print(f"    On road: {road_name}")
            
            key = f"{junction_name}_{route_name}"
            results[key] = {
                'junction_rd': (rd_x, rd_y),
                'route_s': s_val,
                'readback_rd': (readback_rd_x, readback_rd_y),
                'error': error,
                'road_name': road_name,
            }
    
    return results


def test_fine_grained_transitions(road_index, fellow, cd, ts):
    """Test fine-grained s values around transition points to find exact transition."""
    print("\n" + "="*80)
    print("Testing Fine-Grained Transitions")
    print("="*80)
    
    # Test points around transitions (10m increments)
    r1_transition_tests = list(range(870, 1000, 10))  # Around 883.4m and 983.4m
    r2_transition_tests = list(range(975, 1100, 10))  # Around 988.0m and 1088.0m
    
    results = {}
    
    for route_name, test_s_values, expected_transition in [
        ('R1', r1_transition_tests, 883.4),
        ('R2', r2_transition_tests, 988.0),
    ]:
        print(f"\nTesting {route_name} transitions...")
        print(f"  Expected transition at s={expected_transition:.1f}m")
        
        # Get sequence
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            try:
                seq = sequences.Item(1)
            except:
                seq = sequences.Item(0)
        
        # Set route
        route_sel = seq.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0
        route_sel.Activate(route_name)
        
        # Ensure 2 segments
        segs = dutils.ensure_two_segments(seq)
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        dutils.make_endless_transition(segs)
        
        route_results = []
        prev_road = None
        
        for s_val in test_s_values:
            # Place fellow at s value
            dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=0.0)
            
            # Save, download, reset, start
            ts.Save()
            ts.Download()
            time.sleep(0.3)  # Reduced for faster testing
            
            app = Dispatch("ModelDesk.Application")
            exp = app.ActiveProject.ActiveExperiment
            mc = exp.ManeuverControl
            try:
                mc.Stop()
            except:
                pass
            time.sleep(0.1)
            mc.Reset()
            time.sleep(0.1)
            mc.Start(False)
            time.sleep(1.5)
            
            # Step simulation (reduced for faster testing)
            for i in range(15):
                cd.advance_simulation_step()
                time.sleep(0.05)
            time.sleep(0.5)
            
            # Read back position
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            rd_x = float(x_arr[0])
            rd_y = float(y_arr[0])
            
            # Identify road
            road_id = find_road_id_for_position(road_index, rd_x, rd_y)
            road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
            
            transition_detected = (prev_road is not None and road_name != prev_road)
            
            if transition_detected or s_val in [expected_transition, expected_transition + 100]:
                print(f"    s={s_val:>6.1f} -> RD ({rd_x:>8.2f}, {rd_y:>8.2f}) on {road_name:<25} {'<-- TRANSITION' if transition_detected else ''}")
            
            route_results.append({
                's': s_val,
                'rd': (rd_x, rd_y),
                'road_name': road_name,
                'transition': transition_detected,
            })
            
            prev_road = road_name
        
        # Find transition point
        transitions = []
        for i in range(1, len(route_results)):
            if route_results[i]['road_name'] != route_results[i-1]['road_name']:
                transitions.append({
                    's': route_results[i]['s'],
                    'from': route_results[i-1]['road_name'],
                    'to': route_results[i]['road_name'],
                })
        
        if transitions:
            print(f"\n  Transitions found:")
            for trans in transitions:
                print(f"    At s={trans['s']:.1f}: {trans['from']} -> {trans['to']}")
        
        results[route_name] = route_results
    
    return results


def test_round_trip_xodr(coordinate_transform, road_index, fellow, cd, ts, test_coordinates):
    """Test complete round-trip: XODR -> RD -> route s -> ModelDesk -> ControlDesk RD -> XODR."""
    print("\n" + "="*80)
    print("Testing Complete Round-Trip (XODR -> RD -> route s -> ModelDesk -> ControlDesk RD -> XODR)")
    print("="*80)
    
    results = []
    
    for name, coord_data in test_coordinates.items():
        scenic_xodr = coord_data['scenic_xodr']
        print(f"\n{name}:")
        print(f"  Starting XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
        
        # Step 1: XODR → RD
        if coordinate_transform:
            rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        else:
            rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
        print(f"  Step 1 (XODR -> RD): ({rd_x:.3f}, {rd_y:.3f})")
        
        # Step 2: Determine route (simplified - would use detectTrackSegment in real code)
        # For testing, we'll try both routes
        route_candidates = ['R1', 'R2']
        
        best_result = None
        best_error = float('inf')
        
        for route_name in route_candidates:
            route_pref = 'Pit' if route_name == 'R1' else 'Lap'
            
            # Project RD -> route s
            try:
                from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
                s_val, t_val = project_world_to_st_route_specific(
                    road_index, (rd_x, rd_y), route_preference=route_pref
                )
            except:
                s_val, t_val = project_world_to_st(road_index, rd_x, rd_y)
            
            # Place fellow
            sequences = fellow.Sequences
            if sequences.Count == 0:
                seq = sequences.Add()
            else:
                try:
                    seq = sequences.Item(1)
                except:
                    seq = sequences.Item(0)
            
            route_sel = seq.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0
            route_sel.Activate(route_name)
            
            segs = dutils.ensure_two_segments(seq)
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
            dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
            
            # Save, download, reset, start
            ts.Save()
            ts.Download()
            time.sleep(0.5)
            
            app = Dispatch("ModelDesk.Application")
            exp = app.ActiveProject.ActiveExperiment
            mc = exp.ManeuverControl
            try:
                mc.Stop()
            except:
                pass
            time.sleep(0.2)
            mc.Reset()
            time.sleep(0.2)
            mc.Start(False)
            time.sleep(2.0)
            
            # Step simulation
            for i in range(20):
                cd.advance_simulation_step()
                time.sleep(0.1)
            time.sleep(1.0)
            
            # Read back RD
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            readback_rd_x = float(x_arr[0])
            readback_rd_y = float(y_arr[0])
            
            # Step 5: RD -> XODR (inverse transform)
            if coordinate_transform:
                readback_xodr_x, readback_xodr_y = apply_inverse_coordinate_transform(
                    coordinate_transform, (readback_rd_x, readback_rd_y)
                )
            else:
                readback_xodr_x, readback_xodr_y = readback_rd_x, readback_rd_y
            
            # Calculate error
            error = math.sqrt((readback_xodr_x - scenic_xodr[0])**2 + (readback_xodr_y - scenic_xodr[1])**2)
            
            if error < best_error:
                best_error = error
                best_result = {
                    'name': name,
                    'route': route_name,
                    'scenic_xodr': scenic_xodr,
                    'rd': (rd_x, rd_y),
                    'route_s': s_val,
                    'readback_rd': (readback_rd_x, readback_rd_y),
                    'readback_xodr': (readback_xodr_x, readback_xodr_y),
                    'error': error,
                }
            
            print(f"    {route_name}: route s={s_val:.2f}, readback XODR=({readback_xodr_x:.3f}, {readback_xodr_y:.3f}), error={error:.3f}m")
        
        if best_result:
            results.append(best_result)
            print(f"  Best: {best_result['route']} with error={best_result['error']:.3f}m")
    
    return results


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing.
    
    This ensures we work on a clean copy without affecting the original scenario.
    
    Args:
        app: ModelDesk Application object
        exp: Experiment object
        source_scenario: Optional source scenario name (if None, uses currently active)
        new_scenario_name: Name for the new scenario copy (auto-generated if None)
        
    Returns:
        TrafficScenario object for the new copy
    """
    print("\n" + "="*80)
    print("Creating Scenario Copy")
    print("="*80)
    
    # Generate name if not provided
    if new_scenario_name is None:
        new_scenario_name = time.strftime("JunctionTest_%Y%m%d_%H%M%S")
    
    # Activate source scenario if specified
    if source_scenario:
        print(f"[1] Activating source scenario: {source_scenario}")
        try:
            exp.ActivateTrafficScenario(source_scenario)
            print(f"    [OK] Activated '{source_scenario}'")
        except Exception as e:
            print(f"    [WARNING] Could not activate scenario '{source_scenario}': {e}")
            print(f"    Continuing with currently active scenario...")
    else:
        print(f"[1] Using currently active scenario as source")
    
    # Create copy using SaveAs
    print(f"[2] Creating copy as: {new_scenario_name}")
    try:
        exp.TrafficScenario.SaveAs(new_scenario_name, True)
        print(f"    [OK] Created copy '{new_scenario_name}'")
    except Exception as e:
        print(f"    [WARNING] SaveAs failed: {e}, trying alternative method...")
        # Try alternative method via editor
        try:
            editor = exp.EditTrafficScenario()
            try:
                editor.SaveAs(new_scenario_name, True)
                print(f"    [OK] Created copy '{new_scenario_name}' (via editor)")
            finally:
                try:
                    editor.Close(False)
                except:
                    pass
        except Exception as e2:
            print(f"    [ERROR] Failed to create copy: {e2}")
            raise
    
    # Activate the new scenario
    print(f"[3] Activating new scenario: {new_scenario_name}")
    try:
        exp.ActivateTrafficScenario(new_scenario_name)
        print(f"    [OK] Activated '{new_scenario_name}'")
    except Exception as e:
        print(f"    [WARNING] Could not activate new scenario: {e}")
    
    # Rebind handles after a short delay
    pythoncom.PumpWaitingMessages()
    time.sleep(0.2)
    proj = app.ActiveProject
    exp = proj.ActiveExperiment
    ts = exp.TrafficScenario
    
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario after copy.")
    
    print(f"    [OK] Scenario copy ready for testing")
    print("="*80)
    
    return ts


def main():
    """Main test function."""
    print("="*80)
    print("Junction and Round-Trip Testing")
    print("="*80)
    print("\nThis script tests:")
    print("  1. Junction positions and their route s-coordinates")
    print("  2. Fine-grained transitions around the 100m offset")
    print("  3. Complete round-trip: XODR -> RD -> route s -> ModelDesk -> ControlDesk RD -> XODR")
    print("\nGoal: Achieve <1m error in round-trip transformation")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    pythoncom.CoInitialize()
    try:
        # Connect to ModelDesk
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("\n[ERROR] No active project. Please open a ModelDesk project first.")
            return
        exp = proj.ActiveExperiment
        if exp is None:
            print("\n[ERROR] No active experiment. Please activate an experiment.")
            return
        
        # CRITICAL: Create a copy of the current scenario for testing
        # This ensures we don't affect the original scenario
        print("\n[0] Creating scenario copy for testing...")
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="JunctionTest_RoundTrip")
        
        # Connect to ControlDesk
        print("\n[0] Connecting to ControlDesk...")
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        print("    [OK] Connected to ControlDesk")
        
        # Get or create fellow
        print("\n[0] Getting or creating fellow...")
        fellow_name = "TestFellow"
        try:
            fellow = ts.Fellows.Item(fellow_name)
            print(f"    [OK] Found existing fellow: {fellow_name}")
        except:
            fellow = ts.Fellows.Add()
            fellow.Name = fellow_name
            print(f"    [OK] Created new fellow: {fellow_name}")
        
        # Load coordinate transform
        print("\n[0] Loading coordinate transform...")
        transform_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'maps', 'dSPACE', 'Laguna_Seca_transform.json'
        )
        coordinate_transform = None
        if os.path.exists(transform_path):
            coordinate_transform = load_transform(transform_path)
            print(f"    [OK] Loaded coordinate transform from {transform_path}")
        else:
            print(f"    [WARNING] Transform file not found: {transform_path}")
        
        # Load road index
        print("\n[0] Loading road index...")
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        if not os.path.exists(rd_path):
            print(f"    [ERROR] RD file not found: {rd_path}")
            return
        
        road_index = build_rd_road_index(rd_path)
        print(f"    [OK] Loaded road index with {len(road_index['roads'])} roads")
        
        # Test 1: Junction positions
        junction_results = test_junction_positions(road_index, fellow, cd, ts)
        
        # Test 2: Fine-grained transitions
        transition_results = test_fine_grained_transitions(road_index, fellow, cd, ts)
        
        # Test 3: Round-trip with test coordinates
        test_coordinates = {
            'Test_1': {'scenic_xodr': (-101.919, -457.525, 0.0)},  # The Corkscrew1
            'Test_2': {'scenic_xodr': (163.54, 48.30, 0.0)},  # Pit Lane start (approximate XODR)
        }
        round_trip_results = test_round_trip_xodr(coordinate_transform, road_index, fellow, cd, ts, test_coordinates)
        
        # Summary
        print("\n" + "="*80)
        print("Summary")
        print("="*80)
        
        print("\nJunction Results:")
        for key, result in junction_results.items():
            print(f"  {key}: route s={result['route_s']:.2f}, error={result['error']:.2f}m")
        
        print("\nRound-Trip Results:")
        for result in round_trip_results:
            print(f"  {result['name']}: {result['route']}, error={result['error']:.3f}m")
        
        avg_error = sum(r['error'] for r in round_trip_results) / len(round_trip_results) if round_trip_results else 0
        print(f"\nAverage round-trip error: {avg_error:.3f}m")
        print(f"Target: <1.0m")
        
        print("\n" + "="*80)
        print("Testing Complete")
        print("="*80)
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
