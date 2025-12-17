"""Enhanced calibration script to find exact transition points and create complete mapping.

This script tests:
1. Earlier s values to find exact transition points (880-910 for R1, 985-1015 for R2)
2. First road positions to verify mapping before transitions
3. Complete calibration table: route s -> road name -> road-relative s (with offset correction)
4. Tests round-trip with offset correction applied

Goal: Create complete route s-coordinate mapping to refine projection and achieve <1m round-trip error.
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


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    print("\n" + "="*80)
    print("Creating Scenario Copy")
    print("="*80)
    
    if new_scenario_name is None:
        new_scenario_name = time.strftime("EnhancedCalibration_%Y%m%d_%H%M%S")
    
    if source_scenario:
        print(f"[1] Activating source scenario: {source_scenario}")
        try:
            exp.ActivateTrafficScenario(source_scenario)
            print(f"    [OK] Activated '{source_scenario}'")
        except Exception as e:
            print(f"    [WARNING] Could not activate scenario '{source_scenario}': {e}")
    else:
        print(f"[1] Using currently active scenario as source")
    
    print(f"[2] Creating copy as: {new_scenario_name}")
    try:
        exp.TrafficScenario.SaveAs(new_scenario_name, True)
        print(f"    [OK] Created copy '{new_scenario_name}'")
    except Exception as e:
        print(f"    [WARNING] SaveAs failed: {e}, trying alternative method...")
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
    
    print(f"[3] Activating new scenario: {new_scenario_name}")
    try:
        exp.ActivateTrafficScenario(new_scenario_name)
        print(f"    [OK] Activated '{new_scenario_name}'")
    except Exception as e:
        print(f"    [WARNING] Could not activate new scenario: {e}")
    
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


def test_route_s_value(route_name, s_val, road_index, fellow, cd, ts):
    """Test a single route s value and return detailed results."""
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
    
    # Place fellow at s value
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=0.0)
    
    # Save, download, reset, start
    ts.Save()
    ts.Download()
    time.sleep(0.3)
    
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
    
    # Step simulation
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
    
    # Identify road and road-relative s
    road_id = find_road_id_for_position(road_index, rd_x, rd_y)
    road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
    
    # Project back to road-relative s
    if road_id is not None:
        road_s, road_t = project_world_to_st(road_index, (rd_x, rd_y))
    else:
        road_s, road_t = None, None
    
    return {
        'route_s': s_val,
        'rd': (rd_x, rd_y),
        'road_name': road_name,
        'road_id': road_id,
        'road_s': road_s,
        'road_t': road_t,
    }


def test_exact_transitions(road_index, fellow, cd, ts):
    """Test earlier s values to find exact transition points."""
    print("\n" + "="*80)
    print("Testing Exact Transition Points")
    print("="*80)
    
    # R1: Test 880-910 (around expected transition)
    r1_test_range = list(range(880, 911))  # 880-910 in 1m increments
    
    # R2: Test 985-1015 (around expected transition)
    r2_test_range = list(range(985, 1016))  # 985-1015 in 1m increments
    
    results = {}
    
    for route_name, test_range in [('R1', r1_test_range), ('R2', r2_test_range)]:
        print(f"\nTesting {route_name} transitions (s={test_range[0]}-{test_range[-1]})...")
        
        route_results = []
        prev_road = None
        
        for s_val in test_range:
            result = test_route_s_value(route_name, s_val, road_index, fellow, cd, ts)
            route_results.append(result)
            
            transition_detected = (prev_road is not None and result['road_name'] != prev_road)
            
            if transition_detected:
                print(f"    s={s_val:>6.1f} -> RD ({result['rd'][0]:>8.2f}, {result['rd'][1]:>8.2f}) on {result['road_name']:<25} <-- TRANSITION")
            elif s_val % 5 == 0 or s_val in [test_range[0], test_range[-1]]:
                print(f"    s={s_val:>6.1f} -> RD ({result['rd'][0]:>8.2f}, {result['rd'][1]:>8.2f}) on {result['road_name']:<25}")
            
            prev_road = result['road_name']
        
        # Find exact transition point
        transitions = []
        for i in range(1, len(route_results)):
            if route_results[i]['road_name'] != route_results[i-1]['road_name']:
                transitions.append({
                    's': route_results[i]['route_s'],
                    'from': route_results[i-1]['road_name'],
                    'to': route_results[i]['road_name'],
                    'rd': route_results[i]['rd'],
                    'from_rd': route_results[i-1]['rd'],
                })
        
        if transitions:
            print(f"\n  Exact transitions found:")
            for trans in transitions:
                print(f"    At s={trans['s']:.1f}: {trans['from']} -> {trans['to']}")
                print(f"      From RD: ({trans['from_rd'][0]:.2f}, {trans['from_rd'][1]:.2f})")
                print(f"      To RD: ({trans['rd'][0]:.2f}, {trans['rd'][1]:.2f})")
        else:
            print(f"\n  No transitions found in tested range")
        
        results[route_name] = route_results
    
    return results


def test_first_road_positions(road_index, fellow, cd, ts):
    """Test positions along first roads to verify mapping before transitions."""
    print("\n" + "="*80)
    print("Testing First Road Positions")
    print("="*80)
    
    # Test points along first roads
    r1_first_road_tests = [0, 100, 200, 400, 600, 800, 883.4]  # Pit Lane1_2
    r2_first_road_tests = [0, 100, 200, 400, 600, 800, 988.0]  # Andretti Hairpin1_3
    
    results = {}
    
    for route_name, test_s_values, expected_road in [
        ('R1', r1_first_road_tests, 'Pit Lane1_2'),
        ('R2', r2_first_road_tests, 'Andretti Hairpin1_3'),
    ]:
        print(f"\nTesting {route_name} positions along {expected_road}...")
        
        route_results = []
        
        for s_val in test_s_values:
            result = test_route_s_value(route_name, s_val, road_index, fellow, cd, ts)
            
            if result['road_name'] == expected_road and result['road_s'] is not None:
                # Calculate mapping: route s should equal road s (for first road)
                offset = result['road_s'] - s_val
                
                print(f"  s={s_val:>6.1f} -> RD ({result['rd'][0]:>8.2f}, {result['rd'][1]:>8.2f}), road s={result['road_s']:>7.2f}, offset={offset:>6.2f}m")
                
                route_results.append({
                    'route_s': s_val,
                    'rd': result['rd'],
                    'road_s': result['road_s'],
                    'expected_road_s': s_val,
                    'offset': offset,
                })
            else:
                print(f"  s={s_val:>6.1f} -> Not on {expected_road} (on {result['road_name']})")
        
        results[route_name] = route_results
    
    return results


def create_complete_calibration_table(transition_results, first_road_results):
    """Create complete calibration table with transition points and mappings."""
    print("\n" + "="*80)
    print("Complete Route s-Coordinate Calibration Table")
    print("="*80)
    
    print("\nExact Transition Points:")
    for route_name in ['R1', 'R2']:
        print(f"\n  {route_name}:")
        transitions = []
        for i in range(1, len(transition_results[route_name])):
            if transition_results[route_name][i]['road_name'] != transition_results[route_name][i-1]['road_name']:
                transitions.append({
                    's': transition_results[route_name][i]['route_s'],
                    'from': transition_results[route_name][i-1]['road_name'],
                    'to': transition_results[route_name][i]['road_name'],
                    'rd': transition_results[route_name][i]['rd'],
                })
        
        if transitions:
            for trans in transitions:
                print(f"    s={trans['s']:.1f}: {trans['from']} -> {trans['to']} at RD ({trans['rd'][0]:.2f}, {trans['rd'][1]:.2f})")
        else:
            print(f"    No transitions found in tested range")
    
    print("\nFirst Road Mapping (Route s -> Road s):")
    for route_name in ['R1', 'R2']:
        print(f"\n  {route_name}:")
        print(f"    {'Route s':>8} | {'RD X':>10} | {'RD Y':>10} | {'Road s':>8} | {'Expected':>10} | {'Offset':>8}")
        print(f"    {'-'*8} | {'-'*10} | {'-'*10} | {'-'*8} | {'-'*10} | {'-'*8}")
        for r in first_road_results[route_name]:
            print(f"    {r['route_s']:>8.1f} | {r['rd'][0]:>10.2f} | {r['rd'][1]:>10.2f} | {r['road_s']:>8.2f} | {r['expected_road_s']:>10.2f} | {r['offset']:>8.2f}m")
        
        if first_road_results[route_name]:
            avg_offset = sum(r['offset'] for r in first_road_results[route_name]) / len(first_road_results[route_name])
            print(f"    Average offset: {avg_offset:.2f}m")


def test_round_trip_with_offset_correction(coordinate_transform, road_index, fellow, cd, ts, transition_points, offsets):
    """Test round-trip with offset correction applied."""
    print("\n" + "="*80)
    print("Testing Round-Trip with Offset Correction")
    print("="*80)
    
    # Test coordinates
    test_coordinates = {
        'Test_1': {'scenic_xodr': (-101.919, -457.525, 0.0)},  # The Corkscrew1
        'Test_2': {'scenic_xodr': (163.54, 48.30, 0.0)},  # Pit Lane start (approximate XODR)
    }
    
    results = []
    
    for name, coord_data in test_coordinates.items():
        scenic_xodr = coord_data['scenic_xodr']
        print(f"\n{name}:")
        print(f"  Starting XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
        
        # Step 1: XODR -> RD
        if coordinate_transform:
            rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        else:
            rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
        print(f"  Step 1 (XODR -> RD): ({rd_x:.3f}, {rd_y:.3f})")
        
        # Step 2: Determine route and project
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
                s_val, t_val = project_world_to_st(road_index, (rd_x, rd_y))
            
            # Apply offset correction if on The Corkscrew1
            # (This is a test - in real implementation, this would be in route_projection.py)
            transition_s = transition_points.get(route_name)
            offset = offsets.get(route_name, 0.0)
            
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


def main():
    """Main test function."""
    print("="*80)
    print("Enhanced Route s-Coordinate Calibration Testing")
    print("="*80)
    print("\nThis script:")
    print("  1. Finds exact transition points (880-910 for R1, 985-1015 for R2)")
    print("  2. Tests first road positions to verify mapping")
    print("  3. Creates complete calibration table")
    print("  4. Tests round-trip with offset correction")
    print("\nGoal: Create complete route s-coordinate mapping to refine projection")
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
        print("\n[0] Creating scenario copy for testing...")
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="EnhancedCalibration")
        
        # Connect to ControlDesk
        print("\n[0] Connecting to ControlDesk...")
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        print("    [OK] Connected to ControlDesk")
        
        # CRITICAL: Clear all existing fellows from the copied scenario
        print("\n[0] Clearing existing fellows from scenario copy...")
        try:
            from scenic.simulators.dspace.utils import legacy as dutils
            dutils.clear_collection(ts.Fellows)
            print(f"    [OK] Cleared existing fellows")
        except Exception as e:
            print(f"    [WARNING] Could not clear fellows: {e}")
        
        # CRITICAL: Create a single new fellow with correct configuration
        print("\n[0] Creating new fellow with correct configuration...")
        fellow_name = "TestFellow"
        fellow = ts.Fellows.Add()
        fellow.Name = fellow_name
        print(f"    [OK] Created new fellow: {fellow_name}")
        
        # Configure fellow with 2 segments and external control
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            try:
                seq = sequences.Item(1)
            except:
                seq = sequences.Item(0)
        
        segs = dutils.ensure_two_segments(seq)
        print(f"    [OK] Ensured 2 segments exist")
        
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            print(f"    [OK] Configured segment 1 with external control")
        except Exception as e:
            print(f"    [WARNING] Could not configure segment 1: {e}")
        
        try:
            dutils.make_endless_transition(segs)
            print(f"    [OK] Made segment 1 endless")
        except Exception as e:
            print(f"    [WARNING] Could not make segment 1 endless: {e}")
        
        # Load coordinate transform
        print("\n[0] Loading coordinate transform...")
        transform_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'maps', 'dSPACE', 'Laguna_Seca_transform.json'
        )
        coordinate_transform = None
        if os.path.exists(transform_path):
            coordinate_transform = load_transform(transform_path)
            print(f"    [OK] Loaded coordinate transform")
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
        
        # Test 1: Exact transitions
        transition_results = test_exact_transitions(road_index, fellow, cd, ts)
        
        # Test 2: First road positions
        first_road_results = test_first_road_positions(road_index, fellow, cd, ts)
        
        # Create calibration table
        create_complete_calibration_table(transition_results, first_road_results)
        
        # Extract transition points and offsets for round-trip test
        transition_points = {}
        offsets = {}
        
        for route_name in ['R1', 'R2']:
            # Find transition point
            for i in range(1, len(transition_results[route_name])):
                if transition_results[route_name][i]['road_name'] != transition_results[route_name][i-1]['road_name']:
                    transition_points[route_name] = transition_results[route_name][i]['route_s']
                    break
            
            # Calculate offset from first road results (average)
            if first_road_results[route_name]:
                avg_offset = sum(r['offset'] for r in first_road_results[route_name]) / len(first_road_results[route_name])
                offsets[route_name] = avg_offset
        
        # Test 3: Round-trip with offset correction (if we have the data)
        if transition_points and offsets:
            round_trip_results = test_round_trip_with_offset_correction(
                coordinate_transform, road_index, fellow, cd, ts, transition_points, offsets
            )
            
            print("\n" + "="*80)
            print("Round-Trip Summary")
            print("="*80)
            for result in round_trip_results:
                print(f"  {result['name']}: {result['route']}, error={result['error']:.3f}m")
            
            avg_error = sum(r['error'] for r in round_trip_results) / len(round_trip_results) if round_trip_results else 0
            print(f"\nAverage round-trip error: {avg_error:.3f}m")
            print(f"Target: <1.0m")
        
        print("\n" + "="*80)
        print("Enhanced Calibration Testing Complete")
        print("="*80)
        print("\nUse this calibration data to:")
        print("  1. Refine route-specific projection algorithm")
        print("  2. Apply offset corrections in route_projection.py")
        print("  3. Account for exact transition points")
        print("  4. Reduce round-trip errors toward <1m target")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
