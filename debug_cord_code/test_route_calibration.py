"""Test script to create route s-coordinate calibration table.

This script tests:
1. Precise transition points (1m increments around known transitions)
2. Route s=0 positions and offsets
3. Multiple positions along The Corkscrew1 to understand systematic errors
4. Creates calibration table: route s -> RD coordinates -> Road name -> Road-relative s

Goal: Understand route s-coordinate mapping to refine projection and achieve <1m round-trip error.
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
        new_scenario_name = time.strftime("CalibrationTest_%Y%m%d_%H%M%S")
    
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


def test_precise_transitions(road_index, fellow, cd, ts):
    """Test transition points in 1m increments."""
    print("\n" + "="*80)
    print("Testing Precise Transition Points")
    print("="*80)
    
    # R1: Test around 910.0 (found transition point)
    r1_test_range = list(range(905, 921))  # 905-920 in 1m increments
    
    # R2: Test around 1015.0 (found transition point)
    r2_test_range = list(range(1010, 1021))  # 1010-1020 in 1m increments
    
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
            elif s_val in [test_range[0], test_range[-1]] or s_val % 5 == 0:
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
                })
        
        if transitions:
            print(f"\n  Exact transitions found:")
            for trans in transitions:
                print(f"    At s={trans['s']:.1f}: {trans['from']} -> {trans['to']} at RD ({trans['rd'][0]:.2f}, {trans['rd'][1]:.2f})")
        
        results[route_name] = route_results
    
    return results


def test_route_origins(road_index, fellow, cd, ts):
    """Test route s=0 positions and measure offsets."""
    print("\n" + "="*80)
    print("Testing Route Origins (s=0)")
    print("="*80)
    
    results = {}
    
    # Known road start positions (from previous testing)
    expected_starts = {
        'R1': {
            'road': 'Pit Lane1_2',
            'expected_rd': (163.54, 48.30),
        },
        'R2': {
            'road': 'Andretti Hairpin1_3',
            'expected_rd': (172.52, 53.55),
        },
    }
    
    for route_name in ['R1', 'R2']:
        print(f"\nTesting {route_name} at s=0...")
        
        result = test_route_s_value(route_name, 0.0, road_index, fellow, cd, ts)
        
        expected = expected_starts[route_name]
        error = math.sqrt(
            (result['rd'][0] - expected['expected_rd'][0])**2 + 
            (result['rd'][1] - expected['expected_rd'][1])**2
        )
        
        print(f"  Route s=0 -> RD ({result['rd'][0]:.2f}, {result['rd'][1]:.2f})")
        print(f"  Expected: RD ({expected['expected_rd'][0]:.2f}, {expected['expected_rd'][1]:.2f})")
        print(f"  Error: {error:.2f}m")
        print(f"  On road: {result['road_name']}")
        if result['road_s'] is not None:
            print(f"  Road-relative s: {result['road_s']:.2f}m")
        
        results[route_name] = {
            'route_s': 0.0,
            'actual_rd': result['rd'],
            'expected_rd': expected['expected_rd'],
            'error': error,
            'road_name': result['road_name'],
            'road_s': result['road_s'],
        }
    
    return results


def test_corkscrew_positions(road_index, fellow, cd, ts):
    """Test multiple positions along The Corkscrew1 to understand systematic errors."""
    print("\n" + "="*80)
    print("Testing The Corkscrew1 Positions")
    print("="*80)
    
    # Test points along The Corkscrew1 on both routes
    # R1: The Corkscrew1 starts around s=910 (after transition)
    # R2: The Corkscrew1 starts around s=1015 (after transition)
    
    # Test at various positions along The Corkscrew1
    r1_corkscrew_tests = [910, 1000, 1200, 1400, 1600, 1800, 2000]
    r2_corkscrew_tests = [1015, 1100, 1300, 1500, 1700, 1900, 2100]
    
    results = {}
    
    for route_name, test_s_values in [('R1', r1_corkscrew_tests), ('R2', r2_corkscrew_tests)]:
        print(f"\nTesting {route_name} positions along The Corkscrew1...")
        
        route_results = []
        
        for s_val in test_s_values:
            result = test_route_s_value(route_name, s_val, road_index, fellow, cd, ts)
            
            if result['road_name'] == 'The Corkscrew1' and result['road_s'] is not None:
                # Calculate offset: route s - (transition s + road s)
                if route_name == 'R1':
                    transition_s = 910.0
                else:
                    transition_s = 1015.0
                
                expected_road_s = s_val - transition_s
                offset = result['road_s'] - expected_road_s
                
                print(f"  s={s_val:>6.1f} -> RD ({result['rd'][0]:>8.2f}, {result['rd'][1]:>8.2f}), road s={result['road_s']:>7.2f}, offset={offset:>6.2f}m")
                
                route_results.append({
                    'route_s': s_val,
                    'rd': result['rd'],
                    'road_s': result['road_s'],
                    'expected_road_s': expected_road_s,
                    'offset': offset,
                })
            else:
                print(f"  s={s_val:>6.1f} -> Not on The Corkscrew1 (on {result['road_name']})")
        
        results[route_name] = route_results
    
    return results


def create_calibration_table(transition_results, origin_results, corkscrew_results):
    """Create a calibration table mapping route s to physical positions."""
    print("\n" + "="*80)
    print("Route s-Coordinate Calibration Table")
    print("="*80)
    
    print("\nRoute Origins:")
    print(f"  {'Route':<6} | {'Route s':>8} | {'RD X':>10} | {'RD Y':>10} | {'Road':<25} | {'Road s':>8} | {'Error':>8}")
    print(f"  {'-'*6} | {'-'*8} | {'-'*10} | {'-'*10} | {'-'*25} | {'-'*8} | {'-'*8}")
    for route_name in ['R1', 'R2']:
        r = origin_results[route_name]
        road_s_str = f"{r['road_s']:.2f}" if r['road_s'] is not None else "N/A"
        print(f"  {route_name:<6} | {r['route_s']:>8.1f} | {r['actual_rd'][0]:>10.2f} | {r['actual_rd'][1]:>10.2f} | {r['road_name']:<25} | {road_s_str:>8} | {r['error']:>8.2f}m")
    
    print("\nTransition Points:")
    for route_name in ['R1', 'R2']:
        print(f"\n  {route_name}:")
        transitions = []
        for i in range(1, len(transition_results[route_name])):
            if transition_results[route_name][i]['road_name'] != transition_results[route_name][i-1]['road_name']:
                transitions.append(transition_results[route_name][i])
        
        if transitions:
            for trans in transitions:
                print(f"    s={trans['route_s']:.1f}: {transition_results[route_name][transitions.index(trans)-1]['road_name']} -> {trans['road_name']} at RD ({trans['rd'][0]:.2f}, {trans['rd'][1]:.2f})")
    
    print("\nThe Corkscrew1 Calibration (Route s -> Road s mapping):")
    for route_name in ['R1', 'R2']:
        print(f"\n  {route_name}:")
        print(f"    {'Route s':>8} | {'RD X':>10} | {'RD Y':>10} | {'Road s':>8} | {'Expected':>10} | {'Offset':>8}")
        print(f"    {'-'*8} | {'-'*10} | {'-'*10} | {'-'*8} | {'-'*10} | {'-'*8}")
        for r in corkscrew_results[route_name]:
            print(f"    {r['route_s']:>8.1f} | {r['rd'][0]:>10.2f} | {r['rd'][1]:>10.2f} | {r['road_s']:>8.2f} | {r['expected_road_s']:>10.2f} | {r['offset']:>8.2f}m")
        
        if corkscrew_results[route_name]:
            avg_offset = sum(r['offset'] for r in corkscrew_results[route_name]) / len(corkscrew_results[route_name])
            print(f"    Average offset: {avg_offset:.2f}m")


def main():
    """Main test function."""
    print("="*80)
    print("Route s-Coordinate Calibration Testing")
    print("="*80)
    print("\nThis script creates a calibration table mapping route s-coordinates to:")
    print("  - RD coordinates")
    print("  - Road names")
    print("  - Road-relative s coordinates")
    print("\nGoal: Understand route s-coordinate mapping to refine projection")
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
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="CalibrationTest")
        
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
        
        # Load road index
        print("\n[0] Loading road index...")
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        if not os.path.exists(rd_path):
            print(f"    [ERROR] RD file not found: {rd_path}")
            return
        
        road_index = build_rd_road_index(rd_path)
        print(f"    [OK] Loaded road index with {len(road_index['roads'])} roads")
        
        # Test 1: Precise transitions
        transition_results = test_precise_transitions(road_index, fellow, cd, ts)
        
        # Test 2: Route origins
        origin_results = test_route_origins(road_index, fellow, cd, ts)
        
        # Test 3: The Corkscrew1 positions
        corkscrew_results = test_corkscrew_positions(road_index, fellow, cd, ts)
        
        # Create calibration table
        create_calibration_table(transition_results, origin_results, corkscrew_results)
        
        print("\n" + "="*80)
        print("Calibration Testing Complete")
        print("="*80)
        print("\nUse this calibration data to:")
        print("  1. Refine route-specific projection algorithm")
        print("  2. Account for connection segments between roads")
        print("  3. Calibrate route s -> road s mapping")
        print("  4. Reduce round-trip errors toward <1m target")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
