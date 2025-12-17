"""Test script to verify complete route sequences and identify all roads in each route.

This script helps determine:
1. Complete road sequence for R1 (including loop through Corkscrew)
2. Complete road sequence for R2 (including all roads)
3. Where roads connect (end-to-end or with gaps)
4. Route s-coordinate mapping for shared roads (e.g., The Corkscrew1 on both routes)
"""

import sys
import os
import time
import pythoncom
from win32com.client import Dispatch

# Add Scenic to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..'))

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp


def test_route_sequence(route_name, test_s_values, road_index, fellow, cd, ts):
    """Test a route by placing fellows at different s values and reading back RD coordinates.
    
    This helps identify:
    - Which roads are in the route
    - Where roads connect
    - Road lengths and sequence
    
    IMPORTANT: This function REUSES the same fellow object passed in.
    It does NOT create new fellows - it updates the existing fellow's configuration.
    
    Args:
        route_name: Route name ('R1' or 'R2')
        test_s_values: List of s values to test
        road_index: Road index for projection
        fellow: ModelDesk fellow object (REUSED - same object for all tests)
        cd: ControlDesk connection (reused for all tests)
        ts: TrafficScenario object (reused for all tests)
    """
    print(f"\n{'='*80}")
    print(f"Testing Route {route_name} Sequence")
    print(f"{'='*80}")
    
    # CRITICAL: We're reusing the SAME fellow object passed in
    # We do NOT create new fellows - we update this fellow's configuration
    fellow_name = fellow.Name
    print(f"\n[1] Reusing fellow: {fellow_name} (same object, updating configuration)")
    
    # Since we're using a single fellow, it should be at index 0 in the ControlDesk array
    # This assumes the fellow is the first/only fellow in the scenario
    fellow_index = 0
    print(f"    [OK] Reading from ControlDesk array index {fellow_index} (single fellow)")
    
    # Get sequence from the SAME fellow (reusing, not creating new)
    sequences = fellow.Sequences
    if sequences.Count == 0:
        seq = sequences.Add()
    else:
        # Try 1-indexed first, fall back to 0-indexed
        try:
            seq = sequences.Item(1)
        except:
            seq = sequences.Item(0)
    
    # Update route on the SAME fellow (reusing, not creating new)
    route_sel = seq.Route
    route_sel.UseExternal = False  # Matches place_fellow implementation
    route_sel.Direction = 0  # Direct
    route_sel.Activate(route_name)
    print(f"    [OK] Updated route to {route_name} on existing fellow")
    
    # Use utility function to configure segments
    from scenic.simulators.dspace.utils import legacy as dutils
    
    # CRITICAL: Ensure we have 2 segments (segment 0 for initial pose, segment 1 for external control)
    # This matches the pattern used in place_fellow()
    segs = dutils.ensure_two_segments(seq)
    print(f"    [OK] Ensured 2 segments exist (seg0: initial pose, seg1: external control)")
    
    # Configure segment 1 with external control (both movements = "Extern")
    # This enables ControlDesk External Signals to control the fellow
    # Segment 0 will be configured for each test s value below
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        print(f"    [OK] Configured segment 1 with external control (both movements = 'Extern')")
    except Exception as e:
        print(f"    [WARNING] Could not configure segment 1 with external control: {e}")
    
    # Make segment 1 endless so external control can take effect
    try:
        dutils.make_endless_transition(segs)
        print(f"    [OK] Made segment 1 endless (for external control)")
    except Exception as e:
        print(f"    [WARNING] Could not make segment 1 endless: {e}")
    
    results = []
    
    # Ensure route is set and saved before testing
    route_sel.Activate(route_name)
    ts.Save()
    print(f"    [OK] Route {route_name} activated and saved")
    
    # Test each s value - updating the SAME fellow each time
    for s_val in test_s_values:
        print(f"\n[2] Testing s={s_val:.1f} on {route_name} (updating same fellow)...")
        
        # Update s and t on the SAME fellow (reusing, not creating new)
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=0.0)
        
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
        
        # Step simulation (reduced from 20 to 10 for faster testing)
        for i in range(10):
            cd.advance_simulation_step()
            time.sleep(0.1)
        time.sleep(1.0)
        
        # Read back position from ControlDesk array index 0 (single fellow)
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        
        # Use index 0 for single fellow scenario
        if fellow_index < len(x_arr) and fellow_index < len(y_arr):
            rd_x = float(x_arr[fellow_index])
            rd_y = float(y_arr[fellow_index])
        else:
            print(f"    [ERROR] Fellow index {fellow_index} out of range (array size: {len(x_arr)})")
            print(f"    [ERROR] This may indicate multiple fellows exist. Expected only 1.")
            rd_x = float(x_arr[0])  # Fallback
            rd_y = float(y_arr[0])
        
        # Identify which road this projects onto
        from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
        from scenic.simulators.dspace.geometry.route_projection import find_road_name_by_id
        
        road_id = find_road_id_for_position(road_index, rd_x, rd_y)
        road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
        
        results.append({
            's_route': s_val,
            'rd': (rd_x, rd_y),
            'road_name': road_name,
            'road_id': road_id
        })
        
        print(f"    s={s_val:.1f} -> RD ({rd_x:.2f}, {rd_y:.2f}) on road '{road_name}' (ID: {road_id})")
    
    # Analyze results
    print(f"\n[3] Route {route_name} Analysis:")
    print(f"    {'s (route)':>10} | {'RD X':>10} | {'RD Y':>10} | {'Road Name':<25} | {'Road ID':>8}")
    print(f"    {'-'*10} | {'-'*10} | {'-'*10} | {'-'*25} | {'-'*8}")
    
    road_transitions = []
    prev_road = None
    
    for r in results:
        print(f"    {r['s_route']:>10.1f} | {r['rd'][0]:>10.2f} | {r['rd'][1]:>10.2f} | {r['road_name']:<25} | {r['road_id']:>8}")
        
        if prev_road is not None and r['road_name'] != prev_road:
            road_transitions.append({
                's': r['s_route'],
                'from': prev_road,
                'to': r['road_name']
            })
        prev_road = r['road_name']
    
    if road_transitions:
        print(f"\n    Road Transitions:")
        for trans in road_transitions:
            print(f"      At s={trans['s']:.1f}: {trans['from']} -> {trans['to']}")
    else:
        print(f"\n    No road transitions detected (all coordinates on same road)")
    
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
        new_scenario_name = time.strftime("RouteTest_%Y%m%d_%H%M%S")
    
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
    print("Route Sequence Testing")
    print("="*80)
    print("\nThis script tests route sequences by placing fellows at different")
    print("s values and identifying which roads they map to.")
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
        
        # Create a copy of the current scenario for testing
        # This ensures we don't affect the original scenario
        print("\n[0] Creating scenario copy for testing...")
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="RouteTest_Sequence")
        
        print("\n[0] Connecting to ControlDesk...")
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        print("    [OK] Connected to ControlDesk")
        
        # CRITICAL: Clear all existing fellows from the copied scenario
        # This ensures we start with a clean slate and no leftover fellows
        print("\n[0] Clearing existing fellows from scenario copy...")
        try:
            from scenic.simulators.dspace.utils import legacy as dutils
            dutils.clear_collection(ts.Fellows)
            print(f"    [OK] Cleared existing fellows")
        except Exception as e:
            print(f"    [WARNING] Could not clear fellows: {e}")
        
        # CRITICAL: Create a single new fellow with correct configuration
        # After clearing, there should be no fellows, so we always create a new one
        # This follows the best practice: exactly 1 fellow, properly configured
        # See README.md "CRITICAL: How to Correctly Create and Configure a Fellow" section
        print("\n[0] Creating new fellow with correct configuration...")
        fellow_name = "TestFellow"
        fellow = ts.Fellows.Add()
        fellow.Name = fellow_name
        print(f"    [OK] Created new fellow: {fellow_name}")
        
        # Configure fellow with 2 segments and external control
        # Get or create sequence
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            try:
                seq = sequences.Item(1)
            except:
                seq = sequences.Item(0)
        
        # Ensure 2 segments exist (segment 0 for initial pose, segment 1 for external control)
        segs = dutils.ensure_two_segments(seq)
        print(f"    [OK] Ensured 2 segments exist")
        
        # Configure segment 1 with external control (both movements = "Extern")
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            print(f"    [OK] Configured segment 1 with external control (both movements = 'Extern')")
        except Exception as e:
            print(f"    [WARNING] Could not configure segment 1 with external control: {e}")
        
        # Make segment 1 endless so external control can take effect
        try:
            dutils.make_endless_transition(segs)
            print(f"    [OK] Made segment 1 endless (for external control)")
        except Exception as e:
            print(f"    [WARNING] Could not make segment 1 endless: {e}")
        
        # Verify we only have 1 fellow
        num_fellows = ts.Fellows.Count
        if num_fellows != 1:
            print(f"    [WARNING] Found {num_fellows} fellows in scenario. Expected exactly 1.")
        else:
            print(f"    [OK] Scenario has exactly 1 fellow - correct")
        
        # Load road index
        print("\n[0] Loading road index...")
        from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
        
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        if not os.path.exists(rd_path):
            print(f"    [ERROR] RD file not found: {rd_path}")
            return
        
        road_index = build_rd_road_index(rd_path)
        print(f"    [OK] Loaded road index with {len(road_index['roads'])} roads")
        
        # Test strategic s values based on known road sequences:
        # R1 (Pit): Pit Lane1_2 (883.5m) -> The Corkscrew1
        # R2 (Lap): Andretti Hairpin1_3 (988.0m) -> The Corkscrew1
        
        # Get road lengths from index
        pit_lane_length = road_index['roads'].get('Pit Lane1_2', {}).get('length', 883.5)
        andretti_length = road_index['roads'].get('Andretti Hairpin1_3', {}).get('length', 988.0)
        corkscrew_length = road_index['roads'].get('The Corkscrew1', {}).get('length', 2484.6)
        
        # R1 test points: s=0 (Pit Lane start), s=pit_lane_length (transition), s=pit_lane_length+100 (Corkscrew)
        r1_test_s = [0.0, pit_lane_length * 0.5, pit_lane_length, pit_lane_length + 100.0, pit_lane_length + 500.0]
        
        # R2 test points: s=0 (Andretti start), s=andretti_length (transition), s=andretti_length+100 (Corkscrew)
        r2_test_s = [0.0, andretti_length * 0.5, andretti_length, andretti_length + 100.0, andretti_length + 500.0]
        
        print(f"\n[0] Road lengths:")
        print(f"    Pit Lane1_2: {pit_lane_length:.1f}m")
        print(f"    Andretti Hairpin1_3: {andretti_length:.1f}m")
        print(f"    The Corkscrew1: {corkscrew_length:.1f}m")
        print(f"\n[0] Test points:")
        print(f"    R1 (Pit): {r1_test_s}")
        print(f"    R2 (Lap): {r2_test_s}")
        
        # Test R1 using the SAME fellow (reusing, not creating new)
        # The fellow object is passed in and its configuration is updated
        r1_results = test_route_sequence("R1", r1_test_s, road_index, fellow, cd, ts)
        
        # Test R2 using the SAME fellow (just update route, reuse same object)
        # The fellow object is the same - we just change its route configuration
        r2_results = test_route_sequence("R2", r2_test_s, road_index, fellow, cd, ts)
        
        # Compare routes
        print(f"\n{'='*80}")
        print("Route Comparison")
        print(f"{'='*80}")
        
        print("\nExpected route sequences:")
        print(f"  R1 (Pit): Pit Lane1_2 (0-{pit_lane_length:.1f}m) -> The Corkscrew1 ({pit_lane_length:.1f}m+)")
        print(f"  R2 (Lap): Andretti Hairpin1_3 (0-{andretti_length:.1f}m) -> The Corkscrew1 ({andretti_length:.1f}m+)")
        
        print("\nActual results:")
        for route_name, results, expected_transition in [("R1", r1_results, pit_lane_length), ("R2", r2_results, andretti_length)]:
            print(f"\n  {route_name}:")
            for r in results:
                marker = " <-- Expected transition" if abs(r['s_route'] - expected_transition) < 50 else ""
                print(f"    s={r['s_route']:>7.1f} -> {r['road_name']:<25} at RD ({r['rd'][0]:>8.2f}, {r['rd'][1]:>8.2f}){marker}")
            
            # Check for road transitions
            transitions = []
            for i in range(len(results) - 1):
                if results[i]['road_name'] != results[i+1]['road_name']:
                    transitions.append({
                        's': results[i+1]['s_route'],
                        'from': results[i]['road_name'],
                        'to': results[i+1]['road_name']
                    })
            
            if transitions:
                print(f"    Road transitions found:")
                for trans in transitions:
                    print(f"      At s={trans['s']:.1f}: {trans['from']} -> {trans['to']}")
            else:
                print(f"    No road transitions detected in tested range")
        
        print(f"\n{'='*80}")
        print("Testing Complete")
        print(f"{'='*80}")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
