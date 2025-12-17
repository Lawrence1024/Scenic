"""Test offset precision and consistency along The Corkscrew1.

This script tests if the 8.9m (R2) and 9.6m (R1) offsets are truly constant
along The Corkscrew1 by testing multiple positions.

Tests:
1. Places fellows at specific road_s positions on The Corkscrew1 (0, 100, 200, 300, 500, 1000, 1500)
2. Converts to route_s using current formula: route_s = road_s + transition_point + offset
3. Reads back actual position from ControlDesk
4. Measures actual offset at each position
5. Determines if offset is constant or varies

Goal: Verify offset precision to refine route-specific projection and achieve <1m round-trip error.
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
        new_scenario_name = time.strftime("OffsetPrecisionTest_%Y%m%d_%H%M%S")
    
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


def test_offset_at_road_s(route_name, road_s, transition_point, current_offset, road_index, fellow, cd, ts):
    """Test offset at a specific road_s position on The Corkscrew1.
    
    Args:
        route_name: Route name ('R1' or 'R2')
        road_s: Road-relative s position on The Corkscrew1
        transition_point: Known transition point for this route
        current_offset: Current offset value being used (9.6 for R1, 8.9 for R2)
        road_index: Road index for projection
        fellow: Fellow object to use
        cd: ControlDesk connection
        ts: TrafficScenario object
        
    Returns:
        Dictionary with test results including actual offset measurement
    """
    # Calculate route_s using current formula
    route_s = road_s + transition_point + current_offset
    
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
    
    # Place fellow at calculated route_s
    dutils.configure_seg0_absolute_pose(segs, s=float(route_s), t=0.0)
    
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
    for i in range(20):
        cd.advance_simulation_step()
        time.sleep(0.05)
    time.sleep(0.5)
    
    # Read back position
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    x_arr = cd.get_var(f"{base_path}/x")
    y_arr = cd.get_var(f"{base_path}/y")
    
    rd_x = float(x_arr[0])
    rd_y = float(y_arr[0])
    
    # Project readback position to road-relative s
    road_id = find_road_id_for_position(road_index, rd_x, rd_y)
    road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
    
    if road_id is not None and road_name == 'The Corkscrew1':
        actual_road_s, actual_road_t = project_world_to_st(road_index, (rd_x, rd_y))
        
        # Calculate actual offset
        # Expected: actual_road_s should equal road_s (the target position)
        # Actual offset = actual_road_s - road_s
        actual_offset = actual_road_s - road_s
        
        # Also calculate what the offset should be to achieve perfect alignment
        # If we want actual_road_s = road_s, then:
        # route_s = road_s + transition_point + correct_offset
        # But we used: route_s = road_s + transition_point + current_offset
        # So: correct_offset = current_offset - actual_offset
        correct_offset = current_offset - actual_offset
        
        return {
            'route_name': route_name,
            'target_road_s': road_s,
            'route_s_used': route_s,
            'readback_rd': (rd_x, rd_y),
            'actual_road_s': actual_road_s,
            'actual_offset': actual_offset,
            'current_offset': current_offset,
            'correct_offset': correct_offset,
            'error': abs(actual_offset),
        }
    else:
        return {
            'route_name': route_name,
            'target_road_s': road_s,
            'route_s_used': route_s,
            'readback_rd': (rd_x, rd_y),
            'actual_road_s': None,
            'actual_offset': None,
            'current_offset': current_offset,
            'correct_offset': None,
            'error': None,
            'warning': f"Not on The Corkscrew1 (on {road_name})"
        }


def test_offset_precision(road_index, fellow, cd, ts):
    """Test offset precision at multiple positions along The Corkscrew1."""
    print("\n" + "="*80)
    print("Testing Offset Precision Along The Corkscrew1")
    print("="*80)
    
    # Test positions (road-relative s on The Corkscrew1)
    test_road_s_positions = [0.0, 100.0, 200.0, 300.0, 500.0, 1000.0, 1500.0]
    
    # Known transition points and offsets
    route_configs = {
        'R1': {
            'transition_point': 902.0,
            'current_offset': 9.6,
        },
        'R2': {
            'transition_point': 1006.0,
            'current_offset': 8.9,
        },
    }
    
    all_results = {}
    
    for route_name, config in route_configs.items():
        print(f"\n{'='*80}")
        print(f"Testing {route_name} (transition={config['transition_point']:.1f}, offset={config['current_offset']:.1f}m)")
        print(f"{'='*80}")
        
        route_results = []
        
        for road_s in test_road_s_positions:
            print(f"\n  Testing road_s={road_s:.1f}m...")
            
            result = test_offset_at_road_s(
                route_name,
                road_s,
                config['transition_point'],
                config['current_offset'],
                road_index,
                fellow,
                cd,
                ts
            )
            
            route_results.append(result)
            
            if result.get('warning'):
                print(f"    [WARNING] {result['warning']}")
            elif result['actual_road_s'] is not None:
                print(f"    Route s used: {result['route_s_used']:.2f}")
                print(f"    Target road_s: {result['target_road_s']:.2f}")
                print(f"    Actual road_s: {result['actual_road_s']:.2f}")
                print(f"    Actual offset: {result['actual_offset']:.2f}m")
                print(f"    Error: {result['error']:.2f}m")
                print(f"    Correct offset: {result['correct_offset']:.2f}m")
            else:
                print(f"    [ERROR] Could not measure offset")
        
        all_results[route_name] = route_results
    
    return all_results


def analyze_offset_consistency(results):
    """Analyze if offsets are constant or vary along The Corkscrew1."""
    print("\n" + "="*80)
    print("Offset Consistency Analysis")
    print("="*80)
    
    for route_name, route_results in results.items():
        print(f"\n{route_name} Analysis:")
        print(f"  {'Road s':>8} | {'Actual Offset':>14} | {'Error':>8} | {'Correct Offset':>16}")
        print(f"  {'-'*8} | {'-'*14} | {'-'*8} | {'-'*16}")
        
        valid_results = [r for r in route_results if r.get('actual_offset') is not None]
        
        if not valid_results:
            print(f"  No valid measurements")
            continue
        
        for r in valid_results:
            actual_offset_str = f"{r['actual_offset']:.2f}m" if r['actual_offset'] is not None else "N/A"
            error_str = f"{r['error']:.2f}m" if r['error'] is not None else "N/A"
            correct_offset_str = f"{r['correct_offset']:.2f}m" if r['correct_offset'] is not None else "N/A"
            print(f"  {r['target_road_s']:>8.1f} | {actual_offset_str:>14} | {error_str:>8} | {correct_offset_str:>16}")
        
        # Calculate statistics
        actual_offsets = [r['actual_offset'] for r in valid_results]
        correct_offsets = [r['correct_offset'] for r in valid_results if r['correct_offset'] is not None]
        
        if actual_offsets:
            avg_actual_offset = sum(actual_offsets) / len(actual_offsets)
            min_actual_offset = min(actual_offsets)
            max_actual_offset = max(actual_offsets)
            offset_range = max_actual_offset - min_actual_offset
            
            print(f"\n  Statistics:")
            print(f"    Average actual offset: {avg_actual_offset:.2f}m")
            print(f"    Min actual offset: {min_actual_offset:.2f}m")
            print(f"    Max actual offset: {max_actual_offset:.2f}m")
            print(f"    Offset range: {offset_range:.2f}m")
            
            if offset_range < 0.1:
                print(f"    [CONCLUSION] Offset is CONSTANT (range < 0.1m)")
            elif offset_range < 1.0:
                print(f"    [CONCLUSION] Offset is NEARLY CONSTANT (range < 1.0m)")
            else:
                print(f"    [CONCLUSION] Offset VARIES (range >= 1.0m)")
                print(f"    [ACTION] Offset may need position-dependent correction")
        
        if correct_offsets:
            avg_correct_offset = sum(correct_offsets) / len(correct_offsets)
            min_correct_offset = min(correct_offsets)
            max_correct_offset = max(correct_offsets)
            correct_offset_range = max_correct_offset - min_correct_offset
            
            print(f"\n  Recommended Offset Values:")
            print(f"    Average correct offset: {avg_correct_offset:.2f}m")
            print(f"    Min correct offset: {min_correct_offset:.2f}m")
            print(f"    Max correct offset: {max_correct_offset:.2f}m")
            print(f"    Correct offset range: {correct_offset_range:.2f}m")
            
            # Get current offset from first result
            current_offset = valid_results[0]['current_offset']
            print(f"\n  Current offset: {current_offset:.2f}m")
            print(f"  Recommended offset: {avg_correct_offset:.2f}m")
            print(f"  Difference: {abs(avg_correct_offset - current_offset):.2f}m")
            
            if abs(avg_correct_offset - current_offset) < 0.1:
                print(f"    [CONCLUSION] Current offset is CORRECT")
            else:
                print(f"    [CONCLUSION] Current offset should be adjusted to {avg_correct_offset:.2f}m")


def main():
    """Main test function."""
    print("="*80)
    print("Offset Precision Testing")
    print("="*80)
    print("\nThis script tests if the 8.9m (R2) and 9.6m (R1) offsets are truly constant")
    print("along The Corkscrew1 by testing multiple positions at road_s = 0, 100, 200, 300, 500, 1000, 1500")
    print("\nGoal: Verify offset precision to refine route-specific projection")
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
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="OffsetPrecisionTest")
        
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
        
        # Load road index
        print("\n[0] Loading road index...")
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        if not os.path.exists(rd_path):
            print(f"    [ERROR] RD file not found: {rd_path}")
            return
        
        road_index = build_rd_road_index(rd_path)
        print(f"    [OK] Loaded road index with {len(road_index['roads'])} roads")
        
        # Test offset precision
        results = test_offset_precision(road_index, fellow, cd, ts)
        
        # Analyze results
        analyze_offset_consistency(results)
        
        print("\n" + "="*80)
        print("Offset Precision Testing Complete")
        print("="*80)
        print("\nUse this data to:")
        print("  1. Verify if offsets are constant along The Corkscrew1")
        print("  2. Refine offset values if needed")
        print("  3. Determine if position-dependent correction is required")
        print("  4. Update route_projection.py with refined offsets")
        print("  5. Reduce round-trip errors toward <1m target")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
