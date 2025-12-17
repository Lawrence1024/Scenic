"""Comprehensive test to verify offset consistency across many positions.

This script tests many positions along The Corkscrew1 to verify that
the offset is truly constant (not just at the 7 positions tested before).

Tests positions at:
- Fine-grained intervals (every 50m, 100m, 200m)
- Various positions along the entire length of The Corkscrew1
- Both R1 and R2 routes

Goal: Verify offset is constant across the entire road, not just at specific points.
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
    if new_scenario_name is None:
        new_scenario_name = time.strftime("ComprehensiveOffsetTest_%Y%m%d_%H%M%S")
    
    if source_scenario:
        try:
            exp.ActivateTrafficScenario(source_scenario)
        except:
            pass
    
    try:
        exp.TrafficScenario.SaveAs(new_scenario_name, True)
    except:
        try:
            editor = exp.EditTrafficScenario()
            try:
                editor.SaveAs(new_scenario_name, True)
            finally:
                try:
                    editor.Close(False)
                except:
                    pass
        except:
            raise
    
    try:
        exp.ActivateTrafficScenario(new_scenario_name)
    except:
        pass
    
    pythoncom.PumpWaitingMessages()
    time.sleep(0.2)
    proj = app.ActiveProject
    exp = proj.ActiveExperiment
    ts = exp.TrafficScenario
    
    return ts


def test_offset_at_road_s(route_name, road_s, transition_point, current_offset, road_index, fellow, cd, ts):
    """Test offset at a specific road_s position on The Corkscrew1."""
    # Calculate route_s using current formula
    # Use larger offset for positions after transition (road_s >= 1.0)
    if road_s < 1.0:
        # At transition point: use transition offset
        TRANSITION_OFFSETS = {'R1': 9.6, 'R2': 8.9}
        offset = TRANSITION_OFFSETS.get(route_name, current_offset)
    else:
        # After transition: use larger offset
        offset = current_offset
    
    route_s = road_s + transition_point + offset
    
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
        actual_offset = actual_road_s - road_s
        
        # Calculate what the offset should be to achieve perfect alignment
        correct_offset = offset - actual_offset
        
        return {
            'target_road_s': road_s,
            'route_s_used': route_s,
            'actual_road_s': actual_road_s,
            'actual_offset': actual_offset,
            'correct_offset': correct_offset,
            'error': abs(actual_offset),
            'valid': True,
        }
    else:
        return {
            'target_road_s': road_s,
            'route_s_used': route_s,
            'actual_road_s': None,
            'actual_offset': None,
            'correct_offset': None,
            'error': None,
            'valid': False,
            'warning': f"Not on The Corkscrew1 (on {road_name})"
        }


def test_comprehensive_offsets(road_index, fellow, cd, ts):
    """Test offset at many positions along The Corkscrew1."""
    print("\n" + "="*80)
    print("Comprehensive Offset Consistency Testing")
    print("="*80)
    
    # Test positions: mix of fine-grained and coarse intervals
    # Test at: 0, 10, 25, 50, 75, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, 900, 1000, 1200, 1400, 1500, 1700, 2000, 2300
    test_road_s_positions = [
        0.0, 10.0, 25.0, 50.0, 75.0, 100.0, 150.0, 200.0, 250.0, 300.0,
        400.0, 500.0, 600.0, 700.0, 800.0, 900.0, 1000.0, 1200.0, 1400.0,
        1500.0, 1700.0, 2000.0, 2300.0
    ]
    
    # Known transition points and offsets
    route_configs = {
        'R1': {
            'transition_point': 902.0,
            'current_offset': 17.6,  # Larger offset for positions after transition
        },
        'R2': {
            'transition_point': 1006.0,
            'current_offset': 17.9,  # Larger offset for positions after transition
        },
    }
    
    all_results = {}
    
    for route_name, config in route_configs.items():
        print(f"\n{'='*80}")
        print(f"Testing {route_name} (transition={config['transition_point']:.1f}, offset={config['current_offset']:.1f}m)")
        print(f"{'='*80}")
        
        route_results = []
        valid_count = 0
        
        for road_s in test_road_s_positions:
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
            
            if result['valid']:
                valid_count += 1
                if valid_count <= 5 or valid_count % 5 == 0:  # Print first 5 and every 5th after
                    print(f"  road_s={road_s:>7.1f} -> actual_road_s={result['actual_road_s']:>7.2f}, offset={result['actual_offset']:>6.2f}m, error={result['error']:>6.2f}m")
            else:
                if result.get('warning'):
                    print(f"  road_s={road_s:>7.1f} -> [WARNING] {result['warning']}")
        
        all_results[route_name] = route_results
        print(f"\n  Valid measurements: {valid_count}/{len(test_road_s_positions)}")
    
    return all_results


def analyze_comprehensive_results(results):
    """Analyze comprehensive offset results."""
    print("\n" + "="*80)
    print("Comprehensive Offset Analysis")
    print("="*80)
    
    for route_name, route_results in results.items():
        print(f"\n{route_name} Analysis:")
        
        valid_results = [r for r in route_results if r.get('valid')]
        
        if not valid_results:
            print(f"  No valid measurements")
            continue
        
        # Separate transition point from other positions
        transition_results = [r for r in valid_results if r['target_road_s'] < 1.0]
        after_transition_results = [r for r in valid_results if r['target_road_s'] >= 1.0]
        
        print(f"\n  Transition Point (road_s < 1.0m):")
        if transition_results:
            for r in transition_results:
                print(f"    road_s={r['target_road_s']:>6.1f}: offset={r['actual_offset']:>6.2f}m, error={r['error']:>6.2f}m")
            avg_error = sum(r['error'] for r in transition_results) / len(transition_results)
            print(f"    Average error: {avg_error:.3f}m")
        
        print(f"\n  After Transition (road_s >= 1.0m):")
        if after_transition_results:
            actual_offsets = [r['actual_offset'] for r in after_transition_results]
            correct_offsets = [r['correct_offset'] for r in after_transition_results if r['correct_offset'] is not None]
            errors = [r['error'] for r in after_transition_results]
            
            if actual_offsets:
                avg_actual_offset = sum(actual_offsets) / len(actual_offsets)
                min_actual_offset = min(actual_offsets)
                max_actual_offset = max(actual_offsets)
                offset_range = max_actual_offset - min_actual_offset
                std_dev = math.sqrt(sum((x - avg_actual_offset)**2 for x in actual_offsets) / len(actual_offsets))
                
                avg_error = sum(errors) / len(errors)
                max_error = max(errors)
                min_error = min(errors)
                
                print(f"    Statistics:")
                print(f"      Number of positions tested: {len(after_transition_results)}")
                print(f"      Actual offset - Average: {avg_actual_offset:.3f}m")
                print(f"      Actual offset - Min: {min_actual_offset:.3f}m")
                print(f"      Actual offset - Max: {max_actual_offset:.3f}m")
                print(f"      Actual offset - Range: {offset_range:.3f}m")
                print(f"      Actual offset - Std Dev: {std_dev:.3f}m")
                print(f"      Error - Average: {avg_error:.3f}m")
                print(f"      Error - Min: {min_error:.3f}m")
                print(f"      Error - Max: {max_error:.3f}m")
                
                if offset_range < 0.1:
                    print(f"      [CONCLUSION] Offset is CONSTANT (range < 0.1m) [OK]")
                elif offset_range < 0.5:
                    print(f"      [CONCLUSION] Offset is NEARLY CONSTANT (range < 0.5m) [OK]")
                elif offset_range < 1.0:
                    print(f"      [CONCLUSION] Offset is MOSTLY CONSTANT (range < 1.0m) [WARNING]")
                else:
                    print(f"      [CONCLUSION] Offset VARIES (range >= 1.0m) [ERROR]")
                    print(f"      [ACTION] Offset may need position-dependent correction")
                
                if correct_offsets:
                    avg_correct_offset = sum(correct_offsets) / len(correct_offsets)
                    min_correct_offset = min(correct_offsets)
                    max_correct_offset = max(correct_offsets)
                    correct_offset_range = max_correct_offset - min_correct_offset
                    
                    print(f"\n    Recommended Offset Values:")
                    print(f"      Average correct offset: {avg_correct_offset:.3f}m")
                    print(f"      Min correct offset: {min_correct_offset:.3f}m")
                    print(f"      Max correct offset: {max_correct_offset:.3f}m")
                    print(f"      Correct offset range: {correct_offset_range:.3f}m")
                    
                    # Get current offset from config
                    current_offset = 17.6 if route_name == 'R1' else 17.9
                    print(f"\n    Current offset: {current_offset:.2f}m")
                    print(f"    Recommended offset: {avg_correct_offset:.2f}m")
                    print(f"    Difference: {abs(avg_correct_offset - current_offset):.2f}m")
                    
                    if abs(avg_correct_offset - current_offset) < 0.1:
                        print(f"      [CONCLUSION] Current offset is CORRECT [OK]")
                    elif abs(avg_correct_offset - current_offset) < 0.5:
                        print(f"      [CONCLUSION] Current offset is CLOSE (within 0.5m) [WARNING]")
                    else:
                        print(f"      [CONCLUSION] Current offset should be adjusted to {avg_correct_offset:.2f}m [ERROR]")
            
            # Show distribution of offsets
            print(f"\n    Offset Distribution (sample of positions):")
            sample_indices = [0, len(after_transition_results)//4, len(after_transition_results)//2, 
                            3*len(after_transition_results)//4, len(after_transition_results)-1]
            for idx in sample_indices:
                if idx < len(after_transition_results):
                    r = after_transition_results[idx]
                    print(f"      road_s={r['target_road_s']:>7.1f}: offset={r['actual_offset']:>6.2f}m, error={r['error']:>6.2f}m")


def main():
    """Main test function."""
    print("="*80)
    print("Comprehensive Offset Consistency Testing")
    print("="*80)
    print("\nThis script tests many positions along The Corkscrew1 to verify")
    print("that the offset is truly constant across the entire road.")
    print("\nTesting positions at: 0, 10, 25, 50, 75, 100, 150, 200, 250, 300,")
    print("400, 500, 600, 700, 800, 900, 1000, 1200, 1400, 1500, 1700, 2000, 2300")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    pythoncom.CoInitialize()
    try:
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("\n[ERROR] No active project. Please open a ModelDesk project first.")
            return
        exp = proj.ActiveExperiment
        if exp is None:
            print("\n[ERROR] No active experiment. Please activate an experiment.")
            return
        
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="ComprehensiveOffsetTest")
        
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        
        dutils.clear_collection(ts.Fellows)
        
        fellow = ts.Fellows.Add()
        fellow.Name = "TestFellow"
        
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            try:
                seq = sequences.Item(1)
            except:
                seq = sequences.Item(0)
        
        segs = dutils.ensure_two_segments(seq)
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        dutils.make_endless_transition(segs)
        
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        road_index = build_rd_road_index(rd_path)
        
        results = test_comprehensive_offsets(road_index, fellow, cd, ts)
        analyze_comprehensive_results(results)
        
        print("\n" + "="*80)
        print("Comprehensive Testing Complete")
        print("="*80)
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
