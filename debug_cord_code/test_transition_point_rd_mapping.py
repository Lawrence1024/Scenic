"""Test if the transition point RD mapping issue is specific to road_s=0 or general.

This script tests positions very close to the transition point to see if
the ~10m RD error is specific to road_s=0 or affects nearby positions too.
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
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
from scenic.simulators.dspace.geometry.route_projection import find_road_name_by_id
from scenic.simulators.dspace.utils import legacy as dutils


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    if new_scenario_name is None:
        new_scenario_name = time.strftime("TransitionRDTest_%Y%m%d_%H%M%S")
    
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


def test_rd_mapping_at_road_s(route_name, road_s, transition_point, transition_offset, road_index, fellow, cd, ts, expected_rd):
    """Test RD mapping at a specific road_s position."""
    # Calculate route_s
    route_s = road_s + transition_point + transition_offset
    
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
    
    readback_rd_x = float(x_arr[0])
    readback_rd_y = float(y_arr[0])
    
    # Calculate error
    rd_error = math.sqrt((readback_rd_x - expected_rd[0])**2 + (readback_rd_y - expected_rd[1])**2)
    
    # Project readback to road-relative s
    readback_road_s, readback_road_t = project_world_to_st(road_index, (readback_rd_x, readback_rd_y))
    
    return {
        'road_s': road_s,
        'route_s': route_s,
        'readback_rd': (readback_rd_x, readback_rd_y),
        'expected_rd': expected_rd,
        'readback_road_s': readback_road_s,
        'rd_error': rd_error,
        'road_s_error': abs(readback_road_s - road_s),
    }


def main():
    """Main test function."""
    print("="*80)
    print("Transition Point RD Mapping Test")
    print("="*80)
    print("\nTesting if the ~10m RD error is specific to road_s=0 or affects nearby positions")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    # Fellow_1's expected RD coordinate
    fellow_1_xodr = (-101.919263, -457.524908, 0.0)
    
    pythoncom.CoInitialize()
    try:
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("\n[ERROR] No active project")
            return
        exp = proj.ActiveExperiment
        if exp is None:
            print("\n[ERROR] No active experiment")
            return
        
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="TransitionRDTest")
        
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
        
        # Load transform and road index
        transform_path = os.path.join(
            os.path.dirname(__file__), '..', 'assets', 'maps', 'dSPACE', 'Laguna_Seca_transform.json'
        )
        coordinate_transform = None
        if os.path.exists(transform_path):
            coordinate_transform = load_transform(transform_path)
        
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        road_index = build_rd_road_index(rd_path)
        
        # Calculate expected RD for Fellow_1
        if coordinate_transform:
            expected_rd = apply_coordinate_transform(coordinate_transform, fellow_1_xodr[:2])
        else:
            expected_rd = fellow_1_xodr[:2]
        
        print(f"\nFellow_1 Expected RD: ({expected_rd[0]:.6f}, {expected_rd[1]:.6f})")
        
        # Test positions near transition point
        # Test at: 0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0
        test_road_s_positions = [0.0, 0.5, 1.0, 2.0, 5.0, 10.0, 25.0, 50.0, 100.0]
        
        print(f"\n{'='*80}")
        print("Testing R2 (Lap) - Transition Point RD Mapping")
        print(f"{'='*80}")
        
        route_configs = {
            'R2': {
                'transition_point': 1006.0,
                'transition_offset': 8.9,
            }
        }
        
        results = []
        
        for route_name, config in route_configs.items():
            print(f"\nTesting {route_name} positions near transition point:")
            print(f"  {'Road s':>8} | {'Route s':>10} | {'Expected RD X':>14} | {'Readback RD X':>16} | {'RD Error':>10} | {'Road s Error':>12}")
            print(f"  {'-'*8} | {'-'*10} | {'-'*14} | {'-'*16} | {'-'*10} | {'-'*12}")
            
            for road_s in test_road_s_positions:
                # Calculate expected RD for this road_s position
                # We can get this by finding the point on The Corkscrew1 at this road_s
                # The road index has sec_points with (x, y, s) tuples
                expected_rd_for_position = None
                if 'The Corkscrew1' in road_index['roads']:
                    corkscrew = road_index['roads']['The Corkscrew1']
                    sec_points = corkscrew.get('sec_points', [])
                    # Find the point closest to this road_s
                    closest_point = None
                    min_diff = float('inf')
                    for sec in sec_points:
                        if sec and len(sec) >= 3:
                            x, y, s = sec[0], sec[1], sec[2]
                            diff = abs(s - road_s)
                            if diff < min_diff:
                                min_diff = diff
                                closest_point = (x, y)
                    if closest_point:
                        expected_rd_for_position = closest_point
                
                if expected_rd_for_position is None:
                    # Fallback: use Fellow_1's expected RD (not ideal, but for comparison)
                    expected_rd_for_position = expected_rd
                
                result = test_rd_mapping_at_road_s(
                    route_name,
                    road_s,
                    config['transition_point'],
                    config['transition_offset'],
                    road_index,
                    fellow,
                    cd,
                    ts,
                    expected_rd_for_position
                )
                
                results.append(result)
                
                print(f"  {road_s:>8.1f} | {result['route_s']:>10.2f} | {result['expected_rd'][0]:>14.2f} | {result['readback_rd'][0]:>16.2f} | {result['rd_error']:>10.3f}m | {result['road_s_error']:>12.3f}m")
        
        # Analysis
        print(f"\n{'='*80}")
        print("Analysis")
        print(f"{'='*80}")
        
        # Check if error is consistent or varies
        rd_errors = [r['rd_error'] for r in results]
        road_s_errors = [r['road_s_error'] for r in results]
        
        print(f"\nRD Error Statistics:")
        print(f"  Average: {sum(rd_errors) / len(rd_errors):.3f}m")
        print(f"  Min: {min(rd_errors):.3f}m")
        print(f"  Max: {max(rd_errors):.3f}m")
        print(f"  Range: {max(rd_errors) - min(rd_errors):.3f}m")
        
        print(f"\nRoad s Error Statistics:")
        print(f"  Average: {sum(road_s_errors) / len(road_s_errors):.3f}m")
        print(f"  Min: {min(road_s_errors):.3f}m")
        print(f"  Max: {max(road_s_errors):.3f}m")
        
        # Check if error at road_s=0 is different from others
        error_at_0 = results[0]['rd_error']
        errors_after_0 = [r['rd_error'] for r in results[1:]]
        
        if errors_after_0:
            avg_error_after_0 = sum(errors_after_0) / len(errors_after_0)
            print(f"\nError at road_s=0: {error_at_0:.3f}m")
            print(f"Average error at road_s>0: {avg_error_after_0:.3f}m")
            print(f"Difference: {abs(error_at_0 - avg_error_after_0):.3f}m")
            
            if abs(error_at_0 - avg_error_after_0) < 1.0:
                print(f"  [CONCLUSION] Error is consistent across all positions")
                print(f"  [DIAGNOSIS] The ~10m error is not specific to transition point")
            else:
                print(f"  [CONCLUSION] Error is different at transition point")
                print(f"  [DIAGNOSIS] The ~10m error may be specific to road_s=0")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
