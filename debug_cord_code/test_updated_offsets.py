"""Quick test to verify updated offsets work correctly.

This script tests the updated offset values (17.6m for R1, 17.9m for R2)
to verify they reduce round-trip errors.
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
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.utils import legacy as dutils


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    if new_scenario_name is None:
        new_scenario_name = time.strftime("OffsetTest_%Y%m%d_%H%M%S")
    
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


def test_round_trip(coordinate_transform, road_index, scenic_xodr, name, cd, ts, fellow):
    """Test round-trip with updated offsets."""
    print(f"\n{name}:")
    print(f"  Starting XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
    
    # Step 1: XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
    else:
        rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
    print(f"  RD: ({rd_x:.3f}, {rd_y:.3f})")
    
    # Step 2: Project to route-specific (s,t)
    # Try both routes to find best match
    best_result = None
    best_error = float('inf')
    
    for route_pref in ['Lap', 'Pit']:
        try:
            s_val, t_val = project_world_to_st_route_specific(
                road_index, (rd_x, rd_y), route_preference=route_pref
            )
        except:
            continue
        
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
        route_name = 'R2' if route_pref == 'Lap' else 'R1'
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
        
        # RD -> XODR
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
                'route': route_name,
                'route_s': s_val,
                'readback_xodr': (readback_xodr_x, readback_xodr_y),
                'error': error,
            }
        
        print(f"    {route_name}: route_s={s_val:.2f}, error={error:.3f}m")
    
    if best_result:
        print(f"  Best: {best_result['route']}, error={best_result['error']:.3f}m")
        return best_result
    return None


def main():
    """Main test function."""
    print("="*80)
    print("Testing Updated Offsets")
    print("="*80)
    
    # Test coordinates
    test_coords = {
        'Fellow_1': (-101.919263, -457.524908, 0.0),  # The Corkscrew1
        'Fellow_2': (0.948038, -272.443171, 0.0),      # The Corkscrew1
        'Fellow_3': (191.994781, -418.905118, 0.0),    # The Corkscrew1
    }
    
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
        
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="OffsetVerification")
        
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
            from scenic.simulators.dspace.geometry.coordinate_transform import load_transform
            coordinate_transform = load_transform(transform_path)
        
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        road_index = build_rd_road_index(rd_path)
        
        # Test each coordinate
        results = []
        for name, coord in test_coords.items():
            result = test_round_trip(coordinate_transform, road_index, coord, name, cd, ts, fellow)
            if result:
                results.append(result)
        
        # Summary
        print("\n" + "="*80)
        print("Summary")
        print("="*80)
        if results:
            avg_error = sum(r['error'] for r in results) / len(results)
            print(f"Average error: {avg_error:.3f}m")
            print(f"Target: <1.0m")
            for r in results:
                print(f"  {r['route']}: {r['error']:.3f}m")
        else:
            print("No valid results")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
