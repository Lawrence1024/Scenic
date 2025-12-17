"""Verify transition point handling with updated offsets.

This script specifically tests the transition point (road_s=0) to ensure
the conditional offset logic works correctly.
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
from scenic.simulators.dspace.geometry.projection import project_world_to_st
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.utils import legacy as dutils


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    if new_scenario_name is None:
        new_scenario_name = time.strftime("TransitionTest_%Y%m%d_%H%M%S")
    
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


def test_transition_point(route_name, road_index, fellow, cd, ts):
    """Test transition point (road_s=0) with updated logic."""
    print(f"\nTesting {route_name} transition point (road_s=0)...")
    
    # Known transition points
    transition_points = {
        'R1': 902.0,
        'R2': 1006.0,
    }
    
    transition_offsets = {
        'R1': 9.6,   # Should work at transition point
        'R2': 8.9,   # Should work at transition point
    }
    
    transition_point = transition_points[route_name]
    transition_offset = transition_offsets[route_name]
    
    # Calculate route_s for transition point
    # With updated logic: if road_s < 1.0, use transition_offset
    road_s = 0.0
    route_s = road_s + transition_point + transition_offset
    
    print(f"  road_s={road_s:.1f}, transition_point={transition_point:.1f}, transition_offset={transition_offset:.1f}")
    print(f"  Calculated route_s={route_s:.2f}")
    
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
    actual_road_s, actual_road_t = project_world_to_st(road_index, (rd_x, rd_y))
    
    print(f"  Readback RD: ({rd_x:.2f}, {rd_y:.2f})")
    print(f"  Actual road_s: {actual_road_s:.2f}")
    print(f"  Target road_s: {road_s:.2f}")
    print(f"  Error: {abs(actual_road_s - road_s):.2f}m")
    
    return {
        'route_name': route_name,
        'target_road_s': road_s,
        'route_s_used': route_s,
        'actual_road_s': actual_road_s,
        'error': abs(actual_road_s - road_s),
    }


def main():
    """Main test function."""
    print("="*80)
    print("Transition Point Verification")
    print("="*80)
    
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
        
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="TransitionVerification")
        
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
        
        # Load road index
        rd_path = r"C:\Users\bklfh\Documents\Scenic\Scenic\assets\maps\dSPACE\Laguna_Seca.rd"
        road_index = build_rd_road_index(rd_path)
        
        # Test both routes
        results = []
        for route_name in ['R1', 'R2']:
            result = test_transition_point(route_name, road_index, fellow, cd, ts)
            results.append(result)
        
        # Summary
        print("\n" + "="*80)
        print("Summary")
        print("="*80)
        for r in results:
            print(f"  {r['route_name']}: error={r['error']:.3f}m (target road_s={r['target_road_s']:.1f}, actual={r['actual_road_s']:.2f})")
        
        avg_error = sum(r['error'] for r in results) / len(results) if results else 0
        print(f"\nAverage error: {avg_error:.3f}m")
        print(f"Target: <0.1m (transition point should be perfect)")
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
