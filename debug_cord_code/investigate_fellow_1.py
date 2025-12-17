"""Investigate Fellow_1's position to understand why it has high round-trip error.

Fellow_1 has coordinates: (-101.919263, -457.524908, 0.0) in XODR
This should be on The Corkscrew1, but round-trip error is ~10m.

This script will:
1. Transform XODR -> RD
2. Project to (s,t) using route-specific projection
3. Place fellow at calculated (s,t)
4. Read back position
5. Analyze each step to find where the error occurs
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
from scenic.simulators.dspace.geometry.route_projection import (
    project_world_to_st_route_specific, find_road_name_by_id
)
from scenic.simulators.dspace.utils import legacy as dutils


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    if new_scenario_name is None:
        new_scenario_name = time.strftime("Fellow1Investigation_%Y%m%d_%H%M%S")
    
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


def investigate_fellow_1_position(coordinate_transform, road_index, scenic_xodr, cd, ts, fellow):
    """Investigate Fellow_1's position step by step."""
    print("\n" + "="*80)
    print("Fellow_1 Position Investigation")
    print("="*80)
    
    print(f"\nStarting XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
    
    # Step 1: XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        print(f"\nStep 1: XODR -> RD Transform")
        print(f"  RD: ({rd_x:.6f}, {rd_y:.6f})")
    else:
        rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
        print(f"\nStep 1: XODR -> RD (no transform)")
        print(f"  RD: ({rd_x:.6f}, {rd_y:.6f})")
    
    # Step 2: Identify which road this projects onto
    road_id = find_road_id_for_position(road_index, rd_x, rd_y)
    road_name = find_road_name_by_id(road_index, road_id) if road_id is not None else "Unknown"
    print(f"\nStep 2: Road Identification")
    print(f"  Road ID: {road_id}")
    print(f"  Road Name: {road_name}")
    
    # Step 3: Project to road-relative (s,t)
    road_s, road_t = project_world_to_st(road_index, (rd_x, rd_y))
    print(f"\nStep 3: Road-Relative Projection")
    print(f"  Road-relative s: {road_s:.3f}m")
    print(f"  Road-relative t: {road_t:.3f}m")
    
    # Step 4: Try both routes to see which one is correct
    print(f"\nStep 4: Route-Specific Projection")
    
    results = {}
    
    for route_pref in ['Lap', 'Pit']:
        route_name = 'R2' if route_pref == 'Lap' else 'R1'
        print(f"\n  Testing {route_name} ({route_pref}):")
        
        try:
            route_s, route_t = project_world_to_st_route_specific(
                road_index, (rd_x, rd_y), route_preference=route_pref
            )
            print(f"    Route-relative s: {route_s:.3f}m")
            print(f"    Route-relative t: {route_t:.3f}m")
            
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
            dutils.configure_seg0_absolute_pose(segs, s=float(route_s), t=float(route_t))
            
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
            
            print(f"    Readback RD: ({readback_rd_x:.6f}, {readback_rd_y:.6f})")
            
            # Project readback to road-relative s
            readback_road_id = find_road_id_for_position(road_index, readback_rd_x, readback_rd_y)
            readback_road_name = find_road_name_by_id(road_index, readback_road_id) if readback_road_id is not None else "Unknown"
            readback_road_s, readback_road_t = project_world_to_st(road_index, (readback_rd_x, readback_rd_y))
            
            print(f"    Readback road: {readback_road_name}, road_s={readback_road_s:.3f}m, road_t={readback_road_t:.3f}m")
            
            # RD -> XODR
            if coordinate_transform:
                readback_xodr_x, readback_xodr_y = apply_inverse_coordinate_transform(
                    coordinate_transform, (readback_rd_x, readback_rd_y)
                )
            else:
                readback_xodr_x, readback_xodr_y = readback_rd_x, readback_rd_y
            
            print(f"    Readback XODR: ({readback_xodr_x:.6f}, {readback_xodr_y:.6f})")
            
            # Calculate errors
            rd_error = math.sqrt((readback_rd_x - rd_x)**2 + (readback_rd_y - rd_y)**2)
            xodr_error = math.sqrt((readback_xodr_x - scenic_xodr[0])**2 + (readback_xodr_y - scenic_xodr[1])**2)
            road_s_error = abs(readback_road_s - road_s)
            road_t_error = abs(readback_road_t - road_t)
            
            print(f"    RD error: {rd_error:.3f}m")
            print(f"    XODR error: {xodr_error:.3f}m")
            print(f"    Road s error: {road_s_error:.3f}m")
            print(f"    Road t error: {road_t_error:.3f}m")
            
            results[route_name] = {
                'route_s': route_s,
                'route_t': route_t,
                'readback_rd': (readback_rd_x, readback_rd_y),
                'readback_xodr': (readback_xodr_x, readback_xodr_y),
                'readback_road_s': readback_road_s,
                'readback_road_t': readback_road_t,
                'rd_error': rd_error,
                'xodr_error': xodr_error,
                'road_s_error': road_s_error,
                'road_t_error': road_t_error,
            }
            
        except Exception as e:
            print(f"    [ERROR] {e}")
            import traceback
            traceback.print_exc()
    
    # Analysis
    print(f"\n" + "="*80)
    print("Analysis")
    print("="*80)
    
    print(f"\nOriginal Position:")
    print(f"  XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
    print(f"  RD: ({rd_x:.6f}, {rd_y:.6f})")
    print(f"  Road: {road_name}, road_s={road_s:.3f}m, road_t={road_t:.3f}m")
    
    for route_name, result in results.items():
        print(f"\n{route_name} Results:")
        print(f"  Route s: {result['route_s']:.3f}m")
        print(f"  Readback RD: ({result['readback_rd'][0]:.6f}, {result['readback_rd'][1]:.6f})")
        print(f"  Readback XODR: ({result['readback_xodr'][0]:.6f}, {result['readback_xodr'][1]:.6f})")
        print(f"  Readback road_s: {result['readback_road_s']:.3f}m (original: {road_s:.3f}m)")
        print(f"  RD error: {result['rd_error']:.3f}m")
        print(f"  XODR error: {result['xodr_error']:.3f}m")
        print(f"  Road s error: {result['road_s_error']:.3f}m")
        print(f"  Road t error: {result['road_t_error']:.3f}m")
        
        # Check if road_s matches
        if result['road_s_error'] < 0.1:
            print(f"  [OK] Road s matches (error < 0.1m)")
        else:
            print(f"  [WARNING] Road s mismatch (error >= 0.1m)")
        
        # Check if RD matches
        if result['rd_error'] < 1.0:
            print(f"  [OK] RD matches (error < 1.0m)")
        else:
            print(f"  [WARNING] RD mismatch (error >= 1.0m)")
    
    # Find best route
    if results:
        best_route = min(results.keys(), key=lambda k: results[k]['xodr_error'])
        print(f"\nBest Route: {best_route} (XODR error: {results[best_route]['xodr_error']:.3f}m)")
        
        # Check if the issue is in road_s mapping or coordinate transform
        if results[best_route]['road_s_error'] < 0.1 and results[best_route]['rd_error'] > 1.0:
            print(f"\n[DIAGNOSIS] Road s mapping is correct, but RD coordinate doesn't match.")
            print(f"  This suggests the issue is in the route s -> RD conversion (dSPACE internal).")
        elif results[best_route]['road_s_error'] > 0.1:
            print(f"\n[DIAGNOSIS] Road s mapping is incorrect.")
            print(f"  This suggests the issue is in the route-specific projection.")
        else:
            print(f"\n[DIAGNOSIS] Both road s and RD match, but XODR error is high.")
            print(f"  This suggests the issue is in the coordinate transformation.")
    
    return results


def main():
    """Main investigation function."""
    print("="*80)
    print("Fellow_1 Position Investigation")
    print("="*80)
    print("\nInvestigating why Fellow_1 has high round-trip error")
    print("Fellow_1 XODR: (-101.919263, -457.524908, 0.0)")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    # Fellow_1 coordinates
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
        
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="Fellow1Investigation")
        
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
        
        results = investigate_fellow_1_position(
            coordinate_transform, road_index, fellow_1_xodr, cd, ts, fellow
        )
        
        print("\n" + "="*80)
        print("Investigation Complete")
        print("="*80)
        
    finally:
        pythoncom.CoUninitialize()


if __name__ == "__main__":
    main()
