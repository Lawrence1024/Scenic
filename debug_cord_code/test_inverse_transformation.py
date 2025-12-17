#!/usr/bin/env python3
"""
Test inverse transformation: Start from known ControlDesk RD coordinates
and work backwards through the transformation chain to find where it breaks.

This helps identify if the issue is:
- Forward: XODR -> RD -> (s,t) -> ModelDesk -> ControlDesk RD
- Inverse: ControlDesk RD -> (s,t) -> XODR

Usage:
    python debug_cord_code/test_inverse_transformation.py
"""

import sys
import os
import time
import math
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform, apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.utils import legacy as dutils


def test_known_rd_to_st_to_rd(rd_coord, road_index, route="R2"):
    """Test: Given an RD coordinate, project to (s,t), place in ModelDesk, read back RD.
    
    This tests if the round-trip RD -> (s,t) -> RD works correctly.
    """
    print(f"\nTesting round-trip: RD -> (s,t) -> ModelDesk -> ControlDesk RD")
    print(f"Starting RD coordinate: ({rd_coord[0]:.3f}, {rd_coord[1]:.3f})")
    print(f"Route: {route}")
    
    # Step 1: Project RD -> (s,t)
    s_val, t_val = dutils.project_world_to_st(road_index, rd_coord)
    print(f"Projected (s,t): ({s_val:.1f}, {t_val:.3f})")
    
    # Step 2: Place in ModelDesk
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("[SKIP] No ModelDesk project")
            return None
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("[SKIP] No experiment")
            return None
        
        ts = exp.TrafficScenario
        if ts is None:
            print("[SKIP] No traffic scenario")
            return None
        
        # Clear fellows
        try:
            dutils.clear_collection(ts.Fellows)
        except:
            pass
        
        # Create fellow
        F = ts.Fellows.Add()
        F.Name = "TestRoundTrip"
        
        seqs = F.Sequences
        dutils.clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
        segs = dutils.ensure_two_segments(S1)
        
        # Set (s,t)
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
        
        # Set route
        try:
            route_sel = S1.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0
            route_sel.Activate(route)
        except:
            pass
        
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
        except:
            pass
        
        # Save, download and reset
        try:
            ts.Save()
        except:
            pass
        
        ts.Download()
        time.sleep(0.5)
        mc = exp.ManeuverControl
        try:
            mc.Stop()
        except:
            pass
        time.sleep(0.2)
        mc.Reset()
        time.sleep(0.2)
        mc.Start(False)
        time.sleep(2.0)  # Wait longer for initialization
        
        # Connect to ControlDesk and read back
        try:
            cd = ControlDeskApp(
                prog_id="ControlDeskNG.Application",
                outer_platform_name="Platform",
                inner_platform_name="Platform_2"
            ).connect()
            
            # Wait for simulation to initialize
            print("      Waiting 2 seconds for simulation to initialize...")
            time.sleep(2.0)
            
            # Step simulation more times
            print("      Stepping simulation...")
            for i in range(20):  # Step more times
                cd.advance_simulation_step()
                time.sleep(0.1)
                if (i + 1) % 5 == 0:
                    print(f"      Step {i+1}/20 completed")
            print("      Waiting 1 more second for arrays to update...")
            time.sleep(1.0)
            
            # Read position
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            if len(x_arr) > 0 and len(y_arr) > 0:
                readback_rd = (float(x_arr[0]), float(y_arr[0]))
                
                error = math.sqrt((readback_rd[0] - rd_coord[0])**2 + (readback_rd[1] - rd_coord[1])**2)
                
                print(f"Readback RD:    ({readback_rd[0]:.3f}, {readback_rd[1]:.3f})")
                print(f"Error:          {error:.3f} m")
                
                if error < 1.0:
                    print(f"[OK] Round-trip works correctly")
                else:
                    print(f"[ERROR] Round-trip has large error - bug detected!")
                    print(f"  This suggests the (s,t) -> RD mapping is incorrect for route {route}")
                
                return {
                    'input_rd': rd_coord,
                    's_t': (s_val, t_val),
                    'readback_rd': readback_rd,
                    'error': error
                }
            else:
                print("[ERROR] Could not read position from ControlDesk")
                return None
                
        except Exception as e:
            print(f"[ERROR] ControlDesk error: {e}")
            return None
        
    except Exception as e:
        print(f"[ERROR] ModelDesk error: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_route_s_origin(road_index):
    """Test what RD coordinates correspond to s=0 on each route."""
    print("\n" + "="*80)
    print("Testing Route s=0 Origins")
    print("="*80)
    print("\nThis will place fellows at (s=0, t=0) on both R1 and R2")
    print("to see where each route's s-coordinate origin is in RD space")
    
    for route in ["R1", "R2"]:
        print(f"\n--- Testing {route} at (s=0, t=0) ---")
        result = test_known_rd_to_st_to_rd((0.0, 0.0), road_index, route=route)
        
        if result:
            print(f"{route} s=0 origin maps to RD: ({result['readback_rd'][0]:.3f}, {result['readback_rd'][1]:.3f})")
    
    # Also test a known RD coordinate
    print("\n" + "="*80)
    print("Testing Known RD Coordinate")
    print("="*80)
    
    # Use expected RD from Fellow_1
    known_rd = (-96.468, -456.652)
    print(f"\nTesting with known RD coordinate: {known_rd}")
    
    for route in ["R1", "R2"]:
        print(f"\n--- Testing on {route} ---")
        result = test_known_rd_to_st_to_rd(known_rd, road_index, route=route)
        
        if result:
            print(f"  Projected to: (s={result['s_t'][0]:.1f}, t={result['s_t'][1]:.3f})")
            print(f"  Readback:     ({result['readback_rd'][0]:.3f}, {result['readback_rd'][1]:.3f})")
            print(f"  Error:        {result['error']:.3f} m")


def main():
    """Main function."""
    print("="*80)
    print("Inverse Transformation Test")
    print("="*80)
    print("\nThis script tests the round-trip transformation:")
    print("  RD coordinate -> (s,t) -> ModelDesk -> ControlDesk RD")
    print("\nThis helps identify if the bug is in:")
    print("  - The projection step (RD -> s,t)")
    print("  - The route coordinate system (s,t -> RD on specific route)")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    # Load road index
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = None
    if rd_path.exists():
        try:
            road_index = build_rd_road_index(str(rd_path), step=0.5)
            print(f"\n[OK] Loaded road index from {rd_path.name}")
        except Exception as e:
            print(f"\n[ERROR] Could not load road index: {e}")
            import traceback
            traceback.print_exc()
            return 1
    else:
        print(f"\n[ERROR] RD file not found: {rd_path}")
        return 1
    
    # Test route origins
    test_route_s_origin(road_index)
    
    print("\n" + "="*80)
    print("Test Complete")
    print("="*80)
    print("\nInterpretation:")
    print("  - If round-trip errors are small (<1m): Projection is correct")
    print("  - If round-trip errors are large (>10m): Route coordinate system mismatch")
    print("  - Compare R1 vs R2 results to see route-specific behavior")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
