#!/usr/bin/env python3
"""
Test the round-trip coordinate transformation fix.

This verifies that:
1. Scenic XODR coordinate -> RD -> (s,t) -> ModelDesk -> ControlDesk RD -> XODR
2. The final XODR should match the original XODR (within tolerance)

This is the ultimate goal: what goes out of Scenic and what is read back
should be in the same coordinate system.

Usage:
    python debug_cord_code/test_round_trip_fix.py
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
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific


# Test coordinates from Scenic (XODR)
# Include coordinates on different roads to test route sequences
TEST_COORDINATES = {
    'Fellow_1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),  # The Corkscrew1 (R2)
    },
    'Fellow_2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),  # The Corkscrew1 (R2)
    },
    'Fellow_3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),  # The Corkscrew1 (R2)
    },
    'Fellow_4_PitLane': {
        'scenic_xodr': (163.54, 48.30, 0.0),  # Pit Lane start (R1 s=0) - approximate XODR
    },
    'Fellow_5_PitLane': {
        'scenic_xodr': (170.0, 50.0, 0.0),  # Pit Lane middle - approximate XODR
    }
}


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_round_trip(coordinate_transform, road_index, scenic_xodr, name):
    """Test the complete round-trip: XODR -> RD -> (s,t) -> ModelDesk -> ControlDesk RD -> XODR."""
    print(f"\n{name}: Testing round-trip transformation")
    print(f"  Starting XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
    
    # Step 1: XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        print(f"  Step 1 (XODR -> RD): ({rd_x:.3f}, {rd_y:.3f})")
    else:
        rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
        print(f"  Step 1 (XODR -> RD): ({rd_x:.3f}, {rd_y:.3f}) [no transform]")
    
    # Step 2: Determine route from RD coordinate
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("  [SKIP] No ModelDesk project")
            return None
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("  [SKIP] No experiment")
            return None
        
        # We need a simulator instance to use detectTrackSegment, but we can simulate it
        # For now, let's use the route detection logic directly
        from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
        
        # Detect track segment
        params = {}  # Would normally come from scene params
        track_segment = detect_track_segment((rd_x, rd_y), road_index, params, dutils)
        route_pref = assign_route_for_segment(track_segment) if track_segment else 'Lap'
        
        print(f"  Step 2 (Detect Route): {route_pref} (from track segment: {track_segment})")
        
        # Step 3: Project RD -> (s,t) using route-specific index
        s_val, t_val = project_world_to_st_route_specific(
            road_index,
            (rd_x, rd_y),
            route_preference=route_pref
        )
        print(f"  Step 3 (RD -> s,t): ({s_val:.1f}, {t_val:.3f}) [route-relative for {route_pref}]")
        
        # Step 4: Place in ModelDesk
        ts = exp.TrafficScenario
        if ts is None:
            print("  [SKIP] No traffic scenario")
            return None
        
        # Clear existing fellows
        try:
            dutils.clear_collection(ts.Fellows)
        except:
            pass
        
        # Create fellow
        F = ts.Fellows.Add()
        F.Name = f"Test_{name}"
        
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
            route_name_map = {'Pit': 'R1', 'Lap': 'R2'}
            modeldesk_route = route_name_map.get(route_pref, 'R2')
            route_sel.Activate(modeldesk_route)
            print(f"  Step 4 (Set in ModelDesk): (s={s_val:.1f}, t={t_val:.3f}) on route {modeldesk_route}")
        except Exception as e:
            print(f"  [WARNING] Could not set route: {e}")
        
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
        time.sleep(2.0)
        
        # Step 5: Read back from ControlDesk
        try:
            cd = ControlDeskApp(
                prog_id="ControlDeskNG.Application",
                outer_platform_name="Platform",
                inner_platform_name="Platform_2"
            ).connect()
            
            # Wait and step
            print("      Waiting 2 seconds for simulation to initialize...")
            time.sleep(2.0)
            
            print("      Stepping simulation...")
            for i in range(20):
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
                print(f"  Step 5 (ControlDesk RD): ({readback_rd[0]:.3f}, {readback_rd[1]:.3f})")
                
                # Step 6: RD -> XODR (inverse transform)
                if coordinate_transform:
                    readback_xodr = apply_inverse_coordinate_transform(coordinate_transform, readback_rd)
                    print(f"  Step 6 (RD -> XODR): ({readback_xodr[0]:.3f}, {readback_xodr[1]:.3f})")
                    
                    # Calculate errors
                    rd_error = math.sqrt((readback_rd[0] - rd_x)**2 + (readback_rd[1] - rd_y)**2)
                    xodr_error = math.sqrt((readback_xodr[0] - scenic_xodr[0])**2 + (readback_xodr[1] - scenic_xodr[1])**2)
                    
                    print(f"\n  Results:")
                    print(f"    Original XODR:    ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
                    print(f"    Readback XODR:    ({readback_xodr[0]:.3f}, {readback_xodr[1]:.3f})")
                    print(f"    XODR error:       {xodr_error:.3f} m")
                    print(f"    RD error:         {rd_error:.3f} m")
                    
                    if xodr_error < 1.0:
                        print(f"    [OK] Round-trip works! XODR error < 1m")
                    elif xodr_error < 10.0:
                        print(f"    [WARNING] Moderate XODR error (1-10m)")
                    else:
                        print(f"    [ERROR] Large XODR error (>10m) - fix may not be working")
                    
                    return {
                        'original_xodr': scenic_xodr[:2],
                        'readback_xodr': readback_xodr,
                        'xodr_error': xodr_error,
                        'rd_error': rd_error,
                        'route': route_pref
                    }
                else:
                    print(f"  [SKIP] No coordinate transform for inverse")
                    return None
            else:
                print("  [ERROR] Could not read position from ControlDesk")
                return None
                
        except Exception as e:
            print(f"  [ERROR] ControlDesk error: {e}")
            import traceback
            traceback.print_exc()
            return None
        
    except Exception as e:
        print(f"  [ERROR] ModelDesk error: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    """Main function."""
    print("="*80)
    print("Round-Trip Coordinate Transformation Test")
    print("="*80)
    print("\nThis test verifies the ultimate goal:")
    print("  What goes out of Scenic (XODR) should match what is read back (XODR)")
    print("\nTest flow:")
    print("  1. XODR -> RD (coordinate transform)")
    print("  2. Determine Route (R1 or R2)")
    print("  3. RD -> (s,t) [route-specific projection]")
    print("  4. Set in ModelDesk")
    print("  5. Read back RD from ControlDesk")
    print("  6. RD -> XODR (inverse transform)")
    print("  7. Compare original vs readback XODR")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running")
    print("="*80)
    
    # Load coordinate transform
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    coordinate_transform = None
    if transform_path.exists():
        try:
            coordinate_transform = load_transform(str(transform_path))
            print(f"\n[OK] Loaded coordinate transform")
        except Exception as e:
            print(f"\n[WARNING] Could not load transform: {e}")
    else:
        print(f"\n[WARNING] Transform file not found: {transform_path}")
    
    # Load road index
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = None
    if rd_path.exists():
        try:
            road_index = build_rd_road_index(str(rd_path), step=0.5)
            print(f"[OK] Loaded road index")
        except Exception as e:
            print(f"\n[ERROR] Could not load road index: {e}")
            import traceback
            traceback.print_exc()
            return 1
    else:
        print(f"\n[ERROR] RD file not found: {rd_path}")
        return 1
    
    # Test each coordinate
    print_section("Testing Round-Trip Transformations")
    
    results = {}
    for name, data in TEST_COORDINATES.items():
        result = test_round_trip(
            coordinate_transform,
            road_index,
            data['scenic_xodr'],
            name
        )
        if result:
            results[name] = result
    
    # Summary
    print_section("Summary")
    
    if results:
        xodr_errors = [r['xodr_error'] for r in results.values()]
        rd_errors = [r['rd_error'] for r in results.values()]
        
        avg_xodr_error = sum(xodr_errors) / len(xodr_errors)
        max_xodr_error = max(xodr_errors)
        min_xodr_error = min(xodr_errors)
        
        avg_rd_error = sum(rd_errors) / len(rd_errors)
        max_rd_error = max(rd_errors)
        min_rd_error = min(rd_errors)
        
        print(f"\nXODR Round-Trip Errors (Original -> Readback):")
        print(f"  Average: {avg_xodr_error:.3f} m")
        print(f"  Max:     {max_xodr_error:.3f} m")
        print(f"  Min:     {min_xodr_error:.3f} m")
        
        print(f"\nRD Round-Trip Errors (Expected -> Readback):")
        print(f"  Average: {avg_rd_error:.3f} m")
        print(f"  Max:     {max_rd_error:.3f} m")
        print(f"  Min:     {min_rd_error:.3f} m")
        
        print(f"\nRoute Distribution:")
        route_counts = {}
        for name, result in results.items():
            route = result['route']
            route_counts[route] = route_counts.get(route, 0) + 1
        for route, count in route_counts.items():
            print(f"  {route}: {count} coordinate(s)")
        
        # Final verdict
        print(f"\n{'='*80}")
        if avg_xodr_error < 1.0:
            print("[SUCCESS] Round-trip transformation is working correctly!")
            print("  XODR coordinates match within 1m tolerance.")
            print("  Scenic and dSPACE are using the same coordinate system.")
        elif avg_xodr_error < 10.0:
            print("[PARTIAL SUCCESS] Round-trip has moderate errors (1-10m)")
            print("  May need further calibration or route-specific adjustments.")
        else:
            print("[FAILURE] Round-trip has large errors (>10m)")
            print("  The fix may not be working correctly.")
            print("  Check route detection and route-specific projection.")
        print(f"{'='*80}")
    else:
        print("\n[ERROR] No results to analyze")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
