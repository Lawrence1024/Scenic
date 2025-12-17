#!/usr/bin/env python3
"""
Systematic debugging script to isolate the coordinate transformation bug.

This script tests each transformation step independently:
1. Step 1: XODR -> RD (coordinate transform)
2. Step 2: RD (x,y) -> (s,t) projection
3. Step 3: (s,t) -> ModelDesk -> ControlDesk RD (round-trip test)
4. Step 4: Verify route-specific behavior

Usage:
    python debug_cord_code/isolate_transformation_bug.py
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


# Test coordinates from the expected coordinates
TEST_COORDINATES = {
    'Fellow_1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
        'expected_rd': (-96.468, -456.652),
        'expected_s_t': (0.0, -1.653)
    },
    'Fellow_2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
        'expected_rd': (5.082, -273.737),
        'expected_s_t': (279.4, 1.472)
    }
}


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_step1_xodr_to_rd(coordinate_transform):
    """Test Step 1: XODR -> RD coordinate transformation."""
    print_section("STEP 1: Testing XODR -> RD Coordinate Transformation")
    
    if not coordinate_transform:
        print("[SKIP] No coordinate transform available")
        return None
    
    print(f"Transform type: {coordinate_transform.get('type', 'unknown')}")
    
    results = {}
    for name, data in TEST_COORDINATES.items():
        scenic_xodr = data['scenic_xodr'][:2]  # (x, y)
        expected_rd = data['expected_rd']
        
        # Apply forward transform
        actual_rd = apply_coordinate_transform(coordinate_transform, scenic_xodr)
        
        # Calculate error
        error = math.sqrt((actual_rd[0] - expected_rd[0])**2 + (actual_rd[1] - expected_rd[1])**2)
        
        # Test inverse transform
        inverse_xodr = apply_inverse_coordinate_transform(coordinate_transform, actual_rd)
        inverse_error = math.sqrt((inverse_xodr[0] - scenic_xodr[0])**2 + (inverse_xodr[1] - scenic_xodr[1])**2)
        
        results[name] = {
            'scenic_xodr': scenic_xodr,
            'expected_rd': expected_rd,
            'actual_rd': actual_rd,
            'forward_error': error,
            'inverse_xodr': inverse_xodr,
            'inverse_error': inverse_error
        }
        
        print(f"\n{name}:")
        print(f"  XODR:        ({scenic_xodr[0]:8.3f}, {scenic_xodr[1]:8.3f})")
        print(f"  Expected RD: ({expected_rd[0]:8.3f}, {expected_rd[1]:8.3f})")
        print(f"  Actual RD:   ({actual_rd[0]:8.3f}, {actual_rd[1]:8.3f})")
        print(f"  Forward error: {error:.6f} m")
        print(f"  Inverse XODR: ({inverse_xodr[0]:8.3f}, {inverse_xodr[1]:8.3f})")
        print(f"  Inverse error: {inverse_error:.6f} m")
        
        if error > 0.001:
            print(f"  [WARNING] Forward transform error > 1mm")
        if inverse_error > 0.001:
            print(f"  [WARNING] Inverse transform error > 1mm")
    
    # Summary
    avg_forward_error = sum(r['forward_error'] for r in results.values()) / len(results)
    avg_inverse_error = sum(r['inverse_error'] for r in results.values()) / len(results)
    
    print(f"\n[SUMMARY] Step 1 (XODR -> RD):")
    print(f"  Average forward error:  {avg_forward_error:.6f} m")
    print(f"  Average inverse error:   {avg_inverse_error:.6f} m")
    
    if avg_forward_error < 0.001 and avg_inverse_error < 0.001:
        print(f"  [OK] Step 1 is working correctly")
    else:
        print(f"  [ERROR] Step 1 has issues")
    
    return results


def test_step2_rd_to_st(road_index, step1_results):
    """Test Step 2: RD (x,y) -> (s,t) projection."""
    print_section("STEP 2: Testing RD (x,y) -> (s,t) Projection")
    
    if not road_index:
        print("[SKIP] No road index available")
        return None
    
    print(f"Road index contains {len(road_index.get('roads', {}))} roads")
    for road_name, road_data in road_index.get('roads', {}).items():
        length = road_data.get('length', 0)
        num_points = len(road_data.get('sec_points', [[]])[0]) if road_data.get('sec_points') else 0
        print(f"  - {road_name}: length={length:.1f}m, {num_points} points")
    
    results = {}
    for name, step1_data in step1_results.items():
        expected_data = TEST_COORDINATES[name]
        expected_s_t = expected_data['expected_s_t']
        actual_rd = step1_data['actual_rd']
        
        # Project RD -> (s,t)
        actual_s, actual_t = dutils.project_world_to_st(road_index, actual_rd)
        
        # Find which road was used for projection
        from scenic.simulators.dspace.geometry.projection import find_road_id_for_position
        road_id = find_road_id_for_position(road_index, actual_rd[0], actual_rd[1])
        
        # Find road name
        road_name = "Unknown"
        for rname, rdata in road_index.get('roads', {}).items():
            if rdata.get('id') == road_id:
                road_name = rname
                break
        
        # Calculate errors
        s_error = abs(actual_s - expected_s_t[0])
        t_error = abs(actual_t - expected_s_t[1])
        
        results[name] = {
            'rd': actual_rd,
            'expected_s_t': expected_s_t,
            'actual_s_t': (actual_s, actual_t),
            's_error': s_error,
            't_error': t_error,
            'road_id': road_id,
            'road_name': road_name
        }
        
        print(f"\n{name}:")
        print(f"  RD:          ({actual_rd[0]:8.3f}, {actual_rd[1]:8.3f})")
        print(f"  Expected:    s={expected_s_t[0]:7.1f}, t={expected_s_t[1]:6.3f}")
        print(f"  Actual:      s={actual_s:7.1f}, t={actual_t:6.3f}")
        print(f"  Projected onto: {road_name} (id={road_id})")
        print(f"  s error:     {s_error:.3f} m")
        print(f"  t error:     {t_error:.3f} m")
        
        if s_error > 1.0 or t_error > 0.1:
            print(f"  [WARNING] Large projection error")
    
    # Summary
    avg_s_error = sum(r['s_error'] for r in results.values()) / len(results)
    avg_t_error = sum(r['t_error'] for r in results.values()) / len(results)
    
    print(f"\n[SUMMARY] Step 2 (RD -> s,t):")
    print(f"  Average s error: {avg_s_error:.3f} m")
    print(f"  Average t error: {avg_t_error:.3f} m")
    
    if avg_s_error < 1.0 and avg_t_error < 0.1:
        print(f"  [OK] Step 2 projection is working (within expected tolerance)")
    else:
        print(f"  [WARNING] Step 2 has significant errors")
    
    return results


def test_step3_st_to_controldesk_rd(step2_results, route="R2"):
    """Test Step 3: (s,t) -> ModelDesk -> ControlDesk RD (round-trip test)."""
    print_section(f"STEP 3: Testing (s,t) -> ModelDesk -> ControlDesk RD (Route {route})")
    
    print("Connecting to ModelDesk and ControlDesk...")
    
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("[SKIP] No ModelDesk project open")
            return None
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("[SKIP] No experiment active")
            return None
        
        ts = exp.TrafficScenario
        if ts is None:
            print("[SKIP] No traffic scenario")
            return None
        
        print("[OK] Connected to ModelDesk")
        
        # Connect to ControlDesk
        try:
            cd = ControlDeskApp(
                prog_id="ControlDeskNG.Application",
                outer_platform_name="Platform",
                inner_platform_name="Platform_2"
            ).connect()
            print("[OK] Connected to ControlDesk")
        except Exception as e:
            print(f"[SKIP] Could not connect to ControlDesk: {e}")
            return None
        
        # Clear existing fellows
        print("\nClearing existing fellows...")
        try:
            dutils.clear_collection(ts.Fellows)
        except:
            pass
        
        results = {}
        
        # Test each coordinate
        for idx, (name, step2_data) in enumerate(step2_results.items()):
            actual_s, actual_t = step2_data['actual_s_t']
            expected_rd = step2_data['rd']
            
            print(f"\n{name}: Testing (s={actual_s:.1f}, t={actual_t:.3f}) on route {route}")
            
            # Create fellow with this (s,t)
            F = ts.Fellows.Add()
            try:
                F.Name = f"Test_{name}"
            except:
                F.Name = f"Test_{idx}"
            
            # Configure sequence
            seqs = F.Sequences
            dutils.clear_collection(seqs)
            S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
            segs = dutils.ensure_two_segments(S1)
            
            # Set (s,t)
            dutils.configure_seg0_absolute_pose(segs, s=float(actual_s), t=float(actual_t))
            
            # Set route
            try:
                route_sel = S1.Route
                route_sel.UseExternal = False
                route_sel.Direction = 0  # Direct
                route_sel.Activate(route)
                print(f"  Set route to {route}")
            except Exception as e:
                print(f"  [WARNING] Could not set route: {e}")
            
            # Configure segment 1
            try:
                dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            except:
                pass
            
            try:
                dutils.make_endless_transition(segs)
            except:
                pass
            
            results[name] = {
                's_t': (actual_s, actual_t),
                'expected_rd': expected_rd,
                'fellow_name': F.Name
            }
        
        # Save, download and reset
        print("\nSaving scenario...")
        try:
            ts.Save()
            print("[OK] Scenario saved")
        except Exception as e:
            print(f"[WARNING] Save error: {e}")
        
        print("\nDownloading scenario and resetting...")
        try:
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
            time.sleep(1.0)
            print("[OK] Scenario downloaded and reset")
        except Exception as e:
            print(f"[WARNING] Download/reset error: {e}")
        
        # Step simulation a few times
        print("\nStepping simulation to initialize positions...")
        try:
            for i in range(10):
                cd.advance_simulation_step()
                time.sleep(0.1)
            time.sleep(1.0)
            print("[OK] Simulation stepped")
        except Exception as e:
            print(f"[WARNING] Could not step simulation: {e}")
        
        # Read back positions
        print("\nReading back positions from ControlDesk...")
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        
        try:
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            # Read positions for each fellow
            for idx, (name, data) in enumerate(results.items()):
                if idx < len(x_arr) and idx < len(y_arr):
                    actual_rd_x = float(x_arr[idx])
                    actual_rd_y = float(y_arr[idx])
                    
                    expected_rd_x, expected_rd_y = data['expected_rd']
                    
                    # Calculate error
                    error = math.sqrt((actual_rd_x - expected_rd_x)**2 + (actual_rd_y - expected_rd_y)**2)
                    
                    data['actual_rd'] = (actual_rd_x, actual_rd_y)
                    data['error'] = error
                    
                    print(f"\n{name}:")
                    print(f"  (s,t) set:      ({data['s_t'][0]:7.1f}, {data['s_t'][1]:6.3f})")
                    print(f"  Expected RD:    ({expected_rd_x:8.3f}, {expected_rd_y:8.3f})")
                    print(f"  Actual RD:      ({actual_rd_x:8.3f}, {actual_rd_y:8.3f})")
                    print(f"  Error:          {error:.3f} m")
                    
                    if error > 10.0:
                        print(f"  [ERROR] Large mismatch detected!")
                else:
                    print(f"\n{name}: [ERROR] Could not read position from array")
            
            # Summary
            errors = [r['error'] for r in results.values() if 'error' in r]
            if errors:
                avg_error = sum(errors) / len(errors)
                max_error = max(errors)
                min_error = min(errors)
                
                print(f"\n[SUMMARY] Step 3 (s,t -> ControlDesk RD) on route {route}:")
                print(f"  Average error: {avg_error:.3f} m")
                print(f"  Max error:    {max_error:.3f} m")
                print(f"  Min error:    {min_error:.3f} m")
                
                if avg_error < 1.0:
                    print(f"  [OK] Step 3 is working correctly")
                elif avg_error < 10.0:
                    print(f"  [WARNING] Step 3 has moderate errors")
                else:
                    print(f"  [ERROR] Step 3 has large errors - this is likely the bug!")
        
        except Exception as e:
            print(f"[ERROR] Could not read positions: {e}")
            import traceback
            traceback.print_exc()
        
        return results
        
    except Exception as e:
        print(f"[ERROR] Step 3 test failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_step4_route_comparison(step2_results):
    """Test Step 4: Compare R1 vs R2 routes at same (s,t)."""
    print_section("STEP 4: Testing Route-Specific Behavior (R1 vs R2)")
    
    # Test with first coordinate
    name = list(step2_results.keys())[0]
    step2_data = step2_results[name]
    actual_s, actual_t = step2_data['actual_s_t']
    
    print(f"Testing same (s={actual_s:.1f}, t={actual_t:.3f}) on both R1 and R2 routes")
    print("This will show if routes have different coordinate systems")
    
    # Test on R1
    print("\n--- Testing on R1 (pit) ---")
    r1_results = test_step3_st_to_controldesk_rd({name: step2_data}, route="R1")
    
    # Test on R2
    print("\n--- Testing on R2 (lap) ---")
    r2_results = test_step3_st_to_controldesk_rd({name: step2_data}, route="R2")
    
    # Compare
    if r1_results and r2_results and name in r1_results and name in r2_results:
        r1_rd = r1_results[name].get('actual_rd')
        r2_rd = r2_results[name].get('actual_rd')
        
        if r1_rd and r2_rd:
            route_diff = math.sqrt((r1_rd[0] - r2_rd[0])**2 + (r1_rd[1] - r2_rd[1])**2)
            
            print_section("Route Comparison Results")
            print(f"Same (s,t) = ({actual_s:.1f}, {actual_t:.3f})")
            print(f"R1 (pit) -> RD: ({r1_rd[0]:8.3f}, {r1_rd[1]:8.3f})")
            print(f"R2 (lap) -> RD: ({r2_rd[0]:8.3f}, {r2_rd[1]:8.3f})")
            print(f"Difference:     {route_diff:.3f} m")
            
            if route_diff > 1.0:
                print(f"\n[FINDING] Routes R1 and R2 have different coordinate systems!")
                print(f"  The same (s,t) produces different RD coordinates on different routes.")
                print(f"  This confirms that routes have independent s-coordinate origins.")
            
            # Check which route matches expected better
            expected_rd = step2_data['rd']
            r1_error = math.sqrt((r1_rd[0] - expected_rd[0])**2 + (r1_rd[1] - expected_rd[1])**2)
            r2_error = math.sqrt((r2_rd[0] - expected_rd[0])**2 + (r2_rd[1] - expected_rd[1])**2)
            
            print(f"\nComparison with expected RD ({expected_rd[0]:.3f}, {expected_rd[1]:.3f}):")
            print(f"  R1 error: {r1_error:.3f} m")
            print(f"  R2 error: {r2_error:.3f} m")
            
            if r1_error < r2_error:
                print(f"  [FINDING] R1 matches better (by {r2_error - r1_error:.3f} m)")
            elif r2_error < r1_error:
                print(f"  [FINDING] R2 matches better (by {r1_error - r2_error:.3f} m)")
            else:
                print(f"  [FINDING] Both routes have similar errors")


def main():
    """Main debugging function."""
    print("="*80)
    print("Coordinate Transformation Bug Isolation")
    print("="*80)
    print("\nThis script will test each transformation step independently:")
    print("  1. XODR -> RD (coordinate transform)")
    print("  2. RD (x,y) -> (s,t) (projection)")
    print("  3. (s,t) -> ModelDesk -> ControlDesk RD (round-trip)")
    print("  4. Route comparison (R1 vs R2)")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running (for steps 3-4)")
    print("="*80)
    
    # Load coordinate transform
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    coordinate_transform = None
    if transform_path.exists():
        try:
            coordinate_transform = load_transform(str(transform_path))
            print(f"\n[OK] Loaded coordinate transform from {transform_path.name}")
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
            print(f"[OK] Loaded road index from {rd_path.name}")
        except Exception as e:
            print(f"[WARNING] Could not load road index: {e}")
            import traceback
            traceback.print_exc()
    else:
        print(f"[WARNING] RD file not found: {rd_path}")
    
    # Run tests
    step1_results = test_step1_xodr_to_rd(coordinate_transform)
    
    if step1_results:
        step2_results = test_step2_rd_to_st(road_index, step1_results)
        
        if step2_results:
            # Test on R2 first (default route)
            step3_results = test_step3_st_to_controldesk_rd(step2_results, route="R2")
            
            # Compare routes
            test_step4_route_comparison(step2_results)
    
    print_section("Debugging Complete")
    print("\nSummary of findings:")
    print("  - Check Step 1 errors: Should be < 0.001m (coordinate transform)")
    print("  - Check Step 2 errors: Should be < 1.0m for s, < 0.1m for t (projection)")
    print("  - Check Step 3 errors: Should be < 1.0m (round-trip)")
    print("  - Check Step 4: Routes should show different coordinate systems")
    print("\nIf Step 3 has large errors (>10m), the bug is likely:")
    print("  - Route coordinate system mismatch (s,t computed for wrong route)")
    print("  - Or projection using wrong road geometry")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
