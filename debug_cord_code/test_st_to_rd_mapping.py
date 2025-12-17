#!/usr/bin/env python3
"""
Test script to verify (s,t) → RD coordinate mapping in dSPACE.

This script:
1. Connects to ModelDesk
2. Creates a single fellow at a known (s, t) coordinate
3. Downloads scenario to dSPACE
4. Reads back the actual RD coordinates from ControlDesk
5. Compares with expected RD coordinates

This helps verify if the (s,t) → RD mapping is working correctly.
"""

import sys
import os
import time
from pathlib import Path

# Fix encoding for Windows console
if sys.stdout.encoding != 'utf-8':
    sys.stdout.reconfigure(encoding='utf-8')
if sys.stderr.encoding != 'utf-8':
    sys.stderr.reconfigure(encoding='utf-8')

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

import pythoncom
from win32com.client import Dispatch
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
from scenic.simulators.dspace.utils import legacy as dutils


# Test configuration - modify these to test different (s,t) values
# You can test multiple values by creating a list
TEST_S_VALUES = [0.0, 100.0, 500.0, 1000.0, 1500.0]  # Longitudinal positions to test
TEST_T = 0.0      # Lateral deviation (meters)
TEST_ROUTE = "R2"  # Route: "R1" (pit) or "R2" (lap)
TEST_MULTIPLE = False  # Set to True to test multiple s values, False to test single value
TEST_ROUTE_COMPARISON = True  # Set to True to compare R1 vs R2 at same (s,t)


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("="*80)
    print("Connecting to ModelDesk...")
    print("="*80)
    
    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    
    if proj is None:
        raise RuntimeError("Open a ModelDesk project first.")
    
    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("Activate an experiment in ModelDesk.")
    
    ts = exp.TrafficScenario
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")
    
    print("[OK] Connected to ModelDesk")
    print(f"   Project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
    print(f"   Experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
    print(f"   TrafficScenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
    
    return app, proj, exp, ts


def connect_to_controldesk():
    """Connect to ControlDesk COM application."""
    print("\n" + "="*80)
    print("Connecting to ControlDesk...")
    print("="*80)
    
    try:
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        
        print("[OK] Connected to ControlDesk")
        return cd
    except Exception as e:
        print(f"[ERROR] Failed to connect to ControlDesk: {e}")
        print("   Make sure ControlDesk is running with an active experiment")
        return None


def create_test_fellow(ts, s_val, t_val, route_name="R2"):
    """Create a single fellow at specified (s, t) coordinate."""
    print("\n" + "="*80)
    print(f"Creating Test Fellow at (s={s_val:.3f}, t={t_val:.3f}) on route {route_name}")
    print("="*80)
    
    # Clear existing fellows
    try:
        dutils.clear_collection(ts.Fellows)
        print(f"   [OK] Cleared existing fellows")
    except Exception as e:
        print(f"   [WARNING] Could not clear fellows: {e}")
    
    # Create new fellow
    F = ts.Fellows.Add()
    try:
        F.Name = "TestFellow"
        print(f"   [OK] Created fellow: {F.Name}")
    except Exception as e:
        F.Name = f"Fellow_{ts.Fellows.Count}"
        print(f"   [OK] Created fellow with fallback name: {F.Name}")
    
    # Configure sequences and segments
    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # Configure segment 0: absolute pose (s, t)
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    print(f"   [OK] Configured segment 0: s={s_val:.3f}, t={t_val:.3f}")
    
    # Configure segment 1: external control (stationary)
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        print(f"   [OK] Configured segment 1: v=0.0, t=0.0 (stationary)")
    except Exception as e:
        print(f"   [WARNING] Could not configure segment 1: {e}")
    
    # Set endless transition
    try:
        dutils.make_endless_transition(segs)
        print(f"   [OK] Set endless transition")
    except Exception as e:
        print(f"   [WARNING] Could not set endless transition: {e}")
    
    # Set route
    try:
        route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
        route_sel.UseExternal = False
        route_sel.Direction = 0  # Direct
        
        # Check available routes
        available = list(route_sel.AvailableElements)
        available_names = [str(x) for x in available]
        print(f"   Available routes: {available_names}")
        
        if route_name in available_names:
            route_sel.Activate(route_name)
            print(f"   [OK] Activated route: {route_name}")
        else:
            print(f"   [WARNING] Route '{route_name}' not found, using first available: {available_names[0] if available_names else 'None'}")
            if available_names:
                route_sel.Activate(available_names[0])
    except Exception as e:
        print(f"   [WARNING] Could not set route: {e}")
    
    return F


def read_fellow_position(cd, index=0):
    """Read fellow vehicle position from ControlDesk."""
    if not cd:
        return None
    
    print("\n" + "="*80)
    print(f"Reading Fellow Position from ControlDesk (index {index})...")
    print("="*80)
    
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    
    try:
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        z_arr = cd.get_var(f"{base_path}/z")
        yaw_arr = cd.get_var(f"{base_path}/yaw_deg_out")
        
        if isinstance(x_arr, (list, tuple)) and index < len(x_arr):
            x = float(x_arr[index]) if x_arr[index] is not None else 0.0
            y = float(y_arr[index]) if index < len(y_arr) and y_arr[index] is not None else 0.0
            z = float(z_arr[index]) if index < len(z_arr) and z_arr[index] is not None else 0.0
            yaw = float(yaw_arr[index]) if index < len(yaw_arr) and yaw_arr[index] is not None else 0.0
            
            print(f"[OK] Read position from ControlDesk:")
            print(f"   RD coordinates: x={x:10.3f}, y={y:10.3f}, z={z:6.3f}")
            print(f"   Yaw: {yaw:6.1f} degrees")
            
            return {
                'x': x,
                'y': y,
                'z': z,
                'yaw_deg': yaw
            }
        else:
            print(f"[ERROR] Array index {index} out of range or invalid")
            return None
            
    except Exception as e:
        print(f"[ERROR] Error reading fellow position: {e}")
        import traceback
        traceback.print_exc()
        return None


def verify_modeldesk_configuration(ts):
    """Verify the (s,t) configuration in ModelDesk."""
    print("\n" + "="*80)
    print("Verifying ModelDesk Configuration...")
    print("="*80)
    
    try:
        fellows = ts.Fellows
        if fellows.Count == 0:
            print("   [ERROR] No fellows found")
            return None
        
        fellow = fellows.Item(0)
        name = fellow.Name if hasattr(fellow, 'Name') else "Unknown"
        print(f"   Fellow: {name}")
        
        seqs = fellow.Sequences
        if seqs.Count > 0:
            seq = seqs.Item(0)
            
            # Get route
            try:
                route_sel = seq.Route if hasattr(seq, 'Route') else seq.RouteSelection
                route_name = "Unknown"
                if hasattr(route_sel, 'ActiveElement') and hasattr(route_sel.ActiveElement, 'Name'):
                    route_name = route_sel.ActiveElement.Name
                print(f"   Route: {route_name}")
            except Exception as e:
                print(f"   Route: Error reading ({e})")
            
            # Get (s,t) from segment 0
            segs = seq.Segments
            if segs.Count > 0:
                seg0 = segs.Item(0)
                
                s_val = None
                t_val = None
                
                # Read s value
                try:
                    lon_type = seg0.Activity.LongitudinalType
                    ae = lon_type.ActiveElement
                    if hasattr(ae, 'SourceType'):
                        st = ae.SourceType
                        if hasattr(st, 'ActiveElement'):
                            st_ae = st.ActiveElement
                            if hasattr(st_ae, 'Constant'):
                                s_val = st_ae.Constant
                    elif hasattr(ae, 'Constant'):
                        s_val = ae.Constant
                except Exception as e:
                    pass
                
                # Read t value
                try:
                    lat_type = seg0.Activity.LateralType
                    ae = lat_type.ActiveElement
                    if hasattr(ae, 'SourceType'):
                        st = ae.SourceType
                        if hasattr(st, 'ActiveElement'):
                            st_ae = st.ActiveElement
                            if hasattr(st_ae, 'Constant'):
                                t_val = st_ae.Constant
                    elif hasattr(ae, 'Constant'):
                        t_val = ae.Constant
                except Exception as e:
                    pass
                
                print(f"   ModelDesk (s,t): s={s_val}, t={t_val}")
                
                return {
                    'route': route_name,
                    's': s_val,
                    't': t_val
                }
        
        return None
        
    except Exception as e:
        print(f"[ERROR] Error verifying ModelDesk configuration: {e}")
        import traceback
        traceback.print_exc()
        return None


def run_single_test(ts, exp, s_val, t_val, route_name):
    """Run a single test with given (s,t) values."""
    print(f"\n{'='*80}")
    print(f"Testing (s={s_val:.3f}, t={t_val:.3f}) on route {route_name}")
    print(f"{'='*80}")
    
    # Create test fellow
    create_test_fellow(ts, s_val, t_val, route_name)
    
    # Verify ModelDesk configuration
    md_config = verify_modeldesk_configuration(ts)
    
    # Save and download
    try:
        ts.Save()
        ts.Download()
    except Exception as e:
        print(f"   [WARNING] Save/download error: {e}")
    
    # Reset and start maneuver
    try:
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
    except Exception as e:
        print(f"   [WARNING] Maneuver reset error: {e}")
    
    # Step simulation
    cd = connect_to_controldesk()
    if cd:
        try:
            time.sleep(2.0)
            for i in range(5):
                cd.advance_simulation_step()
                time.sleep(0.1)
            time.sleep(1.0)
        except Exception as e:
            print(f"   [WARNING] Step error: {e}")
    
    # Read back position
    actual_pos = read_fellow_position(cd, index=0)
    
    return {
        'input_s': s_val,
        'input_t': t_val,
        'md_config': md_config,
        'actual_rd': actual_pos
    }


def main():
    """Main test function."""
    print("="*80)
    print("(s,t) -> RD Coordinate Mapping Test")
    print("="*80)
    
    if TEST_ROUTE_COMPARISON:
        print(f"\nTest Configuration (Route Comparison):")
        print(f"   Testing R1 vs R2 at same (s,t)")
        print(f"   s (longitudinal): {TEST_S_VALUES[0]:.3f} m")
        print(f"   t (lateral): {TEST_T:.3f} m")
        print(f"   This will prove if R1 and R2 have different coordinate systems")
    elif TEST_MULTIPLE:
        print(f"\nTest Configuration (Multiple s values):")
        print(f"   s values: {TEST_S_VALUES}")
        print(f"   t (lateral): {TEST_T:.3f} m")
        print(f"   Route: {TEST_ROUTE}")
    else:
        print(f"\nTest Configuration (Single value):")
        print(f"   s (longitudinal): {TEST_S_VALUES[0]:.3f} m")
        print(f"   t (lateral): {TEST_T:.3f} m")
        print(f"   Route: {TEST_ROUTE}")
    
    print("="*80)
    
    try:
        # Connect to ModelDesk
        app, proj, exp, ts = connect_to_modeldesk()
        
        results = []
        
        if TEST_ROUTE_COMPARISON:
            # Test R1 and R2 at same (s,t) to compare coordinate systems
            print("\n" + "="*80)
            print("ROUTE COMPARISON TEST: R1 vs R2 at (s=0, t=0)")
            print("="*80)
            print("\nThis test will prove if R1 and R2 have different RD coordinate systems")
            print("by placing a fellow at s=0, t=0 on each route and comparing RD coordinates.")
            print("="*80)
            
            # Test R1
            print("\n>>> Testing R1 (pit lane) at (s=0, t=0)")
            result_r1 = run_single_test(ts, exp, TEST_S_VALUES[0], TEST_T, "R1")
            result_r1['route'] = 'R1'
            results.append(result_r1)
            
            time.sleep(1.0)  # Brief pause between routes
            
            # Test R2
            print("\n>>> Testing R2 (lap/main track) at (s=0, t=0)")
            result_r2 = run_single_test(ts, exp, TEST_S_VALUES[0], TEST_T, "R2")
            result_r2['route'] = 'R2'
            results.append(result_r2)
            
        elif TEST_MULTIPLE:
            # Test multiple s values
            for s_val in TEST_S_VALUES:
                result = run_single_test(ts, exp, s_val, TEST_T, TEST_ROUTE)
                results.append(result)
                time.sleep(0.5)  # Brief pause between tests
        else:
            # Test single value
            result = run_single_test(ts, exp, TEST_S_VALUES[0], TEST_T, TEST_ROUTE)
            results.append(result)
        
        # Summary table
        print("\n" + "="*80)
        print("Test Results Summary")
        print("="*80)
        
        if TEST_ROUTE_COMPARISON:
            # Special format for route comparison
            print(f"\n{'Route':<8} {'s (input)':<12} {'MD s':<12} {'MD t':<12} {'RD x':<12} {'RD y':<12} {'RD z':<10} {'Yaw (deg)':<12}")
            print("-" * 100)
            
            for result in results:
                route = result.get('route', 'Unknown')
                s_in = result['input_s']
                md_s = result['md_config']['s'] if result['md_config'] else 'N/A'
                md_t = result['md_config']['t'] if result['md_config'] else 'N/A'
                rd_x = result['actual_rd']['x'] if result['actual_rd'] else 'N/A'
                rd_y = result['actual_rd']['y'] if result['actual_rd'] else 'N/A'
                rd_z = result['actual_rd']['z'] if result['actual_rd'] else 'N/A'
                yaw = result['actual_rd']['yaw_deg'] if result['actual_rd'] else 'N/A'
                
                print(f"{route:<8} {s_in:<12.1f} {str(md_s):<12} {str(md_t):<12} {str(rd_x):<12} {str(rd_y):<12} {str(rd_z):<10} {str(yaw):<12}")
            
            # Compare R1 vs R2
            if len(results) == 2:
                r1_result = results[0] if results[0].get('route') == 'R1' else results[1]
                r2_result = results[1] if results[0].get('route') == 'R1' else results[0]
                
                if r1_result['actual_rd'] and r2_result['actual_rd']:
                    r1_x, r1_y = r1_result['actual_rd']['x'], r1_result['actual_rd']['y']
                    r2_x, r2_y = r2_result['actual_rd']['x'], r2_result['actual_rd']['y']
                    
                    diff_x = r2_x - r1_x
                    diff_y = r2_y - r1_y
                    diff_dist = (diff_x**2 + diff_y**2)**0.5
                    
                    print("\n" + "="*80)
                    print("ROUTE COMPARISON ANALYSIS")
                    print("="*80)
                    print(f"\nAt (s=0, t=0):")
                    print(f"   R1 (pit) RD coordinates:    ({r1_x:.3f}, {r1_y:.3f})")
                    print(f"   R2 (lap) RD coordinates:    ({r2_x:.3f}, {r2_y:.3f})")
                    print(f"   Difference:                  (Δx={diff_x:.3f}m, Δy={diff_y:.3f}m)")
                    print(f"   Distance between routes:     {diff_dist:.3f} m")
                    
                    if diff_dist > 1.0:
                        print(f"\n[PROOF] R1 and R2 have DIFFERENT coordinate systems!")
                        print(f"   The same (s=0, t=0) maps to different RD coordinates:")
                        print(f"   - R1: ({r1_x:.3f}, {r1_y:.3f})")
                        print(f"   - R2: ({r2_x:.3f}, {r2_y:.3f})")
                        print(f"   - Distance: {diff_dist:.3f} m")
                        print(f"\n   This means:")
                        print(f"   - Each route has its own s-coordinate origin")
                        print(f"   - When projecting RD -> (s,t), you must use the correct route's geometry")
                        print(f"   - The (s,t) values are route-specific, not global")
                    else:
                        print(f"\n[RESULT] R1 and R2 have the SAME coordinate system")
                        print(f"   (s=0, t=0) maps to approximately the same RD coordinates")
        else:
            # Standard format
            print(f"\n{'s (input)':<12} {'MD s':<12} {'MD t':<12} {'RD x':<12} {'RD y':<12} {'RD z':<10} {'Yaw (deg)':<12}")
            print("-" * 90)
            
            for result in results:
                s_in = result['input_s']
                md_s = result['md_config']['s'] if result['md_config'] else 'N/A'
                md_t = result['md_config']['t'] if result['md_config'] else 'N/A'
                rd_x = result['actual_rd']['x'] if result['actual_rd'] else 'N/A'
                rd_y = result['actual_rd']['y'] if result['actual_rd'] else 'N/A'
                rd_z = result['actual_rd']['z'] if result['actual_rd'] else 'N/A'
                yaw = result['actual_rd']['yaw_deg'] if result['actual_rd'] else 'N/A'
                
                print(f"{s_in:<12.1f} {str(md_s):<12} {str(md_t):<12} {str(rd_x):<12} {str(rd_y):<12} {str(rd_z):<10} {str(yaw):<12}")
        
        # Analysis
        if not TEST_ROUTE_COMPARISON:
            print("\n" + "="*80)
            print("Analysis")
            print("="*80)
            
            if len(results) > 1:
                print("\nChecking for linear relationship between s and RD coordinates...")
                valid_results = [r for r in results if r['actual_rd'] is not None]
                if len(valid_results) >= 2:
                    # Calculate differences
                    print("\n   s differences and corresponding RD coordinate differences:")
                    for i in range(1, len(valid_results)):
                        s_diff = valid_results[i]['input_s'] - valid_results[i-1]['input_s']
                        if abs(s_diff) < 0.001:  # Skip if s values are the same
                            continue
                        x_diff = valid_results[i]['actual_rd']['x'] - valid_results[i-1]['actual_rd']['x']
                        y_diff = valid_results[i]['actual_rd']['y'] - valid_results[i-1]['actual_rd']['y']
                        dist_diff = (x_diff**2 + y_diff**2)**0.5
                        
                        print(f"   s: {valid_results[i-1]['input_s']:.1f} -> {valid_results[i]['input_s']:.1f} (Δs={s_diff:.1f}m)")
                        print(f"      RD: ({valid_results[i-1]['actual_rd']['x']:.2f}, {valid_results[i-1]['actual_rd']['y']:.2f}) -> "
                              f"({valid_results[i]['actual_rd']['x']:.2f}, {valid_results[i]['actual_rd']['y']:.2f})")
                        print(f"      ΔRD: (Δx={x_diff:.2f}m, Δy={y_diff:.2f}m, distance={dist_diff:.2f}m)")
                        print(f"      Ratio: |ΔRD|/|Δs| = {dist_diff/abs(s_diff):.3f}")
        
        print("\n" + "="*80)
        print("Test Complete")
        print("="*80)
        print("\nObservations:")
        print("  - If RD coordinates don't change with s, there may be a route/coordinate issue")
        print("  - If |ΔRD|/|Δs| ≈ 1.0, the mapping is approximately correct")
        print("  - If there's a constant offset, check route origin/coordinate system")
        
        return 0
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
