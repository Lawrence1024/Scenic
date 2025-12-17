#!/usr/bin/env python3
"""
Debug script to analyze coordinate transformation between Scenic and dSPACE.

This script:
1. Connects to ModelDesk and ControlDesk
2. Reads actual fellow vehicle positions from ControlDesk
3. Compares with expected coordinates from Scenic
4. Helps identify transformation discrepancies

Usage:
    python debug_cord_code/debug_coordinate_transformation.py
"""

import sys
import os
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


# Expected coordinates from Scenic (XODR) and what they should be in RD
EXPECTED_COORDINATES = {
    'Fellow_1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
        'expected_rd': (-96.468, -456.652),  # From logs
        'expected_s_t': (0.0, -1.653)  # From logs
    },
    'Fellow_2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
        'expected_rd': (5.082, -273.737),
        'expected_s_t': (279.4, 1.472)
    },
    'Fellow_3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),
        'expected_rd': (192.786, -418.186),
        'expected_s_t': (550.3, 0.242)
    },
    'Fellow_4': {
        'scenic_xodr': (162.256104, -693.627649, 0.0),
        'expected_rd': (163.024, -689.557),
        'expected_s_t': (825.4, 1.315)
    },
    'Fellow_5': {
        'scenic_xodr': (302.064561, -815.646205, 0.0),
        'expected_rd': (300.359, -809.920),
        'expected_s_t': (1105.4, 1.098)
    },
    'Fellow_6': {
        'scenic_xodr': (557.639219, -737.139638, 0.0),
        'expected_rd': (551.963, -732.101),
        'expected_s_t': (1385.1, 2.074)
    },
    'Fellow_7': {
        'scenic_xodr': (599.646200, -466.416118, 0.0),
        'expected_rd': (593.789, -464.666),
        'expected_s_t': (1662.8, 1.616)
    }
}


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


def read_fellow_positions(cd, num_fellows=7, wait_for_init=True):
    """Read fellow vehicle positions from ControlDesk."""
    if not cd:
        return None
    
    print("\n" + "="*80)
    print(f"Reading Fellow Positions from ControlDesk (expecting {num_fellows} fellows)...")
    print("="*80)
    
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    
    try:
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        z_arr = cd.get_var(f"{base_path}/z")
        yaw_arr = cd.get_var(f"{base_path}/yaw_deg_out")
        
        print(f"[OK] Successfully read arrays")
        print(f"   x array length: {len(x_arr) if isinstance(x_arr, (list, tuple)) else 'N/A'}")
        print(f"   y array length: {len(y_arr) if isinstance(y_arr, (list, tuple)) else 'N/A'}")
        print(f"   z array length: {len(z_arr) if isinstance(z_arr, (list, tuple)) else 'N/A'}")
        
        # Debug: Print ALL array values to find where fellows actually are
        print(f"\n   [DEBUG] Scanning all array values for non-zero positions:")
        if isinstance(x_arr, (list, tuple)) and len(x_arr) > 0:
            found_count = 0
            for i in range(min(30, len(x_arr))):  # Check all 30 slots
                x_val = x_arr[i] if i < len(x_arr) else None
                y_val = y_arr[i] if i < len(y_arr) else None
                if x_val is not None and y_val is not None:
                    x_float = float(x_val)
                    y_float = float(y_val)
                    # Check if position is non-zero (more than 0.1m away from origin)
                    if abs(x_float) > 0.1 or abs(y_float) > 0.1:
                        print(f"      Array[{i}]: x={x_float:8.2f}, y={y_float:8.2f}, z={z_arr[i] if i < len(z_arr) else 'N/A'}")
                        found_count += 1
            if found_count == 0:
                print(f"      No non-zero positions found in any of the {len(x_arr)} array slots")
            else:
                print(f"      Found {found_count} non-zero positions")
        
        # Check if positions are initialized (not all zeros)
        all_zero = True
        if isinstance(x_arr, (list, tuple)) and len(x_arr) > 0:
            for i in range(min(num_fellows, len(x_arr))):
                if x_arr[i] is not None and abs(float(x_arr[i])) > 0.01:
                    all_zero = False
                    break
        
        if all_zero and wait_for_init:
            print("\n[WARNING] All positions are zero - vehicles may not be initialized yet")
            print("   This could mean:")
            print("   - Vehicles haven't been spawned in the simulation yet")
            print("   - Simulation needs to be started/stepped to initialize positions")
            print("   - Arrays are not yet populated by the plant model")
            print("\n   Try running the simulation for a few steps, then re-run this script")
        
        positions = []
        # ControlDesk arrays are 0-indexed: FellowTrailer[0] = array[0], FellowTrailer[1] = array[1], etc.
        for i in range(num_fellows):
            if isinstance(x_arr, (list, tuple)) and i < len(x_arr):
                x = float(x_arr[i]) if x_arr[i] is not None else 0.0
                y = float(y_arr[i]) if i < len(y_arr) and y_arr[i] is not None else 0.0
                z = float(z_arr[i]) if i < len(z_arr) and z_arr[i] is not None else 0.0
                yaw = float(yaw_arr[i]) if i < len(yaw_arr) and yaw_arr[i] is not None else 0.0
                
                positions.append({
                    'index': i,
                    'x': x,
                    'y': y,
                    'z': z,
                    'yaw_deg': yaw
                })
            else:
                positions.append({
                    'index': i,
                    'x': 0.0,
                    'y': 0.0,
                    'z': 0.0,
                    'yaw_deg': 0.0
                })
        
        return positions
        
    except Exception as e:
        print(f"[ERROR] Error reading fellow positions: {e}")
        import traceback
        traceback.print_exc()
        return None


def check_modeldesk_fellow_configuration(ts, num_fellows=7):
    """Check ModelDesk fellow configuration (routes, s, t values)."""
    print("\n" + "="*80)
    print("Checking ModelDesk Fellow Configuration...")
    print("="*80)
    
    try:
        fellows = ts.Fellows
        print(f"   Found {fellows.Count} fellows in ModelDesk")
        
        configurations = []
        # Iterate through fellows by name instead of index
        fellow_names = []
        try:
            # Try to get all fellow names - try 0-indexed first (as we verified works)
            for i in range(fellows.Count):
                try:
                    # Try 0-indexed first
                    fellow = fellows.Item(i)
                    name = fellow.Name if hasattr(fellow, 'Name') else f"Fellow_{i+1}"
                    fellow_names.append((i, name, fellow))
                except:
                    try:
                        # Try 1-indexed
                        fellow = fellows.Item(i + 1)
                        name = fellow.Name if hasattr(fellow, 'Name') else f"Fellow_{i+1}"
                        fellow_names.append((i, name, fellow))
                    except:
                        pass
        except Exception as e:
            print(f"   [DEBUG] Error getting fellow names: {e}")
        
        # If we couldn't get names, try by expected names
        if not fellow_names:
            print(f"   [DEBUG] Trying to access by expected names...")
            for i in range(min(num_fellows, fellows.Count)):
                try:
                    fellow = fellows.Item(f"Fellow_{i+1}")
                    name = f"Fellow_{i+1}"
                    fellow_names.append((i, name, fellow))
                except:
                    pass
        
        print(f"   [DEBUG] Found {len(fellow_names)} accessible fellows")
        
        for i, name, fellow in fellow_names[:num_fellows]:
            try:
                # Get sequence and route
                seqs = fellow.Sequences
                if seqs.Count > 0:
                    # Try 1-indexed first
                    try:
                        seq = seqs.Item(1)
                    except:
                        seq = seqs.Item(0)  # Fall back to 0-indexed
                    route = seq.Route
                    # Try to get route name
                    route_name = "Unknown"
                    try:
                        if hasattr(route, 'ActiveElement') and hasattr(route.ActiveElement, 'Name'):
                            route_name = route.ActiveElement.Name
                        # Also try to list available routes for debugging
                        if hasattr(route, 'AvailableElements'):
                            available = list(route.AvailableElements)
                            available_names = [str(x) for x in available]
                            print(f"      [DEBUG] Available routes: {available_names}")
                    except Exception as e:
                        print(f"      [DEBUG] Error reading route: {e}")
                    
                    # Get segments and s, t values
                    segs = seq.Segments
                    s_val = None
                    t_val = None
                    
                    if segs.Count > 0:
                        # Try 0-indexed first (as we verified in test script)
                        try:
                            seg0 = segs.Item(0)
                        except:
                            seg0 = segs.Item(1)  # Fall back to 1-indexed
                        
                        # Read s value using EXACT same path as set_activity_constant
                        # Path: LongitudinalType -> ActiveElement -> SourceType -> ActiveElement -> Constant
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
                        
                        # Read t value using EXACT same path
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
                    
                    configurations.append({
                        'name': name,
                        'route': route_name,
                        's': s_val,
                        't': t_val
                    })
                    
                    print(f"   {name}: Route={route_name}, s={s_val}, t={t_val}")
                else:
                    configurations.append({
                        'name': name,
                        'route': 'No sequences',
                        's': None,
                        't': None
                    })
                    
            except Exception as e:
                print(f"   Error reading Fellow {name}: {e}")
                configurations.append({
                    'name': name,
                    'route': 'Error',
                    's': None,
                    't': None
                })
        
        return configurations
        
    except Exception as e:
        print(f"[ERROR] Error checking ModelDesk configuration: {e}")
        import traceback
        traceback.print_exc()
        return None


def verify_transformation_chain(coordinate_transform, scenic_xodr, expected_rd):
    """Verify the transformation chain by doing forward and inverse transforms."""
    if not coordinate_transform:
        return None
    
    try:
        # Forward: XODR -> RD
        forward_rd = apply_coordinate_transform(coordinate_transform, scenic_xodr)
        
        # Inverse: RD -> XODR
        inverse_xodr = apply_inverse_coordinate_transform(coordinate_transform, expected_rd)
        
        # Check consistency
        forward_error = ((forward_rd[0] - expected_rd[0])**2 + (forward_rd[1] - expected_rd[1])**2)**0.5
        inverse_error = ((inverse_xodr[0] - scenic_xodr[0])**2 + (inverse_xodr[1] - scenic_xodr[1])**2)**0.5
        
        return {
            'forward_rd': forward_rd,
            'inverse_xodr': inverse_xodr,
            'forward_error': forward_error,
            'inverse_error': inverse_error
        }
    except Exception as e:
        return {'error': str(e)}


def check_route_assignment_issues(md_configs):
    """Check if route assignment (R1 vs R2) might be causing coordinate issues."""
    print("\n" + "="*80)
    print("Route Assignment Analysis")
    print("="*80)
    
    if not md_configs:
        print("   [WARNING] No ModelDesk configurations available")
        return
    
    route_counts = {}
    for cfg in md_configs:
        route = cfg.get('route', 'Unknown')
        route_counts[route] = route_counts.get(route, 0) + 1
    
    print(f"   Route distribution:")
    for route, count in route_counts.items():
        print(f"      {route}: {count} fellow(s)")
    
    # Check for potential issues
    if 'R1' in route_counts and 'R2' in route_counts:
        print(f"\n   [WARNING] Mixed routes detected: R1 (pit) and R2 (lap)")
        print(f"      This is normal if some fellows are on pit lane")
    elif 'R1' in route_counts and route_counts['R1'] == len(md_configs):
        print(f"\n   [WARNING] All fellows on R1 (pit lane)")
        print(f"      This might cause coordinate issues if they should be on main track")
    elif 'R2' in route_counts and route_counts['R2'] == len(md_configs):
        print(f"\n   [OK] All fellows on R2 (lap/main track)")
        print(f"      Route assignment looks correct")


def compare_coordinates(actual_positions, md_configs=None, coordinate_transform=None):
    """Compare actual positions with expected coordinates, showing full transformation chain."""
    print("\n" + "="*80)
    print("Coordinate Comparison Analysis - Full Transformation Chain")
    print("="*80)
    
    if not actual_positions:
        print("[ERROR] No actual positions to compare")
        return
    
    # Create lookup for ModelDesk configs
    md_lookup = {}
    if md_configs:
        for cfg in md_configs:
            md_lookup[cfg['name']] = cfg
    
    print(f"\n{'Fellow':<12} {'XODR':<20} {'-> RD (exp)':<20} {'-> (s,t)':<15} {'MD (s,t)':<15} {'CD RD':<20} {'Diff (m)':<10}")
    print("-" * 130)
    
    for i, expected_key in enumerate(['Fellow_1', 'Fellow_2', 'Fellow_3', 'Fellow_4', 'Fellow_5', 'Fellow_6', 'Fellow_7']):
        if i >= len(actual_positions):
            break
        
        expected = EXPECTED_COORDINATES[expected_key]
        actual = actual_positions[i]
        
        # Get transformation chain
        scenic_xodr = expected['scenic_xodr']
        exp_rd = expected['expected_rd']
        exp_s_t = expected['expected_s_t']
        
        # Get ModelDesk (s,t) if available
        md_s, md_t = None, None
        if expected_key in md_lookup:
            md_s = md_lookup[expected_key].get('s')
            md_t = md_lookup[expected_key].get('t')
        
        # Actual ControlDesk RD
        act_rd_x, act_rd_y = actual['x'], actual['y']
        
        # Calculate difference
        exp_rd_x, exp_rd_y = exp_rd
        diff_dist = ((act_rd_x - exp_rd_x)**2 + (act_rd_y - exp_rd_y)**2)**0.5
        
        # Format output
        xodr_str = f"({scenic_xodr[0]:6.2f},{scenic_xodr[1]:7.2f})"
        rd_exp_str = f"({exp_rd_x:6.2f},{exp_rd_y:7.2f})"
        st_exp_str = f"({exp_s_t[0]:5.1f},{exp_s_t[1]:5.2f})"
        st_md_str = f"({md_s:5.1f},{md_t:5.2f})" if md_s is not None and md_t is not None else "(N/A)"
        rd_act_str = f"({act_rd_x:6.2f},{act_rd_y:7.2f})"
        
        print(f"{expected_key:<12} {xodr_str:<20} {rd_exp_str:<20} {st_exp_str:<15} {st_md_str:<15} {rd_act_str:<20} {diff_dist:7.2f}")
        
        # Show detailed analysis if there's a significant mismatch
        if diff_dist > 1.0:
            print(f"  [WARNING] MISMATCH DETECTED:")
            print(f"     Expected RD: ({exp_rd_x:.3f}, {exp_rd_y:.3f})")
            print(f"     Actual RD:   ({act_rd_x:.3f}, {act_rd_y:.3f})")
            print(f"     Difference:  {diff_dist:.3f} m")
            
            # Check if ModelDesk (s,t) matches expected
            if md_s is not None and md_t is not None:
                s_diff = abs(md_s - exp_s_t[0])
                t_diff = abs(md_t - exp_s_t[1])
                if s_diff > 0.1 or t_diff > 0.01:
                    print(f"     ModelDesk (s,t) mismatch: expected ({exp_s_t[0]:.1f}, {exp_s_t[1]:.3f}), got ({md_s:.1f}, {md_t:.3f})")
            
            # Try inverse transform to see what XODR coordinate the actual RD would map to
            if coordinate_transform:
                try:
                    inv_xodr = apply_inverse_coordinate_transform(coordinate_transform, (act_rd_x, act_rd_y))
                    print(f"     Inverse transform: Actual RD -> XODR ({inv_xodr[0]:.3f}, {inv_xodr[1]:.3f})")
                    print(f"     Original XODR:     ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
                    inv_diff = ((inv_xodr[0] - scenic_xodr[0])**2 + (inv_xodr[1] - scenic_xodr[1])**2)**0.5
                    print(f"     XODR difference:   {inv_diff:.3f} m")
                except Exception as e:
                    print(f"     Could not compute inverse transform: {e}")
    
    # Summary statistics
    print("\n" + "-" * 130)
    differences = []
    s_t_mismatches = []
    for i, expected_key in enumerate(['Fellow_1', 'Fellow_2', 'Fellow_3', 'Fellow_4', 'Fellow_5', 'Fellow_6', 'Fellow_7']):
        if i >= len(actual_positions):
            break
        expected = EXPECTED_COORDINATES[expected_key]
        actual = actual_positions[i]
        exp_rd_x, exp_rd_y = expected['expected_rd']
        act_rd_x, act_rd_y = actual['x'], actual['y']
        diff_dist = ((act_rd_x - exp_rd_x)**2 + (act_rd_y - exp_rd_y)**2)**0.5
        differences.append(diff_dist)
        
        # Check ModelDesk (s,t) mismatch
        if expected_key in md_lookup:
            md_s = md_lookup[expected_key].get('s')
            md_t = md_lookup[expected_key].get('t')
            if md_s is not None and md_t is not None:
                exp_s, exp_t = expected['expected_s_t']
                s_diff = abs(md_s - exp_s)
                t_diff = abs(md_t - exp_t)
                if s_diff > 0.1 or t_diff > 0.01:
                    s_t_mismatches.append((expected_key, s_diff, t_diff))
    
    if differences:
        import numpy as np
        mean_diff = np.mean(differences)
        max_diff = np.max(differences)
        min_diff = np.min(differences)
        print(f"\nDistance Statistics (Expected RD vs Actual RD):")
        print(f"   Mean difference: {mean_diff:.2f} m")
        print(f"   Max difference:  {max_diff:.2f} m")
        print(f"   Min difference:  {min_diff:.2f} m")
        
        if s_t_mismatches:
            print(f"\n[WARNING] ModelDesk (s,t) Mismatches:")
            for name, s_diff, t_diff in s_t_mismatches:
                print(f"   {name}: s_diff={s_diff:.2f}m, t_diff={t_diff:.3f}m")
        
        # Diagnostic suggestions
        if mean_diff > 5.0:
            print(f"\n[DIAGNOSTIC] SUGGESTIONS:")
            print(f"   - Large mismatch (>5m) suggests coordinate transformation issue")
            print(f"   - Check if transform file is correct and up-to-date")
            print(f"   - Verify RD file matches the XODR file")
            print(f"   - Check if route assignment (R1/R2) is correct")
        elif mean_diff > 1.0:
            print(f"\n[DIAGNOSTIC] SUGGESTIONS:")
            print(f"   - Moderate mismatch (1-5m) suggests projection or route issue")
            print(f"   - Check if (s,t) values in ModelDesk match expected")
            print(f"   - Verify route assignment (R1=pit, R2=lap) is correct")
            print(f"   - Check if t-coordinate scaling (0.3×) is appropriate")


def download_and_reset(exp, ts, step_simulation=False):
    """Download scenario to simulator and reset maneuver."""
    print("\n" + "="*80)
    print("Downloading Scenario and Resetting Maneuver...")
    print("="*80)
    
    try:
        # Download All to simulator
        print("   Downloading scenario to simulator...")
        ts.Download()
        print("   [OK] Download complete")
        
        # Reset maneuver
        print("   Resetting maneuver...")
        mc = exp.ManeuverControl
        try:
            mc.Stop()
        except:
            pass
        import time
        time.sleep(0.2)
        mc.Reset()
        time.sleep(0.2)
        print("   [OK] Reset complete")
        
        # Start maneuver (but don't run simulation)
        # Start(False) means start but don't auto-run
        print("   Starting maneuver...")
        try:
            mc.Start(False)  # False = start but don't auto-run
            time.sleep(1.0)  # Wait longer for initialization
            print("   [OK] Maneuver started")
        except Exception as e:
            print(f"   [WARNING] Could not start maneuver: {e}")
            print("   (This is okay for debugging - positions should still be visible)")
        
        # Optionally step simulation a few times to initialize positions
        if step_simulation:
            print("\n   Stepping simulation a few times to initialize positions...")
            try:
                # Connect to ControlDesk for stepping
                cd = connect_to_controldesk()
                if cd:
                    import time
                    timestep = 0.1  # Default timestep
                    print("      Waiting 2 seconds for simulation to initialize...")
                    time.sleep(2.0)  # Wait for simulation to initialize
                    for i in range(10):  # Step more times
                        try:
                            cd.advance_simulation_step()
                            time.sleep(timestep * 0.3)
                            if (i + 1) % 2 == 0:  # Print every 2 steps
                                print(f"      Step {i+1}/10 completed")
                        except Exception as step_err:
                            print(f"      [WARNING] Step {i+1} failed: {step_err}")
                            time.sleep(timestep)
                    print("   [OK] Simulation stepped 10 times")
                    print("      Waiting 1 more second for arrays to update...")
                    time.sleep(1.0)  # Final wait for arrays to update
                else:
                    print("   [WARNING] Could not connect to ControlDesk for stepping")
            except Exception as e:
                print(f"   [WARNING] Could not step simulation: {e}")
                import traceback
                traceback.print_exc()
        
    except Exception as e:
        print(f"[ERROR] Error during download/reset: {e}")
        import traceback
        traceback.print_exc()


def main():
    """Main debugging function."""
    print("="*80)
    print("dSPACE Coordinate Transformation Debugging Tool")
    print("="*80)
    print("\nThis script will:")
    print("  1. Connect to ModelDesk and ControlDesk")
    print("  2. Check ModelDesk fellow configuration (routes, s, t values)")
    print("  3. Read actual fellow positions from ControlDesk")
    print("  4. Compare with expected coordinates")
    print("\nMake sure:")
    print("  - ModelDesk is open with a project and experiment active")
    print("  - ControlDesk is running with the same experiment loaded")
    print("  - Scenario has been downloaded and maneuver reset")
    print("="*80)
    
    # Connect to ModelDesk
    try:
        app, proj, exp, ts = connect_to_modeldesk()
    except Exception as e:
        print(f"\n[ERROR] Failed to connect to ModelDesk: {e}")
        return 1
    
    # Download and reset to ensure latest configuration
    # Set step_simulation=True to initialize vehicle positions
    download_and_reset(exp, ts, step_simulation=True)
    
    # Check ModelDesk configuration
    md_configs = check_modeldesk_fellow_configuration(ts, num_fellows=7)
    
    # Check route assignment
    check_route_assignment_issues(md_configs)
    
    # Connect to ControlDesk
    cd = connect_to_controldesk()
    if not cd:
        print("\n[WARNING] Cannot read positions without ControlDesk connection")
        return 1
    
    # Read actual positions
    actual_positions = read_fellow_positions(cd, num_fellows=7)
    
    # Verify transformation chain if transform is available
    transform = None
    try:
        map_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        if map_path.exists():
            transform = load_transform(str(map_path))
    except Exception:
        pass
    
    if transform and actual_positions:
        print("\n" + "="*80)
        print("Transformation Chain Verification")
        print("="*80)
        # Test with first fellow
        if len(actual_positions) > 0 and 'Fellow_1' in EXPECTED_COORDINATES:
            expected = EXPECTED_COORDINATES['Fellow_1']
            scenic_xodr = expected['scenic_xodr'][:2]  # (x, y)
            expected_rd = expected['expected_rd']
            
            result = verify_transformation_chain(transform, scenic_xodr, expected_rd)
            if result and 'error' not in result:
                print(f"   Forward transform (XODR -> RD):")
                print(f"      Input:  ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
                print(f"      Output: ({result['forward_rd'][0]:.3f}, {result['forward_rd'][1]:.3f})")
                print(f"      Expected: ({expected_rd[0]:.3f}, {expected_rd[1]:.3f})")
                print(f"      Error: {result['forward_error']:.3f} m")
                print(f"\n   Inverse transform (RD -> XODR):")
                print(f"      Input:  ({expected_rd[0]:.3f}, {expected_rd[1]:.3f})")
                print(f"      Output: ({result['inverse_xodr'][0]:.3f}, {result['inverse_xodr'][1]:.3f})")
                print(f"      Expected: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
                print(f"      Error: {result['inverse_error']:.3f} m")
                
                if result['forward_error'] > 1.0 or result['inverse_error'] > 1.0:
                    print(f"\n   [WARNING] Transformation errors > 1m detected - transform may need recalibration")
            elif result and 'error' in result:
                print(f"   [ERROR] Transformation verification failed: {result['error']}")
    
    if actual_positions:
        print("\n" + "="*80)
        print("Actual Fellow Positions from ControlDesk:")
        print("="*80)
        for pos in actual_positions:
            print(f"  FellowTrailer[{pos['index']}]: "
                  f"x={pos['x']:10.3f}, y={pos['y']:10.3f}, z={pos['z']:6.3f}, "
                  f"yaw={pos['yaw_deg']:6.1f}°")
    
    # Compare coordinates
    if actual_positions:
        # Try to load coordinate transform if available
        transform = None
        try:
            map_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
            if map_path.exists():
                transform = load_transform(str(map_path))
                print(f"\n[OK] Loaded coordinate transformation from {map_path}")
                print(f"   Transform type: {transform.get('type', 'unknown')}")
        except Exception as e:
            print(f"\n[WARNING] Could not load coordinate transform: {e}")
        
        compare_coordinates(actual_positions, md_configs=md_configs, coordinate_transform=transform)
    
    print("\n" + "="*80)
    print("Debugging Complete")
    print("="*80)
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

