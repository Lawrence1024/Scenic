#!/usr/bin/env python3
"""
Test script to verify if expected (s,t) coordinates work correctly on R2 route.

This script:
1. Takes expected (s,t) values from EXPECTED_COORDINATES
2. Places fellows at those (s,t) on R2 route
3. Reads back RD coordinates from ControlDesk
4. Compares with expected RD coordinates

This will verify if the expected (s,t) values are correct for R2 route.
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

# Import expected coordinates from debug script
from debug_coordinate_transformation import EXPECTED_COORDINATES


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
        return None


def create_fellow_at_st(ts, fellow_name, s_val, t_val, route_name="R2"):
    """Create a fellow at specified (s, t) coordinate."""
    # Create new fellow
    F = ts.Fellows.Add()
    try:
        F.Name = fellow_name
    except Exception as e:
        F.Name = f"Fellow_{ts.Fellows.Count}"
    
    # Configure sequences and segments
    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # Configure segment 0: absolute pose (s, t)
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    
    # Configure segment 1: external control (stationary)
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        pass
    
    # Set endless transition
    try:
        dutils.make_endless_transition(segs)
    except:
        pass
    
    # Set route
    try:
        route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
        route_sel.UseExternal = False
        route_sel.Direction = 0  # Direct
        
        if route_name in [str(x) for x in route_sel.AvailableElements]:
            route_sel.Activate(route_name)
    except Exception as e:
        pass
    
    return F


def read_fellow_positions(cd, num_fellows):
    """Read fellow vehicle positions from ControlDesk."""
    if not cd:
        return None
    
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    
    try:
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        z_arr = cd.get_var(f"{base_path}/z")
        yaw_arr = cd.get_var(f"{base_path}/yaw_deg_out")
        
        positions = []
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
        return None


def main():
    """Main test function."""
    print("="*80)
    print("Testing Expected Coordinates on R2 Route")
    print("="*80)
    print("\nThis script will:")
    print("  1. Place fellows at expected (s,t) values on R2 route")
    print("  2. Read back RD coordinates from ControlDesk")
    print("  3. Compare with expected RD coordinates")
    print("\nThis verifies if the expected (s,t) values are correct for R2 route.")
    print("="*80)
    
    try:
        # Connect to ModelDesk
        app, proj, exp, ts = connect_to_modeldesk()
        
        # Clear existing fellows
        try:
            dutils.clear_collection(ts.Fellows)
            print("\n[OK] Cleared existing fellows")
        except Exception as e:
            print(f"\n[WARNING] Could not clear fellows: {e}")
        
        # Connect to ControlDesk once
        cd = connect_to_controldesk()
        
        # Test both R1 and R2 to see which route matches
        TEST_ROUTES = ["R1", "R2"]
        
        all_results = {}
        
        for route in TEST_ROUTES:
            print("\n" + "="*80)
            print(f"Creating Fellows at Expected (s,t) on {route} Route")
            print("="*80)
            
            # Clear existing fellows
            try:
                dutils.clear_collection(ts.Fellows)
            except:
                pass
            
            fellow_names = []
            for i, (key, expected) in enumerate(EXPECTED_COORDINATES.items()):
                s_val, t_val = expected['expected_s_t']
                fellow_name = f"Fellow_{i+1}"
                fellow_names.append(fellow_name)
                
                print(f"\n   {fellow_name}: (s={s_val:.1f}, t={t_val:.3f})")
                create_fellow_at_st(ts, fellow_name, s_val, t_val, route)
            
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
            if route == TEST_ROUTES[0]:  # Only connect once
                cd = connect_to_controldesk()
            if cd:
                try:
                    time.sleep(2.0)
                    for i in range(5):
                        cd.advance_simulation_step()
                        time.sleep(0.1)
                    time.sleep(1.0)
                except Exception as e:
                    pass
            
            # Read back positions
            actual_positions = read_fellow_positions(cd, len(EXPECTED_COORDINATES))
            all_results[route] = actual_positions
            
            time.sleep(0.5)  # Brief pause between routes
        
        # Save and download
        print("\n" + "="*80)
        print("Saving and Downloading Scenario...")
        print("="*80)
        try:
            ts.Save()
            print("   [OK] Scenario saved")
        except Exception as e:
            print(f"   [WARNING] Could not save: {e}")
        
        try:
            ts.Download()
            print("   [OK] Scenario downloaded")
        except Exception as e:
            print(f"   [WARNING] Could not download: {e}")
        
        # Reset and start maneuver
        print("\n" + "="*80)
        print("Resetting and Starting Maneuver...")
        print("="*80)
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
            print("   [OK] Maneuver reset and started")
        except Exception as e:
            print(f"   [WARNING] Could not reset/start: {e}")
        
        # Step simulation
        cd = connect_to_controldesk()
        if cd:
            print("\n   Stepping simulation to initialize positions...")
            try:
                time.sleep(2.0)
                for i in range(5):
                    cd.advance_simulation_step()
                    time.sleep(0.1)
                time.sleep(1.0)
                print("   [OK] Simulation stepped 5 times")
            except Exception as e:
                print(f"   [WARNING] Could not step: {e}")
        
        # Compare results for both routes
        print("\n" + "="*80)
        print("Comparison: Expected vs Actual RD Coordinates")
        print("="*80)
        
        route_stats = {}
        
        for route in TEST_ROUTES:
            actual_positions = all_results.get(route)
            if not actual_positions:
                continue
            
            print(f"\n{'='*80}")
            print(f"Results for {route} Route:")
            print(f"{'='*80}")
            print(f"\n{'Fellow':<12} {'Expected (s,t)':<20} {'Expected RD':<25} {'Actual RD':<25} {'Difference (m)':<15}")
            print("-" * 110)
            
            differences = []
            for i, (key, expected) in enumerate(EXPECTED_COORDINATES.items()):
                if i >= len(actual_positions):
                    break
                
                exp_s, exp_t = expected['expected_s_t']
                exp_rd_x, exp_rd_y = expected['expected_rd']
                actual = actual_positions[i]
                act_rd_x, act_rd_y = actual['x'], actual['y']
                
                diff_x = act_rd_x - exp_rd_x
                diff_y = act_rd_y - exp_rd_y
                diff_dist = (diff_x**2 + diff_y**2)**0.5
                differences.append(diff_dist)
                
                print(f"{key:<12} (s={exp_s:5.1f}, t={exp_t:5.2f})  "
                      f"({exp_rd_x:7.2f}, {exp_rd_y:7.2f})  "
                      f"({act_rd_x:7.2f}, {act_rd_y:7.2f})  "
                      f"{diff_dist:7.2f}")
            
            # Calculate statistics
            if differences:
                import numpy as np
                mean_diff = np.mean(differences)
                max_diff = np.max(differences)
                min_diff = np.min(differences)
                route_stats[route] = {
                    'mean': mean_diff,
                    'max': max_diff,
                    'min': min_diff
                }
                
                print(f"\n   Statistics for {route}:")
                print(f"      Mean difference: {mean_diff:.2f} m")
                print(f"      Max difference:  {max_diff:.2f} m")
                print(f"      Min difference:  {min_diff:.2f} m")
        
        # Final comparison
        print("\n" + "="*80)
        print("Route Comparison Summary")
        print("="*80)
        
        for route, stats in route_stats.items():
            route_name = "R1 (pit)" if route == "R1" else "R2 (lap)"
            mean = stats['mean']
            
            if mean < 1.0:
                status = "[SUCCESS] CORRECT"
            elif mean < 5.0:
                status = "[WARNING] Moderate errors"
            else:
                status = "[ERROR] INCORRECT"
            
            print(f"\n{route_name}:")
            print(f"   Mean error: {mean:.2f} m")
            print(f"   Status: {status}")
        
        # Determine which route matches
        if len(route_stats) == 2:
            r1_mean = route_stats.get('R1', {}).get('mean', float('inf'))
            r2_mean = route_stats.get('R2', {}).get('mean', float('inf'))
            
            print("\n" + "="*80)
            print("CONCLUSION")
            print("="*80)
            
            if r1_mean < r2_mean and r1_mean < 5.0:
                print(f"\n[RESULT] Expected (s,t) values match R1 (pit) route!")
                print(f"   R1 mean error: {r1_mean:.2f}m")
                print(f"   R2 mean error: {r2_mean:.2f}m")
                print(f"   The (s,t) values were computed using R1 route geometry")
            elif r2_mean < r1_mean and r2_mean < 5.0:
                print(f"\n[RESULT] Expected (s,t) values match R2 (lap) route!")
                print(f"   R1 mean error: {r1_mean:.2f}m")
                print(f"   R2 mean error: {r2_mean:.2f}m")
                print(f"   The (s,t) values were computed using R2 route geometry")
            else:
                print(f"\n[RESULT] Expected (s,t) values don't match either route well!")
                print(f"   R1 mean error: {r1_mean:.2f}m")
                print(f"   R2 mean error: {r2_mean:.2f}m")
                print(f"   The (s,t) values may have been computed using wrong/global geometry")
        
        print("\n" + "="*80)
        print("Test Complete")
        print("="*80)
        
        return 0
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())
