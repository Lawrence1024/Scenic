#!/usr/bin/env python3
"""
Find XODR coordinates for specific (s, t) coordinates on main racing road (R2).

This script:
1. Places all fellow vehicles at once at their (s, t) coordinates on R2 route
2. Reads the actual ControlDesk RD positions from array indices 0, 1, 2, etc.
3. Transforms back to XODR coordinates
4. Reports the XODR coordinates for each (s, t) position

Usage:
    python create_new_ttl/find_xodr_for_st_coordinates.py
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
    load_transform, apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.utils import legacy as dutils

# Configuration: Generate coordinates from s=200 to s=3500, step 25, t=0
S_START = 200.0
S_END = 4500.0
S_STEP = 5.0
T_VALUE = 0.0
BATCH_SIZE = 30

# Generate all test coordinates
def generate_test_coordinates():
    """Generate all (s, t) coordinates from s_start to s_end with step s_step."""
    coordinates = []
    s = S_START
    while s <= S_END:
        coordinates.append((s, T_VALUE))
        s += S_STEP
    return coordinates


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario."""
    if new_scenario_name is None:
        new_scenario_name = "ST_Coordinate_Test"
    
    try:
        exp.TrafficScenario.SaveAs(new_scenario_name, True)
        exp.ActivateTrafficScenario(new_scenario_name)
        return exp.TrafficScenario
    except Exception as e:
        print(f"[WARNING] Could not copy scenario: {e}")
        return exp.TrafficScenario


def create_fellow_at_st(ts, fellow_name, s_val, t_val, route_name="R2"):
    """Create a fellow vehicle at specified (s, t) coordinate."""
    print(f"\n  Creating fellow '{fellow_name}' at (s={s_val:.1f}, t={t_val:.3f}) on route {route_name}")
    
    # Create fellow
    fellow = ts.Fellows.Add()
    try:
        fellow.Name = fellow_name
    except Exception as e:
        fellow.Name = f"Fellow_{ts.Fellows.Count}"
    
    # Configure sequences and segments
    seqs = fellow.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # Configure segment 0: absolute pose (s, t)
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    
    # Configure segment 1: external control (stationary)
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"    [WARNING] Could not configure segment 1: {e}")
    
    # Make segment 1 endless
    try:
        dutils.make_endless_transition(segs)
    except Exception as e:
        print(f"    [WARNING] Could not set endless transition: {e}")
    
    # Set route
    try:
        route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
        route_sel.UseExternal = False
        route_sel.Direction = 0  # Direct
        
        available = list(route_sel.AvailableElements)
        available_names = [str(x) for x in available]
        
        if route_name in available_names:
            route_sel.Activate(route_name)
            print(f"    [OK] Activated route: {route_name}")
        else:
            print(f"    [WARNING] Route '{route_name}' not found, using first available: {available_names[0] if available_names else 'None'}")
            if available_names:
                route_sel.Activate(available_names[0])
    except Exception as e:
        print(f"    [WARNING] Could not set route: {e}")
    
    return fellow


def read_fellow_position(cd, index=0):
    """Read fellow vehicle position from ControlDesk."""
    if not cd:
        return None
    
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    
    try:
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        z_arr = cd.get_var(f"{base_path}/z")
        
        if isinstance(x_arr, (list, tuple)) and index < len(x_arr):
            x = float(x_arr[index]) if x_arr[index] is not None else 0.0
            y = float(y_arr[index]) if index < len(y_arr) and y_arr[index] is not None else 0.0
            z = float(z_arr[index]) if index < len(z_arr) and z_arr[index] is not None else 0.0
            
            return {
                'x': x,
                'y': y,
                'z': z
            }
        else:
            print(f"    [ERROR] Array index {index} out of range")
            return None
            
    except Exception as e:
        print(f"    [ERROR] Error reading fellow position: {e}")
        import traceback
        traceback.print_exc()
        return None


def test_all_st_coordinates(test_coordinates, coordinate_transform, ts, exp, cd):
    """Test all (s, t) coordinates in batch and return XODR positions."""
    print("\n" + "="*80)
    print(f"Testing {len(test_coordinates)} coordinates in batch on R2")
    print("="*80)
    
    # Clear existing fellows
    try:
        dutils.clear_collection(ts.Fellows)
        print("  [OK] Cleared existing fellows")
    except Exception as e:
        print(f"  [WARNING] Could not clear fellows: {e}")
    
    # Create all fellows at once
    print(f"\n  Creating {len(test_coordinates)} fellows...")
    for i, (s_val, t_val) in enumerate(test_coordinates):
        fellow_name = f"TestFellow_{i}"
        fellow = create_fellow_at_st(ts, fellow_name, s_val, t_val, route_name="R2")
        print(f"    [OK] Created fellow {i}: {fellow_name} at (s={s_val:.1f}, t={t_val:.3f})")
    
    # Save and download
    print("\n  [OK] Saving scenario...")
    try:
        ts.Save()
    except Exception as e:
        print(f"  [WARNING] Save failed: {e}")
    
    print("  [OK] Downloading to simulator...")
    try:
        ts.Download()
        time.sleep(0.5)
    except Exception as e:
        print(f"  [WARNING] Download failed: {e}")
    
    # Reset and start simulation
    print("  [OK] Resetting and starting simulation...")
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
        time.sleep(2.0)
        print("  [OK] Simulation started")
    except Exception as e:
        print(f"  [WARNING] Could not control simulation: {e}")
    
    # Wait for initialization
    print("  [OK] Waiting for simulation to initialize...")
    time.sleep(2.0)
    
    # Step simulation multiple times
    print("  [OK] Stepping simulation (20 steps)...")
    for i in range(20):
        try:
            cd.advance_simulation_step()
            time.sleep(0.1)
        except Exception as e:
            print(f"  [WARNING] Step {i} failed: {e}")
    
    time.sleep(1.0)
    
    # Read all positions from ControlDesk arrays
    print(f"\n  [OK] Reading positions from ControlDesk (indices 0-{len(test_coordinates)-1})...")
    results = []
    
    for i, (s_val, t_val) in enumerate(test_coordinates):
        position = read_fellow_position(cd, index=i)
        
        if position is None:
            print(f"    [ERROR] Failed to read position for fellow {i}")
            results.append(None)
            continue
        
        rd_x = position['x']
        rd_y = position['y']
        rd_z = position['z']
        
        print(f"    [OK] Fellow {i} (s={s_val:.1f}, t={t_val:.3f}): RD ({rd_x:.6f}, {rd_y:.6f}, {rd_z:.6f})")
        
        # Transform back to XODR
        if coordinate_transform:
            scenic_x, scenic_y = apply_inverse_coordinate_transform(
                coordinate_transform, (rd_x, rd_y)
            )
            print(f"         XODR ({scenic_x:.6f}, {scenic_y:.6f})")
            results.append({
                's': s_val,
                't': t_val,
                'rd_x': rd_x,
                'rd_y': rd_y,
                'rd_z': rd_z,
                'xodr_x': scenic_x,
                'xodr_y': scenic_y
            })
        else:
            print(f"         [WARNING] No coordinate transform, using RD as XODR")
            results.append({
                's': s_val,
                't': t_val,
                'rd_x': rd_x,
                'rd_y': rd_y,
                'rd_z': rd_z,
                'xodr_x': rd_x,
                'xodr_y': rd_y
            })
    
    return results


def main():
    """Main function."""
    print("="*80)
    print("FIND XODR COORDINATES FOR (s, t) COORDINATES ON MAIN RACING ROAD (R2)")
    print("="*80)
    print("\nThis script will:")
    print("  1. Place fellow vehicles in batches at their (s, t) coordinates on R2 route")
    print("  2. Read ControlDesk RD positions from array indices 0, 1, 2, etc.")
    print("  3. Transform back to XODR coordinates")
    print("  4. Report results")
    
    # Generate all coordinates
    all_coordinates = generate_test_coordinates()
    total_coords = len(all_coordinates)
    num_batches = (total_coords + BATCH_SIZE - 1) // BATCH_SIZE  # Ceiling division
    
    print(f"\nConfiguration:")
    print(f"  s range: {S_START:.1f} to {S_END:.1f} (step {S_STEP:.1f})")
    print(f"  t value: {T_VALUE:.3f}")
    print(f"  Total coordinates: {total_coords}")
    print(f"  Batch size: {BATCH_SIZE}")
    print(f"  Number of batches: {num_batches}")
    print(f"\nFirst few coordinates:")
    for i, (s, t) in enumerate(all_coordinates[:5]):
        print(f"  {i+1}. (s={s:.1f}, t={t:.3f})")
    if total_coords > 5:
        print(f"  ... and {total_coords - 5} more")
    
    # Load coordinate transform
    print("\n[1] Loading coordinate transform...")
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    if not transform_path.exists():
        print(f"  [ERROR] Transform file not found: {transform_path}")
        return 1
    
    coordinate_transform = load_transform(str(transform_path))
    if coordinate_transform:
        print(f"  [OK] Loaded transform: {coordinate_transform.get('type', 'unknown')}")
    else:
        print(f"  [WARNING] Could not load transform, will use RD as XODR")
        coordinate_transform = None
    
    # Connect to ModelDesk
    print("\n[2] Connecting to ModelDesk...")
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("  [ERROR] No active project. Please open a ModelDesk project first.")
            return 1
        exp = proj.ActiveExperiment
        if exp is None:
            print("  [ERROR] No active experiment. Please activate an experiment.")
            return 1
        print(f"  [OK] Connected to project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
        print(f"  [OK] Active experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
    except Exception as e:
        print(f"  [ERROR] Failed to connect to ModelDesk: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Create scenario copy
    print("\n[3] Creating scenario copy...")
    try:
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="ST_Coordinate_Test")
        print(f"  [OK] Created scenario: {ts.Name if hasattr(ts, 'Name') else 'ST_Coordinate_Test'}")
    except Exception as e:
        print(f"  [WARNING] Could not create scenario copy: {e}")
        ts = exp.TrafficScenario
    
    # Connect to ControlDesk
    print("\n[4] Connecting to ControlDesk...")
    try:
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        print("  [OK] Connected to ControlDesk")
    except Exception as e:
        print(f"  [ERROR] Failed to connect to ControlDesk: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Test all coordinates in batches
    print("\n[5] Testing all coordinates in batches...")

    all_results = []
    for batch_idx in range(num_batches):
        start = batch_idx * BATCH_SIZE
        end = min(start + BATCH_SIZE, total_coords)
        batch_coords = all_coordinates[start:end]

        print("\n" + "=" * 80)
        print(f"Batch {batch_idx+1}/{num_batches}: testing indices {start}..{end-1} "
            f"(count={len(batch_coords)}), BATCH_SIZE={BATCH_SIZE}")
        print("=" * 80)

        batch_results = test_all_st_coordinates(batch_coords, coordinate_transform, ts, exp, cd)
        # filter out None
        batch_results = [r for r in batch_results if r is not None]
        all_results.extend(batch_results)

    results = all_results

    
    # Print summary
    print("\n" + "="*80)
    print("RESULTS SUMMARY")
    print("="*80)
    print("\nXODR coordinates for (s, t) coordinates on R2 (main racing road):")
    print("\n" + "-"*80)
    print(f"{'s':>10} | {'t':>10} | {'XODR X':>15} | {'XODR Y':>15} | {'RD X':>15} | {'RD Y':>15}")
    print("-"*80)
    
    for result in results:
        print(f"{result['s']:>10.1f} | {result['t']:>10.3f} | "
              f"{result['xodr_x']:>15.6f} | {result['xodr_y']:>15.6f} | "
              f"{result['rd_x']:>15.6f} | {result['rd_y']:>15.6f}")
    
    print("-"*80)
    
    # Save results to file
    output_file = Path(__file__).parent / "st_to_xodr_results.txt"
    print(f"\n[6] Saving results to: {output_file}")
    try:
        with open(output_file, 'w') as f:
            f.write("XODR Coordinates for (s, t) Coordinates on R2 (Main Racing Road)\n")
            f.write("="*80 + "\n\n")
            f.write(f"{'s':>10} | {'t':>10} | {'XODR X':>15} | {'XODR Y':>15} | {'RD X':>15} | {'RD Y':>15}\n")
            f.write("-"*80 + "\n")
            for result in results:
                f.write(f"{result['s']:>10.1f} | {result['t']:>10.3f} | "
                       f"{result['xodr_x']:>15.6f} | {result['xodr_y']:>15.6f} | "
                       f"{result['rd_x']:>15.6f} | {result['rd_y']:>15.6f}\n")
        print(f"  [OK] Results saved")
    except Exception as e:
        print(f"  [WARNING] Could not save results: {e}")
    
    print("\n" + "="*80)
    print("COMPLETE")
    print("="*80)
    
    return 0


if __name__ == "__main__":
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print("\n\n[INTERRUPTED] User cancelled")
        sys.exit(1)
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
    finally:
        try:
            pythoncom.CoUninitialize()
        except:
            pass

