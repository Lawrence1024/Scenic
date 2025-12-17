#!/usr/bin/env python3
"""
Test and analyze t-coordinate (lateral deviation) handling in round-trip transformation.

This script investigates:
1. How t-coordinate affects round-trip errors
2. Whether the 0.3× scale factor is correct
3. If t-coordinate handling differs by position or route
4. Contribution of t-coordinate to overall error

Usage:
    python debug_cord_code/test_t_coordinate_analysis.py
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
from scenic.simulators.dspace.geometry.projection import project_world_to_st


# Test coordinates from round-trip test
TEST_COORDINATES = {
    'Fellow_1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
        'expected_t': -1.653,
    },
    'Fellow_2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
        'expected_t': 1.472,
    },
    'Fellow_3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),
        'expected_t': 0.242,
    },
}


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def analyze_t_coordinate_for_coordinate(coordinate_transform, road_index, scenic_xodr, name, expected_t=None):
    """Analyze t-coordinate handling for a single coordinate."""
    print(f"\n{name}: Analyzing t-coordinate")
    print(f"  Starting XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
    
    # Step 1: XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
    else:
        rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
    print(f"  Step 1 (XODR -> RD): ({rd_x:.3f}, {rd_y:.3f})")
    
    # Step 2: Project to get (s, t) - get both road-relative and raw t
    s_road, t_val = project_world_to_st(road_index, (rd_x, rd_y))
    print(f"  Step 2 (RD -> s,t): road-relative s={s_road:.1f}, t={t_val:.3f}")
    
    if expected_t is not None:
        t_error = abs(t_val - expected_t)
        print(f"  T-coordinate: expected={expected_t:.3f}, actual={t_val:.3f}, error={t_error:.3f}m")
    
    # Step 3: Test different t values to see impact on round-trip
    print(f"\n  Testing different t-coordinate values:")
    
    # Get route
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("    [SKIP] No ModelDesk project")
            return None
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("    [SKIP] No experiment")
            return None
        
        from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
        
        params = {}
        track_segment = detect_track_segment((rd_x, rd_y), road_index, params, dutils)
        route_pref = assign_route_for_segment(track_segment) if track_segment else 'Lap'
        route_name = 'R1' if route_pref == 'Pit' else 'R2'
        
        # Get route-specific (s, t)
        s_route, t_val = project_world_to_st_route_specific(road_index, (rd_x, rd_y), route_pref)
        print(f"  Route-specific: s={s_route:.1f}, t={t_val:.3f} for {route_name}")
        
        # Get traffic scenario (reuse existing one)
        ts = exp.TrafficScenario
        if ts is None:
            print("    [SKIP] No traffic scenario")
            return None
        
        # Clear existing fellows
        try:
            dutils.clear_collection(ts.Fellows)
        except:
            pass
        
        # Create fellow once (will reuse for all t values)
        fellow = ts.Fellows.Add()
        fellow.Name = "TestFellow"
        
        # Configure fellow once
        sequences = fellow.Sequences
        if sequences.Count == 0:
            seq = sequences.Add()
        else:
            seq = sequences.Item(0)
        
        segs = dutils.ensure_two_segments(seq)
        
        # Set route once (same for all t values)
        route_sel = seq.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0
        route_sel.Activate(route_name)
        
        # Configure segment 1 motion once
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        dutils.make_endless_transition(segs)
        
        # Test with different t values
        t_test_values = [t_val, 0.0, t_val * 0.5, t_val * 1.5]
        results = []
        
        for test_t in t_test_values:
            print(f"\n    Testing with t={test_t:.3f}:")
            
            # Update only the t value (reuse same fellow)
            dutils.configure_seg0_absolute_pose(segs, s=s_route, t=test_t)
            dutils.configure_seg1_motion(segs, v=0.0, t=test_t)
            
            # Save and download
            try:
                ts.Save()
            except:
                pass
            ts.Download()
            time.sleep(0.5)
            
            # Reset and start
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
            
            # Read back from ControlDesk
            cd = ControlDeskApp(
                prog_id="ControlDeskNG.Application",
                outer_platform_name="Platform",
                inner_platform_name="Platform_2"
            ).connect()
            
            time.sleep(2.0)
            for i in range(20):
                cd.advance_simulation_step()
                time.sleep(0.1)
            time.sleep(1.0)
            
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            rd_x_readback = float(x_arr[0])
            rd_y_readback = float(y_arr[0])
            
            # Convert back to XODR
            if coordinate_transform:
                xodr_readback = apply_inverse_coordinate_transform(coordinate_transform, (rd_x_readback, rd_y_readback))
            else:
                xodr_readback = (rd_x_readback, rd_y_readback)
            
            # Calculate errors
            rd_error = math.sqrt((rd_x - rd_x_readback)**2 + (rd_y - rd_y_readback)**2)
            xodr_error = math.sqrt((scenic_xodr[0] - xodr_readback[0])**2 + (scenic_xodr[1] - xodr_readback[1])**2)
            
            print(f"      Readback RD: ({rd_x_readback:.3f}, {rd_y_readback:.3f})")
            print(f"      Readback XODR: ({xodr_readback[0]:.3f}, {xodr_readback[1]:.3f})")
            print(f"      RD error: {rd_error:.3f}m, XODR error: {xodr_error:.3f}m")
            
            results.append({
                'test_t': test_t,
                'rd_readback': (rd_x_readback, rd_y_readback),
                'xodr_readback': xodr_readback,
                'rd_error': rd_error,
                'xodr_error': xodr_error,
            })
        
        return {
            'name': name,
            'scenic_xodr': scenic_xodr,
            'rd': (rd_x, rd_y),
            's_route': s_route,
            't_original': t_val,
            'route': route_name,
            'results': results,
        }
        
    except Exception as e:
        print(f"    [ERROR] Exception: {e}")
        import traceback
        traceback.print_exc()
        return None


def main():
    """Main function."""
    print("="*80)
    print("T-Coordinate (Lateral Deviation) Analysis")
    print("="*80)
    print("\nThis script analyzes:")
    print("  1. How t-coordinate affects round-trip errors")
    print("  2. Whether different t values produce different errors")
    print("  3. Contribution of t-coordinate to overall error")
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
    
    # Analyze each coordinate
    print_section("Analyzing T-Coordinate Handling")
    
    results = {}
    for name, data in TEST_COORDINATES.items():
        result = analyze_t_coordinate_for_coordinate(
            coordinate_transform,
            road_index,
            data['scenic_xodr'],
            name,
            data.get('expected_t')
        )
        if result:
            results[name] = result
    
    # Summary
    print_section("Summary and Analysis")
    
    if results:
        print("\nT-Coordinate Impact Analysis:")
        for name, result in results.items():
            print(f"\n{name}:")
            print(f"  Original t: {result['t_original']:.3f}")
            print(f"  Route: {result['route']}, s: {result['s_route']:.1f}")
            print(f"  Error comparison:")
            for test_result in result['results']:
                print(f"    t={test_result['test_t']:>6.3f}: XODR error={test_result['xodr_error']:>6.3f}m, RD error={test_result['rd_error']:>6.3f}m")
            
            # Find minimum error
            min_error_result = min(result['results'], key=lambda x: x['xodr_error'])
            print(f"  Best t value: {min_error_result['test_t']:.3f} (error: {min_error_result['xodr_error']:.3f}m)")
            if abs(min_error_result['test_t'] - result['t_original']) > 0.01:
                print(f"  ⚠️  Optimal t differs from original t by {abs(min_error_result['test_t'] - result['t_original']):.3f}m")
        
        print("\n" + "="*80)
        print("Conclusions:")
        print("  - If errors are similar across t values: t-coordinate is not the main issue")
        print("  - If errors vary significantly with t: t-coordinate scaling may be wrong")
        print("  - If optimal t differs from original: t-calculation or scaling needs adjustment")
        print("="*80)
    else:
        print("\n[ERROR] No results to analyze")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
