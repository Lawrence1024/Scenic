#!/usr/bin/env python3
"""
Test multiple positions along the pit lane to analyze round-trip error consistency.

This script:
1. Tests multiple positions along Pit Lane1_2 (0 to 883.4m)
2. For each position, performs full round-trip: XODR → RD → (s,t) → ModelDesk → ControlDesk RD → XODR
3. Measures round-trip errors
4. Analyzes if errors are consistent or position-dependent

Goal: Determine if the 3.77m error is consistent across all pit lane positions or varies.
"""

import sys
import os
import time
import math
from pathlib import Path
from collections import defaultdict

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
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform, apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.utils import legacy as dutils


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario for testing."""
    if new_scenario_name is None:
        new_scenario_name = time.strftime("PitLaneMultiPosTest_%Y%m%d_%H%M%S")
    
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


def get_rd_coordinate_at_road_s(road_index, road_name, road_s, t=0.0):
    """Get RD coordinate at a specific road-relative s position.
    
    Args:
        road_index: Road index dict
        road_name: Name of the road (e.g., 'Pit Lane1_2')
        road_s: Road-relative s coordinate (0 to road_length)
        t: Lateral deviation (default 0.0)
        
    Returns:
        (rd_x, rd_y) tuple or None if not found
    """
    try:
        roads = road_index.get('roads', {})
        road_data = roads.get(road_name)
        if not road_data:
            return None
        
        sec_points = road_data.get('sec_points', [])
        if not sec_points or not sec_points[0]:
            return None
        
        points = sec_points[0]  # Get the points list
        
        # Find the segment containing road_s
        for i in range(len(points) - 1):
            x0, y0, s0 = points[i]
            x1, y1, s1 = points[i + 1]
            
            if s0 <= road_s <= s1:
                # Interpolate
                if s1 - s0 < 1e-6:
                    return (x0, y0)
                
                u = (road_s - s0) / (s1 - s0)
                x = x0 + u * (x1 - x0)
                y = y0 + u * (y1 - y0)
                
                # Apply lateral offset (perpendicular to segment)
                dx = x1 - x0
                dy = y1 - y0
                seg_len = math.sqrt(dx*dx + dy*dy)
                if seg_len > 1e-6:
                    # Left normal vector
                    nx = -dy / seg_len
                    ny = dx / seg_len
                    x += nx * t
                    y += ny * t
                
                return (x, y)
        
        # If road_s is beyond the road, return the last point
        if points:
            x, y, s = points[-1]
            return (x, y)
        
        return None
    except Exception as e:
        print(f"      [ERROR] Could not get RD coordinate: {e}")
        return None


def test_position_at_road_s(road_s, road_index, coordinate_transform, ts, cd, exp):
    """Test round-trip for a specific road_s position on pit lane.
    
    Args:
        road_s: Road-relative s coordinate (0 to 883.4m for pit lane)
        road_index: Road index dict
        coordinate_transform: Coordinate transform dict
        ts: TrafficScenario object
        cd: ControlDeskApp object
        exp: Experiment object
        
    Returns:
        dict with test results
    """
    print(f"\n   Testing position at road_s = {road_s:.1f}m")
    
    # Get RD coordinate at this road_s
    rd_coord = get_rd_coordinate_at_road_s(road_index, 'Pit Lane1_2', road_s, t=0.0)
    if not rd_coord:
        print(f"      [ERROR] Could not get RD coordinate at road_s={road_s}")
        return None
    
    rd_x, rd_y = rd_coord
    print(f"      RD coordinate: ({rd_x:.6f}, {rd_y:.6f})")
    
    # Transform to XODR (for comparison)
    if coordinate_transform:
        xodr_coord = apply_inverse_coordinate_transform(coordinate_transform, (rd_x, rd_y))
        print(f"      XODR coordinate: ({xodr_coord[0]:.6f}, {xodr_coord[1]:.6f})")
    else:
        xodr_coord = (rd_x, rd_y)
    
    # Project to route-specific (s,t) for R1 (pit lane)
    try:
        s_route, t_route = project_world_to_st_route_specific(
            road_index,
            (rd_x, rd_y),
            route_preference='Pit'
        )
        print(f"      Route-specific (s,t) for R1: (s={s_route:.1f}, t={t_route:.3f})")
    except Exception as e:
        print(f"      [ERROR] Route-specific projection failed: {e}")
        return None
    
    # Clear and create fellow
    try:
        dutils.clear_collection(ts.Fellows)
    except:
        pass
    
    fellow = ts.Fellows.Add()
    fellow.Name = "Test_PitLane_Position"
    
    seqs = fellow.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # Set (s,t) on R1
    dutils.configure_seg0_absolute_pose(segs, s=float(s_route), t=float(t_route))
    
    # Set route to R1
    route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
    route_sel.UseExternal = False
    if hasattr(route_sel, 'Direction'):
        route_sel.Direction = 0
    route_sel.Activate("R1")
    
    # Configure segment 1
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        dutils.make_endless_transition(segs)
    except Exception as e:
        print(f"      [WARNING] Could not configure segment 1: {e}")
    
    # Save, download, reset, start
    ts.Save()
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
    
    # Step simulation
    for i in range(20):
        cd.advance_simulation_step()
        time.sleep(0.1)
    time.sleep(1.0)
    
    # Read back position
    base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
    try:
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        readback_rd = (float(x_arr[0]), float(y_arr[0]))
    except Exception as e:
        print(f"      [ERROR] Could not read position: {e}")
        return None
    
    print(f"      Readback RD: ({readback_rd[0]:.6f}, {readback_rd[1]:.6f})")
    
    # Transform readback to XODR
    if coordinate_transform:
        readback_xodr = apply_inverse_coordinate_transform(coordinate_transform, readback_rd)
    else:
        readback_xodr = readback_rd
    
    # Calculate error
    error_x = readback_xodr[0] - xodr_coord[0]
    error_y = readback_xodr[1] - xodr_coord[1]
    error_distance = math.sqrt(error_x**2 + error_y**2)
    
    # Project readback to road-relative s to verify position
    readback_road_id = find_road_id_for_position(road_index, readback_rd[0], readback_rd[1])
    readback_road_s = None
    if readback_road_id == 1:  # Pit Lane1_2
        readback_road_s, readback_road_t = project_world_to_st(road_index, readback_rd)
        print(f"      Readback road_s: {readback_road_s:.1f}m (expected: {road_s:.1f}m)")
    
    result = {
        'road_s': road_s,
        'rd_coord': (rd_x, rd_y),
        'xodr_coord': xodr_coord,
        'route_s': s_route,
        'route_t': t_route,
        'readback_rd': readback_rd,
        'readback_xodr': readback_xodr,
        'error_x': error_x,
        'error_y': error_y,
        'error_distance': error_distance,
        'readback_road_s': readback_road_s,
        'readback_road_id': readback_road_id
    }
    
    print(f"      Error: {error_distance:.6f}m")
    
    return result


def test_pitlane_multiple_positions():
    """Test multiple positions along the pit lane."""
    print_section("Testing Multiple Positions on Pit Lane")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Load coordinate transform and road index
        print_section("Loading Coordinate Transform and Road Index")
        
        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        coordinate_transform = None
        if transform_path.exists():
            try:
                coordinate_transform = load_transform(str(transform_path))
                print(f"   [OK] Loaded transform: {coordinate_transform.get('type', 'unknown')}")
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        else:
            print(f"   [WARNING] Transform file not found: {transform_path}")
        
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
                print(f"   [OK] Built road index with {len(road_index.get('roads', {}))} roads")
                
                # Verify pit lane exists
                pit_lane = road_index.get('roads', {}).get('Pit Lane1_2')
                if pit_lane:
                    pit_lane_length = pit_lane.get('length', 0)
                    print(f"   [OK] Pit Lane1_2 length: {pit_lane_length:.1f}m")
                else:
                    print(f"   [ERROR] Pit Lane1_2 not found in road index")
                    return False
            except Exception as e:
                print(f"   [ERROR] Could not build road index: {e}")
                import traceback
                traceback.print_exc()
                return False
        else:
            print(f"   [ERROR] RD file not found: {rd_path}")
            return False
        
        # Connect to ModelDesk
        print_section("Connecting to ModelDesk")
        
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        
        if proj is None:
            print("   [ERROR] Open a ModelDesk project first")
            return False
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("   [ERROR] Activate an experiment in ModelDesk")
            return False
        
        # Create scenario copy
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="PitLaneMultiPosTest")
        print(f"   [OK] Created test scenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
        
        # Connect to ControlDesk
        print_section("Connecting to ControlDesk")
        
        try:
            cd = ControlDeskApp(
                prog_id="ControlDeskNG.Application",
                outer_platform_name="Platform",
                inner_platform_name="Platform_2"
            ).connect()
            print("   [OK] Connected to ControlDesk")
        except Exception as e:
            print(f"   [ERROR] Could not connect to ControlDesk: {e}")
            return False
        
        # Test positions along pit lane
        print_section("Testing Multiple Positions")
        
        pit_lane_length = road_index['roads']['Pit Lane1_2']['length']
        print(f"   Pit lane length: {pit_lane_length:.1f}m")
        
        # Test positions: 0, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800, and end
        test_positions = [0, 50, 100, 150, 200, 250, 300, 400, 500, 600, 700, 800]
        if pit_lane_length > 800:
            test_positions.append(pit_lane_length)
        
        results = []
        
        for road_s in test_positions:
            if road_s > pit_lane_length:
                continue
            
            result = test_position_at_road_s(
                road_s, road_index, coordinate_transform, ts, cd, exp
            )
            
            if result:
                results.append(result)
            else:
                print(f"      [WARNING] Test failed for road_s={road_s}")
        
        # Analyze results
        print_section("Results Analysis")
        
        if not results:
            print("   [ERROR] No successful tests")
            return False
        
        print(f"\n   Tested {len(results)} positions along pit lane")
        
        # Calculate statistics
        errors = [r['error_distance'] for r in results]
        avg_error = sum(errors) / len(errors)
        min_error = min(errors)
        max_error = max(errors)
        error_range = max_error - min_error
        
        print(f"\n   Round-Trip Error Statistics:")
        print(f"      Average: {avg_error:.6f}m")
        print(f"      Minimum: {min_error:.6f}m")
        print(f"      Maximum: {max_error:.6f}m")
        print(f"      Range: {error_range:.6f}m")
        
        # Calculate standard deviation
        if len(errors) > 1:
            variance = sum((e - avg_error)**2 for e in errors) / (len(errors) - 1)
            std_dev = math.sqrt(variance)
            print(f"      Std Dev: {std_dev:.6f}m")
        
        # Detailed results table
        print(f"\n   Detailed Results:")
        print(f"   {'Road s':<10} {'Error (m)':<12} {'Readback Road s':<18} {'Route s':<10}")
        print(f"   {'-'*10} {'-'*12} {'-'*18} {'-'*10}")
        for r in results:
            readback_s = f"{r['readback_road_s']:.1f}" if r['readback_road_s'] is not None else "N/A"
            print(f"   {r['road_s']:<10.1f} {r['error_distance']:<12.6f} {readback_s:<18} {r['route_s']:<10.1f}")
        
        # Analysis
        print(f"\n   Analysis:")
        if error_range < 0.1:
            print(f"      ✅ Error is CONSTANT (range < 0.1m)")
        elif error_range < 0.5:
            print(f"      ✅ Error is NEARLY CONSTANT (range < 0.5m)")
        elif error_range < 1.0:
            print(f"      ⚠️  Error varies MODERATELY (range < 1.0m)")
        else:
            print(f"      ❌ Error varies SIGNIFICANTLY (range >= 1.0m)")
        
        if avg_error < 1.0:
            print(f"      ✅ Average error < 1m target")
        elif avg_error < 5.0:
            print(f"      ⚠️  Average error < 5m (acceptable but above target)")
        else:
            print(f"      ❌ Average error >= 5m (unacceptable)")
        
        # Check if errors are position-dependent
        if error_range >= 1.0:
            print(f"\n   [NOTE] Errors vary significantly - may need position-dependent correction")
            # Show trend
            print(f"   Error trend:")
            for r in results:
                print(f"      road_s={r['road_s']:.1f}m: error={r['error_distance']:.6f}m")
        
        return True
        
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    try:
        success = test_pitlane_multiple_positions()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
