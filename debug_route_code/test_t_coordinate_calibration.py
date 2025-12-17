#!/usr/bin/env python3
"""
Test t-coordinate scale factor calibration for different lateral offsets.

This script:
1. Tests multiple lateral offsets (0m, 2m, 4m, 6m, 8m from centerline)
2. Tests multiple scale factors (0.2, 0.25, 0.3, 0.35, 0.4)
3. Performs round-trip tests for each combination
4. Measures errors and finds optimal scale factor

Goal: Find the optimal scale factor that minimizes round-trip error for large lateral offsets (4-8m)
while preserving 2D sampling space.
"""

import sys
import os
import time
import math
from pathlib import Path
from collections import defaultdict
from typing import Dict, List, Tuple, Optional

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


# Global variable to store test scale factor
TEST_SCALE_FACTOR = 0.3


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def get_rd_coordinate_at_road_s_with_t(road_index, road_name, road_s, t_offset=0.0):
    """Get RD coordinate at a specific road-relative s position with lateral offset.
    
    Args:
        road_index: Road index dict
        road_name: Name of the road (e.g., 'Pit Lane1_2')
        road_s: Road-relative s coordinate (0 to road_length)
        t_offset: Lateral deviation from centerline in meters (default 0.0)
        
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
                    x += nx * t_offset
                    y += ny * t_offset
                
                return (x, y)
        
        # If road_s is beyond the road, return the last point
        if points:
            x, y, s = points[-1]
            return (x, y)
        
        return None
    except Exception as e:
        print(f"      [ERROR] Could not get RD coordinate: {e}")
        return None


def project_world_to_st_with_scale_factor(road_index, pos, scale_factor=0.3):
    """Project world coordinates to (s,t) with custom scale factor.
    
    This is a modified version of project_world_to_st that uses a custom scale factor.
    """
    px, py = pos
    all_projections = []
    
    # Get roads from index
    if isinstance(road_index, dict):
        it = road_index.get('roads', {}).values()
    else:
        it = road_index
    
    for road in it:
        sec_list = road.get('sec_points') if isinstance(road, dict) else getattr(road, 'sec_points', [])
        if not sec_list:
            continue
        for pts in sec_list:
            if not pts or len(pts) < 2:
                continue
            for i in range(len(pts) - 1):
                x0, y0, s0 = pts[i]
                x1, y1, s1 = pts[i+1]
                vx, vy = x1 - x0, y1 - y0
                seg_len2 = vx*vx + vy*vy
                if seg_len2 <= 1e-12:
                    continue
                wx, wy = px - x0, py - y0
                u = (wx*vx + wy*vy) / seg_len2
                u = 0.0 if u < 0.0 else (1.0 if u > 1.0 else u)
                qx = x0 + u*vx
                qy = y0 + u*vy
                dx, dy = px - qx, py - qy
                dist2 = dx*dx + dy*dy

                seg_len = seg_len2 ** 0.5
                # Calculate normal vector for t-coordinate
                nx_left, ny_left = -vy/seg_len, vx/seg_len  # left normal
                
                # Calculate t-coordinate using geometric projection with custom scaling
                raw_t = dx*nx_left + dy*ny_left
                t_signed = raw_t * scale_factor  # Use custom scale factor
                s_proj = s0 + u*(s1 - s0)
                
                # Get road ID and name
                road_id = road.get('id') if isinstance(road, dict) else getattr(road, 'id', None)
                road_name = road.get('name') if isinstance(road, dict) else getattr(road, 'name', f'Road_{road_id}')
                
                # Store all projections
                all_projections.append((dist2, s_proj, t_signed, road_id, road_name))
    
    # Find the truly closest projection
    if not all_projections:
        return 0.0, 0.0
    
    # Sort by distance and take the closest
    all_projections.sort(key=lambda x: x[0])
    best = all_projections[0]

    if best is None:
        return 0.0, 0.0
    
    raw_s = float(best[1])
    t_val = float(best[2])
    
    return raw_s, t_val


def test_round_trip_with_scale_factor(
    road_s: float,
    t_offset: float,
    scale_factor: float,
    road_index: Dict,
    coordinate_transform: Optional[Dict],
    ts,
    cd,
    exp
) -> Optional[Dict]:
    """Test round-trip for a specific position, lateral offset, and scale factor.
    
    Args:
        road_s: Road-relative s coordinate
        t_offset: Lateral offset from centerline in meters
        scale_factor: Scale factor to use for t-coordinate calculation
        road_index: Road index dict
        coordinate_transform: Coordinate transform dict
        ts: TrafficScenario object
        cd: ControlDeskApp object
        exp: Experiment object
        
    Returns:
        dict with test results or None if failed
    """
    # Get RD coordinate at this road_s with lateral offset
    rd_coord = get_rd_coordinate_at_road_s_with_t(road_index, 'Pit Lane1_2', road_s, t_offset)
    if not rd_coord:
        return None
    
    rd_x, rd_y = rd_coord
    
    # Transform to XODR (for comparison)
    if coordinate_transform:
        xodr_coord = apply_inverse_coordinate_transform(coordinate_transform, (rd_x, rd_y))
    else:
        xodr_coord = (rd_x, rd_y)
    
    # Project to (s,t) using custom scale factor
    try:
        s_route, t_route = project_world_to_st_with_scale_factor(
            road_index,
            (rd_x, rd_y),
            scale_factor=scale_factor
        )
    except Exception as e:
        print(f"      [ERROR] Projection failed: {e}")
        return None
    
    # Clear and create fellow
    try:
        dutils.clear_collection(ts.Fellows)
    except:
        pass
    
    fellow = ts.Fellows.Add()
    fellow.Name = f"Test_Calibration_s{road_s:.0f}_t{t_offset:.1f}_sf{scale_factor:.2f}"
    
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
        pass
    
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
        return None
    
    # Transform readback to XODR
    if coordinate_transform:
        readback_xodr = apply_inverse_coordinate_transform(coordinate_transform, readback_rd)
    else:
        readback_xodr = readback_rd
    
    # Calculate error
    error_x = readback_xodr[0] - xodr_coord[0]
    error_y = readback_xodr[1] - xodr_coord[1]
    error_distance = math.sqrt(error_x**2 + error_y**2)
    
    result = {
        'road_s': road_s,
        't_offset': t_offset,
        'scale_factor': scale_factor,
        'rd_coord': (rd_x, rd_y),
        'xodr_coord': xodr_coord,
        'route_s': s_route,
        'route_t': t_route,
        'readback_rd': readback_rd,
        'readback_xodr': readback_xodr,
        'error_x': error_x,
        'error_y': error_y,
        'error_distance': error_distance,
    }
    
    return result


def test_t_coordinate_calibration():
    """Test t-coordinate calibration with multiple offsets and scale factors."""
    print_section("T-Coordinate Scale Factor Calibration Test")
    
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
        
        ts = exp.TrafficScenario
        if ts is None:
            print("   [ERROR] Active experiment has no TrafficScenario")
            return False
        
        print(f"   [OK] Connected to ModelDesk")
        print(f"      Project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
        print(f"      Experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
        
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
        
        # Test configuration
        print_section("Test Configuration")
        
        # Test positions: use a fixed position in the middle of pit lane
        pit_lane_length = road_index['roads']['Pit Lane1_2']['length']
        test_road_s = pit_lane_length / 2  # Middle of pit lane
        print(f"   Test position: road_s = {test_road_s:.1f}m (middle of pit lane)")
        
        # Test lateral offsets (meters from centerline)
        test_t_offsets = [0.0, 2.0, 4.0, 6.0, 8.0]
        print(f"   Test lateral offsets: {test_t_offsets} m")
        
        # Test scale factors
        test_scale_factors = [0.2, 0.25, 0.3, 0.35, 0.4]
        print(f"   Test scale factors: {test_scale_factors}")
        
        total_tests = len(test_t_offsets) * len(test_scale_factors)
        print(f"   Total tests: {total_tests}")
        
        # Run tests
        print_section("Running Calibration Tests")
        
        results = []
        test_num = 0
        
        for t_offset in test_t_offsets:
            for scale_factor in test_scale_factors:
                test_num += 1
                print(f"\n   Test {test_num}/{total_tests}: t_offset={t_offset:.1f}m, scale_factor={scale_factor:.2f}")
                
                result = test_round_trip_with_scale_factor(
                    test_road_s,
                    t_offset,
                    scale_factor,
                    road_index,
                    coordinate_transform,
                    ts,
                    cd,
                    exp
                )
                
                if result:
                    results.append(result)
                    print(f"      Error: {result['error_distance']:.6f}m")
                else:
                    print(f"      [ERROR] Test failed")
        
        # Analyze results
        print_section("Results Analysis")
        
        if not results:
            print("   [ERROR] No successful tests")
            return False
        
        print(f"\n   Completed {len(results)}/{total_tests} tests")
        
        # Group results by t_offset
        results_by_offset = defaultdict(list)
        for r in results:
            results_by_offset[r['t_offset']].append(r)
        
        # Analyze each offset
        print(f"\n   Results by Lateral Offset:")
        print(f"   {'Offset (m)':<12} {'Scale Factor':<14} {'Error (m)':<12} {'Best SF':<10}")
        print(f"   {'-'*12} {'-'*14} {'-'*12} {'-'*10}")
        
        best_scale_factors = {}
        for t_offset in sorted(results_by_offset.keys()):
            offset_results = results_by_offset[t_offset]
            
            # Find best scale factor for this offset
            best_result = min(offset_results, key=lambda x: x['error_distance'])
            best_scale_factors[t_offset] = best_result['scale_factor']
            
            # Print all scale factors for this offset
            for r in sorted(offset_results, key=lambda x: x['scale_factor']):
                marker = " <-- BEST" if r['scale_factor'] == best_result['scale_factor'] else ""
                print(f"   {r['t_offset']:<12.1f} {r['scale_factor']:<14.2f} {r['error_distance']:<12.6f}{marker}")
        
        # Summary by scale factor
        print(f"\n   Results by Scale Factor:")
        print(f"   {'Scale Factor':<14} {'Avg Error (m)':<16} {'Min Error (m)':<16} {'Max Error (m)':<16}")
        print(f"   {'-'*14} {'-'*16} {'-'*16} {'-'*16}")
        
        results_by_scale = defaultdict(list)
        for r in results:
            results_by_scale[r['scale_factor']].append(r)
        
        for scale_factor in sorted(results_by_scale.keys()):
            scale_results = results_by_scale[scale_factor]
            errors = [r['error_distance'] for r in scale_results]
            avg_error = sum(errors) / len(errors)
            min_error = min(errors)
            max_error = max(errors)
            print(f"   {scale_factor:<14.2f} {avg_error:<16.6f} {min_error:<16.6f} {max_error:<16.6f}")
        
        # Find overall best scale factor
        print(f"\n   Overall Best Scale Factor Analysis:")
        
        # Calculate average error for each scale factor across all offsets
        scale_factor_scores = {}
        for scale_factor in test_scale_factors:
            scale_results = results_by_scale[scale_factor]
            errors = [r['error_distance'] for r in scale_results]
            avg_error = sum(errors) / len(errors)
            scale_factor_scores[scale_factor] = avg_error
        
        best_overall_scale = min(scale_factor_scores.keys(), key=lambda k: scale_factor_scores[k])
        print(f"   Best overall scale factor: {best_overall_scale:.2f} (avg error: {scale_factor_scores[best_overall_scale]:.6f}m)")
        
        # Compare with current (0.3)
        current_avg_error = scale_factor_scores.get(0.3, float('inf'))
        improvement = ((current_avg_error - scale_factor_scores[best_overall_scale]) / current_avg_error * 100) if current_avg_error > 0 else 0
        print(f"   Current (0.3) avg error: {current_avg_error:.6f}m")
        print(f"   Improvement: {improvement:.1f}%")
        
        # Detailed recommendations
        print(f"\n   Recommendations:")
        if best_overall_scale != 0.3:
            print(f"   ⚠️  Consider changing scale factor from 0.3 to {best_overall_scale:.2f}")
            print(f"      Expected improvement: {improvement:.1f}% reduction in average error")
        else:
            print(f"   ✅ Current scale factor (0.3) appears optimal")
        
        # Check if errors are acceptable
        best_avg_error = scale_factor_scores[best_overall_scale]
        if best_avg_error < 1.0:
            print(f"   ✅ Best average error ({best_avg_error:.6f}m) is below 1m target")
        elif best_avg_error < 5.0:
            print(f"   ⚠️  Best average error ({best_avg_error:.6f}m) is above 1m target but acceptable")
        else:
            print(f"   ❌ Best average error ({best_avg_error:.6f}m) is too high - may need non-linear mapping")
        
        # Export detailed results to CSV
        csv_path = scenic_root / "debug_route_code" / "t_coordinate_calibration_results.csv"
        try:
            import csv
            with open(csv_path, 'w', newline='') as f:
                writer = csv.writer(f)
                writer.writerow(['t_offset', 'scale_factor', 'error_distance', 'road_s', 'route_s', 'route_t'])
                for r in results:
                    writer.writerow([
                        r['t_offset'],
                        r['scale_factor'],
                        r['error_distance'],
                        r['road_s'],
                        r['route_s'],
                        r['route_t']
                    ])
            print(f"\n   [OK] Detailed results exported to: {csv_path}")
        except Exception as e:
            print(f"\n   [WARNING] Could not export CSV: {e}")
        
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
        success = test_t_coordinate_calibration()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
