#!/usr/bin/env python3
"""
Test t-coordinate WITHOUT scaling to verify if dSPACE expects t in meters directly.

This test verifies the hypothesis that dSPACE expects t-coordinate in meters directly,
not scaled by 0.3. If true, we should use t = raw_t (no scaling) instead of t = raw_t * 0.3.
"""

import sys
import os
import time
import math
from pathlib import Path
from typing import Dict, Any

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
from scenic.simulators.dspace.utils import legacy as dutils


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def get_rd_coordinate_at_road_s_with_t(road_index, road_name, road_s, t_offset=0.0):
    """Get RD coordinate at a specific road-relative s position with lateral offset."""
    try:
        roads = road_index.get('roads', {})
        road_data = roads.get(road_name)
        if not road_data:
            return None
        
        sec_points = road_data.get('sec_points', [])
        if not sec_points or not sec_points[0]:
            return None
        
        points = sec_points[0]
        
        for i in range(len(points) - 1):
            x0, y0, s0 = points[i]
            x1, y1, s1 = points[i + 1]
            
            if s0 <= road_s <= s1:
                if s1 - s0 < 1e-6:
                    return (x0, y0)
                
                u = (road_s - s0) / (s1 - s0)
                x = x0 + u * (x1 - x0)
                y = y0 + u * (y1 - y0)
                
                dx = x1 - x0
                dy = y1 - y0
                seg_len = math.sqrt(dx*dx + dy*dy)
                if seg_len > 1e-6:
                    nx = -dy / seg_len
                    ny = dx / seg_len
                    x += nx * t_offset
                    y += ny * t_offset
                
                return (x, y)
        
        if points:
            x, y, s = points[-1]
            return (x, y)
        
        return None
    except Exception as e:
        print(f"      [ERROR] Could not get RD coordinate: {e}")
        return None


def project_world_to_st_no_scaling(road_index, pos):
    """Project world coordinates to (s,t) WITHOUT scaling the t-coordinate.
    
    This is the same as project_world_to_st but with scale_factor = 1.0
    """
    px, py = pos
    all_projections = []
    
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
                nx_left, ny_left = -vy/seg_len, vx/seg_len
                
                # NO SCALING - use raw_t directly
                raw_t = dx*nx_left + dy*ny_left
                t_signed = raw_t  # No scaling!
                s_proj = s0 + u*(s1 - s0)
                
                road_id = road.get('id') if isinstance(road, dict) else getattr(road, 'id', None)
                road_name = road.get('name') if isinstance(road, dict) else getattr(road, 'name', f'Road_{road_id}')
                
                all_projections.append((dist2, s_proj, t_signed, road_id, road_name))
    
    if not all_projections:
        return 0.0, 0.0
    
    all_projections.sort(key=lambda x: x[0])
    best = all_projections[0]

    if best is None:
        return 0.0, 0.0
    
    raw_s = float(best[1])
    t_val = float(best[2])
    
    return raw_s, t_val


def test_round_trip_no_scaling(
    road_s: float,
    t_offset: float,
    road_index: Dict[str, Any],
    coordinate_transform,
    ts,
    cd,
    exp
):
    """Test round-trip WITHOUT scaling the t-coordinate."""
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
    
    # Project to (s,t) WITHOUT scaling
    try:
        s_route, t_route = project_world_to_st_no_scaling(road_index, (rd_x, rd_y))
    except Exception as e:
        print(f"      [ERROR] Projection failed: {e}")
        return None
    
    # Clear and create fellow
    try:
        dutils.clear_collection(ts.Fellows)
    except:
        pass
    
    fellow = ts.Fellows.Add()
    fellow.Name = f"Test_NoScaling_s{road_s:.0f}_t{t_offset:.1f}"
    
    seqs = fellow.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # Set (s,t) on R1 - using t WITHOUT scaling
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


def test_t_coordinate_no_scaling():
    """Test t-coordinate WITHOUT scaling to verify if dSPACE expects meters directly."""
    print_section("T-Coordinate Test WITHOUT Scaling")
    
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
        
        pit_lane_length = road_index['roads']['Pit Lane1_2']['length']
        test_road_s = pit_lane_length / 2
        print(f"   Test position: road_s = {test_road_s:.1f}m (middle of pit lane)")
        
        # Test lateral offsets
        test_t_offsets = [0.0, 2.0, 4.0, 6.0, 8.0]
        print(f"   Test lateral offsets: {test_t_offsets} m")
        print(f"   Using t-coordinate WITHOUT scaling (t = raw_t)")
        
        # Run tests
        print_section("Running Tests WITHOUT Scaling")
        
        results = []
        
        for t_offset in test_t_offsets:
            print(f"\n   Testing t_offset = {t_offset:.1f}m")
            
            result = test_round_trip_no_scaling(
                test_road_s,
                t_offset,
                road_index,
                coordinate_transform,
                ts,
                cd,
                exp
            )
            
            if result:
                results.append(result)
                print(f"      t_coordinate = {result['route_t']:.6f}m")
                print(f"      Error: {result['error_distance']:.6f}m")
            else:
                print(f"      [ERROR] Test failed")
        
        # Analyze results
        print_section("Results Analysis")
        
        if not results:
            print("   [ERROR] No successful tests")
            return False
        
        print(f"\n   Results:")
        print(f"   {'Offset (m)':<12} {'t_coord (m)':<14} {'Error (m)':<12} {'Status':<20}")
        print(f"   {'-'*12} {'-'*14} {'-'*12} {'-'*20}")
        
        for r in results:
            if r['error_distance'] < 0.1:
                status = "✅ Excellent"
            elif r['error_distance'] < 1.0:
                status = "✅ Good"
            elif r['error_distance'] < 5.0:
                status = "⚠️  Acceptable"
            else:
                status = "❌ Poor"
            
            print(f"   {r['t_offset']:<12.1f} {r['route_t']:<14.6f} {r['error_distance']:<12.6f} {status}")
        
        # Summary
        errors = [r['error_distance'] for r in results]
        avg_error = sum(errors) / len(errors)
        max_error = max(errors)
        
        print(f"\n   Summary:")
        print(f"      Average error: {avg_error:.6f}m")
        print(f"      Maximum error: {max_error:.6f}m")
        
        if avg_error < 0.1:
            print(f"   ✅ Hypothesis CONFIRMED: dSPACE expects t in meters directly (no scaling)")
            print(f"      Recommendation: Remove 0.3 scale factor from projection.py")
        elif avg_error < 1.0:
            print(f"   ✅ Hypothesis PARTIALLY CONFIRMED: No scaling works better than 0.3 scaling")
            print(f"      Recommendation: Consider removing or adjusting scale factor")
        else:
            print(f"   ⚠️  Hypothesis INCONCLUSIVE: Errors still significant")
            print(f"      May need further investigation")
        
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
        success = test_t_coordinate_no_scaling()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
