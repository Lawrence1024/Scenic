#!/usr/bin/env python3
"""
Analyze what t-coordinate dSPACE actually uses for EGO vehicles by projecting the readback position.

This test replicates test_t_coordinate_readback_analysis.py but for EGO vehicles instead of Fellow vehicles.
It tests:
1. Sets ego vehicle at known lateral offsets (0m, 2m, 4m from centerline)
2. Sets t-coordinate in ModelDesk for ego vehicle
3. Reads back actual position from ControlDesk
4. Projects readback position to see what t-coordinate it corresponds to
5. Compares expected vs actual t-coordinate

Goal: Determine if ego vehicles exhibit the same t-coordinate ignoring behavior as Fellow vehicles.
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
from scenic.simulators.dspace.geometry.projection import project_world_to_st
from scenic.simulators.dspace.geometry import activate_type, set_activity_constant


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


def test_ego_readback_analysis(
    road_s: float,
    t_offset: float,
    road_index: Dict[str, Any],
    coordinate_transform,
    ts,
    cd,
    exp
):
    """Test and analyze what t-coordinate dSPACE actually uses for EGO vehicles."""
    # Get RD coordinate at this road_s with lateral offset
    rd_coord = get_rd_coordinate_at_road_s_with_t(road_index, 'Pit Lane1_2', road_s, t_offset)
    if not rd_coord:
        return None
    
    rd_x, rd_y = rd_coord
    
    # Project original position to (s,t) - this is what we'll set in ModelDesk
    s_set, t_set = project_world_to_st(road_index, (rd_x, rd_y))
    
    print(f"\n   Original position:")
    print(f"      RD: ({rd_x:.6f}, {rd_y:.6f})")
    print(f"      Expected offset: {t_offset:.3f}m from centerline")
    print(f"      Projected (s,t): ({s_set:.3f}, {t_set:.6f})")
    print(f"      Note: t_set uses 0.3 scale factor, so raw_t = {t_set/0.3:.3f}m")
    
    # Access ego maneuver
    try:
        ego_maneuver = ts.Maneuver.Item(0)
        seq = ego_maneuver.Sequences.Item(0)
    except Exception as e:
        print(f"      [ERROR] Could not access ego maneuver: {e}")
        return None
    
    # Set s-coordinate
    seq.StartPosition = float(s_set)
    print(f"      Set in ModelDesk: s={s_set:.3f}, t={t_set:.6f}")
    
    # Set t-coordinate (lateral deviation)
    try:
        segments = seq.Segments
        if segments.Count > 0:
            seg0 = segments.Item(0)
            lat0 = seg0.Activity.LateralType
            
            # Try to activate Deviation mode (using Activate method directly)
            success = False
            if hasattr(lat0, 'Activate'):
                try:
                    lat0.Activate("Deviation")
                    success = True
                except:
                    # Try using activate_type helper
                    if activate_type(lat0, "Deviation"):
                        success = True
            
            if success:
                # Set DependencyType to Absolute
                if hasattr(lat0, 'ActiveElement'):
                    dep = lat0.ActiveElement.DependencyType
                    if hasattr(dep, 'Activate'):
                        try:
                            dep.Activate("Absolute")
                        except:
                            activate_type(dep, "Absolute")
                    
                    # CRITICAL: Activate SourceType to "Constant" before setting value
                    src_type = lat0.ActiveElement.SourceType
                    if hasattr(src_type, 'Activate'):
                        try:
                            src_type.Activate("Constant")
                        except:
                            activate_type(src_type, "Constant")
                        
                        # Set the constant value
                        if hasattr(src_type, 'ActiveElement'):
                            src_type.ActiveElement.Constant = float(t_set)
                            print(f"      Set lateral deviation: {t_set:.6f}m (via SourceType.Constant)")
                        else:
                            # Fallback to set_activity_constant
                            set_activity_constant(lat0, t_set)
                            print(f"      Set lateral deviation: {t_set:.6f}m (via set_activity_constant)")
                    else:
                        # Fallback to set_activity_constant
                        set_activity_constant(lat0, t_set)
                        print(f"      Set lateral deviation: {t_set:.6f}m (via set_activity_constant)")
                else:
                    print(f"      [WARNING] No ActiveElement found")
            else:
                print(f"      [WARNING] Could not activate Deviation mode")
        else:
            print(f"      [WARNING] No segments in ego sequence")
    except Exception as e:
        print(f"      [WARNING] Could not set lateral position: {e}")
        import traceback
        traceback.print_exc()
    
    # Set other properties
    seq.InitialLongitudinalVelocity = 0.0
    seq.VehicleOrientation = 0.0
    
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
    
    # Read back position from ControlDesk
    base_path = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant"
    try:
        path_x = f"{base_path}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
        path_y = f"{base_path}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"
        
        # Read position (single values, not arrays like Fellow)
        readback_rd_x = float(cd.get_var(path_x))
        readback_rd_y = float(cd.get_var(path_y))
        readback_rd = (readback_rd_x, readback_rd_y)
    except Exception as e:
        print(f"      [ERROR] Could not read position: {e}")
        import traceback
        traceback.print_exc()
        return None
    
    print(f"\n   Readback position:")
    print(f"      RD: ({readback_rd[0]:.6f}, {readback_rd[1]:.6f})")
    
    # Project readback position to see what t-coordinate it corresponds to
    s_readback, t_readback = project_world_to_st(road_index, readback_rd)
    raw_t_readback = t_readback / 0.3  # Reverse the scale factor
    
    print(f"      Projected (s,t): ({s_readback:.3f}, {t_readback:.6f})")
    print(f"      Raw lateral offset: {raw_t_readback:.3f}m from centerline")
    
    # Calculate errors
    error_x = readback_rd[0] - rd_x
    error_y = readback_rd[1] - rd_y
    error_distance = math.sqrt(error_x**2 + error_y**2)
    
    t_error = raw_t_readback - t_offset
    t_set_error = t_readback - t_set
    
    print(f"\n   Analysis:")
    print(f"      Position error: {error_distance:.6f}m")
    print(f"      Lateral offset error: {t_error:.6f}m (expected {t_offset:.3f}m, got {raw_t_readback:.3f}m)")
    print(f"      t-coordinate error: {t_set_error:.6f}m (set {t_set:.6f}, readback {t_readback:.6f})")
    
    result = {
        't_offset': t_offset,
        's_set': s_set,
        't_set': t_set,
        'raw_t_expected': t_offset,
        'readback_rd': readback_rd,
        's_readback': s_readback,
        't_readback': t_readback,
        'raw_t_readback': raw_t_readback,
        'error_distance': error_distance,
        't_error': t_error,
        't_set_error': t_set_error,
    }
    
    return result


def test_ego_t_coordinate_readback_analysis():
    """Analyze what t-coordinate dSPACE actually uses for EGO vehicles."""
    print_section("EGO Vehicle T-Coordinate Readback Analysis")
    
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
        test_t_offsets = [0.0, 2.0, 4.0]
        print(f"   Test lateral offsets: {test_t_offsets} m")
        print(f"   Vehicle type: EGO (not Fellow)")
        
        # Run tests
        print_section("Running Readback Analysis for EGO Vehicle")
        
        results = []
        
        for t_offset in test_t_offsets:
            print(f"\n   {'='*60}")
            print(f"   Testing t_offset = {t_offset:.1f}m")
            print(f"   {'='*60}")
            
            result = test_ego_readback_analysis(
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
        
        # Summary
        print_section("Summary")
        
        if not results:
            print("   [ERROR] No successful tests")
            return False
        
        print(f"\n   Summary of Results:")
        print(f"   {'Offset':<10} {'t_set':<12} {'t_readback':<14} {'t_error':<12} {'Position Error':<16}")
        print(f"   {'-'*10} {'-'*12} {'-'*14} {'-'*12} {'-'*16}")
        
        for r in results:
            print(f"   {r['t_offset']:<10.1f} {r['t_set']:<12.6f} {r['t_readback']:<14.6f} {r['t_error']:<12.6f} {r['error_distance']:<16.6f}")
        
        # Analysis
        print(f"\n   Key Findings:")
        
        # Check if readback positions are identical (like Fellow)
        # Use a tolerance of 0.01m (1cm) to account for small numerical differences
        readback_positions = [r['readback_rd'] for r in results]
        positions_identical = all(
            abs(readback_positions[i][0] - readback_positions[0][0]) < 0.01 and
            abs(readback_positions[i][1] - readback_positions[0][1]) < 0.01
            for i in range(1, len(readback_positions))
        )
        
        # Also check if position errors equal lateral offsets (sign of t-coordinate being ignored)
        errors_match_offsets = all(
            abs(r['error_distance'] - r['t_offset']) < 0.1
            for r in results if r['t_offset'] > 0
        )
        
        if positions_identical or errors_match_offsets:
            print(f"   ❌ EGO vehicles exhibit SAME behavior as Fellow: t-coordinate is being IGNORED")
            print(f"      All readback positions are essentially identical (within 0.01m)")
            print(f"      Position errors equal lateral offsets ({errors_match_offsets})")
            print(f"      This suggests dSPACE is ignoring t-coordinate for EGO vehicles too")
            print(f"      Warning: 'Could not activate Deviation mode' indicates configuration issue")
        else:
            print(f"   ✅ EGO vehicles exhibit DIFFERENT behavior from Fellow: t-coordinate is being APPLIED")
            print(f"      Readback positions are different for different t values")
            print(f"      This suggests EGO vehicles correctly apply lateral deviation")
        
        # Check if t_readback matches t_set
        t_matches = all(abs(r['t_set_error']) < 0.1 for r in results)
        if t_matches:
            print(f"   ✅ t-coordinate is correctly applied (readback matches set value)")
        else:
            print(f"   ❌ t-coordinate is NOT correctly applied (readback differs from set value)")
        
        # Check if raw_t_readback matches expected offset
        raw_t_matches = all(abs(r['t_error']) < 0.1 for r in results)
        if raw_t_matches:
            print(f"   ✅ Lateral offset is correctly preserved")
        else:
            print(f"   ❌ Lateral offset is NOT correctly preserved")
            print(f"      Expected offsets are not matching actual positions")
        
        # Compare with Fellow results
        print(f"\n   Comparison with Fellow Vehicle Results:")
        print(f"   Fellow vehicles: All readback positions were identical")
        print(f"   EGO vehicles: {'Identical' if positions_identical else 'Different'} positions")
        print(f"   EGO position errors match offsets: {errors_match_offsets}")
        
        if positions_identical or errors_match_offsets:
            print(f"\n   ⚠️  CONCLUSION: EGO vehicles have the SAME issue as Fellow vehicles")
            print(f"      Both ignore t-coordinate settings")
            print(f"      'Could not activate Deviation mode' warning indicates configuration problem")
            print(f"      This is a ModelDesk configuration issue affecting both vehicle types")
            print(f"      Need to fix Deviation mode activation for both EGO and Fellow vehicles")
        else:
            print(f"\n   ✅ CONCLUSION: EGO vehicles work correctly, Fellow vehicles have issue")
            print(f"      Need to investigate configuration differences between EGO and Fellow")
        
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
        success = test_ego_t_coordinate_readback_analysis()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
