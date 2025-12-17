#!/usr/bin/env python3
"""
Calibrate direct Scenic XODR → ControlDesk RD transformation.

This script:
1. Takes known Scenic XODR coordinates
2. Uses (s,t) pipeline to place vehicles in ModelDesk
3. Reads actual ControlDesk RD outputs
4. Stores calibration data: (scenic_xodr) → (controldesk_rd_actual)
5. Analyzes offset patterns
6. Saves results to JSON files

Usage:
    python debug_calibrate_cord/calibrate_scenic_to_controldesk.py
"""

import sys
import os
import time
import math
import json
import numpy as np
from pathlib import Path
from datetime import datetime

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
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment


# Test coordinates from fellow_fixed_placing.scenic
TEST_COORDINATES = {
    'ego': {
        'scenic_xodr': (163.545, 48.302, 5.822),
    },
    'fellow1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
    },
    'fellow2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
    },
    'fellow3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),
    },
    'fellow4': {
        'scenic_xodr': (162.256104, -693.627649, 0.0),
    },
    'fellow5': {
        'scenic_xodr': (302.064561, -815.646205, 0.0),
    },
    'fellow6': {
        'scenic_xodr': (557.639219, -737.139638, 0.0),
    },
    'fellow7': {
        'scenic_xodr': (599.646200, -466.416118, 0.0),
    },
    'fellow8': {
        'scenic_xodr': (438.050679, -47.247026, 0.0),
    },
    'fellow9': {
        'scenic_xodr': (211.589136, -18.727096, 0.0),
    },
}


def copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Create a copy of the current scenario."""
    if new_scenario_name is None:
        new_scenario_name = "Calibration_Scenario"
    
    try:
        exp.TrafficScenario.SaveAs(new_scenario_name, True)
        exp.ActivateTrafficScenario(new_scenario_name)
        return exp.TrafficScenario
    except Exception as e:
        print(f"[WARNING] Could not copy scenario: {e}")
        return exp.TrafficScenario


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def measure_controldesk_output(coordinate_transform, road_index, scenic_xodr, name, ts, exp, fellow):
    """Measure actual ControlDesk RD output for a given Scenic XODR coordinate."""
    print(f"\n{name}: Measuring ControlDesk output")
    print(f"  Scenic XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f}, {scenic_xodr[2]:.6f})")
    
    # Step 1: XODR → RD (expected)
    if coordinate_transform:
        rd_expected = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        rd_expected = (rd_expected[0], rd_expected[1])
    else:
        rd_expected = (scenic_xodr[0], scenic_xodr[1])
    
    print(f"  Expected RD: ({rd_expected[0]:.6f}, {rd_expected[1]:.6f})")
    
    # Step 2: Detect route
    try:
        params = {
            'pitLaneRoadIds': ['1'],
            'mainRacingRoadIds': ['0', '2'],
        }
        track_segment = detect_track_segment(rd_expected, road_index, params, dutils)
        route_pref = assign_route_for_segment(track_segment) if track_segment else 'Lap'
        print(f"  Detected route: {route_pref} (segment: {track_segment})")
    except Exception as e:
        print(f"  [WARNING] Route detection failed: {e}")
        route_pref = 'Lap'
        track_segment = None
    
    # Step 3: Project RD → (s,t)
    try:
        s_val, t_val = project_world_to_st_route_specific(
            road_index,
            rd_expected,
            route_preference=route_pref
        )
        print(f"  Projected (s,t): ({s_val:.1f}, {t_val:.3f})")
    except Exception as e:
        print(f"  [ERROR] Projection failed: {e}")
        return None
    
    # Step 4: Place in ModelDesk
    try:
        seqs = fellow.Sequences
        if seqs.Count == 0:
            S1 = seqs.Add()
        else:
            S1 = seqs.Item(0)
        
        segs = dutils.ensure_two_segments(S1)
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
        
        # Set route
        route_sel = S1.Route
        route_sel.UseExternal = False
        route_sel.Direction = 0
        route_name_map = {'Pit': 'R1', 'Lap': 'R2'}
        modeldesk_route = route_name_map.get(route_pref, 'R2')
        route_sel.Activate(modeldesk_route)
        print(f"  Set route: {modeldesk_route}")
        
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
        except:
            pass
        
        # Save, download, reset
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
        
    except Exception as e:
        print(f"  [ERROR] ModelDesk placement failed: {e}")
        return None
    
    # Step 5: Read from ControlDesk
    try:
        cd = ControlDeskApp(
            prog_id="ControlDeskNG.Application",
            outer_platform_name="Platform",
            inner_platform_name="Platform_2"
        ).connect()
        
        time.sleep(2.0)
        
        # Step simulation
        for i in range(20):
            cd.advance_simulation_step()
            time.sleep(0.1)
        
        time.sleep(1.0)
        
        # Read position
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        x_arr = cd.get_var(f"{base_path}/x")
        y_arr = cd.get_var(f"{base_path}/y")
        z_arr = cd.get_var(f"{base_path}/z")
        
        if len(x_arr) > 0 and len(y_arr) > 0:
            rd_actual = (
                float(x_arr[0]),
                float(y_arr[0]),
                float(z_arr[0]) if len(z_arr) > 0 else 0.0
            )
            print(f"  Actual ControlDesk RD: ({rd_actual[0]:.6f}, {rd_actual[1]:.6f}, {rd_actual[2]:.6f})")
            
            # Calculate offsets
            offset_xy = (
                rd_actual[0] - rd_expected[0],
                rd_actual[1] - rd_expected[1]
            )
            offset_z = rd_actual[2] - scenic_xodr[2]
            offset_magnitude = math.sqrt(offset_xy[0]**2 + offset_xy[1]**2)
            
            print(f"  Offset (XY): ({offset_xy[0]:.6f}, {offset_xy[1]:.6f}) = {offset_magnitude:.6f}m")
            print(f"  Offset (Z): {offset_z:.6f}m")
            
            return {
                'name': name,
                'scenic_xodr': list(scenic_xodr),
                'rd_expected': list(rd_expected),
                'rd_actual': list(rd_actual),
                'offset_xy': list(offset_xy),
                'offset_z': offset_z,
                'offset_magnitude': offset_magnitude,
                'route': route_pref,
                'track_segment': track_segment,
                's_t': [s_val, t_val],
                'modeldesk_route': modeldesk_route
            }
        else:
            print("  [ERROR] Could not read position from ControlDesk")
            return None
            
    except Exception as e:
        print(f"  [ERROR] ControlDesk readback failed: {e}")
        import traceback
        traceback.print_exc()
        return None


def analyze_calibration_data(calibration_data):
    """Analyze calibration data to find patterns."""
    if not calibration_data:
        return None
    
    # Extract offsets
    offsets_xy = [d['offset_xy'] for d in calibration_data]
    offsets_z = [d['offset_z'] for d in calibration_data]
    magnitudes = [d['offset_magnitude'] for d in calibration_data]
    
    # Calculate statistics
    offset_x = [o[0] for o in offsets_xy]
    offset_y = [o[1] for o in offsets_xy]
    
    analysis = {
        'timestamp': datetime.now().isoformat(),
        'num_samples': len(calibration_data),
        'offset_xy_statistics': {
            'mean': [float(np.mean(offset_x)), float(np.mean(offset_y))],
            'std': [float(np.std(offset_x)), float(np.std(offset_y))],
            'min': [float(np.min(offset_x)), float(np.min(offset_y))],
            'max': [float(np.max(offset_x)), float(np.max(offset_y))],
            'range': [float(np.max(offset_x) - np.min(offset_x)), float(np.max(offset_y) - np.min(offset_y))]
        },
        'offset_z_statistics': {
            'mean': float(np.mean(offsets_z)),
            'std': float(np.std(offsets_z)),
            'min': float(np.min(offsets_z)),
            'max': float(np.max(offsets_z)),
            'range': float(np.max(offsets_z) - np.min(offsets_z))
        },
        'magnitude_statistics': {
            'mean': float(np.mean(magnitudes)),
            'std': float(np.std(magnitudes)),
            'min': float(np.min(magnitudes)),
            'max': float(np.max(magnitudes)),
            'range': float(np.max(magnitudes) - np.min(magnitudes))
        },
        'route_distribution': {}
    }
    
    # Route distribution
    for d in calibration_data:
        route = d.get('route', 'Unknown')
        analysis['route_distribution'][route] = analysis['route_distribution'].get(route, 0) + 1
    
    # Determine if offset is constant
    xy_range = max(analysis['offset_xy_statistics']['range'])
    z_range = analysis['offset_z_statistics']['range']
    magnitude_range = analysis['magnitude_statistics']['range']
    
    if xy_range < 0.1:
        analysis['offset_pattern'] = 'constant_xy'
        analysis['recommended_correction'] = 'additive_constant'
    elif xy_range < 1.0:
        analysis['offset_pattern'] = 'nearly_constant_xy'
        analysis['recommended_correction'] = 'additive_constant_with_small_variation'
    else:
        analysis['offset_pattern'] = 'position_dependent_xy'
        analysis['recommended_correction'] = 'lookup_table_or_interpolation'
    
    if z_range < 0.1:
        analysis['z_pattern'] = 'constant_z'
    elif z_range < 1.0:
        analysis['z_pattern'] = 'nearly_constant_z'
    else:
        analysis['z_pattern'] = 'position_dependent_z'
    
    return analysis


def save_calibration_data(calibration_data, analysis, output_dir):
    """Save calibration data and analysis to files."""
    output_dir = Path(output_dir)
    output_dir.mkdir(parents=True, exist_ok=True)
    
    # Save raw data
    data_file = output_dir / "calibration_data.json"
    with open(data_file, 'w') as f:
        json.dump(calibration_data, f, indent=2)
    print(f"\n[OK] Saved calibration data to: {data_file}")
    
    # Save analysis
    analysis_file = output_dir / "calibration_analysis.json"
    with open(analysis_file, 'w') as f:
        json.dump(analysis, f, indent=2)
    print(f"[OK] Saved analysis to: {analysis_file}")
    
    # Save human-readable summary
    summary_file = output_dir / "calibration_summary.txt"
    with open(summary_file, 'w') as f:
        f.write("="*80 + "\n")
        f.write("Scenic XODR -> ControlDesk RD Calibration Summary\n")
        f.write("="*80 + "\n\n")
        f.write(f"Generated: {datetime.now().isoformat()}\n")
        f.write(f"Number of samples: {len(calibration_data)}\n\n")
        
        f.write("Offset Statistics (XY plane):\n")
        f.write("-" * 80 + "\n")
        f.write(f"  Mean offset: ({analysis['offset_xy_statistics']['mean'][0]:.6f}, {analysis['offset_xy_statistics']['mean'][1]:.6f}) m\n")
        f.write(f"  Std deviation: ({analysis['offset_xy_statistics']['std'][0]:.6f}, {analysis['offset_xy_statistics']['std'][1]:.6f}) m\n")
        f.write(f"  Range: ({analysis['offset_xy_statistics']['range'][0]:.6f}, {analysis['offset_xy_statistics']['range'][1]:.6f}) m\n")
        f.write(f"  Min: ({analysis['offset_xy_statistics']['min'][0]:.6f}, {analysis['offset_xy_statistics']['min'][1]:.6f}) m\n")
        f.write(f"  Max: ({analysis['offset_xy_statistics']['max'][0]:.6f}, {analysis['offset_xy_statistics']['max'][1]:.6f}) m\n\n")
        
        f.write("Offset Statistics (Z):\n")
        f.write("-" * 80 + "\n")
        f.write(f"  Mean offset: {analysis['offset_z_statistics']['mean']:.6f} m\n")
        f.write(f"  Std deviation: {analysis['offset_z_statistics']['std']:.6f} m\n")
        f.write(f"  Range: {analysis['offset_z_statistics']['range']:.6f} m\n")
        f.write(f"  Min: {analysis['offset_z_statistics']['min']:.6f} m\n")
        f.write(f"  Max: {analysis['offset_z_statistics']['max']:.6f} m\n\n")
        
        f.write("Magnitude Statistics:\n")
        f.write("-" * 80 + "\n")
        f.write(f"  Mean: {analysis['magnitude_statistics']['mean']:.6f} m\n")
        f.write(f"  Std deviation: {analysis['magnitude_statistics']['std']:.6f} m\n")
        f.write(f"  Range: {analysis['magnitude_statistics']['range']:.6f} m\n")
        f.write(f"  Min: {analysis['magnitude_statistics']['min']:.6f} m\n")
        f.write(f"  Max: {analysis['magnitude_statistics']['max']:.6f} m\n\n")
        
        f.write("Route Distribution:\n")
        f.write("-" * 80 + "\n")
        for route, count in analysis['route_distribution'].items():
            f.write(f"  {route}: {count} coordinate(s)\n")
        f.write("\n")
        
        f.write("Offset Pattern Analysis:\n")
        f.write("-" * 80 + "\n")
        f.write(f"  XY Pattern: {analysis['offset_pattern']}\n")
        f.write(f"  Z Pattern: {analysis['z_pattern']}\n")
        f.write(f"  Recommended Correction: {analysis['recommended_correction']}\n\n")
        
        f.write("Detailed Measurements:\n")
        f.write("-" * 80 + "\n")
        f.write(f"{'Name':<12} {'Scenic XODR':<30} {'Expected RD':<30} {'Actual RD':<35} {'Offset XY':<25} {'Offset Z':<12} {'Route':<8}\n")
        f.write("-" * 160 + "\n")
        for d in calibration_data:
            scenic = d['scenic_xodr']
            expected = d['rd_expected']
            actual = d['rd_actual']
            offset = d['offset_xy']
            f.write(f"{d['name']:<12} ({scenic[0]:7.3f},{scenic[1]:7.3f})  ({expected[0]:7.3f},{expected[1]:7.3f})  ({actual[0]:7.3f},{actual[1]:7.3f},{actual[2]:7.3f})  ({offset[0]:7.3f},{offset[1]:7.3f})  {d['offset_z']:7.3f}  {d['route']:<8}\n")
    
    print(f"[OK] Saved summary to: {summary_file}")


def main():
    """Main function."""
    print("="*80)
    print("Scenic XODR -> ControlDesk RD Calibration")
    print("="*80)
    print("\nThis script measures actual ControlDesk RD outputs for known Scenic XODR coordinates.")
    print("\nProcess:")
    print("  1. Transform Scenic XODR -> RD (expected)")
    print("  2. Project RD -> (s,t) using route-specific projection")
    print("  3. Place vehicle in ModelDesk using (s,t)")
    print("  4. Read actual ControlDesk RD output")
    print("  5. Store mapping: (scenic_xodr) -> (controldesk_rd_actual)")
    print("  6. Analyze offset patterns")
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
    
    # Connect to ModelDesk
    try:
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("\n[ERROR] No ModelDesk project active")
            return 1
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("\n[ERROR] No experiment active")
            return 1
        
        # Create scenario copy
        print_section("Creating Scenario Copy")
        ts = copy_scenario(app, exp, source_scenario=None, new_scenario_name="Calibration_Scenario")
        print("[OK] Created scenario copy: Calibration_Scenario")
        
        # Clear all existing fellows
        print("\n[OK] Clearing existing fellows...")
        dutils.clear_collection(ts.Fellows)
        
        # Create a single fellow for all tests
        print("[OK] Creating test fellow...")
        fellow = ts.Fellows.Add()
        fellow.Name = "CalibrationFellow"
        
        # Configure fellow
        seqs = fellow.Sequences
        if seqs.Count == 0:
            seq = seqs.Add()
        else:
            seq = seqs.Item(0)
        
        segs = dutils.ensure_two_segments(seq)
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
        dutils.make_endless_transition(segs)
        
        print("[OK] Fellow configured and ready for calibration")
        
    except Exception as e:
        print(f"\n[ERROR] ModelDesk connection error: {e}")
        import traceback
        traceback.print_exc()
        return 1
    
    # Measure each coordinate
    print_section("Measuring ControlDesk Outputs")
    
    calibration_data = []
    for name, data in TEST_COORDINATES.items():
        result = measure_controldesk_output(
            coordinate_transform,
            road_index,
            data['scenic_xodr'],
            name,
            ts,
            exp,
            fellow
        )
        if result:
            calibration_data.append(result)
        
        # Small delay between tests
        time.sleep(0.5)
    
    # Analyze data
    print_section("Analyzing Calibration Data")
    
    if not calibration_data:
        print("\n[ERROR] No calibration data collected")
        return 1
    
    analysis = analyze_calibration_data(calibration_data)
    
    if analysis:
        print(f"\n[OK] Analysis complete")
        print(f"  Number of samples: {analysis['num_samples']}")
        print(f"  Mean XY offset: ({analysis['offset_xy_statistics']['mean'][0]:.6f}, {analysis['offset_xy_statistics']['mean'][1]:.6f}) m")
        print(f"  XY offset range: ({analysis['offset_xy_statistics']['range'][0]:.6f}, {analysis['offset_xy_statistics']['range'][1]:.6f}) m")
        print(f"  Mean Z offset: {analysis['offset_z_statistics']['mean']:.6f} m")
        print(f"  Z offset range: {analysis['offset_z_statistics']['range']:.6f} m")
        print(f"  Offset pattern: {analysis['offset_pattern']}")
        print(f"  Recommended correction: {analysis['recommended_correction']}")
    
    # Save results
    print_section("Saving Results")
    
    output_dir = Path(__file__).parent
    save_calibration_data(calibration_data, analysis, output_dir)
    
    print_section("Calibration Complete")
    print("\n[SUCCESS] Calibration data collected and saved")
    print(f"\nFiles saved in: {output_dir}")
    print("  - calibration_data.json: Raw measurement data")
    print("  - calibration_analysis.json: Statistical analysis")
    print("  - calibration_summary.txt: Human-readable summary")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

