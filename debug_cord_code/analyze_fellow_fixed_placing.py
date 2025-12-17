#!/usr/bin/env python3
"""
Analyze the coordinates from fellow_fixed_placing.scenic through the integrated framework.

This script:
1. Loads coordinates from the scenic file
2. Transforms them through the complete pipeline:
   - XODR -> RD (coordinate transform)
   - Route detection (pitLane vs mainRacing)
   - RD -> route-specific (s,t) projection
3. If ModelDesk/ControlDesk available: Round-trip test
4. Reports transformation chain and accuracy

Usage:
    python debug_cord_code/analyze_fellow_fixed_placing.py
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

# Coordinates from fellow_fixed_placing.scenic
COORDINATES = {
    'ego': {
        'scenic_xodr': (163.545, 48.302, 5.822),
        'name': 'Ego (Pit Lane)'
    },
    'fellow1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
        'name': 'Fellow_1'
    },
    'fellow2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
        'name': 'Fellow_2'
    },
    'fellow3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),
        'name': 'Fellow_3'
    },
    'fellow4': {
        'scenic_xodr': (162.256104, -693.627649, 0.0),
        'name': 'Fellow_4'
    },
    'fellow5': {
        'scenic_xodr': (302.064561, -815.646205, 0.0),
        'name': 'Fellow_5'
    },
    'fellow6': {
        'scenic_xodr': (557.639219, -737.139638, 0.0),
        'name': 'Fellow_6'
    },
    'fellow7': {
        'scenic_xodr': (599.646200, -466.416118, 0.0),
        'name': 'Fellow_7'
    },
    'fellow8': {
        'scenic_xodr': (438.050679, -47.247026, 0.0),
        'name': 'Fellow_8'
    },
    'fellow9': {
        'scenic_xodr': (211.589136, -18.727096, 0.0),
        'name': 'Fellow_9'
    }
}

def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def analyze_coordinate(coordinate_transform, road_index, scenic_xodr, name, params=None):
    """Analyze a single coordinate through the transformation pipeline."""
    print(f"\n{name}:")
    print(f"  Original XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
    
    # Step 1: XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
        print(f"  Step 1 (XODR -> RD): ({rd_x:.6f}, {rd_y:.6f})")
    else:
        rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
        print(f"  Step 1 (XODR -> RD): ({rd_x:.6f}, {rd_y:.6f}) [no transform - using XODR as RD]")
    
    # Step 2: Detect track segment and route
    from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
    from scenic.simulators.dspace.utils import legacy as dutils
    
    if params is None:
        params = {}
    
    track_segment = detect_track_segment((rd_x, rd_y), road_index, params, dutils)
    route_pref = assign_route_for_segment(track_segment) if track_segment else 'Lap'
    
    print(f"  Step 2 (Route Detection): {route_pref} (track segment: {track_segment})")
    
    # Step 3: Project RD -> route-specific (s,t)
    from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
    
    s_val, t_val = project_world_to_st_route_specific(
        road_index,
        (rd_x, rd_y),
        route_preference=route_pref
    )
    
    print(f"  Step 3 (RD -> route-relative s,t): (s={s_val:.2f}, t={t_val:.6f}) for route {route_pref}")
    
    # Step 4: Find which road this projects onto
    from scenic.simulators.dspace.geometry.projection import find_road_id_for_position
    from scenic.simulators.dspace.geometry.utils import get_road_name_for_id
    
    road_id = find_road_id_for_position(road_index, rd_x, rd_y)
    if road_id is not None:
        road_name = get_road_name_for_id(road_index, road_id)
        if road_name:
            print(f"  Projected onto: {road_name} (ID: {road_id})")
    
    return {
        'xodr': scenic_xodr[:2],
        'rd': (rd_x, rd_y),
        'route': route_pref,
        's': s_val,
        't': t_val,
        'road_id': road_id,
        'road_name': road_name if road_id is not None else None
    }


def test_round_trip_if_available(coordinate_transform, road_index, results, params=None):
    """If ModelDesk/ControlDesk available, test round-trip."""
    try:
        import pythoncom
        from win32com.client import Dispatch
        from scenic.simulators.dspace.controldesk.connection import ControlDeskApp
        from scenic.simulators.dspace.geometry.coordinate_transform import apply_inverse_coordinate_transform
        from scenic.simulators.dspace.utils import legacy as dutils
        
        pythoncom.CoInitialize()
        app = Dispatch("ModelDesk.Application")
        proj = app.ActiveProject
        if proj is None:
            print("\n[SKIP] No ModelDesk project - skipping round-trip test")
            return
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("\n[SKIP] No experiment - skipping round-trip test")
            return
        
        ts = exp.TrafficScenario
        if ts is None:
            print("\n[SKIP] No traffic scenario - skipping round-trip test")
            return
        
        # Try to connect to ControlDesk
        try:
            cd = ControlDeskApp().connect()
            if cd is None:
                print("\n[SKIP] ControlDesk not available - skipping round-trip test")
                return
        except:
            print("\n[SKIP] ControlDesk connection failed - skipping round-trip test")
            return
        
        print_section("ROUND-TRIP TEST (ModelDesk -> ControlDesk -> XODR)")
        
        # Test first coordinate as example
        first_key = list(results.keys())[0]
        result = results[first_key]
        
        print(f"\nTesting {first_key} ({result['name']}):")
        print(f"  Original XODR: ({result['xodr'][0]:.6f}, {result['xodr'][1]:.6f})")
        print(f"  Route-relative (s,t): ({result['s']:.2f}, {result['t']:.6f}) on {result['route']}")
        
        # Clear fellows
        try:
            dutils.clear_collection(ts.Fellows)
        except:
            pass
        
        # Create test fellow
        F = ts.Fellows.Add()
        F.Name = "TestRoundTrip"
        
        seqs = F.Sequences
        dutils.clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
        segs = dutils.ensure_two_segments(S1)
        
        # Set (s,t)
        dutils.configure_seg0_absolute_pose(segs, s=float(result['s']), t=float(result['t']))
        
        # Set route
        route_name_map = {'Pit': 'R1', 'Lap': 'R2'}
        modeldesk_route = route_name_map.get(result['route'], 'R2')
        try:
            route_sel = S1.Route
            route_sel.UseExternal = False
            route_sel.Direction = 0
            route_sel.Activate(modeldesk_route)
            print(f"  Set in ModelDesk: (s={result['s']:.2f}, t={result['t']:.6f}) on route {modeldesk_route}")
        except Exception as e:
            print(f"  [WARNING] Could not set route: {e}")
        
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
        except:
            pass
        
        # Save and download
        ts.Save()
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
        
        # Wait and step
        time.sleep(2.0)
        for i in range(20):
            try:
                cd.advance_simulation_step()
            except:
                pass
            time.sleep(0.1)
        time.sleep(1.0)
        
        # Read back
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        try:
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            if isinstance(x_arr, (list, tuple)) and len(x_arr) > 0:
                rd_readback_x = float(x_arr[0])
                rd_readback_y = float(y_arr[0]) if isinstance(y_arr, (list, tuple)) and len(y_arr) > 0 else 0.0
                
                print(f"  Readback RD: ({rd_readback_x:.6f}, {rd_readback_y:.6f})")
                
                # RD -> XODR (inverse transform)
                if coordinate_transform:
                    xodr_readback_x, xodr_readback_y = apply_inverse_coordinate_transform(
                        coordinate_transform, (rd_readback_x, rd_readback_y)
                    )
                    print(f"  Readback XODR: ({xodr_readback_x:.6f}, {xodr_readback_y:.6f})")
                    
                    # Calculate error
                    dx = xodr_readback_x - result['xodr'][0]
                    dy = xodr_readback_y - result['xodr'][1]
                    error = math.sqrt(dx*dx + dy*dy)
                    
                    print(f"  Round-trip error: {error:.3f}m")
                    
                    if error < 1.0:
                        print(f"  [OK] Error < 1m target")
                    elif error < 10.0:
                        print(f"  [WARNING] Error {error:.3f}m (above 1m target but acceptable)")
                    else:
                        print(f"  [ERROR] Error {error:.3f}m (high error)")
                else:
                    print(f"  [SKIP] No coordinate transform - cannot compute XODR readback")
            else:
                print(f"  [ERROR] Could not read back position from ControlDesk")
        except Exception as e:
            print(f"  [ERROR] Readback failed: {e}")
            import traceback
            traceback.print_exc()
    
    except Exception as e:
        print(f"\n[SKIP] Round-trip test not available: {e}")


def main():
    """Main analysis function."""
    print_section("COORDINATE TRANSFORMATION ANALYSIS")
    print("Analyzing coordinates from fellow_fixed_placing.scenic")
    print(f"Total coordinates: {len(COORDINATES)}")
    
    # Load coordinate transform
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    coordinate_transform = None
    if transform_path.exists():
        from scenic.simulators.dspace.geometry.coordinate_transform import load_transform
        try:
            coordinate_transform = load_transform(str(transform_path))
            print(f"\n[OK] Loaded coordinate transform from: {transform_path}")
        except Exception as e:
            print(f"\n[WARNING] Could not load transform: {e}")
    else:
        print(f"\n[WARNING] Transform file not found: {transform_path}")
    
    # Load road index
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = None
    if rd_path.exists():
        from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
        try:
            road_index = build_rd_road_index(str(rd_path), step=0.5)
            print(f"[OK] Loaded RD road index from: {rd_path}")
            print(f"  Roads: {list(road_index.get('roads', {}).keys())}")
        except Exception as e:
            print(f"[WARNING] Could not load RD index: {e}")
    else:
        print(f"[WARNING] RD file not found: {rd_path}")
    
    if not road_index:
        print("\n[ERROR] Cannot proceed without road index")
        return
    
    # Load params (would normally come from scene, but we'll simulate)
    # For Laguna Seca, we know the road IDs
    params = {
        'pitLaneRoadIds': ['1545702203'],  # Pit Lane1_2
        'mainRacingRoadIds': ['2117817291', '1776499453']  # The Corkscrew1, Andretti Hairpin1_3
    }
    
    print_section("TRANSFORMATION CHAIN ANALYSIS")
    
    results = {}
    for key, coord_data in COORDINATES.items():
        result = analyze_coordinate(
            coordinate_transform,
            road_index,
            coord_data['scenic_xodr'],
            coord_data['name'],
            params
        )
        result['name'] = coord_data['name']
        results[key] = result
    
    # Summary
    print_section("SUMMARY")
    print("\nRoute Distribution:")
    route_counts = {}
    for key, result in results.items():
        route = result['route']
        route_counts[route] = route_counts.get(route, 0) + 1
    for route, count in route_counts.items():
        print(f"  {route}: {count} vehicle(s)")
    
    print("\nRoad Distribution:")
    road_counts = {}
    for key, result in results.items():
        road = result['road_name'] or "Unknown"
        road_counts[road] = road_counts.get(road, 0) + 1
    for road, count in road_counts.items():
        print(f"  {road}: {count} vehicle(s)")
    
    print("\nS-coordinate Range:")
    s_values = [r['s'] for r in results.values()]
    print(f"  Min: {min(s_values):.2f}m")
    print(f"  Max: {max(s_values):.2f}m")
    print(f"  Range: {max(s_values) - min(s_values):.2f}m")
    
    print("\nT-coordinate Range:")
    t_values = [r['t'] for r in results.values()]
    print(f"  Min: {min(t_values):.6f}m")
    print(f"  Max: {max(t_values):.6f}m")
    print(f"  Range: {max(t_values) - min(t_values):.6f}m")
    
    # Round-trip test if available
    test_round_trip_if_available(coordinate_transform, road_index, results, params)
    
    print_section("ANALYSIS COMPLETE")


if __name__ == "__main__":
    from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform
    main()
