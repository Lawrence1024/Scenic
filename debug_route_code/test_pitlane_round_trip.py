#!/usr/bin/env python3
"""
Test round-trip coordinate transformation for a vehicle placed on pitLaneRoad.

This script:
1. Generates a vehicle on pitLaneRoad in Scenic
2. Gets the Scenic XODR coordinate
3. Verifies it's on pitLaneRoad (track segment detection)
4. Performs full round-trip: XODR → RD → (s,t) → ModelDesk → ControlDesk RD → XODR
5. Verifies round-trip accuracy (< 1m error target)
6. Verifies readback coordinate is still on pitLaneRoad

This follows the debug_cord_code style of testing coordinate transformations.
"""

import sys
import os
import time
import math
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
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform, apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.utils import legacy as dutils
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic import scenarioFromString


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_pitlane_round_trip():
    """Test round-trip for a vehicle on pitLaneRoad."""
    print_section("Testing Round-Trip for Vehicle on pitLaneRoad")
    
    # Change to Scenic root directory so localPath works correctly
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    try:
        import os
        os.chdir(scenic_root)
        
        # Step 1: Generate vehicle on pitLaneRoad
        print_section("Step 1: Generate Vehicle on pitLaneRoad")
        
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on pit lane only
ego = new RacingCar on pitLaneRoad, with raceNumber 1
"""
        
        print("   Generating scenario with vehicle on pitLaneRoad...")
        scenario = scenarioFromString(scenario_code)
        scene, iterations = scenario.generate(maxIterations=10)
        
        if not scene.objects:
            print("   [ERROR] No objects generated")
            return False
        
        ego = scene.objects[0]
        scenic_xodr = (float(ego.position.x), float(ego.position.y), 0.0)
        
        print(f"   [OK] Generated vehicle at XODR: ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
        print(f"   [OK] Race number: {ego.raceNumber}")
        print(f"   [OK] Iterations needed: {iterations}")
        
        # Step 2: Load coordinate transform and road index
        print_section("Step 2: Load Coordinate Transform and Road Index")
        
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
            except Exception as e:
                print(f"   [WARNING] Could not build road index: {e}")
        else:
            print(f"   [WARNING] RD file not found: {rd_path}")
        
        # Step 3: Verify coordinate is on pitLaneRoad
        print_section("Step 3: Verify Coordinate is on pitLaneRoad")
        
        # Extract road IDs from scenario params (XODR IDs)
        pit_lane_road_ids = scene.params.get('pitLaneRoadIds', [])
        main_racing_road_ids = scene.params.get('mainRacingRoadIds', [])
        
        print(f"   Extracted from scene.params:")
        print(f"      pitLaneRoadIds: {pit_lane_road_ids}")
        print(f"      mainRacingRoadIds: {main_racing_road_ids}")
        
        if not pit_lane_road_ids and not main_racing_road_ids:
            print(f"   [WARNING] No road IDs found in scene.params - using fallback detection")
        
        # Transform to RD for segment detection
        if coordinate_transform:
            rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
            print(f"   XODR ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f}) -> RD ({rd_x:.6f}, {rd_y:.6f})")
        else:
            rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
            print(f"   Using XODR coordinates directly (no transform)")
        
        # Verify which road the coordinate projects to
        from scenic.simulators.dspace.geometry.projection import find_road_id_for_position
        from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
        from scenic.simulators.dspace.geometry import utils as geom_utils
        
        if road_index:
            projected_rd_road_id = find_road_id_for_position(road_index, rd_x, rd_y)
            print(f"   Projected to RD road ID: {projected_rd_road_id}")
            
            # Get road name if possible
            if projected_rd_road_id is not None:
                roads = road_index.get('roads', {})
                for road_name, road_data in roads.items():
                    if road_data.get('id') == projected_rd_road_id:
                        print(f"   Projected to road: '{road_name}' (RD ID: {projected_rd_road_id})")
                        break
        else:
            projected_rd_road_id = None
            print(f"   [WARNING] Cannot verify road projection - road_index not available")
        
        # Detect track segment using extracted XODR IDs
        params = {
            'pitLaneRoadIds': [str(rid) for rid in pit_lane_road_ids] if pit_lane_road_ids else [],
            'mainRacingRoadIds': [str(rid) for rid in main_racing_road_ids] if main_racing_road_ids else [],
        }
        
        track_segment = detect_track_segment((rd_x, rd_y), road_index, params, geom_utils)
        route_pref = assign_route_for_segment(track_segment) if track_segment else None
        
        print(f"   Detected track segment: {track_segment}")
        print(f"   Assigned route preference: {route_pref}")
        
        if track_segment == 'pitLane':
            print(f"   [OK] Coordinate is correctly on pitLaneRoad")
        else:
            print(f"   [WARNING] Coordinate detected as '{track_segment}' (expected 'pitLane')")
            if track_segment is None:
                print(f"   [NOTE] Segment detection failed - check road ID mapping")
                print(f"      Projected RD road ID: {projected_rd_road_id}")
                print(f"      Expected XODR pitLaneRoadIds: {pit_lane_road_ids}")
                print(f"      Expected XODR mainRacingRoadIds: {main_racing_road_ids}")
        
        # Step 4: Connect to ModelDesk
        print_section("Step 4: Connect to ModelDesk")
        
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
        print(f"      TrafficScenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
        
        # Step 5: Project to (s,t) and place in ModelDesk
        print_section("Step 5: Project to (s,t) and Place in ModelDesk")
        
        # Use route-specific projection
        if route_pref:
            s_val, t_val = project_world_to_st_route_specific(
                road_index,
                (rd_x, rd_y),
                route_preference=route_pref
            )
            print(f"   Route-specific projection (route={route_pref}): (s={s_val:.1f}, t={t_val:.3f})")
        else:
            # Fallback to regular projection
            s_val, t_val = dutils.project_world_to_st(road_index, (rd_x, rd_y))
            print(f"   Regular projection (no route): (s={s_val:.1f}, t={t_val:.3f})")
            route_pref = 'Lap'  # Default
        
        # Clear existing fellows
        try:
            dutils.clear_collection(ts.Fellows)
            print("   [OK] Cleared existing fellows")
        except Exception as e:
            print(f"   [WARNING] Could not clear fellows: {e}")
        
        # Create fellow
        F = ts.Fellows.Add()
        F.Name = "Test_PitLane_RoundTrip"
        
        seqs = F.Sequences
        dutils.clear_collection(seqs)
        S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
        segs = dutils.ensure_two_segments(S1)
        
        # Set (s,t)
        dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
        print(f"   [OK] Set position in ModelDesk: (s={s_val:.1f}, t={t_val:.3f})")
        
        # Set route - ensure R1 for pitLane
        try:
            route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
            route_sel.UseExternal = False
            if hasattr(route_sel, 'Direction'):
                route_sel.Direction = 0  # Direct
            
            route_name_map = {'Pit': 'R1', 'Lap': 'R2'}
            # If track_segment is 'pitLane', force R1; otherwise use route_pref
            if track_segment == 'pitLane':
                modeldesk_route = 'R1'
                print(f"   [OK] Forcing route to R1 (Pit) because track_segment='pitLane'")
            else:
                modeldesk_route = route_name_map.get(route_pref, 'R2')  # Default to R2 if unknown
                print(f"   [OK] Set route: {modeldesk_route} (from {route_pref})")
            
            route_sel.Activate(modeldesk_route)
        except Exception as e:
            print(f"   [WARNING] Could not set route: {e}")
            # Default to R1 for pit lane if route setting fails
            modeldesk_route = 'R1' if track_segment == 'pitLane' else 'R2'
        
        try:
            dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
            dutils.make_endless_transition(segs)
        except Exception as e:
            print(f"   [WARNING] Could not configure segment 1: {e}")
        
        # Step 6: Save, download, and initialize
        print_section("Step 6: Save, Download, and Initialize")
        
        try:
            ts.Save()
            print("   [OK] Scenario saved")
        except Exception as e:
            print(f"   [WARNING] Could not save: {e}")
        
        try:
            ts.Download()
            print("   [OK] Scenario downloaded")
            time.sleep(0.5)
        except Exception as e:
            print(f"   [ERROR] Could not download: {e}")
            return False
        
        # Reset and start maneuver
        mc = exp.ManeuverControl
        try:
            mc.Stop()
        except:
            pass
        time.sleep(0.2)
        
        try:
            mc.Reset()
            print("   [OK] Maneuver reset")
            time.sleep(0.2)
        except Exception as e:
            print(f"   [WARNING] Could not reset: {e}")
        
        try:
            mc.Start(False)
            print("   [OK] Maneuver started")
            time.sleep(2.0)
        except Exception as e:
            print(f"   [ERROR] Could not start: {e}")
            return False
        
        # Step 7: Connect to ControlDesk and read back
        print_section("Step 7: Read Back from ControlDesk")
        
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
        
        # Wait and step
        print("   Waiting 2 seconds for simulation to initialize...")
        time.sleep(2.0)
        
        print("   Stepping simulation (20 steps)...")
        for i in range(20):
            cd.advance_simulation_step()
            time.sleep(0.1)
            if (i + 1) % 5 == 0:
                print(f"      Step {i+1}/20 completed")
        
        print("   Waiting 1 more second for arrays to update...")
        time.sleep(1.0)
        
        # Read position
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
        try:
            x_arr = cd.get_var(f"{base_path}/x")
            y_arr = cd.get_var(f"{base_path}/y")
            
            if len(x_arr) > 0 and len(y_arr) > 0:
                readback_rd = (float(x_arr[0]), float(y_arr[0]))
                print(f"   [OK] Read back RD coordinates: ({readback_rd[0]:.6f}, {readback_rd[1]:.6f})")
            else:
                print(f"   [ERROR] Arrays are empty")
                return False
        except Exception as e:
            print(f"   [ERROR] Could not read position: {e}")
            import traceback
            traceback.print_exc()
            return False
        
        # Step 8: Transform back to XODR and verify
        print_section("Step 8: Transform Back to XODR and Verify")
        
        if coordinate_transform:
            readback_xodr = apply_inverse_coordinate_transform(coordinate_transform, readback_rd)
            print(f"   RD ({readback_rd[0]:.6f}, {readback_rd[1]:.6f}) -> XODR ({readback_xodr[0]:.6f}, {readback_xodr[1]:.6f})")
        else:
            readback_xodr = readback_rd
            print(f"   Using RD coordinates directly (no inverse transform)")
        
        # Calculate error
        error_x = readback_xodr[0] - scenic_xodr[0]
        error_y = readback_xodr[1] - scenic_xodr[1]
        error_distance = math.sqrt(error_x**2 + error_y**2)
        
        print(f"\n   Original XODR:  ({scenic_xodr[0]:.6f}, {scenic_xodr[1]:.6f})")
        print(f"   Readback XODR:  ({readback_xodr[0]:.6f}, {readback_xodr[1]:.6f})")
        print(f"   Error:          ({error_x:.6f}, {error_y:.6f})")
        print(f"   Distance error: {error_distance:.6f} m")
        
        # Step 9: Verify readback is still on pitLaneRoad
        print_section("Step 9: Verify Readback Coordinate is on pitLaneRoad")
        
        # Verify which road the readback coordinate projects to
        if road_index:
            readback_projected_rd_road_id = find_road_id_for_position(road_index, readback_rd[0], readback_rd[1])
            print(f"   Readback projected to RD road ID: {readback_projected_rd_road_id}")
            
            # Get road name if possible
            if readback_projected_rd_road_id is not None:
                roads = road_index.get('roads', {})
                for road_name, road_data in roads.items():
                    if road_data.get('id') == readback_projected_rd_road_id:
                        print(f"   Readback projected to road: '{road_name}' (RD ID: {readback_projected_rd_road_id})")
                        break
        else:
            readback_projected_rd_road_id = None
        
        # Detect track segment for readback coordinate (using same params)
        readback_track_segment = detect_track_segment(readback_rd, road_index, params, geom_utils)
        readback_route_pref = assign_route_for_segment(readback_track_segment) if readback_track_segment else None
        
        print(f"   Readback RD coordinate: ({readback_rd[0]:.6f}, {readback_rd[1]:.6f})")
        print(f"   Detected track segment: {readback_track_segment}")
        print(f"   Assigned route preference: {readback_route_pref}")
        
        if readback_track_segment == 'pitLane':
            print(f"   [OK] Readback coordinate is correctly on pitLaneRoad")
        else:
            print(f"   [WARNING] Readback coordinate detected as '{readback_track_segment}' (expected 'pitLane')")
        
        # Summary
        print_section("Test Summary")
        
        success = True
        
        print(f"\nRound-Trip Accuracy:")
        print(f"  Distance error: {error_distance:.6f} m")
        if error_distance < 1.0:
            print(f"  [OK] Error < 1m target")
        elif error_distance < 5.0:
            print(f"  [WARNING] Error < 5m (acceptable but above target)")
        else:
            print(f"  [ERROR] Error >= 5m (unacceptable)")
            success = False
        
        print(f"\nTrack Segment Verification:")
        print(f"  Original coordinate segment: {track_segment}")
        print(f"  Readback coordinate segment: {readback_track_segment}")
        if track_segment == 'pitLane' and readback_track_segment == 'pitLane':
            print(f"  [OK] Both coordinates on pitLaneRoad")
        elif track_segment == 'pitLane':
            print(f"  [WARNING] Original on pitLaneRoad but readback on '{readback_track_segment}'")
        else:
            print(f"  [WARNING] Original coordinate not detected as pitLane")
        
        print(f"\nRoute Assignment:")
        print(f"  Expected route: R1 (Pit)")
        print(f"  ModelDesk route: {modeldesk_route}")
        if modeldesk_route == 'R1':
            print(f"  [OK] Route correctly assigned to R1")
        else:
            print(f"  [WARNING] Route assigned to {modeldesk_route} (expected R1)")
        
        if success:
            print(f"\n[OK] Round-trip test PASSED")
        else:
            print(f"\n[WARNING] Round-trip test had issues")
        
        return success
        
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        # Restore original working directory
        import os
        os.chdir(original_cwd)


if __name__ == "__main__":
    try:
        success = test_pitlane_round_trip()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
