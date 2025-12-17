#!/usr/bin/env python3
"""
Test if Scenic's "left of" produces the correct t-coordinate sign in dSPACE.

This test:
1. Places fellow1 at a known position
2. Places fellow2 = left of fellow1 by 3m
3. Computes t-coordinates for both
4. Verifies: fellow2 should have more negative t than fellow1
5. Reads back positions and verifies actual spatial relationship
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

from scenic import scenarioFromString
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
from scenic.simulators.dspace.geometry import utils as geom_utils


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_scenic_left_vs_dspace_t():
    """Test if Scenic's 'left of' produces correct t-coordinate sign."""
    print_section("Testing Scenic 'Left Of' vs dSPACE t-Coordinate")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Step 1: Generate scenario with relative positioning
        print_section("Step 1: Generate Scenario with 'Left Of' Placement")
        
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Reference vehicle
fellow1 = new RacingCar on mainRacingRoad, with raceNumber 1

# Vehicle placed left of fellow1
fellow2 = new RacingCar left of fellow1 by 3.0, with raceNumber 2
"""
        
        print("   Generating scenario with relative positioning...")
        scenario = scenarioFromString(scenario_code)
        scene, iterations = scenario.generate(maxIterations=10)
        
        if len(scene.objects) < 2:
            print("   [ERROR] Expected at least 2 objects (fellow1 + fellow2)")
            return False
        
        fellow1 = scene.objects[0]
        fellow2 = scene.objects[1]
        
        print(f"   [OK] Generated {len(scene.objects)} vehicles")
        print(f"   [OK] Iterations needed: {iterations}")
        
        # Step 2: Extract positions and compute t-coordinates
        print_section("Step 2: Compute t-Coordinates for Both Vehicles")
        
        fellow1_pos = (float(fellow1.position.x), float(fellow1.position.y))
        fellow2_pos = (float(fellow2.position.x), float(fellow2.position.y))
        
        print(f"   Fellow1 position (XODR): ({fellow1_pos[0]:.6f}, {fellow1_pos[1]:.6f})")
        print(f"   Fellow2 position (XODR): ({fellow2_pos[0]:.6f}, {fellow2_pos[1]:.6f})")
        
        # Load coordinate transform and road index
        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        coordinate_transform = None
        if transform_path.exists():
            try:
                coordinate_transform = load_transform(str(transform_path))
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
            except Exception as e:
                print(f"   [WARNING] Could not build road index: {e}")
        
        if not road_index:
            print("   [ERROR] Could not build road index")
            return False
        
        # Transform to RD coordinates
        if coordinate_transform:
            rd1_x, rd1_y = apply_coordinate_transform(coordinate_transform, fellow1_pos)
            rd2_x, rd2_y = apply_coordinate_transform(coordinate_transform, fellow2_pos)
        else:
            rd1_x, rd1_y = fellow1_pos
            rd2_x, rd2_y = fellow2_pos
        
        print(f"   Fellow1 position (RD): ({rd1_x:.6f}, {rd1_y:.6f})")
        print(f"   Fellow2 position (RD): ({rd2_x:.6f}, {rd2_y:.6f})")
        
        # Detect route
        params = scene.params if hasattr(scene, 'params') else {}
        track_segment1 = detect_track_segment((rd1_x, rd1_y), road_index, params, geom_utils)
        track_segment2 = detect_track_segment((rd2_x, rd2_y), road_index, params, geom_utils)
        route_pref1 = assign_route_for_segment(track_segment1) if track_segment1 else 'Lap'
        route_pref2 = assign_route_for_segment(track_segment2) if track_segment2 else 'Lap'
        
        print(f"   Fellow1 route: {route_pref1}")
        print(f"   Fellow2 route: {route_pref2}")
        
        # Project to (s,t)
        s1, t1 = project_world_to_st_route_specific(
            road_index, (rd1_x, rd1_y), route_preference=route_pref1
        )
        s2, t2 = project_world_to_st_route_specific(
            road_index, (rd2_x, rd2_y), route_preference=route_pref2
        )
        
        print(f"   Fellow1 (s,t): ({s1:.2f}, {t1:.6f})")
        print(f"   Fellow2 (s,t): ({s2:.2f}, {t2:.6f})")
        
        # Step 3: Analyze t-coordinate relationship
        print_section("Step 3: Analyze t-Coordinate Relationship")
        
        print(f"   Fellow1 t: {t1:.6f}")
        print(f"   Fellow2 t: {t2:.6f}")
        print(f"   Difference (t2 - t1): {t2 - t1:.6f}")
        
        # Expected: If Scenic "left" = dSPACE "left", then t2 < t1 (more negative)
        # If Scenic "left" = dSPACE "right", then t2 > t1 (more positive)
        
        if t2 < t1:
            print(f"   [OK] Fellow2 has more negative t than Fellow1")
            print(f"   [OK] This matches expectation: Scenic 'left' → negative t in dSPACE")
            t_sign_correct = True
        else:
            print(f"   [WARNING] Fellow2 has more positive t than Fellow1")
            print(f"   [WARNING] This suggests: Scenic 'left' → positive t in dSPACE (may be inverted)")
            t_sign_correct = False
        
        # Step 4: Verify spatial relationship
        print_section("Step 4: Verify Spatial Relationship")
        
        # Compute actual distance and direction
        dx = fellow2_pos[0] - fellow1_pos[0]
        dy = fellow2_pos[1] - fellow1_pos[1]
        distance = math.sqrt(dx*dx + dy*dy)
        
        print(f"   Actual distance between vehicles: {distance:.3f}m (expected ~3.0m)")
        
        # Get vehicle orientations
        if hasattr(fellow1, 'heading') and hasattr(fellow2, 'heading'):
            heading1 = fellow1.heading
            heading2 = fellow2.heading
            print(f"   Fellow1 heading: {math.degrees(heading1):.1f}°")
            print(f"   Fellow2 heading: {math.degrees(heading2):.1f}°")
            
            # Compute left vector in Scenic's coordinate system
            # In ENU: left = rotate forward 90° CCW
            # Forward vector: (cos(heading), sin(heading))
            # Left vector: (-sin(heading), cos(heading))
            left_x = -math.sin(heading1)
            left_y = math.cos(heading1)
            
            # Project displacement onto left vector
            left_component = dx * left_x + dy * left_y
            
            print(f"   Displacement component along left vector: {left_component:.3f}m")
            
            if left_component > 0:
                print(f"   [OK] Fellow2 is to the left of Fellow1 in Scenic coordinates")
            else:
                print(f"   [WARNING] Fellow2 is to the right of Fellow1 in Scenic coordinates")
        
        return t_sign_correct
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_scenic_left_vs_dspace_t()
    sys.exit(0 if success else 1)

