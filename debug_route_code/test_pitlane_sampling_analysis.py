#!/usr/bin/env python3
"""
Analyze how Scenic samples coordinates from pitLaneRoad region.

This script:
1. Generates multiple vehicles on pitLaneRoad
2. For each generated coordinate, calculates the lateral distance to the centerline
3. Verifies if the 3.77m error is due to lateral sampling from polygon area vs centerline

Hypothesis: Scenic samples uniformly from lane polygon area, not centerline, causing lateral offset.
"""

import sys
import os
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

from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
from scenic import scenarioFromString


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def calculate_lateral_distance_to_centerline(road_index, road_name, x, y):
    """Calculate lateral distance from a point to the road centerline.
    
    Args:
        road_index: Road index dict
        road_name: Name of the road (e.g., 'Pit Lane1_2')
        x, y: RD coordinates
        
    Returns:
        Lateral distance in meters (positive = left of centerline, negative = right)
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
        
        # Find nearest segment
        min_dist = float('inf')
        lateral_dist = None
        
        for i in range(len(points) - 1):
            x0, y0, s0 = points[i]
            x1, y1, s1 = points[i + 1]
            
            # Vector along segment
            dx = x1 - x0
            dy = y1 - y0
            seg_len2 = dx*dx + dy*dy
            
            if seg_len2 < 1e-12:
                continue
            
            # Vector from segment start to point
            wx = x - x0
            wy = y - y0
            
            # Projection parameter
            u = (wx*dx + wy*dy) / seg_len2
            u = max(0.0, min(1.0, u))  # Clamp to segment
            
            # Projected point on centerline
            proj_x = x0 + u * dx
            proj_y = y0 + u * dy
            
            # Distance from point to projected point
            dist = math.sqrt((x - proj_x)**2 + (y - proj_y)**2)
            
            if dist < min_dist:
                min_dist = dist
                
                # Calculate lateral distance (signed)
                seg_len = math.sqrt(seg_len2)
                # Left normal vector
                nx = -dy / seg_len
                ny = dx / seg_len
                # Lateral distance (signed: positive = left, negative = right)
                lateral_dist = (x - proj_x) * nx + (y - proj_y) * ny
        
        return lateral_dist
    except Exception as e:
        print(f"      [ERROR] Could not calculate lateral distance: {e}")
        return None


def test_pitlane_sampling_analysis():
    """Analyze how Scenic samples coordinates from pitLaneRoad."""
    print_section("Analyzing Scenic Coordinate Sampling on pitLaneRoad")
    
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
            except Exception as e:
                print(f"   [ERROR] Could not build road index: {e}")
                return False
        else:
            print(f"   [ERROR] RD file not found: {rd_path}")
            return False
        
        # Generate multiple vehicles on pitLaneRoad
        print_section("Generating Multiple Vehicles on pitLaneRoad")
        
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on pit lane only
ego = new RacingCar on pitLaneRoad, with raceNumber 1
"""
        
        num_samples = 10
        results = []
        
        print(f"   Generating {num_samples} vehicles on pitLaneRoad...")
        
        for i in range(num_samples):
            try:
                scenario = scenarioFromString(scenario_code)
                scene, iterations = scenario.generate(maxIterations=10)
                
                if not scene.objects:
                    print(f"   [WARNING] Sample {i+1}: No objects generated")
                    continue
                
                ego = scene.objects[0]
                scenic_xodr = (float(ego.position.x), float(ego.position.y), 0.0)
                
                # Transform to RD
                if coordinate_transform:
                    rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr[:2])
                else:
                    rd_x, rd_y = scenic_xodr[0], scenic_xodr[1]
                
                # Verify which road it's on
                projected_road_id = find_road_id_for_position(road_index, rd_x, rd_y)
                
                # Calculate lateral distance to centerline
                lateral_dist = None
                if projected_road_id == 1:  # Pit Lane1_2
                    lateral_dist = calculate_lateral_distance_to_centerline(
                        road_index, 'Pit Lane1_2', rd_x, rd_y
                    )
                
                # Project to get road-relative s and t
                road_s, road_t = project_world_to_st(road_index, (rd_x, rd_y))
                
                result = {
                    'sample': i + 1,
                    'xodr': scenic_xodr,
                    'rd': (rd_x, rd_y),
                    'road_id': projected_road_id,
                    'road_s': road_s,
                    'road_t': road_t,
                    'lateral_dist': lateral_dist,
                    'iterations': iterations
                }
                results.append(result)
                
                print(f"   Sample {i+1}: XODR=({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f}), "
                      f"RD=({rd_x:.3f}, {rd_y:.3f}), "
                      f"road_s={road_s:.1f}m, lateral={lateral_dist:.3f}m" if lateral_dist is not None else f"road_s={road_s:.1f}m, lateral=N/A")
                
            except Exception as e:
                print(f"   [ERROR] Sample {i+1} failed: {e}")
                continue
        
        # Analyze results
        print_section("Results Analysis")
        
        if not results:
            print("   [ERROR] No successful samples")
            return False
        
        print(f"\n   Generated {len(results)} samples")
        
        # Filter to only pit lane samples
        pit_lane_results = [r for r in results if r['road_id'] == 1 and r['lateral_dist'] is not None]
        
        if not pit_lane_results:
            print("   [WARNING] No samples on pit lane")
            return False
        
        print(f"   {len(pit_lane_results)} samples on Pit Lane1_2")
        
        # Calculate statistics for lateral distances
        lateral_dists = [abs(r['lateral_dist']) for r in pit_lane_results]
        avg_lateral = sum(lateral_dists) / len(lateral_dists)
        max_lateral = max(lateral_dists)
        min_lateral = min(lateral_dists)
        
        print(f"\n   Lateral Distance Statistics (distance to centerline):")
        print(f"      Average: {avg_lateral:.6f}m")
        print(f"      Minimum: {min_lateral:.6f}m")
        print(f"      Maximum: {max_lateral:.6f}m")
        print(f"      Range: {max_lateral - min_lateral:.6f}m")
        
        # Compare with the 3.77m error
        print(f"\n   Comparison with Round-Trip Error:")
        print(f"      Average lateral distance: {avg_lateral:.6f}m")
        print(f"      Round-trip error (from test): 3.77m")
        print(f"      Difference: {abs(avg_lateral - 3.77):.6f}m")
        
        if abs(avg_lateral - 3.77) < 1.0:
            print(f"      ✅ Lateral distance matches round-trip error!")
            print(f"      ✅ Hypothesis CONFIRMED: Sampling from polygon area causes lateral offset")
        else:
            print(f"      ⚠️  Lateral distance doesn't match round-trip error exactly")
            print(f"      ⚠️  May be additional factors contributing to error")
        
        # Show detailed results
        print(f"\n   Detailed Results:")
        print(f"   {'Sample':<8} {'XODR (x,y)':<25} {'RD (x,y)':<25} {'Road s':<10} {'Lateral':<10}")
        print(f"   {'-'*8} {'-'*25} {'-'*25} {'-'*10} {'-'*10}")
        for r in pit_lane_results:
            xodr_str = f"({r['xodr'][0]:.2f}, {r['xodr'][1]:.2f})"
            rd_str = f"({r['rd'][0]:.2f}, {r['rd'][1]:.2f})"
            print(f"   {r['sample']:<8} {xodr_str:<25} {rd_str:<25} {r['road_s']:<10.1f} {r['lateral_dist']:<10.3f}")
        
        # Analysis
        print(f"\n   Analysis:")
        if avg_lateral > 1.0:
            print(f"      ⚠️  Coordinates are sampled from polygon area (not centerline)")
            print(f"      ⚠️  Average lateral offset: {avg_lateral:.3f}m")
            print(f"      ⚠️  This explains the round-trip error when projecting to centerline")
        else:
            print(f"      ✅ Coordinates are close to centerline")
            print(f"      ✅ Average lateral offset: {avg_lateral:.3f}m")
        
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
        success = test_pitlane_sampling_analysis()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
