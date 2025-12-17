"""
Diagnostic script to test fellow vehicle coordinates through the complete transformation chain.

This script:
1. Takes fellow coordinates from the scenario
2. Runs them through XODR → RD transformation
3. Tests route detection
4. Tests route-specific projection
5. Shows detailed information at each step
"""

import sys
import os

# Add Scenic to path
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from scenic.simulators.dspace.geometry.pipeline import build_road_index_and_transform
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.utils import legacy as dutils

# Fellow coordinates from fellow_fixed_placing.scenic
FELLOW_COORDINATES = [
    ("Fellow_0", -101.919263, -457.524908, 0.0),
    ("Fellow_1", 0.948038, -272.443171, 0.0),
    ("Fellow_2", 191.994781, -418.905118, 0.0),
    ("Fellow_3", 162.256104, -693.627649, 0.0),
    ("Fellow_4", 302.064561, -815.646205, 0.0),
    ("Fellow_5", 557.639219, -737.139638, 0.0),
    ("Fellow_6", 599.646200, -466.416118, 0.0),
    ("Fellow_7", 438.050679, -47.247026, 0.0),
    ("Fellow_8", 211.589136, -18.727096, 0.0),
]

def test_fellow_coordinate(fellow_name, xodr_x, xodr_y, xodr_z, road_index, coordinate_transform, params):
    """Test a single fellow coordinate through the transformation chain."""
    print(f"\n{'='*80}")
    print(f"Testing {fellow_name}")
    print(f"{'='*80}")
    
    # Step 1: XODR -> RD transformation
    print(f"\n[Step 1] XODR -> RD Transformation")
    print(f"  Input XODR: ({xodr_x:.6f}, {xodr_y:.6f}, {xodr_z:.6f})")
    
    if coordinate_transform is not None:
        from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, (xodr_x, xodr_y))
        print(f"  Output RD: ({rd_x:.6f}, {rd_y:.6f})")
        print(f"  [OK] Coordinate transformation applied")
    else:
        rd_x, rd_y = xodr_x, xodr_y
        print(f"  Output RD: ({rd_x:.6f}, {rd_y:.6f}) (no transform, using XODR)")
        print(f"  [WARN] No coordinate transformation available")
    
    # Step 2: Route detection
    print(f"\n[Step 2] Route Detection")
    position_xy = (rd_x, rd_y)
    track_segment = detect_track_segment(position_xy, road_index, params, dutils)
    
    if track_segment:
        route_pref = assign_route_for_segment(track_segment)
        print(f"  Track segment: '{track_segment}'")
        print(f"  Route preference: '{route_pref}'")
        print(f"  [OK] Route detected")
    else:
        route_pref = 'Lap'  # Default
        print(f"  Track segment: None")
        print(f"  Route preference: '{route_pref}' (default)")
        print(f"  [WARN] Route detection failed, using default")
    
    # Step 3: Route-specific projection
    print(f"\n[Step 3] Route-Specific Projection (RD -> s,t)")
    try:
        s_val, t_val = project_world_to_st_route_specific(
            road_index,
            (rd_x, rd_y),
            route_preference=route_pref
        )
        print(f"  Route: {route_pref}")
        print(f"  Output (s, t): ({s_val:.2f}, {t_val:.6f})")
        print(f"  [OK] Route-specific projection successful")
    except Exception as e:
        print(f"  [ERROR] Route-specific projection failed: {e}")
        # Fallback to regular projection
        try:
            s_val, t_val = dutils.project_world_to_st(road_index, (rd_x, rd_y))
            print(f"  Fallback (s, t): ({s_val:.2f}, {t_val:.6f})")
            print(f"  [WARN] Using fallback projection")
        except Exception as e2:
            print(f"  [ERROR] Fallback projection also failed: {e2}")
            s_val, t_val = 0.0, 0.0
    
    # Step 4: Summary
    print(f"\n[Summary]")
    print(f"  Fellow: {fellow_name}")
    print(f"  XODR: ({xodr_x:.6f}, {xodr_y:.6f})")
    print(f"  RD: ({rd_x:.6f}, {rd_y:.6f})")
    print(f"  Route: {route_pref}")
    print(f"  (s, t): ({s_val:.2f}, {t_val:.6f})")
    
    return {
        'name': fellow_name,
        'xodr': (xodr_x, xodr_y, xodr_z),
        'rd': (rd_x, rd_y),
        'track_segment': track_segment,
        'route': route_pref,
        's_t': (s_val, t_val)
    }

def main():
    """Main diagnostic function."""
    print("="*80)
    print("Fellow Coordinate Transformation Diagnostic")
    print("="*80)
    
    # Map file path
    map_path = os.path.join(
        os.path.dirname(__file__),
        '..', 'assets', 'maps', 'dSPACE', 'LagunaSeca.xodr'
    )
    
    if not os.path.exists(map_path):
        print(f"\n[ERROR] Map file not found: {map_path}")
        print("Please ensure the map file exists.")
        return
    
    print(f"\n[Setup] Loading map and building road index...")
    print(f"  Map file: {map_path}")
    
    # Build road index and coordinate transform
    road_index, coordinate_transform = build_road_index_and_transform(map_path, dutils)
    
    if road_index is None:
        print(f"\n[ERROR] Failed to build road index")
        return
    
    print(f"  [OK] Road index built")
    if coordinate_transform:
        print(f"  [OK] Coordinate transformation loaded")
    else:
        print(f"  [WARN] No coordinate transformation available")
    
    # Get params (simulate what the racing domain would provide)
    # These should match what's in the racing domain model
    params = {
        'pitLaneRoadIds': ['1545702203'],  # Pit Lane1_2
        'mainRacingRoadIds': ['2117817291', '1776499453'],  # The Corkscrew1, Andretti Hairpin1_3
    }
    
    print(f"\n[Params]")
    print(f"  pitLaneRoadIds: {params['pitLaneRoadIds']}")
    print(f"  mainRacingRoadIds: {params['mainRacingRoadIds']}")
    
    # Test each fellow coordinate
    results = []
    for fellow_name, xodr_x, xodr_y, xodr_z in FELLOW_COORDINATES:
        result = test_fellow_coordinate(
            fellow_name, xodr_x, xodr_y, xodr_z,
            road_index, coordinate_transform, params
        )
        results.append(result)
    
    # Summary table
    print(f"\n\n{'='*80}")
    print("SUMMARY TABLE")
    print(f"{'='*80}")
    print(f"{'Fellow':<12} {'XODR (x,y)':<30} {'RD (x,y)':<30} {'Route':<8} {'s':<10} {'t':<10}")
    print(f"{'-'*80}")
    for r in results:
        xodr_str = f"({r['xodr'][0]:.2f}, {r['xodr'][1]:.2f})"
        rd_str = f"({r['rd'][0]:.2f}, {r['rd'][1]:.2f})"
        route = r['route']
        s_val, t_val = r['s_t']
        print(f"{r['name']:<12} {xodr_str:<30} {rd_str:<30} {route:<8} {s_val:<10.2f} {t_val:<10.6f}")
    
    # Route distribution
    print(f"\n[Route Distribution]")
    route_counts = {}
    for r in results:
        route = r['route']
        route_counts[route] = route_counts.get(route, 0) + 1
    
    for route, count in route_counts.items():
        print(f"  {route}: {count} fellow(s)")
    
    # Check for discrepancies
    print(f"\n[Discrepancy Check]")
    lap_count = route_counts.get('Lap', 0)
    pit_count = route_counts.get('Pit', 0)
    
    if pit_count > 0:
        print(f"  [WARN] {pit_count} fellow(s) assigned to Pit route (expected 0 for main racing road)")
        pit_fellows = [r['name'] for r in results if r['route'] == 'Pit']
        print(f"  Fellows on Pit: {', '.join(pit_fellows)}")
    else:
        print(f"  [OK] All fellows assigned to Lap route")
    
    print(f"\n{'='*80}")
    print("Diagnostic complete")
    print(f"{'='*80}")

if __name__ == '__main__':
    main()
