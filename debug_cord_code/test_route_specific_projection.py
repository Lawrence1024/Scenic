#!/usr/bin/env python3
"""
Test route-specific projection behavior.

The hypothesis is that the bug occurs because:
1. Projection uses a global road index (all roads)
2. Routes R1 and R2 have different coordinate systems
3. The same (s,t) on different routes maps to different RD coordinates

This script tests:
- Which road each coordinate projects onto
- Whether that road is part of R1 or R2
- If we can determine the correct route before projection

Usage:
    python debug_cord_code/test_route_specific_projection.py
"""

import sys
import os
import math
from pathlib import Path

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform, apply_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.utils import legacy as dutils
from scenic.simulators.dspace.geometry.projection import find_road_id_for_position


# Test coordinates
TEST_COORDINATES = {
    'Fellow_1': {
        'scenic_xodr': (-101.919263, -457.524908, 0.0),
        'expected_rd': (-96.468, -456.652),
        'expected_s_t': (0.0, -1.653)
    },
    'Fellow_2': {
        'scenic_xodr': (0.948038, -272.443171, 0.0),
        'expected_rd': (5.082, -273.737),
        'expected_s_t': (279.4, 1.472)
    },
    'Fellow_3': {
        'scenic_xodr': (191.994781, -418.905118, 0.0),
        'expected_rd': (192.786, -418.186),
        'expected_s_t': (550.3, 0.242)
    }
}


def analyze_projection(road_index, rd_coord, coordinate_transform=None):
    """Analyze which road a coordinate projects onto and get detailed info."""
    print(f"\nAnalyzing projection for RD coordinate: ({rd_coord[0]:.3f}, {rd_coord[1]:.3f})")
    
    # Find which road it projects onto
    road_id = find_road_id_for_position(road_index, rd_coord[0], rd_coord[1])
    
    # Get road name and details
    road_name = "Unknown"
    road_length = 0
    road_data = None
    
    for rname, rdata in road_index.get('roads', {}).items():
        if rdata.get('id') == road_id:
            road_name = rname
            road_length = rdata.get('length', 0)
            road_data = rdata
            break
    
    print(f"  Projects onto: {road_name} (id={road_id}, length={road_length:.1f}m)")
    
    # Project to get (s,t)
    s_val, t_val = dutils.project_world_to_st(road_index, rd_coord)
    print(f"  Projected (s,t): ({s_val:.1f}, {t_val:.3f})")
    
    # Determine if this road is likely part of R1 (pit) or R2 (lap)
    # Based on README: R1 = pit lane, R2 = main racing track
    likely_route = "Unknown"
    if 'pit' in road_name.lower() or 'lane' in road_name.lower():
        likely_route = "R1 (pit)"
    elif 'corkscrew' in road_name.lower() or 'hairpin' in road_name.lower():
        likely_route = "R2 (lap)"
    else:
        # Check road length - main track is longer
        if road_length > 2000:
            likely_route = "R2 (lap) - long road"
        elif road_length < 1000:
            likely_route = "R1 (pit) - short road"
    
    print(f"  Likely route: {likely_route}")
    
    return {
        'road_id': road_id,
        'road_name': road_name,
        'road_length': road_length,
        's_t': (s_val, t_val),
        'likely_route': likely_route
    }


def test_all_coordinates(road_index, coordinate_transform):
    """Test all test coordinates and analyze their projections."""
    print("="*80)
    print("Route-Specific Projection Analysis")
    print("="*80)
    
    results = {}
    
    for name, data in TEST_COORDINATES.items():
        print(f"\n{'='*80}")
        print(f"{name}")
        print(f"{'='*80}")
        
        scenic_xodr = data['scenic_xodr'][:2]
        expected_rd = data['expected_rd']
        expected_s_t = data['expected_s_t']
        
        print(f"Scenic XODR: ({scenic_xodr[0]:.3f}, {scenic_xodr[1]:.3f})")
        
        # Transform XODR → RD
        if coordinate_transform:
            actual_rd = apply_coordinate_transform(coordinate_transform, scenic_xodr)
            print(f"Transformed RD: ({actual_rd[0]:.3f}, {actual_rd[1]:.3f})")
            print(f"Expected RD:    ({expected_rd[0]:.3f}, {expected_rd[1]:.3f})")
            rd_error = math.sqrt((actual_rd[0] - expected_rd[0])**2 + (actual_rd[1] - expected_rd[1])**2)
            print(f"Transform error: {rd_error:.6f} m")
            test_rd = actual_rd
        else:
            test_rd = expected_rd
        
        # Analyze projection
        analysis = analyze_projection(road_index, test_rd, coordinate_transform)
        
        # Compare with expected (s,t)
        exp_s, exp_t = expected_s_t
        act_s, act_t = analysis['s_t']
        s_error = abs(act_s - exp_s)
        t_error = abs(act_t - exp_t)
        
        print(f"Expected (s,t): ({exp_s:.1f}, {exp_t:.3f})")
        print(f"Actual (s,t):   ({act_s:.1f}, {act_t:.3f})")
        print(f"Errors: s={s_error:.3f}m, t={t_error:.3f}m")
        
        results[name] = {
            'scenic_xodr': scenic_xodr,
            'rd': test_rd,
            'expected_s_t': expected_s_t,
            'actual_s_t': analysis['s_t'],
            'road_name': analysis['road_name'],
            'likely_route': analysis['likely_route'],
            's_error': s_error,
            't_error': t_error
        }
    
    # Summary
    print("\n" + "="*80)
    print("Summary")
    print("="*80)
    
    print("\nRoad Distribution:")
    road_counts = {}
    for name, result in results.items():
        road = result['road_name']
        road_counts[road] = road_counts.get(road, 0) + 1
    
    for road, count in road_counts.items():
        print(f"  {road}: {count} coordinate(s)")
    
    print("\nRoute Distribution:")
    route_counts = {}
    for name, result in results.items():
        route = result['likely_route']
        route_counts[route] = route_counts.get(route, 0) + 1
    
    for route, count in route_counts.items():
        print(f"  {route}: {count} coordinate(s)")
    
    print("\nProjection Errors:")
    avg_s_error = sum(r['s_error'] for r in results.values()) / len(results)
    avg_t_error = sum(r['t_error'] for r in results.values()) / len(results)
    max_s_error = max(r['s_error'] for r in results.values())
    max_t_error = max(r['t_error'] for r in results.values())
    
    print(f"  Average s error: {avg_s_error:.3f} m")
    print(f"  Average t error: {avg_t_error:.3f} m")
    print(f"  Max s error:     {max_s_error:.3f} m")
    print(f"  Max t error:     {max_t_error:.3f} m")
    
    # Analysis
    print("\n" + "="*80)
    print("Analysis")
    print("="*80)
    
    if avg_s_error > 10.0:
        print("\n[FINDING] Large s-coordinate errors detected!")
        print("  This suggests the projection is using the wrong road or coordinate system.")
        print("  Possible causes:")
        print("    - Projection onto wrong road (pit vs main track)")
        print("    - Route coordinate system mismatch (s computed for wrong route)")
    
    if len(set(r['likely_route'] for r in results.values())) > 1:
        print("\n[FINDING] Coordinates project onto different routes!")
        print("  Some coordinates are on R1 (pit), others on R2 (lap).")
        print("  This is expected if coordinates are distributed across the track.")
        print("  However, if all should be on the same route, this indicates a problem.")
    
    # Check if all coordinates should be on the same route
    all_routes = set(r['likely_route'] for r in results.values())
    if len(all_routes) == 1:
        route = list(all_routes)[0]
        print(f"\n[FINDING] All coordinates project onto the same route: {route}")
        print("  If these coordinates should all be on this route, projection is consistent.")
        print("  The bug may be in how (s,t) is interpreted by dSPACE for this route.")
    else:
        print(f"\n[FINDING] Coordinates project onto multiple routes: {all_routes}")
        print("  This may be correct if coordinates span different parts of the track.")
        print("  However, verify that each coordinate is assigned to the correct route in ModelDesk.")
    
    return results


def main():
    """Main function."""
    print("="*80)
    print("Route-Specific Projection Test")
    print("="*80)
    print("\nThis script analyzes which road each coordinate projects onto")
    print("and determines the likely route (R1 pit vs R2 lap).")
    print("\nThis helps identify if:")
    print("  - Coordinates are projecting onto the wrong road")
    print("  - Route assignment is incorrect")
    print("  - Route coordinate systems are causing the mismatch")
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
    
    # Load road index
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = None
    if rd_path.exists():
        try:
            road_index = build_rd_road_index(str(rd_path), step=0.5)
            print(f"[OK] Loaded road index")
            
            # Print road info
            print(f"\nRoads in index:")
            for road_name, road_data in road_index.get('roads', {}).items():
                length = road_data.get('length', 0)
                print(f"  - {road_name}: {length:.1f}m")
        except Exception as e:
            print(f"\n[ERROR] Could not load road index: {e}")
            import traceback
            traceback.print_exc()
            return 1
    else:
        print(f"\n[ERROR] RD file not found: {rd_path}")
        return 1
    
    # Run analysis
    results = test_all_coordinates(road_index, coordinate_transform)
    
    print("\n" + "="*80)
    print("Test Complete")
    print("="*80)
    print("\nNext steps:")
    print("  1. Check if all coordinates should be on the same route")
    print("  2. Verify route assignment in ModelDesk matches the likely route")
    print("  3. If routes differ, test each coordinate on its correct route")
    print("  4. Compare with results from isolate_transformation_bug.py")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())
