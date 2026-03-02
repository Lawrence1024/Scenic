#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
Evaluate TTL CSV files to determine their coordinate system.

Check if TTL files in the 'transformed' folder are in XODR coordinate space
by comparing with known XODR coordinates and the coordinate transformation.
"""
import sys
import os
import csv
from pathlib import Path

# Fix encoding for Windows console
if sys.platform == 'win32':
    sys.stdout.reconfigure(encoding='utf-8')
    sys.stderr.reconfigure(encoding='utf-8')

# Add Scenic src to path
scenic_path = Path(__file__).parent.parent / "src"
if scenic_path.exists():
    sys.path.insert(0, str(scenic_path))

from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_coordinate_transform,
    apply_inverse_coordinate_transform
)
from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
from scenic.simulators.dspace.geometry.route_projection import ROUTE_ROAD_SEQUENCES, ROUTE_TRANSITION_POINTS


def read_ttl_points(csv_path, max_points=100):
    """Read first N points from TTL CSV."""
    points = []
    with open(csv_path, 'r', newline='') as f:
        reader = csv.reader(f)
        try:
            next(reader)  # Skip metadata
        except StopIteration:
            pass
        for i, row in enumerate(reader):
            if i >= max_points:
                break
            if not row or len(row) < 2:
                continue
            try:
                x = float(row[0])
                y = float(row[1])
                points.append((x, y))
            except (ValueError, IndexError):
                continue
    return points


def evaluate_ttl_coordinate_system(ttl_path, transform, road_index):
    """Evaluate what coordinate system TTL points are in."""
    print(f"\nEvaluating: {ttl_path.name}")
    print("=" * 80)
    
    # Read TTL points
    ttl_points = read_ttl_points(ttl_path, max_points=50)
    if not ttl_points:
        print("ERROR: Could not read points from TTL file")
        return None
    
    print(f"Read {len(ttl_points)} sample points")
    print(f"First point: ({ttl_points[0][0]:.6f}, {ttl_points[0][1]:.6f})")
    print(f"Last point: ({ttl_points[-1][0]:.6f}, {ttl_points[-1][1]:.6f})")
    
    # Test hypothesis 1: Points are in XODR space
    print("\n--- Hypothesis 1: Points are in XODR space ---")
    xodr_point = ttl_points[0]
    rd_from_xodr = apply_coordinate_transform(transform, xodr_point)
    print(f"TTL point (assumed XODR): ({xodr_point[0]:.6f}, {xodr_point[1]:.6f})")
    print(f"  -> Transformed to RD: ({rd_from_xodr[0]:.6f}, {rd_from_xodr[1]:.6f})")
    
    # Check if this RD point is on a road
    from scenic.simulators.dspace.geometry.projection import project_world_to_st
    try:
        s_val, t_val = project_world_to_st(road_index, rd_from_xodr)
        print(f"  -> Projects to road: s={s_val:.2f}, t={t_val:.3f}")
        print(f"  [OK] Hypothesis 1: Points ARE in XODR space (projects to road)")
        is_xodr = True
    except Exception as e:
        print(f"  -> Projection error: {e}")
        is_xodr = False
    
    # Test hypothesis 2: Points are in RD space
    print("\n--- Hypothesis 2: Points are in RD space ---")
    rd_point = ttl_points[0]
    print(f"TTL point (assumed RD): ({rd_point[0]:.6f}, {rd_point[1]:.6f})")
    try:
        s_val, t_val = project_world_to_st(road_index, rd_point)
        print(f"  -> Projects to road: s={s_val:.2f}, t={t_val:.3f}")
        print(f"  [OK] Hypothesis 2: Points ARE in RD space (projects to road)")
        is_rd = True
    except Exception as e:
        print(f"  -> Projection error: {e}")
        is_rd = False
    
    # Test hypothesis 3: Points need offset (dx, dy)
    print("\n--- Hypothesis 3: Points need offset (dx=-53.6, dy=-15.7) ---")
    offset_point = (ttl_points[0][0] - 53.6, ttl_points[0][1] - 15.7)
    print(f"TTL point with offset: ({offset_point[0]:.6f}, {offset_point[1]:.6f})")
    
    # Try as XODR
    rd_from_offset_xodr = apply_coordinate_transform(transform, offset_point)
    try:
        s_val, t_val = project_world_to_st(road_index, rd_from_offset_xodr)
        print(f"  -> As XODR -> RD: ({rd_from_offset_xodr[0]:.6f}, {rd_from_offset_xodr[1]:.6f})")
        print(f"  -> Projects to road: s={s_val:.2f}, t={t_val:.3f}")
        is_offset_xodr = True
    except Exception as e:
        print(f"  -> Projection error: {e}")
        is_offset_xodr = False
    
    # Try as RD
    try:
        s_val, t_val = project_world_to_st(road_index, offset_point)
        print(f"  -> As RD: Projects to road: s={s_val:.2f}, t={t_val:.3f}")
        is_offset_rd = True
    except Exception as e:
        print(f"  -> Projection error: {e}")
        is_offset_rd = False
    
    # Summary
    print("\n" + "=" * 80)
    print("SUMMARY")
    print("=" * 80)
    if is_xodr:
        print("[OK] TTL points appear to be in XODR coordinate space")
        print("   -> Can be used directly with Scenic vehicle positions")
        print("   -> No transformation needed for waypoint following")
        return "XODR"
    elif is_rd:
        print("[OK] TTL points appear to be in RD coordinate space")
        print("   -> Need inverse transform to convert to XODR for waypoint following")
        return "RD"
    elif is_offset_xodr:
        print("[OK] TTL points appear to be in XODR space but need offset removed")
        print("   -> Points are already transformed, just need to use directly")
        return "XODR_OFFSET"
    elif is_offset_rd:
        print("[WARN] TTL points appear to be in RD space with offset")
        print("   -> Need to apply offset then inverse transform")
        return "RD_OFFSET"
    else:
        print("[ERROR] Could not determine coordinate system")
        print("   -> Points may be in a different coordinate system")
        return "UNKNOWN"


def main():
    """Main function."""
    scenic_root = Path(__file__).parent.parent
    ttl_folder = scenic_root / "assets" / "ttls" / "LS_ENU_TTL_CSV"
    
    # Load coordinate transform
    transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    transform = load_transform(str(transform_path))
    print("Loaded coordinate transformation")
    
    # Load road index
    rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    road_index = build_rd_road_index(str(rd_path))
    print("Loaded RD road index")
    
    # Evaluate TTL files
    ttl_files = sorted(ttl_folder.glob("ttl_*.csv"))
    if not ttl_files:
        print(f"ERROR: No TTL files found in {ttl_folder}")
        return 1
    
    print(f"\nFound {len(ttl_files)} TTL files to evaluate")
    print("=" * 80)
    
    results = {}
    for ttl_file in ttl_files[:3]:  # Evaluate first 3 files
        result = evaluate_ttl_coordinate_system(ttl_file, transform, road_index)
        results[ttl_file.name] = result
    
    # Overall conclusion
    print("\n" + "=" * 80)
    print("OVERALL CONCLUSION")
    print("=" * 80)
    unique_results = set(results.values())
    if len(unique_results) == 1:
        coord_system = list(unique_results)[0]
        print(f"All TTL files appear to be in: {coord_system} coordinate space")
        if coord_system == "XODR":
            print("\n[OK] RECOMMENDATION: Use TTL points directly (no transformation needed)")
            print("   -> Waypoint following should work correctly")
        elif coord_system == "RD":
            print("\n[WARN] RECOMMENDATION: Transform TTL points from RD -> XODR")
            print("   -> Use apply_inverse_coordinate_transform() before waypoint following")
        else:
            print(f"\n[WARN] RECOMMENDATION: Coordinate system is {coord_system}")
    else:
        print("[WARN] WARNING: TTL files appear to be in different coordinate systems")
        print(f"   Results: {results}")
    
    return 0


if __name__ == "__main__":
    sys.exit(main())

