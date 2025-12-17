#!/usr/bin/env python3
"""
Verify dSPACE RD coordinate system orientation.

This test:
1. Places vehicles at known positions in Scenic (ENU)
2. Transforms to RD coordinates
3. Analyzes the relationship between Scenic ENU and dSPACE RD

Key checks:
- Does +X in RD = East in ENU?
- Does +Y in RD = North in ENU?
- Does left normal computation match dSPACE's left/right?
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
from scenic.simulators.dspace.geometry.projection import project_world_to_st


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_dspace_coordinate_system():
    """Verify dSPACE RD coordinate system orientation."""
    print_section("Testing dSPACE RD Coordinate System Orientation")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Load coordinate transform
        print_section("Step 1: Load Coordinate Transform")
        
        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        coordinate_transform = None
        if transform_path.exists():
            try:
                coordinate_transform = load_transform(str(transform_path))
                print(f"   [OK] Loaded transform: {coordinate_transform.get('type', 'unknown')}")
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        else:
            print(f"   [WARNING] Transform file not found")
        
        # Load road index
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
                print(f"   [OK] Built road index")
            except Exception as e:
                print(f"   [WARNING] Could not build road index: {e}")
        
        if not road_index:
            print("   [ERROR] Could not build road index")
            return False
        
        # Step 2: Analyze coordinate transformation
        print_section("Step 2: Analyze Coordinate Transformation")
        
        # Test points in Scenic ENU (known directions)
        # In ENU: +X = East, +Y = North
        test_points = [
            ("Origin", (0.0, 0.0)),
            ("East", (100.0, 0.0)),  # 100m East
            ("North", (0.0, 100.0)),  # 100m North
            ("West", (-100.0, 0.0)),  # 100m West
            ("South", (0.0, -100.0)),  # 100m South
        ]
        
        print("   Testing coordinate transformation:")
        print("   Scenic ENU → RD")
        print("   " + "-"*70)
        print(f"   {'Name':<10} {'XODR (ENU)':<25} {'RD':<25} {'Delta':<15}")
        print("   " + "-"*70)
        
        origin_rd = None
        for name, (xodr_x, xodr_y) in test_points:
            if coordinate_transform:
                rd_x, rd_y = apply_coordinate_transform(coordinate_transform, (xodr_x, xodr_y))
            else:
                rd_x, rd_y = xodr_x, xodr_y
            
            if name == "Origin":
                origin_rd = (rd_x, rd_y)
                delta = (0.0, 0.0)
            else:
                delta = (rd_x - origin_rd[0], rd_y - origin_rd[1])
            
            print(f"   {name:<10} ({xodr_x:8.2f}, {xodr_y:8.2f})  ({rd_x:8.2f}, {rd_y:8.2f})  ({delta[0]:7.2f}, {delta[1]:7.2f})")
        
        # Step 3: Analyze road segment directions
        print_section("Step 3: Analyze Road Segment Directions")
        
        roads = road_index.get('roads', {})
        if not roads:
            print("   [ERROR] No roads in road index")
            return False
        
        print("   Analyzing road segment directions:")
        print("   " + "-"*70)
        print(f"   {'Road':<25} {'Segment Direction':<25} {'Left Normal':<20}")
        print("   " + "-"*70)
        
        for road_name, road_data in list(roads.items())[:5]:  # First 5 roads
            sec_points = road_data.get('sec_points', [])
            if not sec_points or len(sec_points[0]) < 2:
                continue
            
            points = sec_points[0]
            # Get first segment
            x0, y0, s0 = points[0]
            x1, y1, s1 = points[1]
            
            vx = x1 - x0
            vy = y1 - y0
            seg_len = math.sqrt(vx*vx + vy*vy)
            
            if seg_len < 1e-6:
                continue
            
            # Normalize direction
            dir_x = vx / seg_len
            dir_y = vy / seg_len
            
            # Compute left normal
            nx_left = -vy / seg_len
            ny_left = vx / seg_len
            
            # Compute angle
            dir_angle = math.atan2(dir_y, dir_x)
            left_angle = math.atan2(ny_left, nx_left)
            
            print(f"   {road_name[:24]:<25} ({dir_x:6.3f}, {dir_y:6.3f}) {math.degrees(dir_angle):6.1f}°  ({nx_left:6.3f}, {ny_left:6.3f}) {math.degrees(left_angle):6.1f}°")
        
        print("\n   [NOTE] In standard right-handed coordinate system:")
        print("   - Left normal should be 90° CCW from forward direction")
        print("   - If forward is (1, 0) [East], left should be (0, 1) [North]")
        print("   - If forward is (0, 1) [North], left should be (-1, 0) [West]")
        
        return True
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_dspace_coordinate_system()
    sys.exit(0 if success else 1)

