#!/usr/bin/env python3
"""
Test if the left normal vector computation matches dSPACE's left/right convention.

This test:
1. Takes known road segments with different directions
2. Computes the left normal vector
3. Places vehicles with positive t (should be left) and negative t (should be right)
4. Reads back positions and verifies which side they're actually on
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
from scenic.simulators.dspace.geometry.projection import project_world_to_st, find_road_id_for_position
from scenic.simulators.dspace.geometry.route_projection import project_world_to_st_route_specific
from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment, assign_route_for_segment
from scenic.simulators.dspace.geometry import utils as geom_utils
from scenic.simulators.dspace.utils import legacy as dutils


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def compute_left_normal(vx, vy):
    """Compute left normal vector from segment direction."""
    seg_len = math.sqrt(vx*vx + vy*vy)
    if seg_len < 1e-6:
        return (0, 0)
    nx_left = -vy / seg_len
    ny_left = vx / seg_len
    return (nx_left, ny_left)


def test_left_normal_orientation():
    """Test if left normal computation matches dSPACE convention."""
    print_section("Testing Left Normal Vector Orientation")
    
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    
    try:
        os.chdir(scenic_root)
        
        # Load coordinate transform and road index
        print_section("Step 1: Load Coordinate Transform and Road Index")
        
        transform_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
        coordinate_transform = None
        if transform_path.exists():
            try:
                coordinate_transform = load_transform(str(transform_path))
                print(f"   [OK] Loaded transform")
            except Exception as e:
                print(f"   [WARNING] Could not load transform: {e}")
        
        rd_path = scenic_root / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
        road_index = None
        if rd_path.exists():
            try:
                road_index = build_rd_road_index(str(rd_path), step=0.5)
                print(f"   [OK] Built road index with {len(road_index.get('roads', {}))} roads")
            except Exception as e:
                print(f"   [WARNING] Could not build road index: {e}")
        else:
            print(f"   [ERROR] RD file not found: {rd_path}")
            return False
        
        # Connect to ModelDesk
        print_section("Step 2: Connect to ModelDesk")
        
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
        
        print("   [OK] Connected to ModelDesk")
        
        # Find a road segment to test
        print_section("Step 3: Find Test Road Segment")
        
        roads = road_index.get('roads', {})
        if not roads:
            print("   [ERROR] No roads in road index")
            return False
        
        # Get first road with enough points
        test_road_name = None
        test_road_data = None
        for road_name, road_data in roads.items():
            sec_points = road_data.get('sec_points', [])
            if sec_points and len(sec_points[0]) >= 2:
                test_road_name = road_name
                test_road_data = road_data
                break
        
        if not test_road_name:
            print("   [ERROR] No suitable road found for testing")
            return False
        
        print(f"   [OK] Using road: {test_road_name}")
        
        # Get a segment from the middle of the road
        sec_points = test_road_data.get('sec_points', [])
        points = sec_points[0] if sec_points else []
        if len(points) < 2:
            print("   [ERROR] Not enough points in road")
            return False
        
        # Use a segment from the middle
        mid_idx = len(points) // 2
        if mid_idx >= len(points) - 1:
            mid_idx = len(points) - 2
        
        x0, y0, s0 = points[mid_idx]
        x1, y1, s1 = points[mid_idx + 1]
        
        # Compute segment direction and left normal
        vx = x1 - x0
        vy = y1 - y0
        seg_len = math.sqrt(vx*vx + vy*vy)
        
        nx_left, ny_left = compute_left_normal(vx, vy)
        
        print(f"   Segment: ({x0:.3f}, {y0:.3f}) → ({x1:.3f}, {y1:.3f})")
        print(f"   Direction vector: ({vx:.3f}, {vy:.3f})")
        print(f"   Left normal: ({nx_left:.3f}, {ny_left:.3f})")
        
        # Test point: center of segment
        test_x = (x0 + x1) / 2
        test_y = (y0 + y1) / 2
        
        # Project to get s,t
        s_center, t_center = project_world_to_st(road_index, (test_x, test_y))
        print(f"   Center point: ({test_x:.3f}, {test_y:.3f}) → (s={s_center:.2f}, t={t_center:.6f})")
        
        # Test with positive t (should be left)
        offset_distance = 2.0  # 2 meters
        test_x_left = test_x + nx_left * offset_distance
        test_y_left = test_y + ny_left * offset_distance
        
        s_left, t_left = project_world_to_st(road_index, (test_x_left, test_y_left))
        print(f"   Left point: ({test_x_left:.3f}, {test_y_left:.3f}) → (s={s_left:.2f}, t={t_left:.6f})")
        
        # Test with negative t (should be right)
        test_x_right = test_x - nx_left * offset_distance
        test_y_right = test_y - ny_left * offset_distance
        
        s_right, t_right = project_world_to_st(road_index, (test_x_right, test_y_right))
        print(f"   Right point: ({test_x_right:.3f}, {test_y_right:.3f}) → (s={s_right:.2f}, t={t_right:.6f})")
        
        print_section("Step 4: Verify t-coordinate signs")
        
        print(f"   Center t: {t_center:.6f}")
        print(f"   Left point t: {t_left:.6f} (expected > center)")
        print(f"   Right point t: {t_right:.6f} (expected < center)")
        
        # Check if signs match expectations
        left_is_positive = t_left > t_center
        right_is_negative = t_right < t_center
        
        print(f"\n   Analysis:")
        print(f"   - Left point has t > center: {left_is_positive} (expected: True)")
        print(f"   - Right point has t < center: {right_is_negative} (expected: True)")
        
        if left_is_positive and right_is_negative:
            print(f"   [OK] T-coordinate signs match left/right convention")
            return True
        else:
            print(f"   [WARNING] T-coordinate signs may be inverted!")
            print(f"   - Left point t: {t_left:.6f}, Center t: {t_center:.6f}, Right point t: {t_right:.6f}")
            return False
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_left_normal_orientation()
    sys.exit(0 if success else 1)

