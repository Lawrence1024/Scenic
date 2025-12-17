#!/usr/bin/env python3
"""
Test left/right relative positioning for static vehicle placement.

This script:
1. Generates vehicles using "left of" and "right of" specifiers
2. Verifies relative positions in Scenic coordinates
3. Transforms to dSPACE and places in ModelDesk
4. Reads back from ControlDesk and verifies round-trip accuracy
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
    load_transform,
    apply_coordinate_transform,
    apply_inverse_coordinate_transform
)


def print_section(title):
    """Print a section header."""
    print("\n" + "="*80)
    print(title)
    print("="*80)


def test_left_right_placement():
    """Test left/right relative positioning."""
    print_section("Testing Left/Right Relative Positioning")
    
    # Change to Scenic root directory
    scenic_root = Path(__file__).parent.parent
    original_cwd = Path.cwd()
    try:
        os.chdir(scenic_root)
        
        # Step 1: Generate scenario with relative positioning
        print_section("Step 1: Generate Scenario with Left/Right Placement")
        
        scenario_code = """
param map = localPath('assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Reference vehicle
ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Vehicles placed relative to ego
fellow1 = new RacingCar left of ego by 5.0, with raceNumber 2
fellow2 = new RacingCar right of ego by 5.0, with raceNumber 3
"""
        
        print("   Generating scenario with relative positioning...")
        scenario = scenarioFromString(scenario_code)
        scene, iterations = scenario.generate(maxIterations=10)
        
        if len(scene.objects) < 3:
            print("   [ERROR] Expected at least 3 objects (ego + 2 fellows)")
            return False
        
        ego = scene.objects[0]
        fellow1 = scene.objects[1]
        fellow2 = scene.objects[2]
        
        print(f"   [OK] Generated {len(scene.objects)} vehicles")
        print(f"   [OK] Iterations needed: {iterations}")
        
        # Step 2: Verify relative positions in Scenic
        print_section("Step 2: Verify Relative Positions in Scenic")
        
        ego_pos = (float(ego.position.x), float(ego.position.y))
        fellow1_pos = (float(fellow1.position.x), float(fellow1.position.y))
        fellow2_pos = (float(fellow2.position.x), float(fellow2.position.y))
        
        print(f"   Ego position (XODR): ({ego_pos[0]:.6f}, {ego_pos[1]:.6f})")
        print(f"   Fellow1 position (XODR): ({fellow1_pos[0]:.6f}, {fellow1_pos[1]:.6f})")
        print(f"   Fellow2 position (XODR): ({fellow2_pos[0]:.6f}, {fellow2_pos[1]:.6f})")
        
        # Calculate distances (simplified - would need proper orientation for left/right)
        dist1 = math.sqrt((fellow1_pos[0] - ego_pos[0])**2 + (fellow1_pos[1] - ego_pos[1])**2)
        dist2 = math.sqrt((fellow2_pos[0] - ego_pos[0])**2 + (fellow2_pos[1] - ego_pos[1])**2)
        
        print(f"   Distance ego-fellow1: {dist1:.3f}m (expected ~5.0m)")
        print(f"   Distance ego-fellow2: {dist2:.3f}m (expected ~5.0m)")
        
        # TODO: Add proper left/right verification using orientation
        # TODO: Transform to dSPACE and place in ModelDesk
        # TODO: Read back from ControlDesk and verify round-trip
        
        print("\n   [NOTE] Full dSPACE integration test not yet implemented")
        print("   [NOTE] This is a template - complete the implementation")
        
        return True
        
    except Exception as e:
        print(f"   [ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        return False
    finally:
        os.chdir(original_cwd)


if __name__ == "__main__":
    success = test_left_right_placement()
    sys.exit(0 if success else 1)

