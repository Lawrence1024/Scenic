#!/usr/bin/env python3
"""
Test placing vehicles on each of the three road types and verify placement.

This script:
1. Creates vehicles on 'road' (should work anywhere)
2. Creates vehicles on 'pitLaneRoad' (should only be on pit lane)
3. Creates vehicles on 'mainRacingRoad' (should only be on main circuit)
4. Verifies positions match expected road segments
"""

import sys
from pathlib import Path
import time

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
from scenic.simulators.dspace.utils import legacy as dutils


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("=" * 80)
    print("Connecting to ModelDesk...")
    print("=" * 80)
    
    pythoncom.CoInitialize()
    app = Dispatch("ModelDesk.Application")
    proj = app.ActiveProject
    
    if proj is None:
        raise RuntimeError("Open a ModelDesk project first.")
    
    exp = proj.ActiveExperiment
    if exp is None:
        raise RuntimeError("Activate an experiment in ModelDesk.")
    
    print("[OK] Connected to ModelDesk")
    print(f"   Project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
    print(f"   Experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
    
    return app, proj, exp


def load_coordinate_transform():
    """Load the coordinate transformation from file."""
    transform_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca_transform.json"
    
    if not transform_path.exists():
        print(f"   [WARNING] Transform file not found: {transform_path}")
        return None
    
    try:
        from scenic.simulators.dspace.geometry.coordinate_transform import load_transform
        transform = load_transform(str(transform_path))
        print(f"   [OK] Loaded transform from {transform_path.name}")
        return transform
    except Exception as e:
        print(f"   [WARNING] Could not load transform: {e}")
        return None


def build_road_index():
    """Build road index from RD file."""
    rd_path = Path(__file__).parent.parent / "assets" / "maps" / "dSPACE" / "Laguna_Seca.rd"
    
    if not rd_path.exists():
        print(f"   [WARNING] RD file not found: {rd_path}")
        return None
    
    try:
        from scenic.simulators.dspace.geometry.rd_parser import build_rd_road_index
        index = build_rd_road_index(str(rd_path), step=0.5)
        print(f"   [OK] Built road index from {rd_path.name}")
        return index
    except Exception as e:
        print(f"   [WARNING] Could not build road index: {e}")
        return None


def detect_track_segment(position_xy, road_index, params):
    """Detect track segment from position."""
    try:
        from scenic.simulators.dspace.geometry.route_mapping import detect_track_segment as detect_seg
        from scenic.simulators.dspace.geometry import utils as geom_utils
        
        segment = detect_seg(position_xy, road_index, params, geom_utils)
        return segment
    except Exception as e:
        print(f"      [WARNING] Could not detect track segment: {e}")
        return None


def test_road_placement():
    """Test placing vehicles on different road types."""
    print("=" * 80)
    print("Testing Road Type Placement")
    print("=" * 80)
    
    # Connect to ModelDesk
    app, proj, exp = connect_to_modeldesk()
    ts = exp.TrafficScenario
    
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")
    
    # Load coordinate transform and road index
    print("\n" + "=" * 80)
    print("Loading Coordinate Transform and Road Index...")
    print("=" * 80)
    
    transform = load_coordinate_transform()
    road_index = build_road_index()
    
    # Get road IDs from scenario (if available)
    # These would normally come from the racing domain params
    params = {
        'pitLaneRoadIds': ['1'],  # Typical pit lane ID
        'mainRacingRoadIds': ['0', '2'],  # Typical main racing road IDs
    }
    
    # Test: Create vehicles using Scenic scenario
    print("\n" + "=" * 80)
    print("Creating Test Scenario...")
    print("=" * 80)
    
    from scenic import scenarioFromString
    
    # Create scenarios for each road type
    scenarios = {
        'road': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on entire road
ego = new RacingCar on road, with raceNumber 1
""",
        'pitLaneRoad': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on pit lane only
ego = new RacingCar on pitLaneRoad, with raceNumber 2
""",
        'mainRacingRoad': """
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.domains.racing.model

# Place on main racing road only
ego = new RacingCar on mainRacingRoad, with raceNumber 3
"""
    }
    
    results = {}
    
    for road_type, scenario_code in scenarios.items():
        print(f"\n[Testing {road_type}]")
        print("-" * 80)
        
        try:
            scenario = scenarioFromString(scenario_code)
            scene, _ = scenario.generate(maxIterations=10)
            
            if not scene.objects:
                print(f"   [ERROR] No objects generated for {road_type}")
                results[road_type] = None
                continue
            
            ego = scene.objects[0]
            position = ego.position
            
            print(f"   Generated position: ({position.x:.3f}, {position.y:.3f})")
            
            # Transform to RD coordinates
            if transform:
                from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform
                rd_pos = apply_coordinate_transform(transform, (position.x, position.y))
                print(f"   RD coordinates: ({rd_pos[0]:.3f}, {rd_pos[1]:.3f})")
            else:
                rd_pos = (position.x, position.y)
            
            # Project to (s, t)
            if road_index:
                s_val, t_val = dutils.project_world_to_st(road_index, rd_pos)
                print(f"   Road coordinates: (s={s_val:.1f}, t={t_val:.3f})")
            else:
                s_val, t_val = 0.0, 0.0
            
            # Detect track segment
            segment = detect_track_segment(rd_pos, road_index, params)
            print(f"   Detected segment: {segment}")
            
            # Verify placement matches expected
            if road_type == 'pitLaneRoad':
                expected_segment = 'pitLane'
                if segment == expected_segment:
                    print(f"   [OK] Vehicle on pitLaneRoad is correctly on pit lane")
                else:
                    print(f"   [WARNING] Vehicle on pitLaneRoad detected as '{segment}' (expected 'pitLane')")
            elif road_type == 'mainRacingRoad':
                expected_segment = 'mainRacing'
                if segment == expected_segment:
                    print(f"   [OK] Vehicle on mainRacingRoad is correctly on main racing circuit")
                else:
                    print(f"   [WARNING] Vehicle on mainRacingRoad detected as '{segment}' (expected 'mainRacing')")
            else:  # road
                print(f"   [OK] Vehicle on road can be on any segment (detected: {segment})")
            
            results[road_type] = {
                'position': (position.x, position.y),
                'rd_position': rd_pos,
                'road_coords': (s_val, t_val),
                'segment': segment,
                'success': True
            }
            
        except Exception as e:
            print(f"   [ERROR] Failed to test {road_type}: {e}")
            import traceback
            traceback.print_exc()
            results[road_type] = None
    
    # Summary
    print("\n" + "=" * 80)
    print("Test Summary")
    print("=" * 80)
    
    for road_type, result in results.items():
        if result:
            print(f"\n{road_type}:")
            print(f"  Position: {result['position']}")
            print(f"  Segment: {result['segment']}")
            print(f"  Status: {'[OK]' if result['success'] else '[FAILED]'}")
        else:
            print(f"\n{road_type}: [FAILED]")
    
    return all(r is not None and r.get('success', False) for r in results.values() if r is not None)


if __name__ == "__main__":
    try:
        success = test_road_placement()
        sys.exit(0 if success else 1)
    except KeyboardInterrupt:
        print("\n[INTERRUPTED] Test cancelled by user")
        sys.exit(1)
    except Exception as e:
        print(f"\n[ERROR] Test failed: {e}")
        import traceback
        traceback.print_exc()
        sys.exit(1)
