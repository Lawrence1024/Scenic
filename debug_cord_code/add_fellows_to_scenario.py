#!/usr/bin/env python3
"""
Script to activate a scenario and add fellows with positions from the scenic file.

This script:
1. Connects to ModelDesk
2. Activates/copies a source scenario
3. Adds 7 fellows with positions from fellow_fixed_placing.scenic
4. Uses the same coordinate transformation pipeline as the main simulator
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
from scenic.simulators.dspace.utils import legacy as dutils


# Fellow positions from fellow_fixed_placing.scenic (XODR coordinates)
FELLOW_POSITIONS = [
    (-101.919263, -457.524908, 0.0),  # fellow1
    (0.948038, -272.443171, 0.0),     # fellow2
    (191.994781, -418.905118, 0.0),   # fellow3
    (162.256104, -693.627649, 0.0),   # fellow4
    (302.064561, -815.646205, 0.0),   # fellow5
    (557.639219, -737.139638, 0.0),   # fellow6
    (599.646200, -466.416118, 0.0),   # fellow7
]


def connect_to_modeldesk():
    """Connect to ModelDesk COM application."""
    print("="*80)
    print("Connecting to ModelDesk...")
    print("="*80)
    
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


def activate_or_copy_scenario(app, exp, source_scenario=None, new_scenario_name=None):
    """Activate an existing scenario or create a copy."""
    print("\n" + "="*80)
    print("Activating/Copying Scenario...")
    print("="*80)
    
    if source_scenario:
        print(f"   Activating source scenario: {source_scenario}")
        try:
            exp.ActivateTrafficScenario(source_scenario)
            print(f"   [OK] Activated '{source_scenario}'")
        except Exception as e:
            print(f"   [WARNING] Could not activate scenario: {e}")
            print(f"   Continuing with currently active scenario...")
    
    if new_scenario_name:
        print(f"   Creating copy as: {new_scenario_name}")
        try:
            exp.TrafficScenario.SaveAs(new_scenario_name, True)
            print(f"   [OK] Created copy '{new_scenario_name}'")
        except Exception as e:
            print(f"   [WARNING] Could not SaveAs: {e}")
            # Try alternative method
            try:
                editor = exp.EditTrafficScenario()
                try:
                    editor.SaveAs(new_scenario_name, True)
                    print(f"   [OK] Created copy '{new_scenario_name}' (via editor)")
                finally:
                    try:
                        editor.Close(False)
                    except:
                        pass
            except Exception as e2:
                print(f"   [ERROR] Failed to create copy: {e2}")
        
        # Activate the new scenario
        try:
            exp.ActivateTrafficScenario(new_scenario_name)
            print(f"   [OK] Activated '{new_scenario_name}'")
        except Exception as e:
            print(f"   [WARNING] Could not activate new scenario: {e}")
    
    # Rebind handles
    pythoncom.PumpWaitingMessages()
    time.sleep(0.2)
    proj = app.ActiveProject
    exp = proj.ActiveExperiment
    ts = exp.TrafficScenario
    
    if ts is None:
        raise RuntimeError("Active experiment has no TrafficScenario.")
    
    print(f"   Active scenario: {ts.Name if hasattr(ts, 'Name') else 'Unknown'}")
    return ts


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
        print(f"      Type: {transform.get('type', 'unknown')}")
        return transform
    except Exception as e:
        print(f"   [WARNING] Could not load transform: {e}")
        return None


def apply_coordinate_transform(transform, pos):
    """Apply coordinate transformation from XODR to RD using Scenic's function."""
    if transform is None:
        return pos
    
    try:
        from scenic.simulators.dspace.geometry.coordinate_transform import apply_coordinate_transform as scenic_apply
        result = scenic_apply(transform, pos)
        return result
    except Exception as e:
        print(f"      [WARNING] Could not use Scenic transform function: {e}")
        return pos


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
        import traceback
        traceback.print_exc()
        return None


def detect_route_from_position(pos_xy, road_index):
    """Detect route from position (simplified - always return 'Lap')."""
    # For now, always use 'Lap' route
    return "Lap"


def set_fellow_route(seq, route_name="R1"):
    """Set the route for a fellow sequence."""
    try:
        route_sel = seq.Route
        # Try to activate the route
        success = False
        try:
            route_sel.Activate(route_name)
            # Verify it was set
            if hasattr(route_sel, 'ActiveElement') and hasattr(route_sel.ActiveElement, 'Name'):
                actual_name = route_sel.ActiveElement.Name
                if actual_name == route_name:
                    success = True
                    print(f"      Route set to: {route_name}")
        except Exception as e1:
            print(f"      [DEBUG] Could not activate '{route_name}': {e1}")
        
        if not success:
            # Try to find available routes
            try:
                available = list(route_sel.AvailableElements)
                available_names = [str(x) for x in available]
                print(f"      [DEBUG] Available routes: {available_names}")
                
                # Prefer R1, then R2, then any non-pit route
                if 'R1' in available_names:
                    route_sel.Activate('R1')
                    print(f"      Route set to: R1")
                elif 'R2' in available_names:
                    route_sel.Activate('R2')
                    print(f"      Route set to: R2")
                else:
                    # Prefer non-pit routes
                    non_pit = [n for n in available_names if 'pit' not in n.lower()]
                    if non_pit:
                        route_sel.Activate(non_pit[0])
                        print(f"      Route set to: {non_pit[0]}")
                    elif available_names:
                        route_sel.Activate(available_names[0])
                        print(f"      Route set to: {available_names[0]}")
            except Exception as e2:
                print(f"      [WARNING] Could not set route: {e2}")
    except Exception as e:
        print(f"      [WARNING] Could not set route: {e}")


def add_fellow(ts, fellow_idx, scenic_x, scenic_y, coordinate_transform, road_index):
    """Add a fellow vehicle to the scenario."""
    print(f"\n   Adding Fellow_{fellow_idx + 1} at XODR ({scenic_x:.3f}, {scenic_y:.3f})...")
    
    # 1) Transform XODR -> RD
    if coordinate_transform:
        rd_x, rd_y = apply_coordinate_transform(coordinate_transform, (scenic_x, scenic_y))
        print(f"      XODR ({scenic_x:.3f}, {scenic_y:.3f}) -> RD ({rd_x:.3f}, {rd_y:.3f})")
    else:
        rd_x, rd_y = scenic_x, scenic_y
        print(f"      Using XODR coordinates directly (no transform)")
    
    # 2) Project RD (x,y) -> (s,t)
    if road_index:
        s_val, t_val = dutils.project_world_to_st(road_index, (rd_x, rd_y))
        print(f"      RD ({rd_x:.3f}, {rd_y:.3f}) -> (s={s_val:.1f}, t={t_val:.3f})")
    else:
        s_val, t_val = 0.0, 0.0
        print(f"      [WARNING] No road index, using default (s=0, t=0)")
    
    # 3) Create Fellow
    F = ts.Fellows.Add()
    try:
        F.Name = f"Fellow_{fellow_idx + 1}"
        print(f"      Created Fellow: {F.Name}")
    except Exception as e:
        F.Name = f"Fellow_{ts.Fellows.Count}"
        print(f"      Created Fellow with fallback name: {F.Name} (error: {e})")
    
    # 4) Configure sequences and segments
    seqs = F.Sequences
    dutils.clear_collection(seqs)
    S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
    segs = dutils.ensure_two_segments(S1)
    
    # 5) Configure segment 0: absolute pose (s, t)
    dutils.configure_seg0_absolute_pose(segs, s=float(s_val), t=float(t_val))
    print(f"      Configured segment 0: s={s_val:.1f}, t={t_val:.3f}")
    
    # 6) Configure segment 1: external control
    try:
        dutils.configure_seg1_motion(segs, v=0.0, t=0.0)
    except Exception as e:
        print(f"      [WARNING] Could not configure segment 1: {e}")
    
    # 7) Set endless transition
    try:
        dutils.make_endless_transition(segs)
    except:
        pass
    
    # 8) Set route
    set_fellow_route(S1, "Lap")
    
    return F


def main():
    """Main function."""
    print("="*80)
    print("Add Fellows to Scenario")
    print("="*80)
    print("\nThis script will:")
    print("  1. Connect to ModelDesk")
    print("  2. Activate/copy a scenario")
    print("  3. Add 7 fellows with positions from fellow_fixed_placing.scenic")
    print("="*80)
    
    # Configuration
    SOURCE_SCENARIO = "LagunaSeca_ExternalControl"  # Change if needed
    NEW_SCENARIO_NAME = time.strftime("Scenic_%Y%m%d_%H%M%S")
    
    try:
        # 1) Connect to ModelDesk
        app, proj, exp = connect_to_modeldesk()
        
        # 2) Activate/copy scenario
        ts = activate_or_copy_scenario(app, exp, SOURCE_SCENARIO, NEW_SCENARIO_NAME)
        
        # 3) Clear existing fellows
        print("\n" + "="*80)
        print("Clearing Existing Fellows...")
        print("="*80)
        try:
            dutils.clear_collection(ts.Fellows)
            print(f"   [OK] Cleared {ts.Fellows.Count} existing fellows")
        except Exception as e:
            print(f"   [WARNING] Could not clear fellows: {e}")
        
        # 4) Load coordinate transformation and road index
        print("\n" + "="*80)
        print("Loading Coordinate Transformation and Road Index...")
        print("="*80)
        coordinate_transform = load_coordinate_transform()
        road_index = build_road_index()
        
        # 5) Add fellows
        print("\n" + "="*80)
        print("Adding Fellows...")
        print("="*80)
        
        for i, (x, y, z) in enumerate(FELLOW_POSITIONS):
            try:
                add_fellow(ts, i, x, y, coordinate_transform, road_index)
            except Exception as e:
                print(f"   [ERROR] Failed to add Fellow_{i+1}: {e}")
                import traceback
                traceback.print_exc()
        
        # 6) Save scenario
        print("\n" + "="*80)
        print("Saving Scenario...")
        print("="*80)
        try:
            ts.Save()
            print(f"   [OK] Scenario saved")
        except Exception as e:
            print(f"   [WARNING] Could not save scenario: {e}")
        
        print("\n" + "="*80)
        print("Complete!")
        print("="*80)
        print(f"\nScenario '{ts.Name}' now has {ts.Fellows.Count} fellows.")
        print("You can now:")
        print("  1. Download the scenario to VEOS")
        print("  2. Run the debug_coordinate_transformation.py script to verify positions")
        
        return 0
        
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()
        return 1


if __name__ == "__main__":
    sys.exit(main())

