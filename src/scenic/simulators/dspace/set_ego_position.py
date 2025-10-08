"""
Set Ego Vehicle Starting Position in ModelDesk

This script:
1. Opens the "LagunaSeca_ExternalControl" scenario
2. Creates a copy using "Save As"
3. Modifies the ego vehicle's starting position and properties
4. Saves, downloads, resets, and starts the simulation

Usage:
    python set_ego_position.py --s 800.0 --velocity 15.0 --lane 0
    
Or import and use:
    from set_ego_position import set_ego_vehicle_position
    set_ego_vehicle_position(s_position=800.0, velocity=15.0, lane_index=0)
"""

import pythoncom
from win32com.client import Dispatch
import time
import sys
import argparse


def set_ego_vehicle_position(
    s_position=400.0,
    t_position=0.0,
    velocity=0.0,
    orientation=0.0,
    lane_index=0,
    height=0.1,
    source_scenario="LagunaSeca_ExternalControl",
    new_scenario_name=None
):
    """
    Set the ego vehicle's starting position in ModelDesk.
    
    Args:
        s_position: Longitudinal position along the road (meters)
        t_position: Lateral offset from road centerline (meters)
        velocity: Initial longitudinal velocity (m/s)
        orientation: Vehicle orientation angle (degrees)
        lane_index: Lane index (0 = first lane)
        height: Initial height/z-position (meters)
        source_scenario: Name of the source scenario to copy from
        new_scenario_name: Name for the new scenario (auto-generated if None)
    
    Returns:
        True if successful, False otherwise
    """
    
    print("\n" + "="*80)
    print("SETTING EGO VEHICLE POSITION IN MODELDESK")
    print("="*80)
    print(f"\nParameters:")
    print(f"  Source scenario: {source_scenario}")
    print(f"  S-position: {s_position} m")
    print(f"  T-position: {t_position} m")
    print(f"  Velocity: {velocity} m/s")
    print(f"  Orientation: {orientation} degrees")
    print(f"  Lane index: {lane_index}")
    print(f"  Height: {height} m")
    print()
    
    try:
        # Initialize COM
        pythoncom.CoInitialize()
        print("[1] COM initialized")
        
        # Connect to ModelDesk
        app = Dispatch("ModelDesk.Application")
        print("[2] Connected to ModelDesk.Application")
        
        # Get active project and experiment
        proj = app.ActiveProject
        if proj is None:
            print("[ERROR] No active project. Please open a ModelDesk project first.")
            return False
        print(f"[3] Active project: {proj.Name if hasattr(proj, 'Name') else 'Unknown'}")
        
        exp = proj.ActiveExperiment
        if exp is None:
            print("[ERROR] No active experiment. Please activate an experiment.")
            return False
        print(f"[4] Active experiment: {exp.Name if hasattr(exp, 'Name') else 'Unknown'}")
        
        # Step 1: Activate the source scenario
        print(f"\n[5] Activating source scenario: {source_scenario}")
        try:
            exp.ActivateTrafficScenario(source_scenario)
            print(f"    Successfully activated '{source_scenario}'")
        except Exception as e:
            print(f"    Warning: Could not activate scenario: {e}")
            print(f"    Continuing with currently active scenario...")
        
        # Step 2: Create a copy using SaveAs
        if new_scenario_name is None:
            new_scenario_name = time.strftime("EgoTest_%Y%m%d_%H%M%S")
        
        print(f"\n[6] Creating copy as: {new_scenario_name}")
        try:
            exp.TrafficScenario.SaveAs(new_scenario_name, True)
            print(f"    Successfully saved as '{new_scenario_name}'")
        except Exception as e:
            # Try alternative method using editor
            print(f"    Primary SaveAs failed, trying via editor...")
            try:
                editor = exp.EditTrafficScenario()
                try:
                    editor.SaveAs(new_scenario_name, True)
                    print(f"    Successfully saved via editor")
                finally:
                    try:
                        editor.Close(False)
                    except:
                        pass
            except Exception as e2:
                print(f"    Warning: SaveAs failed: {e2}")
        
        # Step 3: Activate the new scenario
        print(f"\n[7] Activating new scenario: {new_scenario_name}")
        try:
            exp.ActivateTrafficScenario(new_scenario_name)
            print(f"    Successfully activated")
        except Exception as e:
            print(f"    Warning: Could not activate: {e}")
        
        # Refresh handles after SaveAs/Activate
        pythoncom.PumpWaitingMessages()
        time.sleep(0.2)
        proj = app.ActiveProject
        exp = proj.ActiveExperiment
        ts = exp.TrafficScenario
        
        if ts is None:
            print("[ERROR] No TrafficScenario available")
            return False
        
        # Step 4: Access the ego vehicle (Maneuver.Item(0))
        print(f"\n[8] Accessing ego vehicle...")
        maneuver_collection = ts.Maneuver
        
        if maneuver_collection.Count == 0:
            print("[ERROR] No ego maneuver found in scenario")
            return False
        
        ego_maneuver = maneuver_collection.Item(0)
        print(f"    Ego maneuver: {ego_maneuver.Name if hasattr(ego_maneuver, 'Name') else 'Unknown'}")
        
        # Access sequences
        sequences = ego_maneuver.Sequences
        if sequences.Count == 0:
            print("[ERROR] No sequences found in ego maneuver")
            return False
        
        seq = sequences.Item(0)
        print(f"    Accessed sequence (current StartPosition: {seq.StartPosition})")
        
        # Step 5: Modify ego vehicle properties
        print(f"\n[9] Modifying ego vehicle properties...")
        
        # Set starting position
        seq.StartPosition = float(s_position)
        print(f"    Set StartPosition: {s_position} m")
        
        # Set vehicle orientation
        # Note: VehicleOrientation is relative to road direction
        #   0.0 = aligned with road
        #   positive = counter-clockwise from road
        #   negative = clockwise from road
        seq.VehicleOrientation = float(orientation)
        print(f"    Set VehicleOrientation: {orientation} degrees (relative to road)")
        
        # Set lane index
        seq.InitialLaneIndex = int(lane_index)
        print(f"    Set InitialLaneIndex: {lane_index}")
        
        # Set height
        seq.InitialHeight = float(height)
        print(f"    Set InitialHeight: {height} m")
        
        # Set initial velocity
        seq.InitialLongitudinalVelocity = float(velocity)
        print(f"    Set InitialLongitudinalVelocity: {velocity} m/s")
        
        # Optional: If you want to set lateral position (t-coordinate)
        # This would require accessing segments and setting lateral deviation
        # Similar to how we do it for Fellows
        if t_position != 0.0:
            print(f"\n    Setting lateral position (t-coordinate): {t_position} m")
            try:
                segments = seq.Segments
                if segments.Count > 0:
                    seg0 = segments.Item(0)
                    
                    # Access lateral type
                    lat0 = seg0.Activity.LateralType
                    
                    # Try to activate Deviation mode
                    if hasattr(lat0, 'Activate'):
                        try:
                            lat0.Activate("Deviation")
                            
                            # Set dependency to Absolute
                            if hasattr(lat0, 'ActiveElement'):
                                dep = lat0.ActiveElement.DependencyType
                                if hasattr(dep, 'Activate'):
                                    dep.Activate("Absolute")
                            
                            # Set the constant value
                            if hasattr(lat0, 'ActiveElement'):
                                src_type = lat0.ActiveElement.SourceType
                                if hasattr(src_type, 'Activate'):
                                    src_type.Activate("Constant")
                                if hasattr(src_type, 'ActiveElement'):
                                    src_type.ActiveElement.Constant = float(t_position)
                                    print(f"    Successfully set lateral deviation: {t_position} m")
                        except Exception as e:
                            print(f"    Warning: Could not set lateral position: {e}")
            except Exception as e:
                print(f"    Warning: Could not access segments for lateral positioning: {e}")
        
        # Step 6: Save the scenario
        print(f"\n[10] Saving scenario...")
        try:
            ts.Save()
            print(f"    Scenario saved")
        except Exception as e:
            print(f"    Warning: Save failed: {e}")
        
        # Step 7: Download to simulator
        print(f"\n[11] Downloading to simulator...")
        try:
            ts.Download()
            print(f"    Downloaded to simulator")
        except Exception as e:
            print(f"    Warning: Download failed: {e}")
        
        # Step 8: Reset and start simulation
        print(f"\n[12] Resetting and starting simulation...")
        try:
            mc = exp.ManeuverControl
            
            # Stop any running simulation
            try:
                mc.Stop()
                print(f"    Stopped previous simulation")
            except:
                pass
            
            time.sleep(0.2)
            
            # Reset
            mc.Reset()
            print(f"    Reset simulation")
            
            time.sleep(0.2)
            
            # Start
            mc.Start(False)  # False = don't wait for completion
            print(f"    Started simulation")
            
        except Exception as e:
            print(f"    Warning: Could not control simulation: {e}")
        
        print("\n" + "="*80)
        print("SUCCESS: Ego vehicle position set and simulation started!")
        print("="*80 + "\n")
        
        return True
        
    except Exception as e:
        print(f"\n[FATAL ERROR] {e}")
        import traceback
        traceback.print_exc()
        return False
        
    finally:
        try:
            pythoncom.CoUninitialize()
        except:
            pass


def main():
    """Command-line interface for setting ego vehicle position."""
    
    parser = argparse.ArgumentParser(
        description="Set ego vehicle starting position in ModelDesk",
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog="""
Examples:
  # Set ego at s=800m with 15 m/s initial velocity
  python set_ego_position.py --s 800 --velocity 15
  
  # Set ego at s=1200m in lane 1 (second lane)
  python set_ego_position.py --s 1200 --lane 1
  
  # Full control
  python set_ego_position.py --s 600 --t 2.0 --velocity 20 --orientation 45 --lane 0
        """
    )
    
    parser.add_argument('--s', type=float, default=400.0,
                        help='Longitudinal position (s-coordinate) in meters (default: 400.0)')
    parser.add_argument('--t', type=float, default=0.0,
                        help='Lateral position (t-coordinate) in meters (default: 0.0)')
    parser.add_argument('--velocity', type=float, default=0.0,
                        help='Initial velocity in m/s (default: 0.0)')
    parser.add_argument('--orientation', type=float, default=0.0,
                        help='Vehicle orientation in degrees (default: 0.0)')
    parser.add_argument('--lane', type=int, default=0,
                        help='Lane index, 0=first lane (default: 0)')
    parser.add_argument('--height', type=float, default=0.1,
                        help='Initial height/z-position in meters (default: 0.1)')
    parser.add_argument('--source', type=str, default='LagunaSeca_ExternalControl',
                        help='Source scenario name (default: LagunaSeca_ExternalControl)')
    parser.add_argument('--name', type=str, default=None,
                        help='New scenario name (default: auto-generated)')
    
    args = parser.parse_args()
    
    success = set_ego_vehicle_position(
        s_position=args.s,
        t_position=args.t,
        velocity=args.velocity,
        orientation=args.orientation,
        lane_index=args.lane,
        height=args.height,
        source_scenario=args.source,
        new_scenario_name=args.name
    )
    
    return 0 if success else 1


if __name__ == "__main__":
    sys.exit(main())

