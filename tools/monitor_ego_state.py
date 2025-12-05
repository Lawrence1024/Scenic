"""Monitor ego vehicle state variables in ControlDesk in real-time.

This script continuously reads ego state variables to verify they're updating correctly.

Usage:
    python monitor_ego_state.py
    
Press Ctrl+C to stop.
"""

import pythoncom
from win32com.client import Dispatch
import time

def monitor_ego_state():
    """Monitor ego vehicle state variables in real-time."""
    try:
        # Initialize COM
        pythoncom.CoInitialize()
        
        # Connect to ControlDesk
        print("[ControlDesk] Connecting...")
        app = Dispatch("ControlDeskNG.Application")
        print("[ControlDesk] Connected!")
        
        # Get platform and variables
        exp = app.ActiveExperiment
        platforms = exp.Platforms
        
        try:
            outer = platforms.Item("Platform")
        except:
            outer = platforms.Item(0)
        
        try:
            inner_plats = outer.Platforms
            inner = inner_plats.Item("Platform_2")
        except:
            inner = outer
        
        vdesc = inner.ActiveVariableDescription
        variables = vdesc.Variables
        
        # Define paths to test
        base_path = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel"
        
        paths_to_test = {
            "Current x": f"{base_path}/Ego_x/Value",
            "Current y": f"{base_path}/Ego_y/Value",
            "Current z": f"{base_path}/Ego_z/Value",
            "Current yaw": f"{base_path}/Ego_yaw/Value",
            "Current velocity": f"{base_path}/Ego_velocity/Value",
        }
        
        print(f"\n{'='*80}")
        print("MONITORING EGO VEHICLE STATE (Press Ctrl+C to stop)")
        print(f"{'='*80}\n")
        
        print("Testing which paths are accessible...\n")
        
        # Test which paths work
        working_paths = {}
        for name, path in paths_to_test.items():
            try:
                value = variables[path].ValueConverted
                working_paths[name] = path
                print(f"✓ {name}: {path}")
            except Exception as e:
                print(f"✗ {name}: {path}")
                print(f"  Error: {e}")
        
        if not working_paths:
            print("\n[ERROR] None of the expected paths are accessible!")
            print("\nRun dump_controldesk_variables.py to find the correct paths.")
            return
        
        print(f"\n{'='*80}")
        print(f"MONITORING {len(working_paths)} VARIABLE(S)")
        print(f"{'='*80}\n")
        print(f"{'Time':>8} | {'Variable':<20} | {'Value':>15} | {'Change':>10}")
        print("-" * 80)
        
        # Store previous values to detect changes
        prev_values = {}
        iteration = 0
        
        while True:
            iteration += 1
            current_time = time.strftime("%H:%M:%S")
            
            for name, path in working_paths.items():
                try:
                    value = variables[path].ValueConverted
                    
                    # Check if value changed
                    if name in prev_values:
                        if abs(value - prev_values[name]) > 0.001:
                            change = f"+{value - prev_values[name]:.3f}"
                            marker = "📈"
                        else:
                            change = "---"
                            marker = "⏸️ "
                    else:
                        change = "INITIAL"
                        marker = "🆕"
                    
                    # Print every 10 iterations or when value changes
                    if iteration % 10 == 0 or change != "---":
                        print(f"{current_time} | {marker} {name:<17} | {value:>15.3f} | {change:>10}")
                    
                    prev_values[name] = value
                    
                except Exception as e:
                    print(f"{current_time} | ❌ {name:<17} | Error: {e}")
            
            if iteration % 10 == 0:
                print("-" * 80)
            
            time.sleep(0.1)  # Read at 10 Hz
            
    except KeyboardInterrupt:
        print("\n\n[Stopped] Monitoring stopped by user")
    except Exception as e:
        print(f"\n[ERROR] {e}")
        import traceback
        traceback.print_exc()

if __name__ == "__main__":
    monitor_ego_state()


