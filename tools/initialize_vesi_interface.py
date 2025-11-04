# -*- coding: utf-8 -*-
"""Standalone script to initialize VesiInterface manual control interface.

This script sets all required master switches, race control configuration, 
and enable flags to activate the VesiInterface manual control system in dSPACE.

Usage:
  python tools/initialize_vesi_interface.py

Prerequisites:
  - ControlDesk application must be open
  - Active experiment must be loaded
  - Platform must be available for connection
"""

import sys
import os

# Add the src directory to the path so we can import ControlDeskApp
sys.path.insert(0, os.path.join(os.path.dirname(__file__), '..', 'src'))

from scenic.simulators.dspace.controldesk.connection import ControlDeskApp


def initialize_vesi_interface():
    """Initialize VesiInterface manual control interface.
    
    Sets all required master switches, race control configuration, and enable flags
    to activate the VesiInterface manual control system.
    """
    print("[VesiInterface] Initializing manual control interface...")
    print("=" * 70)
    
    cd = None
    try:
        # Step 1: Connect to ControlDesk
        print("\n[Step 1] Connecting to ControlDesk...")
        cd = ControlDeskApp().connect()
        print("[OK] Connected to ControlDesk")
        
        # Step 2: Go online
        print("\n[Step 2] Going online (starting online calibration)...")
        cd.go_online()
        print("[OK] Online calibration started")
        
        # Step 3: Start measurement
        print("\n[Step 3] Starting measurement...")
        cd.start_measurement()
        print("[OK] Measurement started")
        
        # Step 4: VesiInterface Master Switches
        print("\n[Step 4] Setting master switches...")
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value",
            0.0
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value",
            1.0  # CRITICAL: Enable manual VESI control
        )
        print("[OK] Master switches set")
        
        # Step 5: Race Control Configuration
        print("\n[Step 5] Configuring race control...")
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value",
            0.0  # Intern mode (required for manual control)
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value",
            9  # CRITICAL: System state constant
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value",
            1
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value",
            0
        )
        print("[OK] Race control configured")
        
        # Step 6: Enable Individual Control Channels
        print("\n[Step 6] Enabling control channels...")
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_brake_cmd/Value",
            1
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_gear_cmd/Value",
            1
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_steering_cmd/Value",
            1
        )
        cd.set_var(
            "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_throttle_cmd/Value",
            1
        )
        print("[OK] Control channels enabled")
        
        # Step 7: Initialize all control values to 0
        print("\n[Step 7] Initializing control values to 0...")
        KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
        KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
        KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
        KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
        KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
        KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
        
        cd.set_var(KEY_THROTTLE, 0.0)
        cd.set_var(KEY_BRAKE_FRONT, 0.1)
        cd.set_var(KEY_BRAKE_REAR, 0.1)
        cd.set_var(KEY_STEERING, 0)
        cd.set_var(KEY_GEAR, 0.0)
        cd.set_var(KEY_CLUTCH, 0.0)
        print("[OK] All control values initialized")
        
        print("\n" + "=" * 70)
        print("[VesiInterface] ✅ Initialization complete - manual control ready")
        print("=" * 70)
        return 0
        
    except Exception as e:
        print("\n" + "=" * 70)
        print(f"[VesiInterface] ❌ ERROR - Initialization failed: {e}")
        print("=" * 70)
        import traceback
        traceback.print_exc()
        return 1


def main():
    """Main entry point."""
    print("VesiInterface Manual Control Initialization")
    print("=" * 70)
    print("\nPrerequisites:")
    print("  - ControlDesk application must be open")
    print("  - Active experiment must be loaded")
    print("  - Platform must be available for connection")
    print()
    
    return initialize_vesi_interface()


if __name__ == '__main__':
    sys.exit(main())

