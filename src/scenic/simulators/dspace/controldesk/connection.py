# -*- coding: utf-8 -*-
"""Lightweight ControlDesk COM automation wrapper.

Provides a minimal API for:
 - Going online/offline (online calibration)
 - Starting/stopping measurement
 - Reading/writing model variables via ActiveVariableDescription

The COM application object is created similarly to ModelDesk automation.
"""

from typing import Any


class ControlDeskApp:
    def __init__(self, prog_id: str = "ControlDeskNG.Application",
                 outer_platform_name: str = "Platform",
                 inner_platform_name: str = "Platform_2"):
        self._prog_id = prog_id
        self._outer_platform_name = outer_platform_name
        self._inner_platform_name = inner_platform_name
        self.app = None

    def connect(self):
        import pythoncom
        from win32com.client import Dispatch
        pythoncom.CoInitialize()
        self.app = Dispatch(self._prog_id)
        return self

    # Online calibration (Go Online / Offline)
    def go_online(self):
        self.app.CalibrationManagement.StartOnlineCalibration()

    def go_offline(self):
        self.app.CalibrationManagement.StopOnlineCalibration()

    # Measurement (Start/Stop measuring)
    def start_measurement(self):
        self.app.MeasurementDataManagement.Start()

    def stop_measurement(self):
        self.app.MeasurementDataManagement.Stop()

    # Variable access
    def _get_variables(self):
        exp = self.app.ActiveExperiment
        plats = exp.Platforms
        try:
            outer = plats.Item(self._outer_platform_name)
        except Exception:
            outer = plats[0]
        try:
            inner_plats = outer.Platforms
            inner = inner_plats.Item(self._inner_platform_name)
        except Exception:
            inner = outer
        vdesc = inner.ActiveVariableDescription
        return vdesc.Variables

    def get_var(self, path: str) -> Any:
        vars_obj = self._get_variables()
        return vars_obj[path].ValueConverted

    def set_var(self, path: str, value: Any):
        vars_obj = self._get_variables()
        vars_obj[path].ValueConverted = value

    # Maneuver control
    def start_maneuver(self):
        """Start the active experiment's maneuver.
        
        Pulses the MANEUVER_START variable: sets it to 1, waits briefly, then resets to 0.
        """
        import time
        
        maneuver_start_path = (
            "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
            "UserInterface/PAR_Plant/ManeuverControl/MANEUVER_START/MDLDCtrl_ManeuverStart"
        )
        
        self.set_var(maneuver_start_path, 1.0)
        time.sleep(0.1)
        self.set_var(maneuver_start_path, 0.0)
        print("[ControlDesk] Maneuver started")

    def stop_maneuver(self):
        """Stop the active experiment's maneuver."""
        exp = self.app.ActiveExperiment
        mc = exp.ManeuverControl
        mc.Stop()

    def reset_maneuver(self):
        """Reset the active experiment's maneuver."""
        exp = self.app.ActiveExperiment
        mc = exp.ManeuverControl
        mc.Reset()

    # Simulation control
    def set_simulation_step(self, step=0.01):
        """Set the simulation time step."""
        try:
            platform = self.app.PlatformManagement.Platforms.Item(0)
            platform.SimulationTimeOptions.SingleStepTime = str(step)
            print(f"[ControlDesk] Simulation step set to {step} seconds")
        except Exception as e:
            print(f"[ControlDesk] Error setting simulation step: {e}")

    def pause_simulation(self):
        """Pause the dSPACE simulation for step-by-step control.
        
        This should be called once during setup to put the simulation into
        a paused state where it can be advanced step-by-step.
        """
        try:
            print("[ControlDesk] Getting PlatformManagement...")
            platforms = self.app.PlatformManagement.Platforms
            print(f"[ControlDesk] Found {platforms.Count} platform(s)")
            
            print("[ControlDesk] Getting platform 0...")
            platform = platforms.Item(0)
            print(f"[ControlDesk] Platform: {platform.Name if hasattr(platform, 'Name') else 'Unknown'}")
            
            print("[ControlDesk] Getting RealTimeApplications...")
            rta = platform.RealTimeApplications.Item(0)
            print(f"[ControlDesk] RealTimeApplication found")
            
            print("[ControlDesk] Calling rta.Pause()...")
            rta.Pause()
            print("[ControlDesk] ✅ Simulation paused for step-by-step control")
        except Exception as e:
            print(f"[ControlDesk] ❌ Error pausing simulation: {e}")
            import traceback
            traceback.print_exc()

    def advance_simulation_step(self):
        """Advance the dSPACE simulation by one timestep.
        
        Uses ControlDesk COM interface to execute a single simulation step.
        This should be called after control variables have been written.
        """
        try:
            platforms = self.app.PlatformManagement.Platforms
            platform = platforms.Item(0)
            rta = platform.RealTimeApplications.Item(0)
            rta.SingleStep()
        except Exception as e:
            print(f"[ControlDesk] ❌ Error advancing simulation step: {e}")
            import traceback
            traceback.print_exc()
            raise

    def initialize_vesi_interface(self):
        """Initialize VesiInterface manual control interface.
        
        Sets all required master switches, race control configuration, and enable flags
        to activate the VesiInterface manual control system.
        """
        print("[ControlDesk] Initializing VesiInterface manual control interface...")
        
        try:
            # Step 1: VesiInterface Master Switches
            print("[ControlDesk] Setting master switches...")
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value",
                0.0
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value",
                1.0  # CRITICAL: Enable manual VESI control
            )
            print("[ControlDesk] Master switches set")
            
            # Step 2: Race Control Configuration
            print("[ControlDesk] Configuring race control...")
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value",
                0.0  # Intern mode (required for manual control)
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value",
                9  # CRITICAL: System state constant
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value",
                1
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value",
                0
            )
            print("[ControlDesk] Race control configured")
            
            # Step 3: Enable Individual Control Channels
            print("[ControlDesk] Enabling control channels...")
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_brake_cmd/Value",
                1
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_gear_cmd/Value",
                1
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_steering_cmd/Value",
                1
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_throttle_cmd/Value",
                1
            )
            print("[ControlDesk] Control channels enabled")
            
            # Step 4: Initialize all control values to 0
            print("[ControlDesk] Initializing control values to 0...")
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            
            self.set_var(KEY_THROTTLE, 0.0)
            self.set_var(KEY_BRAKE_FRONT, 0.1)
            self.set_var(KEY_BRAKE_REAR, 0.1)
            self.set_var(KEY_STEERING, 0)
            self.set_var(KEY_GEAR, 0.0)
            self.set_var(KEY_CLUTCH, 0.0)
            print("[ControlDesk] All control values initialized to 0")
            
            print("[ControlDesk] VesiInterface initialization complete - manual control ready")
            
        except Exception as e:
            print(f"[ControlDesk] ERROR - VesiInterface initialization failed: {e}")
            import traceback
            traceback.print_exc()


