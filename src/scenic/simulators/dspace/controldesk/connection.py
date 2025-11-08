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


