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
    def __init__(self, prog_id: str = "ControlDeskNG.Application"):
        self._prog_id = prog_id
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
    def get_var(self, path: str) -> Any:
        plat = self.app.ActiveExperiment.Platforms[0]
        vdesc = plat.ActiveVariableDescription
        return vdesc.Variables[path].ValueConverted

    def set_var(self, path: str, value: Any):
        plat = self.app.ActiveExperiment.Platforms[0]
        vdesc = plat.ActiveVariableDescription
        vdesc.Variables[path].ValueConverted = value


