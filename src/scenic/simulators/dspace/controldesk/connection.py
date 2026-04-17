# -*- coding: utf-8 -*-
"""Lightweight ControlDesk COM automation wrapper.

Provides a minimal API for:
 - Going online/offline (online calibration)
 - Starting/stopping measurement
 - Reading/writing model variables via ActiveVariableDescription

The COM application object is created similarly to ModelDesk automation.
"""

import time
from typing import Any
import pythoncom
from win32com.client import Dispatch


class ControlDeskApp:
    def __init__(self, prog_id: str = "ControlDeskNG.Application",
                 outer_platform_name: str = "Platform",
                 inner_platform_name: str = "Platform_2"):
        self._prog_id = prog_id
        self._outer_platform_name = outer_platform_name
        self._inner_platform_name = inner_platform_name
        self.app = None
        # Cache COM objects to reduce overhead
        self._platform = None
        self._rta = None
        # Variable access cache: avoid re-resolving Variables collection every call
        self._variables_cache = None
        # Per-path ref cache for writes only (reads use fresh lookup to avoid stale values)
        self._write_refs_cache = {}
        # Per-call timing for COM analysis (path, 'get'|'set', duration_sec)
        self._timing_log = []

    def connect(self):
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

    # Variable access (cached: Variables object + per-path variable reference)
    def _get_variables(self):
        if self._variables_cache is not None:
            return self._variables_cache
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
        self._variables_cache = vdesc.Variables
        return self._variables_cache

    def get_var(self, path: str) -> Any:
        """Read variable; always fresh lookup (no ref cache) so values are never stale."""
        t0 = time.perf_counter()
        try:
            vars_obj = self._get_variables()
            return vars_obj[path].ValueConverted
        finally:
            self._timing_log.append((path, "get", time.perf_counter() - t0))

    def _get_write_var_ref(self, path: str):
        """Cached variable reference for writes only. Safe: writes push to target, no stale read."""
        if path in self._write_refs_cache:
            return self._write_refs_cache[path]
        vars_obj = self._get_variables()
        ref = vars_obj[path]
        self._write_refs_cache[path] = ref
        return ref

    def set_var(self, path: str, value: Any):
        """Write variable; uses cached ref for writes to reduce path lookup cost (correctness unchanged)."""
        t0 = time.perf_counter()
        try:
            ref = self._get_write_var_ref(path)
            ref.ValueConverted = value
        finally:
            self._timing_log.append((path, "set", time.perf_counter() - t0))

    # Maneuver control
    def start_maneuver(self, var_access=None):
        """Start the active experiment's maneuver.

        Pulses the MANEUVER_START variable: sets it to 1, waits briefly, then resets to 0.

        var_access: optional object with a ``set_var(path, value)`` method (e.g. MAPortApp).
            When provided, the pulse is written through that backend instead of ControlDesk
            COM. Scenic passes ``_var_access`` here so under CoSim the pulse goes through
            MAPort — proven reliable in the tester even when ControlDesk COM reads fail.
        """
        maneuver_start_path = (
            "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
            "UserInterface/PAR_Plant/ManeuverControl/MANEUVER_START/MDLDCtrl_ManeuverStart"
        )
        writer = var_access if var_access is not None else self
        print(f"[ControlDesk] Pulse: MANEUVER_START (backend={type(writer).__name__})")
        writer.set_var(maneuver_start_path, 1.0)
        time.sleep(0.5)
        writer.set_var(maneuver_start_path, 0.0)

    def stop_maneuver(self, var_access=None):
        """Stop the active experiment's maneuver via variable pulse (MANEUVER_STOP).

        See ``start_maneuver`` for ``var_access`` semantics.
        """
        maneuver_stop_path = (
            "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
            "UserInterface/PAR_Plant/ManeuverControl/MANEUVER_STOP/MDLDCtrl_ManeuverStop"
        )
        writer = var_access if var_access is not None else self
        print(f"[ControlDesk] Pulse: MANEUVER_STOP (backend={type(writer).__name__})")
        writer.set_var(maneuver_stop_path, 1.0)
        time.sleep(0.5)
        writer.set_var(maneuver_stop_path, 0.0)

    def reset_maneuver(self, var_access=None):
        """Reset the active experiment's maneuver via variable pulse (RESET).

        See ``start_maneuver`` for ``var_access`` semantics.
        """
        maneuver_reset_path = (
            "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
            "UserInterface/PAR_Plant/ManeuverControl/RESET/MDLDCtrl_Reset"
        )
        writer = var_access if var_access is not None else self
        print(f"[ControlDesk] Pulse: MANEUVER_RESET (backend={type(writer).__name__})")
        writer.set_var(maneuver_reset_path, 1.0)
        time.sleep(0.5)
        writer.set_var(maneuver_reset_path, 0.0)

    # Simulation control
    def _get_platform(self):
        """Get cached platform object, creating if needed."""
        if self._platform is None:
            self._platform = self.app.PlatformManagement.Platforms.Item(0)
        return self._platform
    
    def _get_rta(self):
        """Get cached RealTimeApplication object, creating if needed."""
        if self._rta is None:
            platform = self._get_platform()
            self._rta = platform.RealTimeApplications.Item(0)
        return self._rta
    
    def _clear_cache(self):
        """Clear cached COM objects (useful for reconnection scenarios)."""
        self._platform = None
        self._rta = None
        self._variables_cache = None
        self._write_refs_cache = {}

    def print_timing_summary(self):
        """Print per-path COM timing summary for analysis (call at end of run)."""
        if not self._timing_log:
            return
        # Aggregate by (path, op): total_sec, count
        agg = {}
        for path, op, duration in self._timing_log:
            key = (path, op)
            if key not in agg:
                agg[key] = [0.0, 0]
            agg[key][0] += duration
            agg[key][1] += 1
        # Sort by total duration descending
        rows = [(path, op, total, count) for (path, op), (total, count) in agg.items()]
        rows.sort(key=lambda x: -x[2])
        print("[COM Timing] Per-path summary (total_sec, count, mean_ms):")
        for path, op, total, count in rows:
            mean_ms = (total / count) * 1000.0 if count else 0
            # Shorten path for readability: keep last two path segments
            short = path
            if "/" in path:
                parts = path.split("/")
                short = "/".join(parts[-2:]) if len(parts) >= 2 else path
            print(f"  [{op:3s}] {total:.3f}s  n={count:6d}  mean={mean_ms:6.2f}ms  {short}")
        total_all = sum(d for _, _, d in self._timing_log)
        print(f"[COM Timing] TOTAL: {total_all:.3f}s over {len(self._timing_log)} calls")
    
    def set_simulation_step(self, step=0.01):
        """Set the simulation time step (SingleStepTime in ControlDesk).

        0D) Frozen-controller tests suggest the step API may advance a different internal
        tick than assumed, or the configured timestep may not be applied as expected.
        If simulated time delta per step does not match this value, investigate
        SimulationTimeOptions / SingleStep semantics in the experiment.
        """
        try:
            platform = self._get_platform()
            platform.SimulationTimeOptions.SingleStepTime = str(step)
            print(f"[ControlDesk] set_simulation_step(SingleStepTime={step}) applied")
        except Exception as e:
            print(f"[ControlDesk] Error setting simulation step: {e}")

    def start_simulation(self):
        """Start the real-time application (RTA Start) so the simulation is running (time advances).
        Call this before pause_simulation() when using step-by-step control, so that
        SingleStep() advances SimulationTime. Without this, time may stay 0.
        """
        try:
            rta = self._get_rta()
            if hasattr(rta, "Start"):
                rta.Start()
                print("[ControlDesk] start_simulation (RTA.Start) called")
            else:
                print("[ControlDesk] RTA has no Start method; simulation may already be started or use different API")
        except Exception as e:
            print(f"[ControlDesk] start_simulation failed: {e}")

    def pause_simulation(self):
        """Pause the dSPACE simulation for step-by-step control.
        
        This should be called once during setup to put the simulation into
        a paused state where it can be advanced step-by-step.
        """
        try:
            rta = self._get_rta()
            rta.Pause()
        except Exception as e:
            print(f"[ControlDesk] Error pausing simulation: {e}")

    def advance_simulation_step(self):
        """Advance the dSPACE simulation by one timestep.
        
        Uses ControlDesk COM interface to execute a single simulation step.
        This should be called after control variables have been written.
        """
        try:
            rta = self._get_rta()
            rta.SingleStep()
        except Exception as e:
            raise

    def initialize_vesi_interface(self):
        """Initialize VesiInterface manual control interface.
        
        Sets all required master switches, race control configuration, and enable flags
        to activate the VesiInterface manual control system.
        """
        try:
            # Step 1: VesiInterface Master Switches
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value",
                0.0
            )
            self.set_var(
                "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value",
                1.0  # CRITICAL: Enable manual VESI control
            )
            
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
            
            # Step 3: Enable Individual Control Channels
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
            
            # Step 4: Initialize all control values to 0
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            
            self.set_var(KEY_THROTTLE, 0.0)
            self.set_var(KEY_BRAKE_FRONT, 0.0)
            self.set_var(KEY_BRAKE_REAR, 0.0)
            self.set_var(KEY_STEERING, 0)
            self.set_var(KEY_GEAR, 0.0)
            self.set_var(KEY_CLUTCH, 0.0)
            
        except Exception as e:
            print(f"[ControlDesk] ERROR - VesiInterface initialization failed: {e}")


