# -*- coding: utf-8 -*-
"""
Per-Tick Control Module for dSPACE VEOS

This module handles per-tick control functionality for dSPACE vehicles,
separated from the main simulator for better organization.

Based on manual4.md requirements:
- 10ms timing (dt = 0.01) for per-tick control
- Variable paths: Environment.Vehicle.Fx.Driver.Throttle/Brake/SteeringWheelAngle
- External control flags enabled via ASM_Maneuver.py script
"""

import time
import threading
import subprocess
import os
from typing import Dict, Optional, Callable, Tuple


class PerTickController:
    """Per-tick control controller for dSPACE vehicles."""
    
    def __init__(self, controldesk_app=None):
        """Initialize per-tick controller.
        
        Args:
            controldesk_app: ControlDesk COM application object
        """
        self.controldesk_app = controldesk_app
        self.control_variables = {}  # Cache control variables
        self.active_loops = {}  # Track active control loops
        
    def connectControlDesk(self):
        """Connect to ControlDesk COM application for runtime control.
        
        Based on manual4.md: Connect CD to VEOS through CTun and ensure
        the same ASM_Traffic.sdf is assigned in CD.
        """
        try:
            from win32com.client import Dispatch
            self.controldesk_app = Dispatch("ControlDeskNG.Application")
            
            # Verify connection and project setup
            proj = self.controldesk_app.ActiveProject
            exp = proj.ActiveExperiment
            
            print("[ControlDesk] Connected to ControlDesk application")
            print(f"[ControlDesk] Active project: {proj.Name}")
            print(f"[ControlDesk] Active experiment: {exp.Name}")
            
            # Check if we can access variables (indicates CTun connection)
            try:
                # Try to get a test variable to verify connectivity
                test_var = exp.GetVariable("Environment.Vehicle.F1.Driver.Throttle")
                print("[ControlDesk] ✅ CTun connectivity verified - can access vehicle variables")
                return True
            except Exception as e:
                print(f"[ControlDesk] ⚠️  CTun connectivity issue: {e}")
                print("[ControlDesk] Ensure CTun client is running: .\\bin\\ctun.exe client 127.0.0.1 --dest 10.6.0.2")
                print("[ControlDesk] Ensure VEOS is registered at 192.168.100.101")
                return False
                
        except Exception as e:
            print(f"[ControlDesk] Failed to connect: {e}")
            return False
    
    def getControlVariables(self, exp, vehicle_name):
        """Get control variables for a vehicle from ControlDesk.
        
        Args:
            exp: ControlDesk ActiveExperiment object
            vehicle_name: Name of the vehicle (e.g., "F1")
            
        Returns:
            Dictionary with control variable objects
        """
        control_vars = {}
        
        # Variable path patterns based on manual4.md
        # Format: Environment.Vehicle.<vehicle_name>.Driver.<command>
        variable_patterns = {
            'throttle': f"Environment.Vehicle.{vehicle_name}.Driver.Throttle",
            'brake': f"Environment.Vehicle.{vehicle_name}.Driver.Brake", 
            'steering': f"Environment.Vehicle.{vehicle_name}.Driver.SteeringWheelAngle"
        }
        
        for control_type, var_path in variable_patterns.items():
            try:
                var_obj = exp.GetVariable(var_path)
                control_vars[control_type] = var_obj
                print(f"[ControlDesk] Found {control_type} variable: {var_path}")
            except Exception as e:
                print(f"[ControlDesk] Could not find {control_type} variable at {var_path}: {e}")
                # Try alternative paths
                alternative_paths = [
                    f"Environment.Vehicle.{vehicle_name}.Driver.{control_type.title()}",
                    f"Vehicle.{vehicle_name}.Driver.{control_type.title()}",
                    f"{vehicle_name}.Driver.{control_type.title()}"
                ]
                
                for alt_path in alternative_paths:
                    try:
                        var_obj = exp.GetVariable(alt_path)
                        control_vars[control_type] = var_obj
                        print(f"[ControlDesk] Found {control_type} variable at alternative path: {alt_path}")
                        break
                    except:
                        continue
        
        return control_vars
    
    def setVehicleControl(self, vehicle_name, throttle=None, brake=None, steering=None, velocity=None):
        """Set dynamic control inputs for a fellow vehicle using ControlDesk.
        
        This implements Phase 2 of the dSPACE architecture:
        - Connect to ControlDesk COM application (assumes CTun is already connected)
        - Access vehicle control variables
        - Write throttle/brake/steering values per tick
        
        Based on manual4.md: 10ms timing (dt = 0.01) for per-tick control
        
        Args:
            vehicle_name: Name of the fellow vehicle (e.g., "F1", "F2")
            throttle: Throttle input (0.0 to 1.0)
            brake: Brake input (0.0 to 1.0) 
            steering: Steering angle (-1.0 to 1.0)
            velocity: Target velocity in m/s
        """
        if not self.controldesk_app:
            if not self.connectControlDesk():
                return False
        
        try:
            # Access active experiment (assumes CTun connectivity is established)
            proj = self.controldesk_app.ActiveProject
            exp = proj.ActiveExperiment
            
            # Get or cache control variables for this vehicle
            if vehicle_name not in self.control_variables:
                self.control_variables[vehicle_name] = self.getControlVariables(exp, vehicle_name)
            
            control_vars = self.control_variables[vehicle_name]
            if not control_vars:
                print(f"[ControlDesk] No control variables found for {vehicle_name}")
                return False
            
            # Apply control inputs with proper timing
            success = True
            
            if throttle is not None and 'throttle' in control_vars:
                try:
                    control_vars['throttle'].Value = float(throttle)
                    print(f"[ControlDesk] Set throttle for {vehicle_name}: {throttle}")
                except Exception as e:
                    print(f"[ControlDesk] Throttle control error for {vehicle_name}: {e}")
                    success = False
            
            if brake is not None and 'brake' in control_vars:
                try:
                    control_vars['brake'].Value = float(brake)
                    print(f"[ControlDesk] Set brake for {vehicle_name}: {brake}")
                except Exception as e:
                    print(f"[ControlDesk] Brake control error for {vehicle_name}: {e}")
                    success = False
            
            if steering is not None and 'steering' in control_vars:
                try:
                    # Convert steering from [-1,1] to degrees as per manual4.md
                    steering_degrees = float(steering) * 30.0  # Scale to reasonable degrees
                    control_vars['steering'].Value = steering_degrees
                    print(f"[ControlDesk] Set steering for {vehicle_name}: {steering_degrees}°")
                except Exception as e:
                    print(f"[ControlDesk] Steering control error for {vehicle_name}: {e}")
                    success = False
            
            return success
            
        except Exception as e:
            print(f"[ControlDesk] Control error for {vehicle_name}: {e}")
            return False
    
    def startPerTickControl(self, vehicle_name, control_function=None, dt=0.01):
        """Start per-tick control loop for a vehicle.
        
        Based on manual4.md: dt = 0.01 (10ms) for per-tick control
        
        Args:
            vehicle_name: Name of the vehicle to control (e.g., "F1", "F2")
            control_function: Function that returns (throttle, brake, steering) tuple
            dt: Time step in seconds (default 0.01 = 10ms)
        """
        if not self.controldesk_app:
            if not self.connectControlDesk():
                return False
        
        def control_loop():
            """Per-tick control loop."""
            print(f"[PerTick] Starting control loop for {vehicle_name} (dt={dt}s)")
            
            try:
                while True:
                    start_time = time.time()
                    
                    # Get control inputs from function
                    if control_function:
                        try:
                            throttle, brake, steering = control_function()
                            self.setVehicleControl(vehicle_name, throttle, brake, steering)
                        except Exception as e:
                            print(f"[PerTick] Control function error: {e}")
                    
                    # Maintain timing
                    elapsed = time.time() - start_time
                    sleep_time = dt - elapsed
                    if sleep_time > 0:
                        time.sleep(sleep_time)
                    else:
                        print(f"[PerTick] Warning: Control loop lagging by {-sleep_time:.3f}s")
                        
            except KeyboardInterrupt:
                print(f"[PerTick] Control loop stopped for {vehicle_name}")
            except Exception as e:
                print(f"[PerTick] Control loop error for {vehicle_name}: {e}")
        
        # Start control loop in separate thread
        control_thread = threading.Thread(target=control_loop, daemon=True)
        control_thread.start()
        
        # Track active loop
        self.active_loops[vehicle_name] = control_thread
        
        print(f"[PerTick] Control loop started for {vehicle_name}")
        return True
    
    def stopPerTickControl(self, vehicle_name):
        """Stop per-tick control loop for a vehicle."""
        if vehicle_name in self.active_loops:
            # Note: Thread will stop on KeyboardInterrupt or when daemon=True
            del self.active_loops[vehicle_name]
            print(f"[PerTick] Control loop stopped for {vehicle_name}")
            return True
        return False


class ExternalControlManager:
    """Manager for external control flags via ASM_Maneuver.py script."""
    
    @staticmethod
    def enableExternalControlViaScript(scene_objects):
        """Enable external control using ASM_Maneuver.py script.
        
        Based on manual4.md: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4
        
        Args:
            scene_objects: List of Scenic objects to enable external control for
        """
        try:
            # Method 1: Try Docker exec (preferred for containerized VEOS)
            if ExternalControlManager._tryDockerExec(scene_objects):
                return
            
            # Method 2: Try direct script execution (if running inside VEOS container)
            if os.path.exists('/home/dspace/scripts/ASM_Maneuver.py'):
                ExternalControlManager._runScriptDirectly(scene_objects)
            else:
                print("[ASM_Maneuver] Script not found - external control may need manual setup")
                print("[ASM_Maneuver] Try: docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4")
                
        except Exception as e:
            print(f"[ASM_Maneuver] Error running script: {e}")
    
    @staticmethod
    def _tryDockerExec(scene_objects):
        """Try to run ASM_Maneuver.py via Docker exec."""
        try:
            # Enable external control for each fellow vehicle
            for scenic_obj in scene_objects:
                if hasattr(scenic_obj, 'raceNumber') and scenic_obj is not getattr(scene_objects[0], 'egoObject', None):
                    fellow_number = scenic_obj.raceNumber
                    vehicle_flag = fellow_number + 1  # F1 = vehicleflag_2, F2 = vehicleflag_3, etc.
                    
                    # Docker exec command as per manual4.md
                    cmd = ['docker', 'exec', '-it', 'veos', 'python3', 
                           '/home/dspace/scripts/ASM_Maneuver.py', 
                           f'vehicleflag_{vehicle_flag}', 'trackflag_4']
                    
                    try:
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            print(f"[ASM_Maneuver] ✅ Enabled external control for F{fellow_number}")
                        else:
                            print(f"[ASM_Maneuver] ❌ Failed for F{fellow_number}: {result.stderr}")
                            return False
                    except subprocess.TimeoutExpired:
                        print(f"[ASM_Maneuver] ⏰ Timeout for F{fellow_number}")
                        return False
                    except FileNotFoundError:
                        print("[ASM_Maneuver] Docker not found - trying direct script execution")
                        return False
                    except Exception as e:
                        print(f"[ASM_Maneuver] Error for F{fellow_number}: {e}")
                        return False
            
            return True
            
        except Exception as e:
            print(f"[ASM_Maneuver] Docker exec error: {e}")
            return False
    
    @staticmethod
    def _runScriptDirectly(scene_objects):
        """Run ASM_Maneuver.py script directly (when inside VEOS container)."""
        try:
            for scenic_obj in scene_objects:
                if hasattr(scenic_obj, 'raceNumber') and scenic_obj is not getattr(scene_objects[0], 'egoObject', None):
                    fellow_number = scenic_obj.raceNumber
                    vehicle_flag = fellow_number + 1
                    
                    cmd = ['python3', '/home/dspace/scripts/ASM_Maneuver.py', 
                           f'vehicleflag_{vehicle_flag}', 'trackflag_4']
                    
                    try:
                        result = subprocess.run(cmd, capture_output=True, text=True, timeout=10)
                        if result.returncode == 0:
                            print(f"[ASM_Maneuver] ✅ Enabled external control for F{fellow_number}")
                        else:
                            print(f"[ASM_Maneuver] ❌ Failed for F{fellow_number}: {result.stderr}")
                    except Exception as e:
                        print(f"[ASM_Maneuver] Error for F{fellow_number}: {e}")
                        
        except Exception as e:
            print(f"[ASM_Maneuver] Direct script error: {e}")
