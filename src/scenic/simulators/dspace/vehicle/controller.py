"""Vehicle controller for dSPACE simulator.

This module handles the application of control commands to vehicles in the
dSPACE simulation environment, including both ego and fellow vehicles.
"""


class VehicleController:
    """Controller for applying vehicle commands to ControlDesk.
    
    This class handles the translation from Scenic control commands to
    ControlDesk variable writes, supporting both:
    - Ego vehicle: VesiInterface physics-based control
    - Fellow vehicles: Kinematic control via external signals
    
    Attributes:
        simulation: Reference to parent DSpaceSimulation instance
        cd: ControlDesk connection object
    """
    
    def __init__(self, simulation):
        """Initialize the vehicle controller.
        
        Args:
            simulation: The parent DSpaceSimulation instance
        """
        self.simulation = simulation
        self.cd = simulation._cd
    
    def apply_ego_control(self, obj):
        """Apply VesiInterface control for ego vehicle.
        
        Ego uses physics-based control: throttle/brake/steering → VesiInterface → physics engine.
        Control inputs are written to VesiInterface manual control paths which feed into
        the VEOS vehicle dynamics model.
        
        Args:
            obj: The ego vehicle object with _control_state attribute
            
        Control Flow:
            1. Extract throttle/brake/steering from _control_state
            2. Scale to ControlDesk command ranges
            3. Write to VesiInterface manual control variables
            4. Apply one-shot actions (gear, clutch)
        """
        if not hasattr(obj, '_control_state') or not obj._control_state:
            return
        
        control = obj._control_state
        
        try:
            # VesiInterface manual control variable paths
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            
            # Apply throttle (0-1 → 0-100 command range)
            if 'throttle' in control and control['throttle'] is not None:
                throttle_val = float(max(0.0, min(1.0, control['throttle'])) * 100.0)
                self.cd.set_var(KEY_THROTTLE, throttle_val)
            
            # Apply brake (0-1 → 0-100 command range, front and rear)
            if 'braking' in control and control['braking'] is not None:
                brake_val = float(max(0.0, min(1.0, control['braking'])) * 100.0)
                self.cd.set_var(KEY_BRAKE_FRONT, brake_val)
                self.cd.set_var(KEY_BRAKE_REAR, brake_val)
            
            # Apply steering (-1 to 1 → -70 to +70 command range)
            if 'steering' in control and control['steering'] is not None:
                steer_val = -float(max(-1.0, min(1.0, control['steering'])) * 70.0)
                self.cd.set_var(KEY_STEERING, steer_val)
                
        except Exception as e:
            print(f"[VehicleController:EgoControl] Error: {e}")
        
        # Apply one-shot actions (gear, clutch)
        if hasattr(obj, '_oneshot_actions') and obj._oneshot_actions:
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            
            for action_type, value in obj._oneshot_actions:
                try:
                    if action_type == 'gear':
                        gear_int = max(0, min(6, int(value)))
                        self.cd.set_var(KEY_GEAR, gear_int)
                    elif action_type == 'clutch':
                        clutch_pct = float(value * 100.0)
                        self.cd.set_var(KEY_CLUTCH, clutch_pct)
                except Exception as e:
                    print(f"[VehicleController:EgoControl] {action_type} error: {e}")
    
    def apply_fellow_control(self, obj):
        """Apply kinematic control for fellow vehicle using physics model.
        
        Fellows use kinematic control: throttle/brake/steering → physics model → velocity/deviation.
        The physics model computes realistic motion, then velocity and lateral deviation are
        written to ControlDesk External_Signals for kinematic control.
        
        Args:
            obj: The fellow vehicle object with _control_state and dspaceActor attributes
            
        Control Flow:
            1. Get fellow index (F1→0, F2→1, etc.)
            2. Extract throttle/brake/steering from _control_state
            3. Update physics model to compute new velocity/deviation
            4. Write velocity (km/h) and deviation (m) to ControlDesk with correct indexing
        """
        if not hasattr(obj, '_control_state') or not obj._control_state:
            return
        
        # Get fellow index
        fellow_index = self.get_fellow_index(obj)
        if fellow_index is None:
            print(f"[VehicleController:FellowControl] Could not determine index for {obj}")
            return
        
        control = obj._control_state
        
        # Extract controls (default to 0 if not present)
        throttle = float(control.get('throttle', 0.0))
        brake = float(control.get('braking', 0.0))
        steering = float(control.get('steering', 0.0))
        
        try:
            # Update physics model
            actor = obj.dspaceActor
            new_velocity, new_deviation = actor.physics.update(
                throttle=throttle,
                brake=brake,
                steering=steering,
                dt=self.simulation.timestep
            )
            
            # Write to ControlDesk external signals with correct indexing
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
            
            # Velocity (m/s → km/h conversion)
            v_path = f"{base_path}/Const_v_Fellows_External[km|h]/Value[{fellow_index}]"
            self.cd.set_var(v_path, new_velocity * 3.6)  # m/s to km/h
            
            # Lateral deviation (meters)
            d_path = f"{base_path}/Const_d_Fellows_External[m]/Value[{fellow_index}]"
            self.cd.set_var(d_path, new_deviation)
            
            print(f"[Fellow {fellow_index}] Controls: throttle={throttle:.2f}, brake={brake:.2f}, steering={steering:.2f}")
            print(f"[Fellow {fellow_index}] → v={new_velocity:.2f} m/s ({new_velocity*3.6:.1f} km/h), d={new_deviation:.2f} m")
            
        except Exception as e:
            print(f"[VehicleController:FellowControl] Fellow {fellow_index} error: {e}")
            import traceback
            traceback.print_exc()
    
    def get_fellow_index(self, obj):
        """Get the array index for a fellow vehicle (0-based).
        
        Fellow vehicles are numbered F1, F2, F3, etc. by their raceNumber.
        ControlDesk arrays are 0-indexed, so this method converts:
        - F1 (raceNumber=1) → index 0
        - F2 (raceNumber=2) → index 1
        - F3 (raceNumber=3) → index 2
        - etc.
        
        Args:
            obj: The fellow vehicle object
            
        Returns:
            int: 0-based index for ControlDesk arrays, or None if not found
        """
        if hasattr(obj, 'raceNumber'):
            # Fellow vehicles are numbered F1, F2, F3...
            # ControlDesk arrays are 0-indexed, so F1→index 0, F2→index 1, etc.
            return obj.raceNumber - 1
        return None
    
    def read_fellow_state(self, obj):
        """Read fellow vehicle state from ControlDesk external signals with correct indexing.
        
        Reads the current velocity and lateral deviation for a fellow vehicle from
        ControlDesk External_Signals arrays. Uses the same indexing scheme as writing
        to ensure consistency.
        
        Args:
            obj: The fellow vehicle object
            
        Returns:
            dict: State dictionary with keys:
                - 'velocity': Velocity in m/s
                - 'deviation': Lateral deviation in meters
                - 'fellow_index': The 0-based array index
            None if read fails or vehicle not found
        """
        if not self.cd:
            return None
        
        fellow_index = self.get_fellow_index(obj)
        if fellow_index is None:
            return None
        
        try:
            base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
            
            # Read velocity (km/h → m/s conversion)
            v_path = f"{base_path}/Const_v_Fellows_External[km|h]/Value[{fellow_index}]"
            velocity_kmh = self.cd.get_var(v_path)
            velocity_ms = velocity_kmh / 3.6 if velocity_kmh is not None else 0.0
            
            # Read lateral deviation (meters)
            d_path = f"{base_path}/Const_d_Fellows_External[m]/Value[{fellow_index}]"
            deviation = self.cd.get_var(d_path)
            deviation = deviation if deviation is not None else 0.0
            
            return {
                'velocity': velocity_ms,
                'deviation': deviation,
                'fellow_index': fellow_index
            }
            
        except Exception as e:
            print(f"[VehicleController:ReadState] Fellow {fellow_index} error: {e}")
            return None

