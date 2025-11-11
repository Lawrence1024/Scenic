"""Vehicle controller for dSPACE simulator.

This module handles the application of control commands to vehicles in the
dSPACE simulation environment, including both ego and fellow vehicles.
"""

from ..vehicle.physics import VehiclePhysicsState


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
        control = getattr(obj, '_control_state', None)
        
        try:
            # VesiInterface manual control variable paths
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            
            # Apply throttle (0-1 → 0-100 command range)
            if control and 'throttle' in control and control['throttle'] is not None:
                throttle_val = float(max(0.0, min(1.0, control['throttle'])) * 100.0)
                self.cd.set_var(KEY_THROTTLE, throttle_val)
            
            # Apply brake (0-1 → 0-100 command range, front and rear)
            if control and 'braking' in control and control['braking'] is not None:
                brake_val = float(max(0.0, min(1.0, control['braking'])) * 100.0)
                self.cd.set_var(KEY_BRAKE_FRONT, brake_val)
                self.cd.set_var(KEY_BRAKE_REAR, brake_val)
            
            # Apply steering (-1 to 1 → -70 to +70 command range)
            if control and 'steering' in control and control['steering'] is not None:
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
        
        # Ensure fellow arrays are initialized before attempting to write
        self.simulation._ensureFellowArraysInitialized()
        
        # Get fellow index
        fellow_index = self.get_fellow_index(obj)
        if fellow_index is None:
            print(f"[VehicleController:FellowControl] Could not determine index for {obj}")
            return
        # Adjust for base (0-based vs 1-based arrays) for writing
        eff_index = fellow_index + (self.simulation._fellow_index_base or 0)
        
        control = obj._control_state
        
        # Extract controls (default to 0 if not present)
        throttle = float(control.get('throttle', 0.0))
        brake = float(control.get('braking', 0.0))
        steering = float(control.get('steering', 0.0))
        
        try:
            # Update physics model
            actor = obj.dspaceActor
            # Ensure physics model exists for kinematic update
            if getattr(actor, "physics", None) is None:
                actor.physics = VehiclePhysicsState(initial_velocity=0.0, initial_deviation=0.0)
            new_velocity, new_deviation = actor.physics.update(
                throttle=throttle,
                brake=brake,
                steering=steering,
                dt=self.simulation.timestep
            )
            
            # Write to ControlDesk external signals using bulk arrays (robust when element addressing is not supported)
            self.simulation._probe_external_index_base()
            v_path_bulk = self.simulation._ext_v_path
            d_path_bulk = self.simulation._ext_d_path
            ext_base = self.simulation._ext_index_base or 0
            eff_ext_index = fellow_index + ext_base

            # Read current arrays (bulk), extend if needed
            try:
                v_arr = self.cd.get_var(v_path_bulk)
            except Exception:
                v_arr = None
            try:
                d_arr = self.cd.get_var(d_path_bulk)
            except Exception:
                d_arr = None

            if not isinstance(v_arr, list):
                v_arr = list(v_arr) if isinstance(v_arr, tuple) else []
            if not isinstance(d_arr, list):
                d_arr = list(d_arr) if isinstance(d_arr, tuple) else []

            need_len = eff_ext_index + 1
            if len(v_arr) < need_len:
                v_arr.extend([0.0] * (need_len - len(v_arr)))
            if len(d_arr) < need_len:
                d_arr.extend([0.0] * (need_len - len(d_arr)))

            # Update slots
            v_value = float(new_velocity * 3.6)  # m/s → km/h
            d_value = float(new_deviation)       # meters
            v_prev = v_arr[eff_ext_index]
            d_prev = d_arr[eff_ext_index]
            v_arr[eff_ext_index] = v_value
            d_arr[eff_ext_index] = d_value

            # Bulk write back
            self.cd.set_var(v_path_bulk, v_arr)
            self.cd.set_var(d_path_bulk, d_arr)

            # Read-back verification (bulk) for the element we just updated
            try:
                v_back_arr = self.cd.get_var(v_path_bulk)
                d_back_arr = self.cd.get_var(d_path_bulk)
                v_echo = v_back_arr[eff_ext_index] if isinstance(v_back_arr, (list, tuple)) and len(v_back_arr) > eff_ext_index else None
                d_echo = d_back_arr[eff_ext_index] if isinstance(d_back_arr, (list, tuple)) and len(d_back_arr) > eff_ext_index else None
                print(f"[Fellow {fellow_index}] Controls: throttle={throttle:.2f}, brake={brake:.2f}, steering={steering:.2f}")
                print(f"[Fellow {fellow_index}] → v={new_velocity:.2f} m/s ({v_value:.1f} km/h), d={d_value:.2f} m (eff_idx={eff_ext_index})")
                print(f"[Fellow {fellow_index}] ExternalSignals bulk write/read: v {v_prev}→{v_echo}, d {d_prev}→{d_echo} @ {v_path_bulk}[{eff_ext_index}]")
            except Exception as es_err:
                print(f"[Fellow {fellow_index}] ExternalSignals bulk feedback read error (idx={eff_ext_index}): {es_err}")
            # For the first fellow, also try to read plant pose if available (bulk-safe)
            if fellow_index == 0:
                try:
                    plant_base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                    x_arr = self.cd.get_var(f"{plant_base}/x")
                    y_arr = self.cd.get_var(f"{plant_base}/y")
                    yaw_arr = self.cd.get_var(f"{plant_base}/yaw_deg_out")
                    x = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and len(x_arr) > eff_index else None
                    y = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and len(y_arr) > eff_index else None
                    yaw = yaw_arr[eff_index] if isinstance(yaw_arr, (list, tuple)) and len(yaw_arr) > eff_index else None
                    if x is not None and y is not None and yaw is not None:
                        print(f"[Fellow {fellow_index}] Plant pose: x={x}, y={y}, yaw_deg={yaw} (idx={eff_index})")
                except Exception:
                    # Silently ignore; arrays may not be ready in the first few ticks
                    pass
            
            # Clear warning flag if we successfully wrote (arrays are now ready)
            if hasattr(obj, '_write_array_bounds_warning_shown'):
                delattr(obj, '_write_array_bounds_warning_shown')
            
        except Exception as e:
            error_msg = str(e)
            if "Index was outside the bounds" in error_msg or "bounds of the array" in error_msg:
                # Array bounds error - arrays may not be initialized yet
                if not hasattr(obj, '_write_array_bounds_warning_shown'):
                    print(f"[VehicleController:FellowControl] Warning: Fellow {fellow_index} array not ready yet (arrays may not be initialized)")
                    obj._write_array_bounds_warning_shown = True
            else:
                print(f"[VehicleController:FellowControl] Fellow {fellow_index} error: {e}")
                import traceback
                traceback.print_exc()
    
    def get_fellow_index(self, obj):
        """Get the array index for a fellow vehicle (0-based).
        
        Uses the simulation's _getFellowIndex method which properly handles
        index calculation from fellow_vehicles dict, not from raceNumber.
        
        Args:
            obj: The fellow vehicle object
            
        Returns:
            int: 0-based index for ControlDesk arrays, or None if not found
        """
        # Delegate to simulation's method which has proper logic
        return self.simulation._getFellowIndex(obj)
    
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

