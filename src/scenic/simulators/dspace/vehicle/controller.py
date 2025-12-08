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
        
        # Track for debug
        if not hasattr(obj, '_ego_control_count'):
            obj._ego_control_count = 0
        obj._ego_control_count += 1
        
        try:
            # VesiInterface manual control variable paths
            KEY_THROTTLE = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
            KEY_BRAKE_FRONT = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
            KEY_BRAKE_REAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
            KEY_STEERING = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            
            throttle_scenic = control.get('throttle', 0.0) if control else 0.0
            brake_scenic = control.get('braking', 0.0) if control else 0.0
            steer_scenic = control.get('steering', 0.0) if control else 0.0
            
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
            
            # Debug every 50 steps
            if obj._ego_control_count % 50 == 0:
                print(f"[EgoControl #{obj._ego_control_count}] Writing: throttle={throttle_scenic:.3f}→{throttle_scenic*100:.1f}, brake={brake_scenic:.3f}→{brake_scenic*100:.1f}, steer={steer_scenic:.3f}→{-steer_scenic*70:.1f}")
                
        except Exception as e:
            print(f"[VehicleController:EgoControl] Error: {e}")
            import traceback
            traceback.print_exc()
        
        # Apply one-shot actions (gear, clutch)
        if hasattr(obj, '_oneshot_actions') and obj._oneshot_actions:
            KEY_GEAR = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"
            KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
            
            for action_type, value in obj._oneshot_actions:
                try:
                    if action_type == 'gear':
                        gear_int = max(0, min(6, int(value)))
                        self.cd.set_var(KEY_GEAR, gear_int)
                        if obj._ego_control_count <= 5 or obj._ego_control_count % 50 == 0:
                            print(f"[EgoControl] Setting gear to {gear_int}")
                    elif action_type == 'clutch':
                        clutch_pct = float(value * 100.0)
                        self.cd.set_var(KEY_CLUTCH, clutch_pct)
                        print(f"[EgoControl] Setting clutch to {clutch_pct}%")
                except Exception as e:
                    print(f"[VehicleController:EgoControl] {action_type} error: {e}")
                    import traceback
                    traceback.print_exc()
    
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
        from ..controldesk.arrays import ensure_fellow_arrays_initialized
        ensure_fellow_arrays_initialized(self.simulation)
        
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
        
        # CRITICAL: Get the actual CTE (cross-track error) from the behavior
        # The behavior stores this in _current_cte (not in _control_state)
        cte_from_behavior = getattr(obj, '_current_cte', None)
        
        # Track control calls for debug
        if not hasattr(obj, '_fellow_control_count'):
            obj._fellow_control_count = 0
        obj._fellow_control_count += 1
        
        try:
            # Update physics model
            actor = obj.dspaceActor
            # Ensure physics model exists for kinematic update
            if getattr(actor, "physics", None) is None:
                actor.physics = VehiclePhysicsState(initial_velocity=0.0, initial_deviation=0.0)
                print(f"[Fellow {fellow_index}] Physics model created (initial velocity=0.0 m/s)")
            
            # Sync physics model with actual velocity from ControlDesk
            # Read velocity from v_Fellows (plant output, actual simulation velocity)
            # This provides proper feedback from the simulator's physics engine
            actual_speed = 0.0
            read_source = "none"
            
            # First, try to read from plant output v_Fellows (actual simulation velocity)
            try:
                base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                v_arr = self.cd.get_var(f"{base_path}/v_Fellows")
                if isinstance(v_arr, (list, tuple)) and eff_index < len(v_arr):
                    v_value = v_arr[eff_index]
                    if v_value is not None:
                        # v_Fellows is in km/h, convert to m/s
                        actual_speed = float(v_value) / 3.6
                        read_source = "v_Fellows"
            except Exception:
                pass
            
            # If v_Fellows read failed or was 0, try reading from External Signals (commanded velocity)
            if actual_speed == 0.0 or read_source == "none":
                try:
                    self.simulation._probe_external_index_base()
                    v_path_bulk = self.simulation._ext_v_path
                    ext_base = self.simulation._ext_index_base or 0
                    eff_ext_index = fellow_index + ext_base
                    
                    v_arr_ext = self.cd.get_var(v_path_bulk)
                    if isinstance(v_arr_ext, (list, tuple)) and eff_ext_index < len(v_arr_ext):
                        v_ext_value = v_arr_ext[eff_ext_index]
                        if v_ext_value is not None:
                            # External Signals are in km/h, convert to m/s
                            actual_speed = float(v_ext_value) / 3.6
                            read_source = "ExternalSignals"
                except Exception:
                    pass
            
            # Final fallback: use cached linvel
            if actual_speed == 0.0 and hasattr(actor, 'linvel') and actor.linvel is not None:
                try:
                    actual_speed = float(actor.linvel.norm())
                    read_source = "linvel"
                except Exception:
                    import math
                    actual_speed = math.sqrt(actor.linvel.x**2 + actor.linvel.y**2 + actor.linvel.z**2)
                    actual_speed = float(actual_speed)
                    read_source = "linvel_manual"
            
            # Sync physics model's internal velocity with actual velocity from ControlDesk
            old_physics_velocity = actor.physics.velocity
            actor.physics.velocity = actual_speed
            
            # Debug: Log physics sync on first few calls
            if obj._fellow_control_count <= 3:
                print(f"[Fellow {fellow_index} Physics] Synced velocity: {old_physics_velocity:.2f} → {actual_speed:.2f} m/s (from {read_source})")
            
            # CRITICAL: Initialize deviation on first step to match the starting CTE
            # This ensures the Fellow knows its actual starting lateral position
            if obj._fellow_control_count == 1 and cte_from_behavior is not None:
                # Initialize physics model deviation to current CTE
                actor.physics.deviation = float(cte_from_behavior)
                print(f"\n[Fellow {fellow_index} Physics] 🚗 INITIALIZATION")
                print(f"  Starting CTE = {cte_from_behavior:.2f} m")
                print(f"  Setting initial deviation = {actor.physics.deviation:.2f} m")
            
            # Store old deviation to calculate change
            old_deviation = actor.physics.deviation
            
            # Update physics: integrates steering to change lateral position over time
            new_velocity, new_deviation = actor.physics.update(
                throttle=throttle,
                brake=brake,
                steering=steering,
                dt=self.simulation.timestep
            )
            
            # Calculate how much the deviation changed
            deviation_delta = new_deviation - old_deviation
            
            # Log the physics behavior for first 10 steps
            if obj._fellow_control_count <= 10:
                velocity_factor = min(actual_speed / 20.0, 1.0)
                lateral_vel = steering * 2.0 * velocity_factor  # steering_sensitivity = 2.0
                print(f"\n[Fellow {fellow_index} Physics] Step {obj._fellow_control_count}")
                print(f"  velocity = {actual_speed:.2f} m/s → velocity_factor = {velocity_factor:.2f}")
                print(f"  steering = {steering:.3f} → lateral_vel = {lateral_vel:.3f} m/s")
                print(f"  deviation: {old_deviation:.2f} → {new_deviation:.2f} m (Δ = {deviation_delta:.3f} m)")
                print(f"  CTE from behavior = {cte_from_behavior:.2f} m")
                if abs(deviation_delta) < 0.01:
                    print(f"  ⚠️  Deviation not changing! (velocity too low or steering = 0)")
            
            # Debug: Log physics update on first few calls
            if obj._fellow_control_count <= 3:
                print(f"[Fellow {fellow_index} Physics] Update: throttle={throttle:.2f}, brake={brake:.2f}, steering={steering:.2f}")
                print(f"[Fellow {fellow_index} Physics] Result: velocity={new_velocity:.2f} m/s, deviation={new_deviation:.2f} m")
            
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
                
                # Always print (not just first 3 times) so we can see what's happening
                print(f"\n[Fellow {fellow_index}] Step {obj._fellow_control_count}")
                print(f"  Controls IN:  throttle={throttle:.2f}, brake={brake:.2f}, steering={steering:.2f}")
                print(f"  Physics OUT:  velocity={new_velocity:.2f} m/s ({v_value:.1f} km/h), deviation={d_value:.2f} m")
                print(f"  Written to dSPACE: v={v_value:.1f} km/h, d={d_value:.2f} m @ index {eff_ext_index}")
                print(f"  Read back:    v={v_echo} km/h, d={d_echo} m")
            except Exception as es_err:
                print(f"[Fellow {fellow_index}] ExternalSignals bulk feedback read error (idx={eff_ext_index}): {es_err}")
            # Read actual plant pose to see where the fellow really is
            try:
                plant_base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                x_arr = self.cd.get_var(f"{plant_base}/x")
                y_arr = self.cd.get_var(f"{plant_base}/y")
                yaw_arr = self.cd.get_var(f"{plant_base}/yaw_deg_out")
                x = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and len(x_arr) > eff_index else None
                y = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and len(y_arr) > eff_index else None
                yaw = yaw_arr[eff_index] if isinstance(yaw_arr, (list, tuple)) and len(yaw_arr) > eff_index else None
                
                if x is not None and y is not None and yaw is not None:
                    print(f"  Actual position in dSPACE: x={x:.2f} m, y={y:.2f} m, yaw={yaw:.1f}°")
                    
                    # Check if we have waypoints to compare against
                    if hasattr(obj, 'waypoints') and obj.waypoints and len(obj.waypoints) > 0:
                        # Find nearest waypoint
                        min_dist = float('inf')
                        nearest_wp = None
                        for wp in obj.waypoints:
                            wp_x, wp_y = float(wp[0]), float(wp[1])
                            dist = ((x - wp_x)**2 + (y - wp_y)**2)**0.5
                            if dist < min_dist:
                                min_dist = dist
                                nearest_wp = (wp_x, wp_y)
                        
                        if nearest_wp:
                            print(f"  Nearest waypoint: ({nearest_wp[0]:.2f}, {nearest_wp[1]:.2f}), distance={min_dist:.2f} m")
                            print(f"  ⚠️  Fellow is {min_dist:.2f}m away from TTL path!")
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

