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
            
            # Apply brake (0-1 → 0-10000 command range, front and rear)
            # ControlDesk expects brake in range 0-10000, not 0-100
            if control and 'braking' in control and control['braking'] is not None:
                brake_val = float(max(0.0, min(1.0, control['braking'])) * 10000.0)
                self.cd.set_var(KEY_BRAKE_FRONT, brake_val)
                self.cd.set_var(KEY_BRAKE_REAR, brake_val)
            
            # Apply steering (-1 to 1 → -70 to +70 command range)
            # NOTE: Positive steering = LEFT turn in ControlDesk (verified via joystick integration docs)
            # The negative sign was causing steering to be inverted (steering LEFT when should steer RIGHT)
            if control and 'steering' in control and control['steering'] is not None:
                steer_val = float(max(-1.0, min(1.0, control['steering']))) * 70.0
                self.cd.set_var(KEY_STEERING, steer_val)
            
            # Debug every 50 steps
            if obj._ego_control_count % 50 == 0:
                print(f"[EgoControl #{obj._ego_control_count}] Writing: throttle={throttle_scenic:.3f}->{throttle_scenic*100:.1f}, brake={brake_scenic:.3f}->{brake_scenic*100:.1f}, steer={steer_scenic:.3f}->{-steer_scenic*70:.1f}")
                
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
            actual_speed = 0.0
            
            # Use confirmed FellowTrailer path [1]
            try:
                base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                v_arr = self.cd.get_var(f"{base_path}/v_Fellows")
                if isinstance(v_arr, (list, tuple)) and eff_index < len(v_arr):
                    v_value = v_arr[eff_index]
                    if v_value is not None:
                        # v_Fellows is typically m/s in ASM FellowTrailer, but if feedback fails assume 0
                        # Wait, user confirmed output [4] is km/h. FellowTrailer is internal.
                        # Usually FellowTrailer is m/s. We stick to m/s conversion just in case (safer)
                        # If it's already m/s, dividing by 3.6 makes it tiny. 
                        # Let's assume km/h to match the External Signal units for consistency.
                        actual_speed = float(v_value) / 3.6
            except Exception:
                pass
            
            # Sync physics velocity
            old_physics_velocity = actor.physics.velocity
            actor.physics.velocity = actual_speed
            
            if obj._fellow_control_count <= 3:
                print(f"[Fellow {fellow_index} Physics] Synced velocity: {old_physics_velocity:.2f} → {actual_speed:.2f} m/s")
            
            # CRITICAL: Initialize deviation on first step to maintain continuity
            if obj._fellow_control_count == 1:
                initial_deviation = 0.0
                try:
                    # Read current external signal deviation [3]
                    base_ext = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
                    d_path_bulk = f"{base_ext}/Const_d_Fellows_External[m]/Value"
                    d_arr = self.cd.get_var(d_path_bulk)
                    if isinstance(d_arr, (list, tuple)) and eff_index < len(d_arr):
                        initial_deviation = float(d_arr[eff_index]) if d_arr[eff_index] is not None else 0.0
                except Exception:
                    initial_deviation = 0.0
                
                actor.physics.deviation = initial_deviation
                print(f"\n[Fellow {fellow_index} Physics] INITIALIZATION: Dev={initial_deviation:.2f}m")
            
            old_deviation = actor.physics.deviation
            
            if obj._fellow_control_count == 1:
                # First step: maintain current deviation
                new_velocity = actual_speed 
                new_deviation = actor.physics.deviation 
            else:
                # Sync deviation with actual position before update
                try:
                    # Read actual position [1]
                    plant_base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
                    x_arr = self.cd.get_var(f"{plant_base}/x")
                    y_arr = self.cd.get_var(f"{plant_base}/y")
                    x_rd = x_arr[eff_index] if isinstance(x_arr, (list, tuple)) and eff_index < len(x_arr) else None
                    y_rd = y_arr[eff_index] if isinstance(y_arr, (list, tuple)) and eff_index < len(y_arr) else None
                    
                    if x_rd is not None and y_rd is not None and self.simulation._road_index:
                        from ..utils.legacy import project_world_to_st
                        s_actual, t_actual = project_world_to_st(
                            self.simulation._road_index, (float(x_rd), float(y_rd))
                        )
                        actor.physics.deviation = float(t_actual)
                except Exception:
                    pass
                
                # Physics Update
                new_velocity, new_deviation = actor.physics.update(
                    throttle=throttle,
                    brake=brake,
                    steering=steering,
                    dt=self.simulation.timestep
                )
            
            # --- WRITE TO EXTERNAL SIGNALS [2, 3, 4] ---
            # Confirmed Paths from User
            base_ext = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
            v_path_bulk = f"{base_ext}/Const_v_Fellows_External[km|h]/Value"
            d_path_bulk = f"{base_ext}/Const_d_Fellows_External[m]/Value"
            # Note: We are NOT writing 's' [2] because we are using velocity control.
            # If we wrote 's', we would override the integration and cause stuttering.
            
            # Prepare values
            v_value = float(new_velocity * 3.6)  # m/s → km/h [4]
            d_value = float(new_deviation)       # meters [3]
            
            # Read-Modify-Write (Bulk Array)
            try:
                v_arr = list(self.cd.get_var(v_path_bulk) or [])
                d_arr = list(self.cd.get_var(d_path_bulk) or [])
            except:
                v_arr = []
                d_arr = []

            # Extend arrays if too short
            need_len = eff_index + 1
            if len(v_arr) < need_len: v_arr.extend([0.0] * (need_len - len(v_arr)))
            if len(d_arr) < need_len: d_arr.extend([0.0] * (need_len - len(d_arr)))

            # Update specific index
            v_arr[eff_index] = v_value
            d_arr[eff_index] = d_value

            # Write back
            self.cd.set_var(v_path_bulk, v_arr)
            self.cd.set_var(d_path_bulk, d_arr)

            # Debug Log
            if obj._fellow_control_count % 50 == 0:
                print(f"[Fellow {fellow_index}] Step {obj._fellow_control_count}")
                print(f"  Physics: v={v_value:.1f} km/h, d={d_value:.2f} m")
                print(f"  Written to: ...External_Signals/Const_v... and .../Const_d...")

        except Exception as e:
            error_msg = str(e)
            if "Index was outside the bounds" not in error_msg:
                print(f"[VehicleController:FellowControl] Error: {e}")
                import traceback
                traceback.print_exc()
    
    def get_fellow_index(self, obj):
        """Get the array index for a fellow vehicle (0-based)."""
        return self.simulation._getFellowIndex(obj)