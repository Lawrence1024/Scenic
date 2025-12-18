"""Racing-specific behaviors for dynamic agents.

These behaviors extend the driving domain behaviors with racing-specific
strategies and maneuvers, using abstract racing protocols that simulators
must implement.
"""

from scenic.domains.driving.behaviors import *
from scenic.domains.driving.controllers import PIDLateralController, PIDLongitudinalController
from scenic.domains.driving.actions import SetThrottleAction, SetBrakeAction, SetSteerAction
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction, SetSpeedLimitAction, SetTTLSelectionAction, SetTargetGapAction, SetStrategyAction, SetPowertrainModeAction, SetScaleFactorAction, SetPush2PassAction, StopCarAction, SetGearAction
import scenic.domains.racing.model as _racing
import math

# Import waypoint finding utility
import sys
import os
_scenic_root = os.path.abspath(os.path.join(os.path.dirname(__file__), '..', '..', '..', '..'))
_tools_path = os.path.join(_scenic_root, 'tools')
if _tools_path not in sys.path:
    sys.path.insert(0, _tools_path)
from get_map_bounds import find_best_racing_waypoint

behavior FollowRacingLineBehavior(target_speed=30, manage_gears=True, use_waypoints=True, lookahead=20.0):
    """Follow the car's TTL using controllers.
    
    Outputs NORMALIZED control signals (-1.0 to 1.0).
    The Simulator (simulator.py) automatically scales these to dSPACE VesiInterface units.
    """
    
    # SETUP & DEFAULTS
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad)
    take SetMaxSpeedAction(target_speed)
    
    throttle_limit = 1.0

    # Ego: keep a conservative base throttle, we may lower further on large CTE
    if self is simulation().scene.egoObject:
        throttle_limit = 0.1

    # Steering slew-rate and CTE safety thresholds
    max_steer_delta = 0.2          # per step (normalized units)
    cte_slowdown_threshold = 15.0  # m: start slowing down
    cte_stop_threshold = 50.0      # m: full brake to avoid runaway

    # Get Controllers (Standard Scenic PIDs return -1.0 to 1.0)
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    
    # Gear thresholds (m/s)
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]

    wp_last_idx = 0

    # CRITICAL: Wait for simulation to initialize and position to be available
    # This ensures arrays are ready for fellows before behavior tries to read position
    wait
    # Additional wait to ensure position is actually readable
    while not hasattr(self, 'position') or self.position is None:
        wait
    
    # Initialize waypoint index based on starting position (helps with waypoint following)
    wp_list_init = (self.waypoints if hasattr(self, 'waypoints') else None)
    if use_waypoints and wp_list_init and len(wp_list_init) >= 2:
        try:
            px = float(self.position.x); py = float(self.position.y)
            
            # Get car heading if available
            car_heading = None
            car_heading_src = None
            # Prefer dSPACE/ControlDesk readback heading if available
            if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
                try:
                    car_heading = float(self.dspaceActor.heading)
                    car_heading_src = "dspaceActor.heading"
                except:
                    pass
            if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
                    car_heading_src = "self.heading"
                except:
                    pass
            
            # First, find the actual nearest waypoint (simple distance-based search)
            # This ensures we start from a reasonable starting point
            nearest_idx = 0
            best_d2 = 1e18
            for i in range(len(wp_list_init)):
                wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            
            # Now use find_best_racing_waypoint starting from the nearest waypoint
            # This ensures we start from a reasonable position and consider forward direction
            result = find_best_racing_waypoint(
                car_position=(px, py),
                car_heading=car_heading if car_heading is not None else 0.0,
                waypoints=wp_list_init,
                last_known_index=nearest_idx,  # Start from nearest, not index 0
                max_search_distance=50.0,  # Smaller search radius since we're starting from nearest
                forward_bias=0.9,
                min_forward_distance=5.0,
                forward_only=False  # Allow any waypoint for initialization
            )
            
            if result:
                wp_last_idx = result['index']
                print(f"[FollowRacingLineBehavior] Initialized: starting at ({px:.2f}, {py:.2f}), "
                      f"waypoint index={wp_last_idx}, distance={result['distance']:.2f}m")
            else:
                # Fallback: use the nearest waypoint we found
                wp_last_idx = nearest_idx
                print(f"[FollowRacingLineBehavior] Initialized (fallback): starting at ({px:.2f}, {py:.2f}), "
                      f"nearest waypoint index={nearest_idx}, distance={best_d2**0.5:.2f}m")
        except Exception as e:
            print(f"[FollowRacingLineBehavior] Warning: Could not initialize waypoint index: {e}, starting from index 0")
            wp_last_idx = 0

    while True:
        # Calculate Control Signals (Standard Logic)
        current_speed = (self.speed if self.speed is not None else 0)
        
        # --- CTE & Waypoints ---
        cte = None
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        
        # If waypoints exist, use lookahead logic
        if use_waypoints and wp_list and len(wp_list) >= 2:
            # Ensure position is available before accessing
            if not hasattr(self, 'position') or self.position is None:
                wait
                continue
            px = float(self.position.x); py = float(self.position.y)
            
            # DEBUG: Log position being used for CTE calculation
            if not hasattr(self, '_cte_debug_count'):
                self._cte_debug_count = 0
            self._cte_debug_count += 1
            if self._cte_debug_count <= 5:
                print(f"[FollowRacingLineBehavior] Step {self._cte_debug_count}: Using position ({px:.2f}, {py:.2f}) for CTE calculation")
                print(f"  First waypoint: ({wp_list[0][0]}, {wp_list[0][1]}), distance = {((px-wp_list[0][0])**2 + (py-wp_list[0][1])**2)**0.5:.2f} m")
            
            # A. Find best waypoint using forward-only racing waypoint finder
            # This guarantees forward progress and considers heading alignment
            car_heading = None
            if hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
                except:
                    pass
            
            # Use find_best_racing_waypoint for robust forward-only waypoint selection
            try:
                result = find_best_racing_waypoint(
                    car_position=(px, py),
                    car_heading=car_heading if car_heading is not None else 0.0,
                    waypoints=wp_list,
                    last_known_index=wp_last_idx,
                    max_search_distance=100.0,
                    forward_bias=0.9,  # Strong forward preference
                    min_forward_distance=5.0,
                    forward_only=True  # CRITICAL: No backtracking allowed
                )
                
                if result:
                    nearest_idx = result['index']
                    wp_last_idx = nearest_idx
                    if self._cte_debug_count <= 5:
                        wp_coord = wp_list[nearest_idx]
                        print(f"  [Waypoint] Found forward waypoint: index={nearest_idx}, "
                              f"coordinate=({wp_coord[0]:.2f}, {wp_coord[1]:.2f}), "
                              f"distance={result['distance']:.2f}m, "
                              f"forward_score={result['forward_score']:.3f}")
                        # Check if waypoint is actually on road
                        if nearest_idx > 0:
                            prev_wp = wp_list[nearest_idx - 1]
                            next_wp = wp_list[nearest_idx + 1] if nearest_idx < len(wp_list) - 1 else wp_list[0]
                            print(f"  [Waypoint] Previous wp {nearest_idx-1}: ({prev_wp[0]:.2f}, {prev_wp[1]:.2f}), "
                                  f"Next wp {nearest_idx+1 if nearest_idx < len(wp_list)-1 else 0}: ({next_wp[0]:.2f}, {next_wp[1]:.2f})")
                else:
                    # Fallback: No forward waypoint found, use simple nearest (shouldn't happen often)
                    print(f"[FollowRacingLineBehavior] Warning: No forward waypoint found, using fallback")
                    nearest_idx = wp_last_idx
                    if nearest_idx >= len(wp_list):
                        nearest_idx = len(wp_list) - 1
            except Exception as e:
                # Fallback to simple nearest if function fails
                print(f"[FollowRacingLineBehavior] Warning: Waypoint finder error: {e}, using fallback")
                nearest_idx = wp_last_idx
                if nearest_idx >= len(wp_list):
                    nearest_idx = len(wp_list) - 1
            
            # B. Find Target Point (Lookahead) - use this for CTE calculation
            # This provides better control by looking ahead instead of using nearest waypoint
            Ld = float(lookahead)
            rem = Ld
            j = nearest_idx
            found_target = False
            lookahead_seg_idx = nearest_idx
            tgt_x = None
            tgt_y = None
            
            # Walk forward along polyline for lookahead target
            while rem > 0.0 and j < len(wp_list) - 1:
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j+1][0]), float(wp_list[j+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len <= 1e-6:
                    j += 1; continue
                
                lookahead_seg_idx = j  # Track this segment for CTE calculation
                
                if rem <= seg_len:
                    u = rem / seg_len
                    tgt_x = x0 + u * seg_dx
                    tgt_y = y0 + u * seg_dy
                    found_target = True
                    break
                else:
                    rem -= seg_len
                    j += 1
            
            # C. Calculate CTE using the lookahead segment (not nearest segment)
            # This provides better control by looking ahead where we want to go
            if found_target and tgt_x is not None and tgt_y is not None:
                # Use the lookahead segment for CTE calculation
                x0, y0 = float(wp_list[lookahead_seg_idx][0]), float(wp_list[lookahead_seg_idx][1])
                x1, y1 = float(wp_list[lookahead_seg_idx+1][0]), float(wp_list[lookahead_seg_idx+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len > 1e-6:
                    # Project vehicle position onto the lookahead segment
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    u_proj = max(0.0, min(1.0, u_proj))
                    proj_x = x0 + u_proj * seg_dx
                    proj_y = y0 + u_proj * seg_dy
                    
                    # Calculate CTE: Project vehicle pos onto segment normal
                    # Left Normal: (-dy, dx) - points LEFT of forward direction
                    # Positive CTE = LEFT of path, Negative CTE = RIGHT of path
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - proj_x)*nx + (py - proj_y)*ny
                    
                    # DEBUG: Log CTE calculation details
                    if self._cte_debug_count <= 5:
                        print(f"  [CTE Debug] Lookahead segment: wp[{lookahead_seg_idx}]=({x0:.2f}, {y0:.2f}) -> wp[{lookahead_seg_idx+1}]=({x1:.2f}, {y1:.2f})")
                        print(f"  [CTE Debug] Lookahead target: ({tgt_x:.2f}, {tgt_y:.2f})")
                        print(f"  [CTE Debug] Vehicle projection on segment: ({proj_x:.2f}, {proj_y:.2f}), u={u_proj:.3f}")
                        print(f"  [CTE Debug] Normal vector: ({nx:.3f}, {ny:.3f})")
                        print(f"  [CTE Debug] CTE = {cte:.3f}m ({'LEFT' if cte > 0 else 'RIGHT'} of path)")
                else:
                    cte = 0.0
            else:
                # Fallback: use nearest segment if lookahead not found
                best_seg_idx = nearest_idx
                x0, y0 = float(wp_list[best_seg_idx][0]), float(wp_list[best_seg_idx][1])
                x1, y1 = float(wp_list[best_seg_idx+1][0]), float(wp_list[best_seg_idx+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len > 1e-6:
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    u_proj = max(0.0, min(1.0, u_proj))
                    proj_x = x0 + u_proj * seg_dx
                    proj_y = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - proj_x)*nx + (py - proj_y)*ny
                else:
                    cte = 0.0

        # Fallback: Use TTL Geometry if waypoints didn't provide CTE
        if cte is None:
            line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else mainRacingRoad)
            if hasattr(line, 'signedDistanceTo'):
                cte = line.signedDistanceTo(self.position)
            else:
                cte = 0.0

        # --- PID Calculation ---
        speed_error = min(self.maxSpeed if hasattr(self, 'maxSpeed') else 200, target_speed) - current_speed
        throttle_pid = _lon_controller.run_step(speed_error)

        # CRITICAL: The steering PID should use NEGATIVE CTE
        # If CTE > 0 (too far left), we want negative steering (steer right)
        # If CTE < 0 (too far right), we want positive steering (steer left)
        
        # Scale down large CTE to prevent PID saturation
        # For racing, we want conservative steering (0.3-0.5 range) to avoid dangerous oversteer
        # For dSPACE with K_P=0.3, K_D=0.15, max_dterm_contribution=0.1:
        # To keep total output ≤ 0.5: PTerm + DTerm ≤ 0.5
        # With DTerm limited to 0.1, max PTerm = 0.4, so max CTE = 0.4/0.3 = 1.33m
        # Using 1.5m for moderate correction: PTerm = 0.3 * 1.5 = 0.45, DTerm ≤ 0.1, total ≤ 0.55
        cte_for_pid = -cte
        max_cte_for_pid = 1.5  # Conservative for racing: PTerm = 0.45, DTerm ≤ 0.1, total ≤ 0.55
        if abs(cte_for_pid) > max_cte_for_pid:
            # Clamp to ±1.5m for safe, controlled steering in racing scenarios
            cte_for_pid = max_cte_for_pid * (1.0 if cte_for_pid > 0 else -1.0)
        
        steer_pid = _lat_controller.run_step(cte_for_pid)

        # --- CTE-aware safety envelope ---
        cte_mag = abs(cte)
        local_throttle_limit = throttle_limit
        final_brake = 0.0

        if cte_mag >= cte_stop_threshold:
            # Way off track: stop and point back toward path gently
            local_throttle_limit = 0.0
            final_brake = 1.0
            steer_pid = -0.5 if cte > 0 else 0.5  # steer toward path without saturation
        elif cte_mag >= cte_slowdown_threshold:
            # Far from path: reduce throttle aggressively, allow some brake
            local_throttle_limit = min(local_throttle_limit, 0.3)
            final_brake = min(1.0, (cte_mag - cte_slowdown_threshold) / (cte_stop_threshold - cte_slowdown_threshold))

        # --- Normalization & slew limiting ---
        final_steer = max(-1.0, min(1.0, steer_pid))
        # Apply simple slew-rate limiter to avoid oscillations
        if not hasattr(self, '_last_final_steer'):
            self._last_final_steer = final_steer
        steer_delta = final_steer - self._last_final_steer
        if steer_delta > max_steer_delta:
            final_steer = self._last_final_steer + max_steer_delta
        elif steer_delta < -max_steer_delta:
            final_steer = self._last_final_steer - max_steer_delta
        self._last_final_steer = final_steer

        final_throttle = 0.0
        if throttle_pid >= 0:
            final_throttle = max(0.0, min(local_throttle_limit, throttle_pid))
        else:
            final_brake = max(final_brake, min(1.0, abs(throttle_pid)))

        # Store CTE in a separate attribute (NOT _control_state which is managed by actions)
        self._current_cte = cte
        
        # Build Action List 
        actions_to_take = [
            SetSteerAction(final_steer), 
            SetThrottleAction(final_throttle), 
            SetBrakeAction(final_brake)
        ]

        # Gear Logic
        gear_changed = False
        new_gear = None
        if manage_gears and hasattr(self, 'setGear'):
            # Default to 0 (Neutral) if unknown, NOT 1
            current_gear = getattr(self, 'gear', 0) 
            
            # Case A: Start from Neutral
            if current_gear < 1:
                actions_to_take.append(SetGearAction(1))
                self.gear = 1
                gear_changed = True
                new_gear = 1
                print(f"  [Gear] Shifting from {current_gear} to 1 (starting from neutral)")
            
            # Case B: Shifting while moving
            elif current_speed is not None:
                if current_gear < 6 and current_speed > gear_up_thresholds[min(current_gear, 5)]:
                    new_gear = current_gear + 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting up from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s > threshold={gear_up_thresholds[min(current_gear, 5)]:.2f})")
                elif current_gear > 1 and current_speed < gear_down_thresholds[min(current_gear - 1, 5)]: # Fixed index
                    new_gear = current_gear - 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting down from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s < threshold={gear_down_thresholds[min(current_gear - 1, 5)]:.2f})")

        # Debug Print
        if hasattr(self, '_behavior_step_count'):
            self._behavior_step_count += 1
        else:
            self._behavior_step_count = 0
        
        # Print every step (not just every 50) so we can debug the TTL following
        gear_val = getattr(self, 'gear', 0)
        print(f"\n[FollowRacingLine] Step {self._behavior_step_count}:")
        print(f"  Position: ({self.position.x:.2f}, {self.position.y:.2f})")
        print(f"  Speed: {current_speed:.2f} m/s")
        print(f"  CTE (Cross-Track Error): {cte:.3f} m {'(LEFT of path)' if cte > 0 else '(RIGHT of path)'}")
        if abs(cte) > 5.0:
            print(f"  PID inputs: speed_error={speed_error:.2f}, cte_input={cte_for_pid:.3f} (clamped from {cte:.3f})")
        else:
            print(f"  PID inputs: speed_error={speed_error:.2f}, cte_input={cte_for_pid:.3f}")
        print(f"  PID outputs: throttle_pid={throttle_pid:.3f}, steer_pid={steer_pid:.3f}")
        print(f"  Final controls: throttle={final_throttle:.3f}, brake={final_brake:.3f}, steer={final_steer:.3f} {'(LEFT)' if final_steer > 0 else '(RIGHT)'}, gear={gear_val}")
        if gear_changed and new_gear is not None:
            print(f"  [Gear Change] Applied: {new_gear}")
        print(f"  → If CTE is positive (left), steering should be negative (right) to correct")

        # Execute all actions together
        take actions_to_take

behavior FollowRacingLineMPCBehavior(target_speed=30, manage_gears=True, use_waypoints=True, lookahead=20.0, mpc_config_path=None):
    """Follow the car's TTL using MPC (Model Predictive Control) for lateral control.
    
    This behavior uses MPC for steering control instead of PID, providing better
    predictive control for racing scenarios, especially in high-speed cornering.
    
    Outputs NORMALIZED control signals (-1.0 to 1.0).
    The Simulator (simulator.py) automatically scales these to dSPACE VesiInterface units.
    
    Args:
        target_speed: Target speed in m/s
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based control
        lookahead: Lookahead distance for waypoint following (meters)
        mpc_config_path: Path to MPC config YAML file (optional, uses default if None)
    """
    
    # SETUP & DEFAULTS
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad)
    take SetMaxSpeedAction(target_speed)
    
    throttle_limit = 1.0

    # Ego: keep a conservative base throttle, we may lower further on large CTE
    if self is simulation().scene.egoObject:
        throttle_limit = 0.1

    # Steering slew-rate and CTE safety thresholds
    max_steer_delta = 0.2          # per step (normalized units)
    cte_slowdown_threshold = 15.0  # m: start slowing down
    cte_stop_threshold = 50.0      # m: full brake to avoid runaway
    
    # Progressive throttle reduction thresholds (for better control when CTE is large)
    cte_throttle_reduction_start = 2.0   # m: start reducing throttle progressively (lowered from 5.0)
    cte_throttle_reduction_max = 10.0    # m: maximum throttle reduction zone
    min_throttle_at_large_cte = 0.03     # minimum throttle when CTE > 10m

    # Get Controllers: Longitudinal PID + Lateral MPC
    _lon_controller, _lat_controller = simulation().getRacingControllers(self, use_mpc=True, mpc_config_path=mpc_config_path)
    
    # Gear thresholds (m/s)
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]

    wp_last_idx = 0

    # CRITICAL: Wait for simulation to initialize and position to be available
    wait
    while not hasattr(self, 'position') or self.position is None:
        wait
    # Also wait for dSPACE heading readback so waypoint-ahead initialization uses a real heading.
    while (not hasattr(self, 'dspaceActor') or self.dspaceActor is None
           or not hasattr(self.dspaceActor, 'heading') or self.dspaceActor.heading is None):
        wait
    
    # Initialize waypoint index based on starting position
    # CRITICAL FIX: Find the first waypoint that is AHEAD of the vehicle
    wp_list_init = (self.waypoints if hasattr(self, 'waypoints') else None)
    if use_waypoints and wp_list_init and len(wp_list_init) >= 2:
        try:
            px = float(self.position.x); py = float(self.position.y)
            
            car_heading = None
            car_heading_src = None
            if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
                try:
                    car_heading = float(self.dspaceActor.heading)
                    car_heading_src = "dspaceActor.heading"
                except:
                    pass
            if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
                    car_heading_src = "self.heading"
                except:
                    pass
            if car_heading is None:
                print("[FollowRacingLineMPCBehavior] Warning: heading unavailable at init; dot-product ahead search disabled")
            
            # Step 1: Find the nearest waypoint (by distance)
            nearest_idx = 0
            best_d2 = 1e18
            for i in range(len(wp_list_init)):
                wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            
            # Step 2: If we have a valid heading, find the first waypoint AHEAD of the vehicle
            # This prevents trying to track waypoints behind the vehicle
            if car_heading is not None:
                # Vehicle forward direction (math is already imported at module level)
                veh_fx = math.cos(car_heading)
                veh_fy = math.sin(car_heading)
                
                # Starting from nearest waypoint, find first one ahead
                # A waypoint is "ahead" if the dot product of (vehicle_to_waypoint) and (vehicle_forward) > 0
                wp_last_idx = nearest_idx
                for i in range(nearest_idx, min(len(wp_list_init), nearest_idx + 100)):
                    wx, wy = float(wp_list_init[i][0]), float(wp_list_init[i][1])
                    to_wp_x = wx - px
                    to_wp_y = wy - py
                    dot_product = to_wp_x * veh_fx + to_wp_y * veh_fy
                    
                    if dot_product > 0:  # Waypoint is ahead
                        wp_last_idx = i
                        wp_dist = (to_wp_x*to_wp_x + to_wp_y*to_wp_y) ** 0.5
                        print(f"[FollowRacingLineMPCBehavior] Initialized: starting at ({px:.2f}, {py:.2f}), heading={car_heading*180/math.pi:.1f}deg (src={car_heading_src})")
                        print(f"  Found first waypoint AHEAD: index={wp_last_idx} at ({wx:.2f}, {wy:.2f}), distance={wp_dist:.2f}m")
                        print(f"  Dot product={dot_product:.2f} (positive means ahead)")
                        break
                else:
                    # No waypoint ahead found in search window, use nearest
                    wp_last_idx = nearest_idx
                    print(f"[FollowRacingLineMPCBehavior] Warning: No waypoint ahead found in search window, using nearest waypoint {nearest_idx}")
            else:
                # No heading available, use nearest waypoint
                wp_last_idx = nearest_idx
                print(f"[FollowRacingLineMPCBehavior] Initialized (no heading): nearest waypoint index={nearest_idx}, distance={best_d2**0.5:.2f}m")
        except Exception as e:
            print(f"[FollowRacingLineMPCBehavior] Warning: Could not initialize waypoint index: {e}, starting from index 0")
            wp_last_idx = 0
        
        # Initialize waypoint progress tracking
        if not hasattr(self, '_waypoints_passed'):
            self._waypoints_passed = 0

    while True:
        # Calculate Control Signals
        current_speed = (self.speed if self.speed is not None else 0)
        
        # --- Waypoint Management for MPC ---
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        
        # Ensure position is available
        if not hasattr(self, 'position') or self.position is None:
            wait
            continue
        
        px = float(self.position.x); py = float(self.position.y)
        car_heading = None
        # Prefer dSPACE/ControlDesk readback heading if available
        if hasattr(self, 'dspaceActor') and self.dspaceActor is not None and hasattr(self.dspaceActor, 'heading') and self.dspaceActor.heading is not None:
            try:
                car_heading = float(self.dspaceActor.heading)
            except:
                pass
        if car_heading is None and hasattr(self, 'heading') and self.heading is not None:
            try:
                car_heading = float(self.heading)
            except:
                pass
        
        # Update waypoint index for MPC
        # SIMPLIFIED (but robust): advance waypoint index using a hit-threshold, with
        # pass-through detection to handle large timestep/high speed (can "jump over" a waypoint).
        old_wp_idx = wp_last_idx
        if use_waypoints and wp_list and len(wp_list) >= 2:
            try:
                # Track previous position for pass-through detection
                if not hasattr(self, '_prev_pos') or self._prev_pos is None:
                    self._prev_pos = (px, py)
                prev_px, prev_py = self._prev_pos

                # NOTE: Scenic behaviors.scenic does NOT support nested Python defs.
                # We compute point-to-segment distance inline below.

                # Calculate distance to current waypoint
                current_wp_dist = None
                if wp_last_idx < len(wp_list):
                    wp_x, wp_y = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                    dx = px - wp_x; dy = py - wp_y
                    current_wp_dist = (dx*dx + dy*dy) ** 0.5
                
                # Hit threshold (meters)
                #
                # IMPORTANT:
                # With large Scenic timesteps (e.g., 1s) and moderate/high speed, we can easily
                # pass *near* a waypoint without ever getting within a tiny fixed radius (3m).
                # If we fail to advance, the controller will keep targeting an old waypoint and
                # can "turn back" (exactly what the log shows for wp=1: min dist ~5.4m).
                #
                # So we scale the hit threshold with the actual travel distance over the last step.
                travel_dx = px - prev_px
                travel_dy = py - prev_py
                travel_dist = (travel_dx*travel_dx + travel_dy*travel_dy) ** 0.5
                HIT_THRESHOLD = 3.0  # base meters
                # Dynamic component: ~60% of last-step travel, capped to avoid skipping too aggressively
                dyn_thr = 0.6 * travel_dist
                if dyn_thr > HIT_THRESHOLD:
                    HIT_THRESHOLD = dyn_thr
                if HIT_THRESHOLD > 12.0:
                    HIT_THRESHOLD = 12.0

                # Advance as many waypoints as we plausibly "hit" this step
                advanced_any = False
                while wp_last_idx < len(wp_list) - 1:
                    wp_x, wp_y = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                    dx_now = px - wp_x; dy_now = py - wp_y
                    d_now = (dx_now*dx_now + dy_now*dy_now) ** 0.5

                    # Pass-through detection: distance from waypoint to (prev_pos -> curr_pos) segment
                    # Compute point-to-segment distance inline:
                    ax = prev_px
                    ay = prev_py
                    bx = px
                    by_ = py
                    vx = bx - ax
                    vy = by_ - ay
                    wx0 = wp_x - ax
                    wy0 = wp_y - ay
                    vv = vx*vx + vy*vy
                    if vv <= 1e-12:
                        # Segment degenerate
                        d_seg = d_now
                    else:
                        t = (wx0*vx + wy0*vy) / vv
                        if t < 0.0:
                            t = 0.0
                        elif t > 1.0:
                            t = 1.0
                        cx = ax + t * vx
                        cy = ay + t * vy
                        ddx = wp_x - cx
                        ddy = wp_y - cy
                        d_seg = (ddx*ddx + ddy*ddy) ** 0.5

                    if d_now < HIT_THRESHOLD or d_seg < HIT_THRESHOLD:
                        advanced_any = True
                        reason = "within_radius" if d_now < HIT_THRESHOLD else "passed_through"
                        print(f"[Waypoint Increment] {reason}: advancing {wp_last_idx} -> {wp_last_idx + 1} (d_now={d_now:.2f}m, d_seg={d_seg:.2f}m, travel={travel_dist:.2f}m, thr={HIT_THRESHOLD:.2f}m)")
                        wp_last_idx += 1
                        continue
                    break

                # Update current_wp_dist for logging below
                current_wp_dist = None
                if wp_last_idx < len(wp_list):
                    wp_x, wp_y = float(wp_list[wp_last_idx][0]), float(wp_list[wp_last_idx][1])
                    dx = px - wp_x; dy = py - wp_y
                    current_wp_dist = (dx*dx + dy*dy) ** 0.5

                # Store current as previous for next step
                self._prev_pos = (px, py)
                
                # Log current waypoint
                if wp_last_idx < len(wp_list):
                    current_wp = wp_list[wp_last_idx]
                    current_wp_x, current_wp_y = float(current_wp[0]), float(current_wp[1])
                    if wp_last_idx != old_wp_idx:
                        # Initialize waypoint progress tracking
                        if not hasattr(self, '_waypoints_passed'):
                            self._waypoints_passed = 0
                        self._waypoints_passed += 1
                        progress_pct = (self._waypoints_passed / len(wp_list)) * 100.0 if len(wp_list) > 0 else 0.0
                        print(f"[FollowRacingLineMPCBehavior] WAYPOINT HIT: index {old_wp_idx} -> {wp_last_idx} at ({current_wp_x:.2f}, {current_wp_y:.2f}), distance={current_wp_dist:.2f}m")
                        print(f"[FollowRacingLineMPCBehavior] Progress: {self._waypoints_passed} waypoints passed ({progress_pct:.1f}% of {len(wp_list)} total waypoints)")
                    else:
                        print(f"[FollowRacingLineMPCBehavior] Waypoint index: {wp_last_idx} at ({current_wp_x:.2f}, {current_wp_y:.2f}), distance={current_wp_dist:.2f}m")
                
                # DISABLED: Complex search logic - using simple increment for testing
                # All waypoint search code below is commented out for testing
                
                """
                # OLD COMPLEX LOGIC: If distance to current waypoint is very large (>10m), find nearest waypoint first
                # This prevents the waypoint index from getting stuck when vehicle has moved far from the waypoint
                # The issue: using wp_last_idx as starting point biases search toward that index
                if current_wp_dist and current_wp_dist > 10.0:
                    print(f"[Waypoint Search] Large distance to waypoint ({current_wp_dist:.1f}m), finding nearest waypoint first...")
                    # First, find the actual nearest waypoint by brute force (scan all waypoints)
                    # This ensures we start from the closest waypoint, not the last known index
                    nearest_idx = wp_last_idx
                    best_d2 = current_wp_dist * current_wp_dist  # Start with current distance
                    # Scan a large window around current index first (faster)
                    scan_window = 500
                    scan_start = max(0, wp_last_idx - scan_window)
                    scan_end = min(len(wp_list), wp_last_idx + scan_window)
                    for i in range(scan_start, scan_end):
                        wp_x, wp_y = float(wp_list[i][0]), float(wp_list[i][1])
                        dx = px - wp_x; dy = py - wp_y
                        d2 = dx*dx + dy*dy
                        if d2 < best_d2:
                            best_d2 = d2
                            nearest_idx = i
                    
                    # If nearest is still far, scan entire waypoint list
                    if best_d2 > 100.0:  # If nearest is still >10m, scan everything
                        print(f"[Waypoint Search] Nearest in window still far ({best_d2**0.5:.1f}m), scanning all waypoints...")
                        nearest_idx = 0
                        best_d2 = 1e18
                        for i in range(len(wp_list)):
                            wp_x, wp_y = float(wp_list[i][0]), float(wp_list[i][1])
                            dx = px - wp_x; dy = py - wp_y
                            d2 = dx*dx + dy*dy
                            if d2 < best_d2:
                                best_d2 = d2
                                nearest_idx = i
                    
                    nearest_dist = best_d2 ** 0.5
                    print(f"[Waypoint Search] Found nearest waypoint: index={nearest_idx}, distance={nearest_dist:.2f}m (was {wp_last_idx} at {current_wp_dist:.2f}m)")
                    
                    # CRITICAL: Always enforce forward progress - never use a backward waypoint as starting point
                    # If nearest waypoint is backward, use current wp_last_idx instead to maintain forward progress
                    search_start_idx = nearest_idx if nearest_idx >= wp_last_idx else wp_last_idx
                    if nearest_idx < wp_last_idx:
                        print(f"[Waypoint Search] Nearest waypoint ({nearest_idx}) is backward, using current index ({wp_last_idx}) to enforce forward progress")
                    
                    # Now use the search_start_idx as starting point for aggressive search
                    # CRITICAL: Always use forward_only=True to prevent backward waypoint jumps
                    result = find_best_racing_waypoint(
                        car_position=(px, py),
                        car_heading=car_heading if car_heading is not None else 0.0,
                        waypoints=wp_list,
                        last_known_index=search_start_idx,  # Always forward from wp_last_idx
                        max_search_distance=500.0,  # Very large search distance
                        forward_bias=0.95,  # Strong forward bias to prefer forward progress
                        min_forward_distance=0.0,  # Remove minimum forward distance requirement
                        forward_only=True  # CRITICAL: Always enforce forward-only to prevent backward jumps
                    )
                    if result:
                        print(f"[Waypoint Search] Found waypoint with aggressive search: index={result['index']}, distance={result.get('distance', 0.0):.2f}m")
                        # If result is still the old index and distance is large, enforce forward progress
                        # Never use a backward waypoint even if it's nearest
                        if result['index'] == wp_last_idx and result.get('distance', 0.0) > 10.0:
                            # Only use nearest if it's forward; otherwise keep current index
                            if nearest_idx >= wp_last_idx:
                                print(f"[Waypoint Search] Aggressive search returned old index with large distance, using forward nearest waypoint ({nearest_idx}) instead")
                                result = {
                                    'index': nearest_idx,
                                    'waypoint': wp_list[nearest_idx],
                                    'distance': nearest_dist,
                                    'forward_score': 1.0,
                                    'heading_alignment': 0.5,
                                    'total_score': 0.0,
                                    'next_index': (nearest_idx + 1) % len(wp_list),
                                    'next_waypoint': wp_list[(nearest_idx + 1) % len(wp_list)]
                                }
                            else:
                                print(f"[Waypoint Search] Aggressive search returned old index with large distance, but nearest ({nearest_idx}) is backward - keeping current index ({wp_last_idx}) to enforce forward progress")
                
                # If aggressive search didn't find anything, try standard forward-only search
                if not result:
                    # Adaptive search distance: increase when CTE is large or distance to waypoint is large
                    # This helps catch waypoints even when vehicle is far off-track
                    base_search_dist = 100.0
                    if current_wp_dist and current_wp_dist > 50.0:
                        # If distance to current waypoint is > 50m, we've likely missed it - search more aggressively
                        search_dist = max(base_search_dist, current_wp_dist * 2.0)  # Search at least 2x the distance
                        print(f"[Waypoint Search] Very large distance to waypoint ({current_wp_dist:.1f}m), using extended search distance ({search_dist:.1f}m)")
                    else:
                        search_dist = base_search_dist
                    
                    # Try standard forward-only search
                    result = find_best_racing_waypoint(
                        car_position=(px, py),
                        car_heading=car_heading if car_heading is not None else 0.0,
                        waypoints=wp_list,
                        last_known_index=wp_last_idx,
                        max_search_distance=search_dist,
                        forward_bias=0.9,
                        min_forward_distance=5.0,
                        forward_only=True
                    )
                
                # Fallback: If no forward waypoint found and distance is large, try more aggressive search
                if not result and current_wp_dist and current_wp_dist > 30.0:
                    print(f"[Waypoint Search] No waypoint found with standard search, trying extended search...")
                    # Try with much larger search distance and relaxed forward constraint
                    extended_result = find_best_racing_waypoint(
                        car_position=(px, py),
                        car_heading=car_heading if car_heading is not None else 0.0,
                        waypoints=wp_list,
                        last_known_index=wp_last_idx,
                        max_search_distance=500.0,  # Very large search distance
                        forward_bias=0.7,  # Slightly less forward bias
                        min_forward_distance=0.0,  # Remove minimum forward distance requirement
                        forward_only=False  # Allow backward search when distance is very large
                    )
                    if extended_result:
                        result = extended_result
                        print(f"[Waypoint Search] Found waypoint with extended search: index={extended_result['index']}, distance={extended_result.get('distance', 0.0):.2f}m")
                
                # Final fallback: If still no result, find nearest waypoint by scanning (forward and backward if needed)
                if not result:
                    print(f"[Waypoint Search] No waypoint found with standard/extended search, scanning waypoints...")
                    # Manually scan waypoints to find the nearest one
                    # If distance is large, scan both forward and backward
                    best_idx = None
                    best_dist = float('inf')
                    
                    if current_wp_dist and current_wp_dist > 20.0:
                        # Very large distance - scan both forward and backward
                        print(f"[Waypoint Search] Large distance ({current_wp_dist:.1f}m), scanning both forward and backward waypoints")
                        scan_start = max(0, wp_last_idx - 200)  # Look back up to 200 waypoints
                        scan_end = min(len(wp_list), wp_last_idx + 500)  # Scan up to 500 waypoints ahead
                    else:
                        # Normal case - scan forward only
                        scan_start = wp_last_idx
                        scan_end = min(len(wp_list), wp_last_idx + 500)  # Scan up to 500 waypoints ahead
                    
                    for i in range(scan_start, scan_end):
                        wp_x, wp_y = float(wp_list[i][0]), float(wp_list[i][1])
                        dx = px - wp_x; dy = py - wp_y
                        dist = (dx*dx + dy*dy) ** 0.5
                        if dist < best_dist:
                            best_dist = dist
                            best_idx = i
                    
                    if best_idx is not None and best_dist < 500.0:  # Only use if within 500m
                        # Create a result-like dict for consistency
                        result = {
                            'index': best_idx,
                            'waypoint': wp_list[best_idx],
                            'distance': best_dist,
                            'forward_score': 1.0 if best_idx >= wp_last_idx else 0.5,  # Forward if ahead, partial if behind
                            'heading_alignment': 0.5,  # Unknown
                            'total_score': 0.0,
                            'next_index': (best_idx + 1) % len(wp_list),
                            'next_waypoint': wp_list[(best_idx + 1) % len(wp_list)]
                        }
                        direction = "forward" if best_idx >= wp_last_idx else "backward"
                        print(f"[Waypoint Search] Found waypoint via manual scan ({direction}): index={best_idx}, distance={best_dist:.2f}m")
                
                if result:
                    new_wp_idx = result['index']
                    wp_last_idx = new_wp_idx
                    # Debug logging: show waypoint index updates
                    wp_dist = result.get('distance', 0.0)
                    
                    if new_wp_idx != old_wp_idx:
                        # Initialize waypoint progress tracking
                        if not hasattr(self, '_waypoints_passed'):
                            self._waypoints_passed = 0
                        
                        # Track forward progress (but handle backward jumps)
                        if new_wp_idx > old_wp_idx:
                            # Forward progress
                            progress = new_wp_idx - old_wp_idx
                            self._waypoints_passed += progress
                        elif new_wp_idx < old_wp_idx:
                            # Backward jump detected (shouldn't happen with forward_only=True, but log it)
                            print(f"[FollowRacingLineMPCBehavior] WARNING: Backward waypoint jump detected: {old_wp_idx} -> {new_wp_idx}")
                        
                        # WAYPOINT HIT: Index changed
                        if new_wp_idx < len(wp_list):
                            hit_wp = wp_list[new_wp_idx]
                            hit_wp_x, hit_wp_y = float(hit_wp[0]), float(hit_wp[1])
                            progress_pct = (self._waypoints_passed / len(wp_list)) * 100.0 if len(wp_list) > 0 else 0.0
                            print(f"[FollowRacingLineMPCBehavior] WAYPOINT HIT: index {old_wp_idx} -> {new_wp_idx} at ({hit_wp_x:.2f}, {hit_wp_y:.2f}), distance={wp_dist:.2f}m")
                            print(f"[FollowRacingLineMPCBehavior] Progress: {self._waypoints_passed} waypoints passed ({progress_pct:.1f}% of {len(wp_list)} total waypoints)")
                        else:
                            print(f"[FollowRacingLineMPCBehavior] Waypoint index updated: {old_wp_idx} -> {new_wp_idx} (distance={wp_dist:.2f}m)")
                    else:
                        # Log current waypoint index with coordinates (every step for now)
                        if wp_last_idx < len(wp_list):
                            current_wp = wp_list[wp_last_idx]
                            current_wp_x, current_wp_y = float(current_wp[0]), float(current_wp[1])
                            print(f"[FollowRacingLineMPCBehavior] Waypoint index: {wp_last_idx} at ({current_wp_x:.2f}, {current_wp_y:.2f}), distance={wp_dist:.2f}m")
                        else:
                            print(f"[FollowRacingLineMPCBehavior] Waypoint index: {wp_last_idx} (distance={wp_dist:.2f}m)")
                else:
                    # Still no waypoint found - log warning but keep current index
                    print(f"[FollowRacingLineMPCBehavior] WARNING: Could not find any forward waypoint, keeping index {wp_last_idx}")
                """  # End of commented-out complex search logic
                
            except Exception as e:
                print(f"[FollowRacingLineMPCBehavior] Warning: Waypoint finder error: {e}")
        
        # --- Longitudinal Control (PID) ---
        # Compute CTE magnitude early for speed error modification
        # (We'll compute full CTE later, but need magnitude now for PID tuning)
        # This makes the PID aware of CTE so it commands braking when off-track
        # IMPORTANT: Use the updated waypoint index (wp_last_idx) and expand search window
        # to handle cases where vehicle has overshot waypoints
        cte_for_pid = None
        if use_waypoints and wp_list and len(wp_list) >= 2:
            # CTE estimate for PID tuning (use nearest waypoint segment)
            # Use adaptive search window: expand when vehicle might be far off-track
            try:
                nearest_idx = wp_last_idx
                best_d2 = 1e18
                # Adaptive search window: start with larger window to handle overshoot
                # Base: ±50 waypoints (~10m at 0.2m spacing)
                # If we find a waypoint far from current index, expand search
                search_window = 50  # Increased from 10
                for i in range(max(0, wp_last_idx - search_window), min(len(wp_list), wp_last_idx + search_window)):
                    wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                    dx = px - wx; dy = py - wy
                    d2 = dx*dx + dy*dy
                    if d2 < best_d2:
                        best_d2 = d2; nearest_idx = i
                
                # If nearest waypoint is far from wp_last_idx, expand search further
                if abs(nearest_idx - wp_last_idx) > search_window * 0.8:
                    # Nearest waypoint is near edge of search window, expand search
                    expanded_window = search_window * 2
                    for i in range(max(0, wp_last_idx - expanded_window), min(len(wp_list), wp_last_idx + expanded_window)):
                        wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                        dx = px - wx; dy = py - wy
                        d2 = dx*dx + dy*dy
                        if d2 < best_d2:
                            best_d2 = d2; nearest_idx = i
                
                if nearest_idx < len(wp_list) - 1:
                    x0, y0 = float(wp_list[nearest_idx][0]), float(wp_list[nearest_idx][1])
                    x1, y1 = float(wp_list[nearest_idx+1][0]), float(wp_list[nearest_idx+1][1])
                    seg_dx = x1 - x0; seg_dy = y1 - y0
                    seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                    if seg_len > 1e-6:
                        wx = px - x0; wy = py - y0
                        u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                        u_proj = max(0.0, min(1.0, u_proj))
                        proj_x = x0 + u_proj * seg_dx
                        proj_y = y0 + u_proj * seg_dy
                        nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                        cte_for_pid = (px - proj_x)*nx + (py - proj_y)*ny
            except:
                pass
        
        # Fallback: Use TTL Geometry for quick CTE estimate
        if cte_for_pid is None:
            line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else mainRacingRoad)
            if hasattr(line, 'signedDistanceTo'):
                try:
                    cte_for_pid = line.signedDistanceTo(self.position)
                except:
                    cte_for_pid = 0.0
            else:
                cte_for_pid = 0.0
        
        cte_mag_for_pid = abs(cte_for_pid) if cte_for_pid is not None else 0.0
        
        # Universal max speed limit: 40 mph = 17.88 m/s
        MAX_SPEED_LIMIT_MS = 17.88  # 40 mph in m/s
        
        # When CTE is large, modify target speed to encourage slowing down
        # This makes the PID aware that we want to reduce speed when off-track
        # The vehicle naturally accelerates even without throttle, so we need to
        # tell the PID to aim for a lower speed (or even brake) when CTE is large
        # FIX 2: Gradual speed limiting for CTE 2-5m (not binary threshold)
        if cte_mag_for_pid >= 10.0:
            # At 10m+ CTE: set target to current speed - 2 m/s (encourages braking)
            # This ensures speed_error is negative, commanding braking
            effective_target_speed = max(0.0, current_speed - 2.0)
        elif cte_mag_for_pid >= 5.0:
            # At 5-10m CTE: set target to current speed (zero throttle)
            # This ensures speed_error is near zero, commanding no throttle
            effective_target_speed = current_speed
        elif cte_mag_for_pid >= 3.0:
            # FIX: 3-5m CTE: Limit to 4 m/s (prevent overshooting when approaching track)
            effective_target_speed = 4.0
        elif cte_mag_for_pid >= 2.0:
            # FIX: 2-3m CTE: Limit to 5 m/s (prevent overshooting when close to track)
            effective_target_speed = 5.0
        elif cte_mag_for_pid >= cte_stop_threshold:
            # At 50m+ CTE: aim for very low speed (encourages heavy braking)
            effective_target_speed = target_speed * 0.1
        elif cte_mag_for_pid >= cte_slowdown_threshold:
            # Between 15-50m: aim for 30% of target speed (encourages braking)
            factor = 0.3
            effective_target_speed = target_speed * factor
        elif cte_mag_for_pid >= cte_throttle_reduction_max:
            # Between 10-15m: linear from 50% to 30% of target speed
            factor = 0.5 - ((cte_mag_for_pid - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.2
            effective_target_speed = target_speed * factor
        else:
            # CTE < 2m: use full target speed (but still respect max speed limit)
            effective_target_speed = target_speed
        
        # Apply universal max speed limit
        effective_target_speed = min(effective_target_speed, MAX_SPEED_LIMIT_MS)
        
        # Debug logging for CTE-aware PID (only log when CTE is significant)
        if cte_mag_for_pid >= cte_throttle_reduction_start:
            print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m, target_speed={target_speed:.1f}m/s -> effective={effective_target_speed:.1f}m/s (current={current_speed:.2f}m/s, max_limit={MAX_SPEED_LIMIT_MS:.1f}m/s)")
        
        speed_error = min(self.maxSpeed if hasattr(self, 'maxSpeed') else 200, effective_target_speed) - current_speed
        throttle_pid = _lon_controller.run_step(speed_error)
        # Clamp PID output to reasonable range (prevent excessive throttle commands)
        throttle_pid = max(-1.0, min(1.0, throttle_pid))
        
        # FIX 1: Force zero/negative throttle AND braking when CTE is very large
        # Override PID output to command braking when far off-track
        # Also apply brake to counteract natural acceleration
        # CRITICAL FIX: Don't apply brake when speed is zero/very low - brake should only slow down moving vehicles
        # Use hysteresis to prevent deadlock at threshold boundary
        
        # Initialize hysteresis state tracking
        if not hasattr(self, '_was_cte_large'):
            self._was_cte_large = False
        
        # Hysteresis: Enter "large CTE" mode at 5.5m, exit at 4.5m (prevents oscillation at boundary)
        if cte_mag_for_pid >= 5.5:
            self._was_cte_large = True
        elif cte_mag_for_pid < 4.5:
            self._was_cte_large = False
        
        # Speed threshold: Only apply brake if speed is significant
        # When stopped, allow throttle to enable movement
        # Progressive brake and throttle control to prevent brake-accelerate cycles
        SPEED_THRESHOLD_FOR_BRAKE = 2.0  # Increased from 1.0 m/s - allows more movement before braking
        MIN_THROTTLE_WHEN_STOPPED = 0.1  # Increased from 0.05 - stronger throttle to start moving
        MIN_THROTTLE_WHEN_MOVING_SLOW = 0.05  # Allow small throttle when moving slowly to enable progress
        
        # SMOOTH DRIVING: Prefer throttle reduction over braking for smoother control
        # Strategy: Use throttle reduction as primary speed control, brake only when necessary
        # This prevents drive-brake-drive cycles and creates smoother deceleration
        cte_brake = 0.0  # Brake command for CTE-based braking
        throttle_override = None  # Optional throttle override (None = use PID output)
        
        if cte_mag_for_pid >= 10.0:
            # Very far off-track (>10m): need active braking
            if current_speed > 4.0:  # Only brake when speed is high
                # High speed + large CTE: apply brake to slow down quickly
                throttle_override = -0.3  # Negative throttle (braking via throttle)
                cte_brake = 0.2  # Moderate brake
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 10m, speed={current_speed:.2f}m/s: APPLYING BRAKE (throttle={throttle_override:.3f}, brake={cte_brake:.3f})")
            elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                # Moderate speed: reduce throttle, light brake
                throttle_override = 0.0  # Zero throttle
                cte_brake = 0.1  # Light brake
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 10m, speed={current_speed:.2f}m/s: REDUCING THROTTLE + LIGHT BRAKE (throttle=0.0, brake={cte_brake:.3f})")
            else:
                # Low speed: allow small throttle to enable movement
                throttle_override = MIN_THROTTLE_WHEN_STOPPED
                cte_brake = 0.0
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 10m but slow (speed={current_speed:.2f}m/s): ALLOWING MIN THROTTLE ({throttle_override:.3f})")
        elif self._was_cte_large or cte_mag_for_pid >= 5.0:
            # Moderate CTE (5-10m): prefer throttle reduction over braking for smoothness
            if current_speed > 5.0:
                # High speed (>5 m/s): use brake to slow down (necessary for safety)
                if cte_mag_for_pid >= 7.0:
                    throttle_override = 0.0  # Zero throttle
                    cte_brake = 0.08  # Light brake
                else:
                    throttle_override = 0.0  # Zero throttle
                    cte_brake = 0.05  # Very light brake
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 5m, speed={current_speed:.2f}m/s: REDUCING THROTTLE + LIGHT BRAKE (throttle=0.0, brake={cte_brake:.3f})")
            elif current_speed > 3.0:
                # Moderate speed (3-5 m/s): reduce throttle, no brake (smooth deceleration)
                throttle_override = 0.0  # Zero throttle - let vehicle coast/slow naturally
                cte_brake = 0.0  # No brake - throttle reduction is sufficient
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 5m, speed={current_speed:.2f}m/s: REDUCING THROTTLE ONLY (throttle=0.0, brake=0.0) - smooth deceleration")
            elif current_speed > SPEED_THRESHOLD_FOR_BRAKE:
                # Low speed (2-3 m/s): allow small throttle for progress, no brake
                # Use PID output but reduce it gradually based on CTE
                throttle_reduction_factor = 1.0 - (cte_mag_for_pid - 5.0) / 5.0  # 0.0 at 10m CTE, 1.0 at 5m CTE
                throttle_reduction_factor = max(0.3, min(1.0, throttle_reduction_factor))  # Clamp to 30-100%
                throttle_override = None  # Use PID output, but will be reduced later
                cte_brake = 0.0  # No brake
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 5m, speed={current_speed:.2f}m/s: GRADUAL THROTTLE REDUCTION (factor={throttle_reduction_factor:.2f}, brake=0.0)")
            else:
                # Stopped or very slow: allow min throttle to start moving
                throttle_override = MIN_THROTTLE_WHEN_STOPPED
                cte_brake = 0.0
                print(f"[CTE-Aware PID] CTE={cte_mag_for_pid:.2f}m >= 5m but stopped (speed={current_speed:.2f}m/s): ALLOWING MIN THROTTLE ({throttle_override:.3f}) TO START MOVING")
        
        # Apply throttle override if set
        if throttle_override is not None:
            throttle_pid = throttle_override
        elif current_speed > SPEED_THRESHOLD_FOR_BRAKE and current_speed <= 3.0 and (self._was_cte_large or cte_mag_for_pid >= 5.0):
            # Apply gradual throttle reduction for low-moderate speed
            throttle_reduction_factor = 1.0 - (cte_mag_for_pid - 5.0) / 5.0  # 0.0 at 10m CTE, 1.0 at 5m CTE
            throttle_reduction_factor = max(0.3, min(1.0, throttle_reduction_factor))  # Clamp to 30-100%
            throttle_pid = throttle_pid * throttle_reduction_factor
        
        # Debug logging for PID output when CTE is large
        if cte_mag_for_pid >= cte_throttle_reduction_start:
            print(f"[CTE-Aware PID] speed_error={speed_error:.2f}m/s, throttle_pid={throttle_pid:.3f}, cte_brake={cte_brake:.3f}")
        
        # --- Lateral Control (MPC) ---
        # Build vehicle state for MPC
        vehicle_state = {
            'x': px,
            'y': py,
            'yaw': car_heading if car_heading is not None else 0.0,
            'speed': current_speed,
        }
        
        # Add gear information for MPC (check before gear change logic)
        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', 0)
            vehicle_state['gear'] = current_gear
        
        # Add optional yaw_rate if available
        if hasattr(self, 'angularVelocity') and self.angularVelocity is not None:
            try:
                vehicle_state['yaw_rate'] = float(self.angularVelocity.z) if hasattr(self.angularVelocity, 'z') else 0.0
            except:
                pass
        
        # Try to read steering feedback from ControlDesk
        # This provides actual steering angle (delta) for MPC state
        sim = simulation()
        if hasattr(sim, 'mpc_config') and sim.mpc_config:
            from scenic.domains.racing.mpc.io_adapter import read_state_from_controldesk
            try:
                cd_state = read_state_from_controldesk(sim, self)
                if 'steer_actual' in cd_state:
                    vehicle_state['steer_actual'] = cd_state['steer_actual']
            except Exception as e:
                # If reading fails, MPC will use previous state estimate
                pass
        
        # Convert waypoints to list of tuples for MPC
        waypoints_for_mpc = None
        if wp_list and len(wp_list) >= 2:
            waypoints_for_mpc = [(float(wp[0]), float(wp[1])) for wp in wp_list]
        
        # Compute steering using MPC
        # Pass CTE magnitude for adaptive waypoint search
        try:
            steer_mpc = _lat_controller.run_step(
                vehicle_state, 
                waypoints_for_mpc, 
                wp_last_idx,
                cte_magnitude=cte_mag_for_pid  # Pass CTE magnitude for adaptive search
            )
        except Exception as ex:
            print(f"[FollowRacingLineMPCBehavior] MPC error: {ex}, using fallback")
            steer_mpc = 0.0
        
        # --- CTE-aware safety envelope (for throttle/brake) ---
        # Compute CTE for safety checks (similar to PID behavior)
        cte = None
        if use_waypoints and wp_list and len(wp_list) >= 2:
            # Use lookahead segment for CTE calculation
            Ld = float(lookahead)
            rem = Ld
            j = wp_last_idx
            found_target = False
            lookahead_seg_idx = wp_last_idx
            
            while rem > 0.0 and j < len(wp_list) - 1:
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j+1][0]), float(wp_list[j+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len <= 1e-6:
                    j += 1; continue
                
                lookahead_seg_idx = j
                
                if rem <= seg_len:
                    found_target = True
                    break
                else:
                    rem -= seg_len
                    j += 1
            
            if found_target:
                x0, y0 = float(wp_list[lookahead_seg_idx][0]), float(wp_list[lookahead_seg_idx][1])
                x1, y1 = float(wp_list[lookahead_seg_idx+1][0]), float(wp_list[lookahead_seg_idx+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len > 1e-6:
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    u_proj = max(0.0, min(1.0, u_proj))
                    proj_x = x0 + u_proj * seg_dx
                    proj_y = y0 + u_proj * seg_dy
                    
                    # Compute normal vector: (-dy, dx) points LEFT of forward direction
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    
                    # Apply heading flip logic to match MPC's internal CTE calculation
                    # This ensures displayed CTE matches what MPC uses for control
                    if car_heading is not None:
                        seg_heading = math.atan2(seg_dy, seg_dx)
                        heading_diff = seg_heading - car_heading
                        # Normalize to [-pi, pi]
                        heading_diff = math.atan2(math.sin(heading_diff), math.cos(heading_diff))
                        # If heading difference > 90°, flip the normal vector (same as MPC does)
                        if abs(heading_diff) > math.pi / 2:
                            nx = -nx
                            ny = -ny
                    
                    cte = (px - proj_x)*nx + (py - proj_y)*ny
        
        # Fallback: Use TTL Geometry if waypoints didn't provide CTE
        if cte is None:
            line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else mainRacingRoad)
            if hasattr(line, 'signedDistanceTo'):
                cte = line.signedDistanceTo(self.position)
            else:
                cte = 0.0
        
        cte_mag = abs(cte)
        local_throttle_limit = throttle_limit
        final_brake = 0.0

        # Progressive throttle reduction based on CTE magnitude
        # This prevents the vehicle from accelerating too fast when off-track
        if cte_mag >= cte_stop_threshold:
            local_throttle_limit = 0.0
            final_brake = 1.0
            steer_mpc = -0.5 if cte > 0 else 0.5
        elif cte_mag >= cte_slowdown_threshold:
            # Between 15m and 50m: progressive braking
            local_throttle_limit = min(local_throttle_limit, 0.3)
            final_brake = min(1.0, (cte_mag - cte_slowdown_threshold) / (cte_stop_threshold - cte_slowdown_threshold))
        elif cte_mag >= cte_throttle_reduction_max:
            # Between 10m and 15m: aggressive throttle reduction
            # Linear reduction from base throttle_limit to 0.3
            throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_max) / (cte_slowdown_threshold - cte_throttle_reduction_max)) * 0.7
            local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
            local_throttle_limit = max(local_throttle_limit, 0.3)  # Don't go below 0.3 in this zone
        elif cte_mag >= cte_throttle_reduction_start:
            # Between 2m and 10m: progressive throttle reduction
            # Linear reduction from base throttle_limit to min_throttle_at_large_cte
            throttle_factor = 1.0 - ((cte_mag - cte_throttle_reduction_start) / (cte_throttle_reduction_max - cte_throttle_reduction_start)) * (1.0 - min_throttle_at_large_cte / throttle_limit)
            local_throttle_limit = min(local_throttle_limit, throttle_limit * throttle_factor)
            local_throttle_limit = max(local_throttle_limit, min_throttle_at_large_cte)
        
        # Speed-based throttle reduction when CTE is large (prevents overshooting)
        # At higher speeds with large CTE, reduce throttle more aggressively
        if cte_mag >= cte_throttle_reduction_start and current_speed > 3.0:
            # Speed penalty: more aggressive reduction at high speeds
            # At 3 m/s: 0% reduction, at 8 m/s: 50% reduction, at 13+ m/s: 80% reduction
            if current_speed <= 8.0:
                speed_penalty = (current_speed - 3.0) / 10.0  # 0 at 3 m/s, 0.5 at 8 m/s
            else:
                speed_penalty = 0.5 + ((current_speed - 8.0) / 10.0) * 0.3  # 0.5 at 8 m/s, 0.8 at 13+ m/s
            speed_penalty = min(0.8, speed_penalty)  # Cap at 80% reduction
            speed_factor = 1.0 - speed_penalty
            local_throttle_limit = local_throttle_limit * speed_factor
        
        # Additional throttle reduction for moderate CTE (2-4m) when speed is high
        # This prevents overshooting when approaching the track at high speed
        if cte_mag >= 2.0 and cte_mag < 5.0 and current_speed > 4.0:
            # More aggressive reduction: at 4 m/s: 0%, at 6 m/s: 50%, at 8+ m/s: 80%
            if current_speed <= 6.0:
                moderate_cte_penalty = (current_speed - 4.0) / 4.0  # 0 at 4 m/s, 0.5 at 6 m/s
            else:
                moderate_cte_penalty = 0.5 + ((current_speed - 6.0) / 4.0) * 0.3  # 0.5 at 6 m/s, 0.8 at 8+ m/s
            moderate_cte_penalty = min(0.8, moderate_cte_penalty)  # Cap at 80% reduction
            moderate_cte_factor = 1.0 - moderate_cte_penalty
            local_throttle_limit = local_throttle_limit * moderate_cte_factor

        # --- Normalization & slew limiting ---
        steer_mpc_raw = steer_mpc
        final_steer = max(-1.0, min(1.0, steer_mpc_raw))
        # Apply simple slew-rate limiter
        if not hasattr(self, '_last_final_steer'):
            self._last_final_steer = final_steer
        prev_steer = self._last_final_steer
        steer_delta = final_steer - prev_steer
        limited = False
        if steer_delta > max_steer_delta:
            final_steer = prev_steer + max_steer_delta
            limited = True
        elif steer_delta < -max_steer_delta:
            final_steer = prev_steer - max_steer_delta
            limited = True
        self._last_final_steer = final_steer
        print(
            f"[Steer Slew DBG] mpc_raw={float(steer_mpc_raw):+.3f} clamp={float(max(-1.0, min(1.0, steer_mpc_raw))):+.3f} "
            f"prev={float(prev_steer):+.3f} delta={float(steer_delta):+.3f} max_delta={float(max_steer_delta):+.3f} "
            f"limited={limited} final={float(final_steer):+.3f}"
        )

        # Ensure local_throttle_limit doesn't exceed base throttle_limit
        local_throttle_limit = min(local_throttle_limit, throttle_limit)
        
        # SMOOTH DRIVING: Prefer throttle reduction over braking, avoid simultaneous throttle+brake
        # Combine CTE-based brake with existing brake logic
        final_brake = max(final_brake, cte_brake)
        
        final_throttle = 0.0
        if throttle_pid >= 0:
            # Clamp throttle to local_throttle_limit (which includes CTE and speed reductions)
            final_throttle = max(0.0, min(local_throttle_limit, throttle_pid))
        else:
            # If PID commands negative throttle (braking), add to brake
            final_brake = max(final_brake, min(1.0, abs(throttle_pid)))
        
        # SMOOTH DRIVING FIX: Avoid simultaneous throttle and brake for moderate CTE (5-7m)
        # When CTE is moderate and we have both throttle and brake, prefer throttle reduction
        if final_throttle > 0.0 and final_brake > 0.0 and cte_mag >= 5.0 and cte_mag < 7.0:
            # Moderate CTE: prefer throttle reduction over braking for smoothness
            if current_speed < 4.0:
                # Low-moderate speed: remove brake, reduce throttle instead
                throttle_reduction = final_brake * 0.5  # Reduce throttle by brake amount
                final_throttle = max(0.0, final_throttle - throttle_reduction)
                final_brake = 0.0  # Remove brake
                print(f"[Smooth Driving] CTE={cte_mag:.2f}m, speed={current_speed:.2f}m/s: Removing brake, reducing throttle instead (throttle={final_throttle:.3f}, brake=0.0)")
            else:
                # Higher speed: keep brake, remove throttle (brake is necessary)
                final_throttle = 0.0
                print(f"[Smooth Driving] CTE={cte_mag:.2f}m, speed={current_speed:.2f}m/s: Removing throttle, keeping brake (throttle=0.0, brake={final_brake:.3f})")
        
        # Apply universal max speed limit: if current speed exceeds limit, reduce throttle/apply brake
        if current_speed > MAX_SPEED_LIMIT_MS:
            # Speed exceeds limit: reduce throttle and apply brake
            speed_excess = current_speed - MAX_SPEED_LIMIT_MS
            # More aggressive braking for larger excess
            if speed_excess > 2.0:
                final_throttle = 0.0
                final_brake = max(final_brake, 0.5)  # Strong brake
            elif speed_excess > 1.0:
                final_throttle = 0.0
                final_brake = max(final_brake, 0.3)  # Moderate brake
            else:
                final_throttle = max(0.0, final_throttle * 0.5)  # Reduce throttle
                final_brake = max(final_brake, 0.1)  # Light brake
            print(f"[Speed Limit] Speed {current_speed:.2f}m/s exceeds limit {MAX_SPEED_LIMIT_MS:.1f}m/s, applying brake={final_brake:.3f}")

        # Store CTE for debugging
        self._current_cte = cte
        
        # Build Action List 
        actions_to_take = [
            SetSteerAction(final_steer), 
            SetThrottleAction(final_throttle), 
            SetBrakeAction(final_brake)
        ]

        # Gear Logic (same as PID behavior)
        gear_changed = False
        new_gear = None
        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', 0) 
            
            if current_gear < 1:
                actions_to_take.append(SetGearAction(1))
                self.gear = 1
                gear_changed = True
                new_gear = 1
                print(f"  [Gear] Shifting from {current_gear} to 1 (starting from neutral)")
            
            elif current_speed is not None:
                if current_gear < 6 and current_speed > gear_up_thresholds[min(current_gear, 5)]:
                    new_gear = current_gear + 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting up from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")
                elif current_gear > 1 and current_speed < gear_down_thresholds[min(current_gear - 1, 5)]:
                    new_gear = current_gear - 1
                    actions_to_take.append(SetGearAction(new_gear))
                    self.gear = new_gear
                    gear_changed = True
                    print(f"  [Gear] Shifting down from {current_gear} to {new_gear} (speed={current_speed:.2f} m/s)")

        # Debug Print
        if hasattr(self, '_behavior_step_count'):
            self._behavior_step_count += 1
        else:
            self._behavior_step_count = 0
        
        gear_val = getattr(self, 'gear', 0)
        print(f"\n[FollowRacingLineMPC] Step {self._behavior_step_count}:")
        print(f"  Position: ({px:.2f}, {py:.2f})")
        print(f"  Speed: {current_speed:.2f} m/s")
        print(f"  CTE: {cte:.3f} m {'(LEFT)' if cte > 0 else '(RIGHT)'}")
        
        # Add waypoint information
        if use_waypoints and wp_list and len(wp_list) >= 2 and wp_last_idx is not None:
            if wp_last_idx < len(wp_list):
                current_wp = wp_list[wp_last_idx]
                current_wp_x, current_wp_y = float(current_wp[0]), float(current_wp[1])
                # Calculate distance to current waypoint
                dx_to_wp = px - current_wp_x
                dy_to_wp = py - current_wp_y
                dist_to_wp = (dx_to_wp*dx_to_wp + dy_to_wp*dy_to_wp) ** 0.5
                
                print(f"  Current waypoint: index={wp_last_idx}, coord=({current_wp_x:.2f}, {current_wp_y:.2f}), distance={dist_to_wp:.2f}m")
                
                # Next waypoint information
                if wp_last_idx < len(wp_list) - 1:
                    next_wp = wp_list[wp_last_idx + 1]
                    next_wp_x, next_wp_y = float(next_wp[0]), float(next_wp[1])
                    # Calculate segment heading (orientation)
                    seg_dx = next_wp_x - current_wp_x
                    seg_dy = next_wp_y - current_wp_y
                    seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                    if seg_len > 1e-6:
                        seg_heading = math.atan2(seg_dy, seg_dx)  # radians
                        seg_heading_deg = seg_heading * 180.0 / math.pi
                        print(f"  Next waypoint: index={wp_last_idx + 1}, coord=({next_wp_x:.2f}, {next_wp_y:.2f}), segment_heading={seg_heading_deg:.1f}deg")
                elif wp_last_idx == len(wp_list) - 1:
                    print(f"  Next waypoint: N/A (at last waypoint)")
        else:
            print(f"  Current waypoint: N/A (no waypoints available)")
        
        print(f"  MPC steering: {steer_mpc:.3f} -> {final_steer:.3f} (after slew limit)")
        print(f"  Final controls: throttle={final_throttle:.3f}, brake={final_brake:.3f}, steer={final_steer:.3f}, gear={gear_val}")

        # Execute all actions together
        take actions_to_take

behavior PitStopBehavior(manage_gears=True):
    """Execute a pit stop using racing-specific systems.
    
    This behavior demonstrates the use of racing-specific actions like
    pit limiter and ERS deployment.
    """
    
    # Enter pit lane with speed limiter
    take PitLimiterAction(activate=True)
    do FollowRacingLineBehavior(target_speed=20, manage_gears=manage_gears)
    
    # Stop for pit stop
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take PitLimiterAction(activate=False)

behavior OvertakingBehavior(target_car, aggressive=False):
    """Attempt to overtake target car using racing systems.
    
    This behavior uses DRS and ERS systems for overtaking maneuvers.
    
    Args:
        target_car: The car to overtake
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    # Close the gap
    while (distance from self to target_car) > 5:
        do FollowRacingLineBehavior(target_speed=35)
    
    # Execute overtake with racing systems
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)
        take DRSAction(activate=True)
    
    # Move to side and accelerate
    take SetThrottleAction(1.0)
    
    # Complete overtake
    do FollowRacingLineBehavior() until (distance from self to target_car) > 10
    
    # Return to racing line
    do FollowRacingLineBehavior()

behavior DefensiveBehavior():
    """Defend position using racing-specific systems.
    
    This behavior uses traction control and brake bias adjustments
    for defensive driving.
    """
    
    # Adjust racing systems for defense
    take TractionControlAction(level=8)  # More conservative TC
    take BrakeBiasAction(bias=0.6)  # More front bias for stability
    
    # Follow racing line defensively
    do FollowRacingLineBehavior(target_speed=25)


## Decision tree behaviors (for race decision engine integration)

behavior FlagBasedSpeedBehavior(speed_type="green", speed_limit=None, manage_gears=True):
    """Set speed based on flag type (decision tree behavior).
    
    This behavior sets the speed limit based on race flags (yellow, green, etc.)
    and applies it to the vehicle.
    
    Args:
        speed_type: Speed type string - "yellow", "double_yellow", "green", "round", etc.
        speed_limit: Speed limit in m/s (if None, uses default for speed_type)
    """
    
    # Set speed limit based on type
    if speed_limit is None:
        # Default speeds (can be overridden with params)
        speed_limits = {
            "pit_crawl": 10.0,
            "pit_lane": 20.0,
            "pit_road": 25.0,
            "yellow": 40.0,
            "double_yellow": 90.0,
            "green": 120.0,
            "round": 120.0,
            "stop": 0.0
        }
        speed_limit = speed_limits.get(speed_type, 120.0)
    
    take SetSpeedLimitAction(speed_limit=speed_limit, speed_type=speed_type)
    
    # Apply speed limit via FollowRacingLineBehavior
    do FollowRacingLineBehavior(target_speed=speed_limit, manage_gears=manage_gears)


behavior LaneSelectionBehavior(ttl_selection="race", manage_gears=True):
    """Select TTL based on attacker/defender flags (decision tree behavior).
    
    This behavior selects the appropriate TTL (left for defender, right for attacker,
    race for optimal) and sets the speed accordingly.
    
    Args:
        ttl_selection: TTL selection string - "left", "right", "race", "optimal", or "pit"
    """
    
    take SetTTLSelectionAction(selection=ttl_selection)
    
    # Set speed based on selection (green speed for racing, slower for pit)
    if ttl_selection == "pit":
        do FlagBasedSpeedBehavior(speed_type="pit_lane", speed_limit=20.0, manage_gears=manage_gears)
    else:
        do FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0, manage_gears=manage_gears)


behavior StopBehavior(stop_type="safe"):
    """Stop car with specified stop type (decision tree behavior).
    
    This behavior implements emergency, immediate, or safe stop behavior.
    
    Args:
        stop_type: Stop type string - "emergency", "immediate", or "safe"
    """
    
    take StopCarAction(stop_type=stop_type)
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")


behavior FollowModeBehavior(target_car, target_gap=31.0, manage_gears=True, use_waypoints=True, lookahead=20.0):
    """Follow another car maintaining target gap (decision tree behavior).
    
    This behavior implements follow mode strategy where the car maintains
    a target gap distance to the car ahead.
    
    Args:
        target_car: The car to follow
        target_gap: Target gap distance in meters
    """
    
    take SetStrategyAction(strategy_type="follow_mode")
    take SetTargetGapAction(gap=target_gap, gap_type="attacker_preparing")
    
    # Get controllers
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    past_steer_angle = 0
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]
    
    # Waypoint state (nearest index), used only if waypoints are available
    wp_last_idx = 0
    
    while True:
        # Compute gap to target car
        current_gap = distance from self to target_car
        
        # Compute speed error based on gap
        gap_error = current_gap - target_gap
        
        # Adjust speed to maintain gap
        if gap_error > 5.0:  # Too far, speed up
            target_speed = (target_car.speed if target_car.speed is not None else 0) + 2.0
        elif gap_error < -5.0:  # Too close, slow down
            target_speed = (target_car.speed if target_car.speed is not None else 0) - 2.0
        else:
            target_speed = target_car.speed if target_car.speed is not None else 0
        
        # Clamp to max speed
        target_speed = min(target_speed, self.maxSpeed if hasattr(self, 'maxSpeed') else 120.0)
        
        # Get TTL to follow
        line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else (track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad))
        
        # Cross-track error (waypoint-targeted if available)
        cte = None
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)
        if use_waypoints and wp_list and len(wp_list) >= 2:
            px = float(self.position.x); py = float(self.position.y)
            nearest_idx = 0; best_d2 = 1e18
            for i in range(max(0, wp_last_idx - 25), min(len(wp_list), wp_last_idx + 26)):
                wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            wp_last_idx = nearest_idx
            Ld = float(lookahead)
            tgt_idx = nearest_idx; rem = Ld; j = nearest_idx
            while rem > 0.0 and j < len(wp_list) - 1:
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j+1][0]), float(wp_list[j+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    j += 1; continue
                if rem <= seg_len:
                    u = rem / seg_len
                    # projection for signed error
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
                    break
                else:
                    rem -= seg_len; j += 1; tgt_idx = j
            if cte is None:
                k0 = max(0, min(len(wp_list)-2, wp_last_idx))
                x0, y0 = float(wp_list[k0][0]), float(wp_list[k0][1])
                x1, y1 = float(wp_list[k0+1][0]), float(wp_list[k0+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                if seg_len <= 1e-6:
                    cte = 0.0
                else:
                    wx = px - x0; wy = py - y0
                    u_proj = (wx*seg_dx + wy*seg_dy) / (seg_len*seg_len)
                    if u_proj < 0.0: u_proj = 0.0
                    if u_proj > 1.0: u_proj = 1.0
                    qx = x0 + u_proj * seg_dx; qy = y0 + u_proj * seg_dy
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - qx)*nx + (py - qy)*ny
        if cte is None:
            cte = line.signedDistanceTo(self.position)
        current_speed = (self.speed if self.speed is not None else 0)
        speed_error = target_speed - current_speed

        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', None)
            if current_gear is None or current_gear < 1:
                take SetGearAction(1)
                self.gear = 1
                current_gear = 1
            elif current_speed is not None:
                if current_gear < 6 and current_speed > gear_up_thresholds[current_gear]:
                    take SetGearAction(current_gear + 1)
                    self.gear = current_gear + 1
                    current_gear = self.gear
                elif current_gear > 1 and current_speed < gear_down_thresholds[current_gear - 1]:
                    take SetGearAction(current_gear - 1)
                    self.gear = current_gear - 1
                    current_gear = self.gear
        
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer


behavior PitLaneBehavior(manage_gears=True):
    """Handle pit lane speeds (decision tree behavior).
    
    This behavior implements pit lane speed limits: pit crawl (10 m/s),
    pit lane (20 m/s), and pit road (25 m/s).
    """
    
    # Determine which pit zone we're in (simplified - would need location detection)
    # For now, use pit_lane speed as default
    take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
    take SetTTLSelectionAction(selection="pit")
    take SetTargetGapAction(gap=0.0, gap_type="no_gap")
    
    # Apply pit lane speed
    do FollowRacingLineBehavior(target_speed=20.0, manage_gears=manage_gears)


behavior SimpleRaceBehavior(manage_gears=True, use_waypoints=True, lookahead=20.0, 
                           out_of_bounds_tolerance=5.0):
    """Simplified race decision tree behavior.
    
    Priority-based decision making:
    1. Emergency stop (if out of bounds)
    2. Pit lane behavior (if in pit lane)
    3. Green flag behavior (normal racing)
    
    Args:
        manage_gears: Whether to automatically manage gears
        use_waypoints: Whether to use waypoint-based steering
        lookahead: Lookahead distance for waypoint steering (meters)
        out_of_bounds_tolerance: Distance tolerance for out-of-bounds check (meters)
    """
    
    while True:
        # ============================================================
        # PRIORITY 1: Emergency Stop Check (Out of Bounds)
        # ============================================================
        
        # Check if car is still within track bounds
        # Option 1: Check if position is in road region
        is_in_bounds = road.contains(self.position) if hasattr(road, 'contains') else True
        
        # Option 2: Check distance to road (more lenient)
        if not is_in_bounds:
            # Check if we're close enough to road (within tolerance)
            distance_to_road = road.distanceTo(self.position) if hasattr(road, 'distanceTo') else 0.0
            is_in_bounds = distance_to_road <= out_of_bounds_tolerance
        
        # Emergency stop if out of bounds
        if not is_in_bounds:
            take StopCarAction(stop_type="emergency")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            # Emergency stop - exit behavior
            break
        
        # ============================================================
        # PRIORITY 2: Pit Lane vs Green Flag
        # ============================================================
        
        # Check if we're in pit lane
        in_pit_lane = False
        if hasattr(track, 'pitLaneRoad') and track.pitLaneRoad:
            in_pit_lane = track.pitLaneRoad.contains(self.position)
        
        if in_pit_lane:
            # PIT LANE BEHAVIOR
            take SetSpeedLimitAction(speed_limit=20.0, speed_type="pit_lane")
            take SetTTLSelectionAction(selection="pit")
            take SetTargetGapAction(gap=0.0, gap_type="no_gap")
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute pit lane behavior
            do FollowRacingLineBehavior(target_speed=20.0, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints, lookahead=lookahead)
        else:
            # GREEN FLAG BEHAVIOR (Normal Racing)
            green_speed = 120.0  # Default green speed (m/s)
            take SetSpeedLimitAction(speed_limit=green_speed, speed_type="green")
            take SetTTLSelectionAction(selection="race")  # Use race TTL
            take SetStrategyAction(strategy_type="cruise_control")
            
            # Execute green flag behavior
            do FollowRacingLineBehavior(target_speed=green_speed, manage_gears=manage_gears,
                                       use_waypoints=use_waypoints, lookahead=lookahead)
        
        wait  # Wait one timestep before re-evaluating