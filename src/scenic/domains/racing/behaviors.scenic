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
            if hasattr(self, 'heading') and self.heading is not None:
                try:
                    car_heading = float(self.heading)
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