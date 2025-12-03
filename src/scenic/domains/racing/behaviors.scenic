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

behavior FollowRacingLineBehavior(target_speed=30, manage_gears=True, use_waypoints=True, lookahead=20.0):
    """Follow the car's TTL using controllers.
    
    Outputs NORMALIZED control signals (-1.0 to 1.0).
    The Simulator (simulator.py) automatically scales these to dSPACE VesiInterface units.
    """
    
    # --- 1. SETUP & DEFAULTS ---
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(track.racingLine if hasattr(track, 'racingLine') and track.racingLine else mainRacingRoad)
    take SetMaxSpeedAction(target_speed)
    
    # Get Controllers (Standard Scenic PIDs return -1.0 to 1.0)
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    
    # Gear thresholds (m/s)
    gear_up_thresholds = [0.0, 12.0, 22.0, 32.0, 42.0, 52.0]
    gear_down_thresholds = [0.0, 9.0, 18.0, 28.0, 38.0, 48.0]
    
    wp_last_idx = 0

    def distance_to_ttl(fellow_x, fellow_y, ttl):
        steer_sign = 0.0
        closest_seg_distance = float('inf')

        for i in range(len(ttl) - 1):
            # defining ttl segment
            x1, y1 = ttl[i]
            x2, y2 = ttl[i + 1]

            # segment vectors
            dx, dy = x2-x1, y2-y1
            seg_len_sq = dx**2 + dy**2
            if seg_len_sq < 1e-6:
                continue
            
            # fellow position, vectors, and projection
            fellow_ttl_dx, fellow_ttl_dy = fellow_x - x1, fellow_y - y1
            proj_fellow_ttl_seg_len = (fellow_ttl_dx*dx + fellow_ttl_dy*dy) / seg_len_sq
            proj_fellow_ttl_seg_len = max(0.0, min(1.0, proj_fellow_ttl_seg_len))
            
            # closest point on segment
            closest_x = x1 + proj_fellow_ttl_seg_len * dx
            closest_y = y1 + proj_fellow_ttl_seg_len * dy

            # perpendicular distance from fellow to TTL segment closes point
            perp_dx = fellow_ttl_dx - closest_x
            perp_dy = fellow_ttl_dy - closest_y
            dist = (perp_dx**2 + perp_dy**2)**0.5

            cross_prod = dx*perp_dy - dy*perp_dx
            signed = dist if cross_prod > 0.0 else -dist

            if abs(dist) < closest_seg_distance:
                closest_seg_distance = abs(dist)
                steer_sign = signed

        return steer_sign
    
    while True:
        current_speed = (self.speed if self.speed is not None else 0)
        
        # --- 2. CTE CALCULATION (Waypoints Priority) ---
        cte = None
        wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)

                
        if hasattr(self, 'isFellow') and self.isFellow:
            act_pos_x = float(actor.position.x)
            act_pos_y = float(actor.position.y)

            ttl = getattr(obj, 'ttl', None)
            if ttl and len(ttl) >= 2:
                cte = distance_to_ttl(act_pos_x, act_pos_y, ttl)

                # ttl distance adjustment
                steer = -0.40 * float(cte)
                steer = max(-1.0, min(1.0, steer))

                TARGET_SPEED = getattr(self, 'maxSpeed', 30.0)
                err = TARGET_SPEED - speed

                if err > 1.0:
                    throttle = min(err * 0.05, 1.0)
                    brake = 0.0
                elif err < -1.0:
                    throttle = 0.0
                    brake = min(-err * 0.05, 1.0)
                else:
                    throttle = 1.0
                    brake = 0.0
                
                take RegulatedControlAction(throttle, steer, past_steer_angle)
                past_steer_angle = angle

                wait
                continue

        
        # If Waypoints exist (Step 2 Requirement), use lookahead logic
        if use_waypoints and wp_list and len(wp_list) >= 2:
            px = float(self.position.x); py = float(self.position.y)
            
            # A. Find nearest waypoint index (Windowed search for efficiency)
            nearest_idx = 0
            best_d2 = 1e18
            start_search = max(0, wp_last_idx - 25)
            end_search = min(len(wp_list), wp_last_idx + 26)
            
            for i in range(start_search, end_search):
                wx, wy = float(wp_list[i][0]), float(wp_list[i][1])
                dx = px - wx; dy = py - wy
                d2 = dx*dx + dy*dy
                if d2 < best_d2:
                    best_d2 = d2; nearest_idx = i
            wp_last_idx = nearest_idx
            
            # B. Find Target Point (Lookahead)
            Ld = float(lookahead)
            rem = Ld
            j = nearest_idx
            found_target = False
            
            # Walk forward along polyline
            while rem > 0.0 and j < len(wp_list) - 1:
                x0, y0 = float(wp_list[j][0]), float(wp_list[j][1])
                x1, y1 = float(wp_list[j+1][0]), float(wp_list[j+1][1])
                seg_dx = x1 - x0; seg_dy = y1 - y0
                seg_len = (seg_dx*seg_dx + seg_dy*seg_dy) ** 0.5
                
                if seg_len <= 1e-6:
                    j += 1; continue
                
                if rem <= seg_len:
                    u = rem / seg_len
                    tgt_x = x0 + u * seg_dx
                    tgt_y = y0 + u * seg_dy
                    
                    # Calculate CTE: Project vehicle pos onto this segment normal
                    # Left Normal: (-dy, dx)
                    nx = -seg_dy / seg_len; ny = seg_dx / seg_len
                    cte = (px - tgt_x)*nx + (py - tgt_y)*ny
                    found_target = True
                    break
                else:
                    rem -= seg_len
                    j += 1
            
            # C. End of track handling
            if not found_target:
                cte = 0.0

        # Fallback: Use TTL Geometry (Step 3 Requirement)
        if cte is None:
            line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else mainRacingRoad)
            if hasattr(line, 'signedDistanceTo'):
                cte = line.signedDistanceTo(self.position)
            else:
                 cte = 0.0

        # --- 3. PID EXECUTION ---
        speed_error = min(self.maxSpeed if hasattr(self, 'maxSpeed') else 200, target_speed) - current_speed
        
        # Raw PID outputs (-1.0 to 1.0)
        throttle_pid = _lon_controller.run_step(speed_error)
        steer_pid = _lat_controller.run_step(cte)

        # --- 4. SIGNAL CONVERSION (NORMALIZED) ---
        # Note: Do NOT scale to 100 or 70 here. actions.py checks for [-1, 1].
        # simulator.py will handle the scaling to hardware units.
        
        # Steer: [-1, 1]
        final_steer = max(-1.0, min(1.0, steer_pid))

        # Throttle/Brake Split [0, 1]
        final_throttle = 0.0
        final_brake = 0.0

        if throttle_pid >= 0:
            final_throttle = max(0.0, min(1.0, throttle_pid))
        else:
            final_brake = max(0.0, min(1.0, abs(throttle_pid)))

        # --- 5. GEAR MANAGEMENT ---
        if manage_gears and hasattr(self, 'setGear'):
            current_gear = getattr(self, 'gear', 1) or 1
            if current_speed is not None:
                if current_gear < 6 and current_speed > gear_up_thresholds[min(current_gear, 5)]:
                    take SetGearAction(current_gear + 1)
                    self.gear = current_gear + 1
                elif current_gear > 1 and current_speed < gear_down_thresholds[min(current_gear - 1, 4)]:
                    take SetGearAction(current_gear - 1)
                    self.gear = current_gear - 1

        # --- 6. EXECUTE ---
        take SetSteerAction(final_steer), SetThrottleAction(final_throttle), SetBrakeAction(final_brake)

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