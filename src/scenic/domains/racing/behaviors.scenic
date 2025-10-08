"""Racing-specific behaviors for dynamic agents.

These behaviors extend the driving domain behaviors with racing-specific
strategies and maneuvers.
"""

from scenic.domains.driving.behaviors import *
import scenic.domains.racing.model as _racing

behavior FollowRacingLineBehavior(target_speed=30, aggressive=False):
    """Follow the optimal racing line at target speed.
    
    This behavior attempts to keep the car on the racing line while maintaining
    the target speed. More aggressive settings will push closer to track limits.
    
    Args:
        target_speed: Target speed in m/s (default: 30 m/s = 108 km/h)
        aggressive: If True, use more of the track width and higher speeds
    """
    # Get the racing line from the track
    racing_line = _racing.track.racingLine
    
    # Use the underlying lane following with racing-specific parameters
    if aggressive:
        actual_speed = target_speed * 1.1  # 10% faster when aggressive
    else:
        actual_speed = target_speed
    
    # Follow the racing line
    do FollowLaneBehavior(target_speed=actual_speed)

behavior OvertakingBehavior(target_car, target_speed=35):
    """Attempt to overtake another car.
    
    This behavior will try to pass the target car by:
    1. Closing the gap
    2. Moving to the side (using more track width)
    3. Accelerating past
    4. Returning to racing line
    
    Args:
        target_car: The car to overtake
        target_speed: Speed to use during overtake (m/s)
    """
    # Phase 1: Close the gap
    while (distance from self to target_car) > 5:
        # Accelerate towards the car
        do FollowLaneBehavior(target_speed=target_speed * 0.9)
    
    # Phase 2: Pull alongside (simplified - real implementation would check track width)
    while (distance from self to target_car) > 0.5:
        take SetThrottleAction(0.9), SetBrakeAction(0.0)
    
    # Phase 3: Complete the pass
    while self.position.x < target_car.position.x:  # Simplified position check
        take SetThrottleAction(1.0), SetBrakeAction(0.0)
    
    # Phase 4: Return to racing line
    do FollowRacingLineBehavior(target_speed=target_speed)

behavior DefensiveDrivingBehavior(target_speed=28):
    """Drive defensively, protecting position from overtaking attempts.
    
    This behavior will:
    - Monitor cars behind
    - Adjust line to defend position
    - Maintain consistent speed
    
    Args:
        target_speed: Target speed in m/s
    """
    while True:
        # Check for cars behind trying to overtake
        closest_behind = None
        min_distance = float('inf')
        
        for obj in simulation().objects:
            if obj is self or not isinstance(obj, _racing.RacingCar):
                continue
            
            # Check if car is behind
            track_distance_self = _racing.track.distanceAlongTrack(self.position)
            track_distance_obj = _racing.track.distanceAlongTrack(obj.position)
            
            if track_distance_obj is not None and track_distance_self is not None:
                distance_diff = track_distance_self - track_distance_obj
                
                # If car is behind and close
                if 0 < distance_diff < 20 and distance_diff < min_distance:
                    min_distance = distance_diff
                    closest_behind = obj
        
        # If car is close behind, defend position
        if closest_behind and min_distance < 10:
            # Hold the racing line firmly
            take SetThrottleAction(0.7), SetBrakeAction(0.0)
        else:
            # Normal racing
            do FollowRacingLineBehavior(target_speed=target_speed)

behavior PitStopBehavior(duration=25):
    """Execute a pit stop.
    
    This behavior will:
    1. Enter the pit lane
    2. Navigate to pit box
    3. Stop for service
    4. Exit pit lane
    
    Args:
        duration: Duration of pit stop in seconds (default: 25s for tire change)
    """
    # Check if track has a pit lane
    if _racing.track.pitLane is None:
        print("Warning: No pit lane on this track, cannot execute pit stop")
        return
    
    pit_lane = _racing.track.pitLane
    
    # Phase 1: Enter pit lane
    entry_point = pit_lane.entryPoint
    if entry_point:
        # Navigate towards pit entry
        while (distance from self to entry_point) > 5:
            take SetThrottleAction(0.5), SetBrakeAction(0.0)
    
    # Phase 2: Reduce speed to pit lane limit
    while self.speed > pit_lane.speedLimit:
        take SetThrottleAction(0.0), SetBrakeAction(0.3)
    
    # Phase 3: Navigate pit lane at speed limit
    while not pit_lane.isPitBox(self.position):
        take SetThrottleAction(0.3), SetBrakeAction(0.0)
    
    # Phase 4: Stop in pit box
    while self.speed > 0.1:
        take SetThrottleAction(0.0), SetBrakeAction(1.0)
    
    # Phase 5: Service (stationary)
    wait duration
    
    # Reset tire wear and fuel (game-like simulation)
    self.tireWear = 0.0
    self.fuelLevel = 1.0
    
    # Phase 6: Exit pit lane
    exit_point = pit_lane.exitPoint
    if exit_point:
        # Accelerate out of pit box
        while (distance from self to exit_point) > 5:
            take SetThrottleAction(0.7), SetBrakeAction(0.0)
    
    # Phase 7: Rejoin track
    do FollowRacingLineBehavior()

behavior QualifyingLapBehavior(target_speed=35):
    """Execute a fast qualifying lap.
    
    In qualifying, the goal is to achieve the fastest single lap time.
    This behavior will:
    - Push to maximum speed
    - Use optimal racing line
    - Take more risks than race pace
    
    Args:
        target_speed: Target speed in m/s (higher than race pace)
    """
    # Push hard for one lap
    do FollowRacingLineBehavior(target_speed=target_speed, aggressive=True)

behavior FormationLapBehavior(leader=None, spacing=8.0):
    """Follow the car ahead during a formation lap before race start.
    
    Formation laps are slow laps where cars maintain grid order and spacing
    before the actual race starts.
    
    Args:
        leader: The car to follow (typically the car ahead on grid)
        spacing: Distance to maintain from leader in meters
    """
    FORMATION_SPEED = 15  # ~54 km/h
    
    if leader is None:
        # If no leader, just circulate at formation speed
        do FollowRacingLineBehavior(target_speed=FORMATION_SPEED)
    else:
        # Follow the leader at safe distance
        while True:
            current_distance = distance from self to leader
            
            if current_distance > spacing + 2:
                # Too far, speed up
                take SetThrottleAction(0.6), SetBrakeAction(0.0)
            elif current_distance < spacing - 2:
                # Too close, slow down
                take SetThrottleAction(0.2), SetBrakeAction(0.3)
            else:
                # Just right, maintain speed
                take SetThrottleAction(0.4), SetBrakeAction(0.0)

behavior RaceStartBehavior(reaction_time=0.3):
    """Handle the race start from stationary position.
    
    This behavior manages:
    1. Waiting for start signal
    2. Launch (with reaction time)
    3. Acceleration to racing speed
    4. Avoiding first-corner incidents
    
    Args:
        reaction_time: Time delay for reacting to start signal (seconds)
    """
    # Phase 1: Stationary on grid
    take SetThrottleAction(0.0), SetBrakeAction(1.0), SetHandBrakeAction(True)
    
    # Phase 2: Wait for start signal (simplified - would need actual signal)
    # In real implementation, this would wait for a signal from the simulation
    wait reaction_time
    
    # Phase 3: Launch!
    take SetHandBrakeAction(False)
    
    # Phase 4: Full acceleration
    for i in range(50):  # Accelerate for 5 seconds
        take SetThrottleAction(1.0), SetBrakeAction(0.0)
        wait 0.1
    
    # Phase 5: Transition to normal racing
    do FollowRacingLineBehavior(target_speed=30)

behavior ConserveFuelBehavior(target_speed=25, fuel_save_amount=0.15):
    """Drive conservatively to save fuel.
    
    Used in endurance racing or when fuel is low. Reduces speed and
    acceleration to extend fuel range.
    
    Args:
        target_speed: Reduced target speed in m/s
        fuel_save_amount: Amount to reduce throttle (0.0-1.0)
    """
    while self.fuelLevel > 0.05:  # Continue until nearly empty
        # Drive at reduced speed
        if self.speed < target_speed:
            # Gentle acceleration
            throttle = 0.5 * (1.0 - fuel_save_amount)
            take SetThrottleAction(throttle), SetBrakeAction(0.0)
        elif self.speed > target_speed + 2:
            # Gentle braking
            take SetThrottleAction(0.0), SetBrakeAction(0.2)
        else:
            # Coast when possible
            take SetThrottleAction(0.3 * (1.0 - fuel_save_amount)), SetBrakeAction(0.0)
        
        # Simulate fuel consumption (simplified)
        self.fuelLevel -= 0.0001 * self.speed / 30.0  # Proportional to speed
        
        wait 0.1
    
    # When fuel is critically low, pit stop
    do PitStopBehavior()

behavior TrafficManagementBehavior(target_speed=30):
    """Manage traffic during a race - decide when to overtake or defend.
    
    This behavior analyzes the race situation and chooses appropriate actions:
    - If car ahead is slower: attempt overtake
    - If car behind is faster: defend position  
    - Otherwise: follow racing line
    
    Args:
        target_speed: Target racing speed in m/s
    """
    while True:
        # Find nearest car ahead
        nearest_ahead = None
        min_ahead_distance = float('inf')
        
        # Find nearest car behind
        nearest_behind = None
        min_behind_distance = float('inf')
        
        for obj in simulation().objects:
            if obj is self or not isinstance(obj, _racing.RacingCar):
                continue
            
            track_distance_self = _racing.track.distanceAlongTrack(self.position)
            track_distance_obj = _racing.track.distanceAlongTrack(obj.position)
            
            if track_distance_obj is None or track_distance_self is None:
                continue
            
            distance_diff = track_distance_obj - track_distance_self
            
            # Car ahead
            if 0 < distance_diff < 50 and distance_diff < min_ahead_distance:
                min_ahead_distance = distance_diff
                nearest_ahead = obj
            
            # Car behind
            if -50 < distance_diff < 0 and abs(distance_diff) < min_behind_distance:
                min_behind_distance = abs(distance_diff)
                nearest_behind = obj
        
        # Decision making
        if nearest_ahead and min_ahead_distance < 15:
            # Car close ahead - try to overtake
            try:
                do OvertakingBehavior(nearest_ahead, target_speed=target_speed * 1.1) for 10 seconds
            interrupt when True:
                pass  # Return to main loop
        elif nearest_behind and min_behind_distance < 10:
            # Car close behind - defend position
            try:
                do DefensiveDrivingBehavior(target_speed=target_speed) for 5 seconds
            interrupt when True:
                pass
        else:
            # Clear track - normal racing
            do FollowRacingLineBehavior(target_speed=target_speed)

