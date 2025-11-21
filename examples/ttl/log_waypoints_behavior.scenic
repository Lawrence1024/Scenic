"""Example: Log Waypoints Behavior

This behavior can be added to any vehicle to log its TTL waypoints during simulation.
Useful for debugging and verifying that waypoints are correctly loaded.

Usage:
    Add this behavior to a vehicle:
    vehicle.behavior = LogWaypointsBehavior(print_interval=10)
    
    This will print waypoint information every N simulation steps.
"""

behavior LogWaypointsBehavior(print_interval=10):
    """Log waypoint information periodically during simulation.
    
    Args:
        print_interval: How many simulation steps between each log message
    """
    step_count = 0
    
    while True:
        step_count += 1
        
        if step_count % print_interval == 0:
            if hasattr(self, 'waypoints') and self.waypoints:
                print(f"[WAYPOINTS] Vehicle {getattr(self, 'raceNumber', 'unknown')} has {len(self.waypoints)} waypoints")
                
                # Print current position and nearest waypoint
                if hasattr(self, 'position') and self.position:
                    current_pos = (self.position.x, self.position.y)
                    
                    # Find nearest waypoint
                    min_dist = float('inf')
                    nearest_idx = 0
                    for i, (wx, wy) in enumerate(self.waypoints):
                        dist = ((current_pos[0] - wx)**2 + (current_pos[1] - wy)**2)**0.5
                        if dist < min_dist:
                            min_dist = dist
                            nearest_idx = i
                    
                    print(f"  Current position: ({current_pos[0]:.2f}, {current_pos[1]:.2f})")
                    print(f"  Nearest waypoint: {nearest_idx}/{len(self.waypoints)} "
                          f"at ({self.waypoints[nearest_idx][0]:.2f}, {self.waypoints[nearest_idx][1]:.2f}), "
                          f"distance: {min_dist:.2f}m")
                    
                    # Print next few waypoints ahead
                    lookahead = min(5, len(self.waypoints) - nearest_idx - 1)
                    if lookahead > 0:
                        print(f"  Next {lookahead} waypoints:")
                        for i in range(1, lookahead + 1):
                            idx = nearest_idx + i
                            if idx < len(self.waypoints):
                                wx, wy = self.waypoints[idx]
                                print(f"    [{idx}] ({wx:.2f}, {wy:.2f})")
            else:
                print(f"[WAYPOINTS] Vehicle {getattr(self, 'raceNumber', 'unknown')} has no waypoints loaded")
        
        wait

