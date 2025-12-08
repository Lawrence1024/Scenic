
behavior RecoverToRacingLine():
    wp_last_idx = 0  # CRITICAL: Track last known waypoint index

    while True:
        # Check if car is out of bounds
        is_in_bounds = road.contains(self.position) if hasattr(road, 'contains') else True

        if not is_in_bounds:
            # Get waypoints
            wp_list = (self.waypoints if hasattr(self, 'waypoints') else None)

            if wp_list and len(wp_list) > 0:
                px = float(self.position.x)
                py = float(self.position.y)
                heading = float(self.heading) if hasattr(self, 'heading') else None

                # METHOD 1: Simple forward-only (RECOMMENDED)
                from tools.get_map_bounds import find_forward_waypoint
                result = find_forward_waypoint(
                    (px, py),
                    wp_list,
                    last_known_index=wp_last_idx,  # REQUIRED
                    car_heading=heading,           # Optional but recommended
                    max_search_distance=100.0
                )

                # METHOD 2: Full control (alternative)
                # from tools.get_map_bounds import find_best_racing_waypoint
                # result = find_best_racing_waypoint(
                #     (px, py), heading, wp_list,
                #     last_known_index=wp_last_idx,
                #     forward_only=True,  # CRITICAL: Prevents backtracking
                #     forward_bias=0.9,
                #     max_search_distance=100.0
                # )

                if result:
                    target_wp = result['waypoint']
                    wp_last_idx = result['index']  # Update last known index

                    # Navigate towards target_wp
                    # This waypoint is GUARANTEED to be forward
                    # ... your steering/control logic here ...
                else:
                    # No forward waypoint found - may need to increase search distance
                    # or handle edge case (e.g., end of track)
                    pass