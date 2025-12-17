# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param time_step = 1.0

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Fellow cars placed at fixed coordinates ---
# These coordinates should be validated to be on-road
# Route R2 s=175: Found XODR coordinate with buffer for vehicle bounding box (2.0m x 4.5m)
# This ensures the entire vehicle fits in the drivable region, not just the center point
ego = new RacingCar at (72.567889, 107.574718, 0.0), with behavior FollowRacingLineBehavior()
# Place fellow car at lookahead target (20m ahead) for visualization
# This is where the ego is steering towards, not the nearest waypoint
# Lookahead target calculated from waypoint 3422: (66.428759, 87.330550)
fellow1 = new RacingCar at (66.428759, 87.330550, 0.0)

