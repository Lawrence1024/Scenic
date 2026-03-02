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
# Fellow placed ahead of ego for visualization
fellow1 = new RacingCar at (55.7661373818, 88.2693869080), with regionContainedIn everywhere

