# Example: Ego vehicle using MPC controller for racing line following
#
# This example demonstrates how to use the MPC-based behavior for improved
# racing performance compared to PID controllers.

# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 1  # 20 Hz control frequency

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Racing domain model (brings in RacingCar/racing behaviors) ---
model scenic.domains.racing.model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)
ego = new RacingCar at (72.567889, 107.574718, 0.0), \
    with raceNumber 1, \
    with ttlFileName 'ttl_fellow_test_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV/transformed'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,      # 30 m/s (~108 km/h)
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    lookahead=20.0,       # 20m lookahead distance
    mpc_config_path=None  # Use default MPC config (debug_mpc/vehicle_mpc.yaml)
)

# Using main racing road centerline TTL (3541 waypoints, ~4.2km total length)
# Waypoints are in XODR coordinate system and guaranteed to be on-road
# TTL file: ttl_fellow_test_xodr.csv (XODR coordinates, transformed from dSPACE)

fellow1 = new RacingCar at (66.428759, 87.330550, 0.0)