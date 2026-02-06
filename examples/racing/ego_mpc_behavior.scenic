# Example: Ego vehicle using MPC controller for racing line following
#
# This example demonstrates how to use the MPC-based behavior for improved
# racing performance compared to PID controllers.

# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.05 
param batch_steps = 1   

# --- Driving world model (brings in Car/road/behaviors) ---
model scenic.domains.driving.model

# --- Racing domain model (brings in RacingCar/racing behaviors) ---
model scenic.domains.racing.model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)

ego = new RacingCar at (55.766137,88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_17.csv', \
    with ttlFileName 'ttl_fellow_test_xodr_all.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV/transformed'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,      # 30 m/s (~108 km/h)
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    lookahead=20.0,       # 20m lookahead distance
    mpc_config_path=None  # Use default MPC config (src/scenic/domains/racing/mpc/vehicle_mpc.yaml)
)

# Using main racing road centerline TTL (3541 waypoints, ~4.2km total length)
# Waypoints are in XODR coordinate system and guaranteed to be on-road
# TTL file: ttl_fellow_test_xodr_all.csv (XODR coordinates, transformed from dSPACE)
# Fellow vehicles placed every 100m starting from 200m (s = 200, 300, 400, ..., 3100)

fellow0 = new RacingCar at (-130.271358,-544.854676), with regionContainedIn everywhere
fellow1 = new RacingCar at (-125.230382,-544.238104), with regionContainedIn everywhere
fellow2 = new RacingCar at (-120.276175,-543.141248), with regionContainedIn everywhere
fellow3 = new RacingCar at (-115.465428,-541.552996), with regionContainedIn everywhere
fellow4 = new RacingCar at (-110.860394,-539.441584), with regionContainedIn everywhere
fellow5 = new RacingCar at (-106.570478,-536.768906), with regionContainedIn everywhere
fellow6 = new RacingCar at (-102.738364,-533.488527), with regionContainedIn everywhere