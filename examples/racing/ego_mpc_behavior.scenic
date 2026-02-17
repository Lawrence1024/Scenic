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

# ego = new RacingCar at (614.659946,-302.782016),\
ego = new RacingCar at (-110.956171,-151.841778,8.331000),\
# ego = new RacingCar at (55.766137,88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_17.csv', \
    # with ttlFileName 'laguna_main_racing_line', \
    # with ttlFileName 'laguna_raceable_xy.csv', \
    with ttlFileName 'ttl_fellow_test_xodr_all.csv', \
    # with ttlFileName 'ttl_racing_line_xodr.csv', \
    # with ttlFileName 'temp_aligned_to_centerline.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV/transformed'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,      # 60 m/s (~216 km/h) nominal; capped at 140 mph (62.58 m/s) by MAX_SPEED_LIMIT_MS
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    lookahead=20.0,       # 20m lookahead distance
    mpc_config_path=None  # Use default MPC config (src/scenic/domains/racing/mpc/vehicle_mpc.yaml)
)

# Using main racing road centerline TTL (3541 waypoints, ~4.2km total length)
# Waypoints are in XODR coordinate system and guaranteed to be on-road
# TTL file: ttl_fellow_test_xodr_all.csv (XODR coordinates, transformed from dSPACE)
# Fellow vehicles placed every 100m starting from 200m (s = 200, 300, 400, ..., 3100)

# fellow0 = new RacingCar at (616.120555,-297.938762), with regionContainedIn everywhere
# fellow1 = new RacingCar at (617.586835,-293.097982), with regionContainedIn everywhere
# fellow2 = new RacingCar at (618.881724,-288.204668), with regionContainedIn everywhere
# fellow3 = new RacingCar at (619.597641,-283.187822), with regionContainedIn everywhere
# fellow4 = new RacingCar at (619.682079,-278.129827), with regionContainedIn everywhere
# fellow5 = new RacingCar at (618.989290,-273.124863), with regionContainedIn everywhere
# fellow6 = new RacingCar at (617.237850,-268.391681), with regionContainedIn everywhere




