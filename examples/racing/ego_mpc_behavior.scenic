# Example: Ego vehicle using MPC controller for racing line following
#
# This example demonstrates how to use the MPC-based behavior for improved
# racing performance compared to PID controllers.

# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
# 100 Hz simulation step, 20 Hz control and readback (0.05 s period)
param time_step = 0.01
param control_period = 0.05
# Temporary: use centerline TTLs for segment map instead of OpenDRIVE (when the map is flawed)
param use_ttl_segments = True

# --- dSPACE racing model (RacingCar, behaviors, 100 Hz step / 20 Hz control & readback) ---
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)
# Pitlane Start
# ego = new RacingCar at (79.766382000,97.055717000), \
# Pitlane End
# ego = new RacingCar at (196.952588000,9.974341000), \
# Main End
# ego = new RacingCar at (134.131413,125.953041),\  
# Weird Curve
# ego = new RacingCar at (614.659946,-302.782016),\  
# Werid Curve
# ego = new RacingCar at (-110.956171,-151.841778,8.331000),\ 
# Main Start
ego = new RacingCar at (55.766137,88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_main_road.csv', \
    # with ttlFileName 'ttl_pitlane.csv', \
    #with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFileName 'ttl_right_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,      # 60 m/s (~216 km/h) nominal; capped at 140 mph (62.58 m/s) by MAX_SPEED_LIMIT_MS
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    mpc_config_path=None  # Use default MPC config (src/scenic/domains/racing/mpc/vehicle_mpc.yaml)
)

# fellow0 = new RacingCar at (178.553226000,43.855936000), with regionContainedIn everywhere
# fellow1 = new RacingCar at (174.210186000,51.888402000), with regionContainedIn everywhere
# fellow2 = new RacingCar at (161.870429800,56.357831800), with regionContainedIn everywhere
# fellow3 = new RacingCar at (161.393517000,53.432218000), with regionContainedIn everywhere
# fellow4 = new RacingCar at (155.526641000,61.715180000), with regionContainedIn everywhere
# fellow5 = new RacingCar at (150.865108000,68.365371000), with regionContainedIn everywhere
# fellow6 = new RacingCar at (146.798926000,74.193057000), with regionContainedIn everywhere













