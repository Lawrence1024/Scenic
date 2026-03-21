param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
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
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,      # 60 m/s (~216 km/h) nominal; capped at 140 mph (62.58 m/s) by MAX_SPEED_LIMIT_MS
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    mpc_config_path=None  # Use default MPC config (src/scenic/domains/racing/mpc/vehicle_mpc.yaml)
)













