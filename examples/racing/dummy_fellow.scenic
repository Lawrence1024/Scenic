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
# Light-step mode: disable COM read/write to test step_time only (vehicle will not move). Set True to test; False for full analytics (COM on).
param light_step = False
# Optional: describe this run for analysis (logged as [RacingRun] edit_note=... and stored in result_data)
# param edit_note = 'baseline'  # e.g. 'curvature cap 0.08', 'TTL v2'
# Temporary: use centerline TTLs for segment map instead of OpenDRIVE (when the map is flawed)
param use_ttl_segments = True
# Dummy fellows: follow centerline at constant speed (Velocity and Lateral deviation set to Extern in dSPACE)
param manual_control = False

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
ego = new RacingCar at (60.3811498,103.7450019), \
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
# ego.behavior = FollowRacingLineMPCBehavior(
#     target_speed=60,      # 60 m/s (~216 km/h) nominal; capped at 140 mph (62.58 m/s) by MAX_SPEED_LIMIT_MS
#     manage_gears=True,    # Auto gear shifting
#     use_waypoints=True,   # Use waypoint-based control
#     mpc_config_path=None  # Use default MPC config (src/scenic/domains/racing/mpc/vehicle_mpc.yaml)
# )


ego.behavior = ARTStackControlBehavior()

# Dummy fellow: no behavior; driven by External_Signals (Velocity/Lateral deviation = Extern in dSPACE)
param fellow_dummy_centerline = True
param fellow_dummy_velocity_kmh = 50
fellow0 = new RacingCar at (49.3895,87.9318), with regionContainedIn everywhere, with raceNumber 2













