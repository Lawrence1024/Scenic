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

# --- dSPACE racing model (RacingCar, behaviors, 100 Hz step / 20 Hz control & readback) ---
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)

ego = new RacingCar at (134.131413,125.953041),\
# ego = new RacingCar at (614.659946,-302.782016),\
# ego = new RacingCar at (-110.956171,-151.841778,8.331000),\
# ego = new RacingCar at (55.766137,88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_racing_line_xodr_closed.csv', \
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

# fellow0 = new RacingCar at (616.120555,-297.938762), with regionContainedIn everywhere
# fellow1 = new RacingCar at (617.586835,-293.097982), with regionContainedIn everywhere
# fellow2 = new RacingCar at (618.881724,-288.204668), with regionContainedIn everywhere
# fellow3 = new RacingCar at (619.597641,-283.187822), with regionContainedIn everywhere
# fellow4 = new RacingCar at (619.682079,-278.129827), with regionContainedIn everywhere
# fellow5 = new RacingCar at (618.989290,-273.124863), with regionContainedIn everywhere
# fellow6 = new RacingCar at (617.237850,-268.391681), with regionContainedIn everywhere




