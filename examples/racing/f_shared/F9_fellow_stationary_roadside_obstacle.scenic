# Shared F-bank scenario F9: stationary fellow on the shoulder (roadside "obstacle").
# Fellow is placed ahead on the track but offset ~4.5 m to the **right** (negative t),
# with **zero** commanded speed — a non-threatening obstacle. Ego should bypass on the
# left / optimal without treating it like a closing race car.
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = False
param scenic_control = True
param fellowHarnessLog = True
param phase7_prediction_enabled = False
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903, -112.41203639782893), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(target_speed=60, manage_gears=True, use_waypoints=True, mpc_config_path=None, phase6_orchestration_enabled=True, phase7_prediction_enabled=globalParameters.phase7_prediction_enabled)

# (delta_s, delta_t): ~32 m ahead along route, ~4.5 m right of centerline (shoulder).
opponent = new RacingCar with _racing_st_offset (32, -4.5), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowConstantSpeedTrackOffsetBehavior(speed_mph=0)
