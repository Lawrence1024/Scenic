param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = False
param scenic_control = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903, -112.41203639782893), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(target_speed=60, manage_gears=True, use_waypoints=True, mpc_config_path=None, tactical_planner_enabled=True)

opponent = new RacingCar with _racing_st_offset ('ahead', 40), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowConstantSpeedTrackOffsetBehavior(speed_mph=60)
