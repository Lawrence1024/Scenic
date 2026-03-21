param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
param launch_veos_ipc_client = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903,-112.41203639782893), \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = ARTStackControlBehavior()

fellow1 = new RacingCar with _racing_st_offset ('ahead', 20), \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

fellow1.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,     
    manage_gears=True,  
    use_waypoints=True,
)











