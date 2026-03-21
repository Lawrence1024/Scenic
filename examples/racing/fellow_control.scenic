param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
param fellow_dummy_centerline = False
param launch_veos_ipc_client = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

fellow1 = new RacingCar at (-84.0609596333669,-120.92319628460945), \
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











