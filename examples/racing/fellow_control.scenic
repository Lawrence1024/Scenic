param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = True
param fellow_dummy_centerline = False
model scenic.simulators.dspace.racing_model

ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

fellow1 = new RacingCar at (-73.49710545241894,-103.91194317950539), \
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











