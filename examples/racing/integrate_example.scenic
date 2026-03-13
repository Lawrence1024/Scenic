param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
model scenic.simulators.dspace.racing_model


ego = new RacingCar at (55.766137,88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_main_road.csv', \
    # with ttlFileName 'ttl_pitlane.csv', \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

# fellow0 = new RacingCar at (178.553226000,43.855936000), with regionContainedIn everywhere













