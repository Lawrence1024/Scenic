param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = True
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')


fellow1 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('left', 3)
fellow2 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('right', 3)
fellow3 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('behind', 20)
fellow4 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('ahead', 30)



