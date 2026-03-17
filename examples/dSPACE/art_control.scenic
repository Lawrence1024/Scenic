param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = ARTStackControlBehavior()












