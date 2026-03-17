param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = True
param fellow_dummy_centerline = True
param fellow_dummy_velocity_kmh = 50
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

fellow0 = new RacingCar at (49.3895,87.9318), with regionContainedIn everywhere, with raceNumber 2













