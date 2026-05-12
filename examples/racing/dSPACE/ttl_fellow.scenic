param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = True
param launch_veos_ipc_client = True
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

fellow0 = new RacingCar at (-78.86454576530903,-112.41203639782893), 
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
fellow0.behavior = FellowFollowTTLGeometricBehavior(speed_mph=150)













