# Phase 12 scenario: corner-entry spawn with fellow on right TTL, slower cruise.
# Paired with F7 (straight spawn, same fellow script) to compare segment-conditioned behavior.
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = False
param scenic_control = True
param fellowHarnessLog = True
param prediction_enabled = False
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (146.6773, -311.8879), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(target_speed=60, manage_gears=True, use_waypoints=True, mpc_config_path=None, prediction_enabled=globalParameters.prediction_enabled)

opponent = new RacingCar with _racing_st_offset ('ahead', 45), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_right_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=45)
