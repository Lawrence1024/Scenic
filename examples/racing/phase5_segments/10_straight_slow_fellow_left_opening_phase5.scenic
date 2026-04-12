# Phase 5: explicit straight bypass — fellow crawls on the LEFT TTL line; ego on optimal has a clear right opening.
# Fellow ~20 mph vs ego 60 mph target → large closing speed on typical main-straight segments early in the lap.
# Mirror of 09_straight_slow_fellow_right_opening_phase5 (pass left vs pass right); expect commit_pass_right / SETUP_RIGHT.
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = False
param scenic_control = True
param fellowHarnessLog = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903, -112.41203639782893), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    tactical_planner_enabled=True,
    pass_commit_shield_enabled=True,
    phase5_segment_tactics_enabled=True,
)

opponent = new RacingCar with _racing_st_offset ('ahead', 45), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=20)
