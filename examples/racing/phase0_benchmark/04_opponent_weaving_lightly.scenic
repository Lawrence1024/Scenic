# Phase benchmark: ego [Phase0]/[Phase2]; optional [FellowHarness] if fellowHarnessLog.
# Fellow (v,d) comes from the *behavior*; ttlFileName/ttlFolder attach route/polyline (not a TTL planner switch).
# Details: examples/racing/fellow_smoke/README.md (TTL files and fellow behaviors).
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = True
param launch_veos_ipc_client = False
param fellowHarnessLog = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903, -112.41203639782893), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(target_speed=60, manage_gears=True, use_waypoints=True, mpc_config_path=None)

opponent = new RacingCar with _racing_st_offset ('ahead', 35), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowSwerveOutOfControlBehavior(
    interval=8,
    swerve_right_s=1.0,
    swerve_left_s=1.0,
    swerve_amp_m=1.5,
    swerve_d_rate_m_s=1.8,
    stop_hold_d=False,
)
