# Shared F-bank scenario F5: fellow ahead, right-left swerve then stop.
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
param scenic_control = True
param fellowHarnessLog = True
param prediction_enabled = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-72.78951200758087, -61.6425846392769), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = FollowRacingLineMPCBehavior(target_speed=60, manage_gears=True, use_waypoints=True, mpc_config_path=None, prediction_enabled=globalParameters.prediction_enabled, tactical_planner_enabled=True, stability_guard_enabled=True)

opponent = new RacingCar with _racing_st_offset ('ahead', 35), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowSwerveOutOfControlBehavior(
    speed_mph=60,
    interval=8.0,
    swerve_right_s=1.6,
    swerve_left_s=2.0,
    swerve_amp_m=5.0,
    swerve_d_rate_m_s=5.5,
)
