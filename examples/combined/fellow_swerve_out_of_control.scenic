param map = localPath('../../assets/maps/dSPACE/LGS_v1.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
param launch_veos_ipc_client = True
param record_ros2_bag = False
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-78.86454576530903,-112.41203639782893), \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = ARTStackControlBehavior()

fellow0 = new RacingCar with _racing_st_offset ('behind', 30), \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')
fellow0.behavior = FellowSwerveOutOfControlBehavior(
    interval=10,
    swerve_right_s=1.8,
    swerve_left_s=2.0,
    swerve_amp_m=6.0,
    swerve_d_rate_m_s=6.5,
    stop_hold_d=True,
)
