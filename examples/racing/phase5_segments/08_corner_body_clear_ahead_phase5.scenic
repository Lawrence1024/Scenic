# Phase 5 targeted bank: corner_body + Phase 3 SETUP_* -> expect [Phase5Event]
# reason=body_no_new_setup (force FOLLOW on optimal).
#
# Ego pose is a ttl_optimal_xodr waypoint on "main curve" with run progress ~0.49
# (corner_body). Derived offline via build_waypoint_segment_map_from_ttl +
# planner_segment_context (same pipeline as runtime segment_map).
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
param scenic_control = True
param fellowHarnessLog = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (167.6397, -328.6813), \
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

opponent = new RacingCar with _racing_st_offset ('ahead', 22), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=52)
