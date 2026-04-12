# Phase 5 targeted bank: corner_entry + Phase 3 SETUP_* + overlap NOT in
# {partial_overlap, side_by_side} -> expect [Phase5Event] reason=entry_conservative.
#
# Ego pose is a ttl_optimal_xodr waypoint whose segment map is "main curve" with
# run progress ~0.07 (corner_entry per situation_assessment.planner_segment_context).
# Derived offline via build_waypoint_segment_map_from_ttl + planner_segment_context.
param map = localPath('../../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = False
param scenic_control = True
param fellowHarnessLog = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (146.6773, -311.8879), \
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

# In-line ahead (~20 m): blocked for Phase 3, typically clear_ahead overlap (not
# side_by_side / partial_overlap), slower car keeps collision_risk in pass band.
opponent = new RacingCar with _racing_st_offset ('ahead', 20), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=52)
