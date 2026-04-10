# Phase benchmark: ego [Phase0]/[Phase2]; optional [FellowHarness] if fellowHarnessLog.
# Fellow (v,d) comes from the *behavior*; ttlFileName/ttlFolder attach route/polyline (not a TTL planner switch).
# Details: examples/racing/fellow_smoke/README.md (TTL files and fellow behaviors).
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
    with ttlFileName 'ttl_left_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

# Scripted planner handoff test: left -> right at t=10s.
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    planner_enabled=True,
    ttl_schedule='10:right',
    target_speed_cap=60,
)
