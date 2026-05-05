# Shared F-bank scenario F9: stationary fellow on the shoulder (roadside "obstacle").
# Fellow is placed ahead on the track but offset ~4.5 m to the **right** (negative t),
# with **zero** commanded speed — a non-threatening obstacle. Ego should bypass on the
# left / optimal without treating it like a closing race car.
#
# SD-11 NOTE (2026-04-27): tactical_planner_enabled and prediction_enabled are now
# True so the SD-11 trajectory-prediction strategy pipeline can run on this scenario.
# Pre-SD-11 the simple non-tactical FollowRacingLineMPC behavior could not actually
# exhibit the F9 deadlock — the deadlock was visible only when the tactical planner
# was active (F9 in the full_stack runner). Now this file mirrors that runtime
# configuration so the deadlock and its SD-11 fix are both reproducible by running
# this .scenic directly.
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

ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    prediction_enabled=globalParameters.prediction_enabled,
    tactical_planner_enabled=True,
    stability_guard_enabled=True,
)

# (delta_s, delta_t): ~32 m ahead along route, ~4.5 m right of centerline (shoulder).
opponent = new RacingCar with _racing_st_offset (32, -4.5), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowConstantSpeedTrackOffsetBehavior(speed_mph=0)
