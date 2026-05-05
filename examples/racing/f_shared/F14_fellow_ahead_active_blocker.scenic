# Shared F-bank scenario F14: fellow ahead, actively blocks ego from overtaking.
#
# Negative passing test. Fellow has full visibility of ego state and uses it
# adversarially: each control tick the blocker mirrors ego's lateral position
# on its TTL (rate-limited slew, no teleporting) and matches ego speed minus a
# small offset, keeping the gap collapsing toward the blocker. Surfaces ego-
# controller weaknesses against a defender that actively cuts off both pass
# sides.
#
# Setup discipline (so the blocker is actually exercised, not bypassed):
#   - Ego runs with the tactical planner ON (prediction + strategy + safety
#     guard). Without this, ego is in pure FREE_RUN MPC and barrels straight
#     into anything ahead -- the blocker never gets to defend.
#   - Blocker placed 3 m to the RIGHT of the optimal centerline (t = -3),
#     leaving a clear pass-left visual for ego. When ego commits left, the
#     blocker mirrors left to cut off the pass -- that's the test.
#   - min_speed_mph = 20 so the speed-matching loop can actually keep the
#     blocker slower than ego at low speeds. Earlier (40 mph) the blocker
#     accelerated past ego at startup and the gap grew before collapsing.
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
    # SD-39rev3: lowered from 60 m/s to 20 m/s for F14 specifically. With the
    # active blocker, ego doesn't need racing-line speed -- the blocker is at
    # ~12 m/s. Ego target=60 made the racing-line "pull" so dominant that
    # the slew limiter on effective_target_speed (behaviors.scenic:1711, ~15
    # m/s² when tactical cap is binding) couldn't drag the MPC's tracked
    # target down to the planner cap fast enough during commits. Result:
    # ~7 m/s slew-induced overshoot, ego at ~22 m/s into a 12 m/s blocker.
    # With target=20, the slew has only 6 m/s to traverse (20 -> 14 cap),
    # which converges in ~0.4 s; overshoot drops to ~3 m/s.
    #
    # NOTE on road grade: the longitudinal MPC already does gravity-aware
    # throttle/brake conversion (mpc_longitudinal.py:399-434) -- on downhill
    # it asks for more brake to fight the gravity-assisted acceleration; on
    # uphill it asks for less brake (gravity helps slow ego). So the "ramp
    # we're on" is accounted for at the actuator level. The remaining
    # overshoot we observed is from the target-speed slew limiter, not from
    # missing grade compensation.
    target_speed=20,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    prediction_enabled=globalParameters.prediction_enabled,
    tactical_planner_enabled=True,
    # SD-36: enable the stability guard. Without this, the guard is dead code
    # and EMERGENCY_STABLE never activates -- predicted_collision goes nowhere.
    # F14 is a deliberate negative-passing test, so we need the safety layer
    # actively defending. This is a per-scenario opt-in; existing F-bank scenarios
    # remain unchanged until they explicitly want the guard.
    stability_guard_enabled=True,
)

opponent = new RacingCar at (0, 0), \
    with regionContainedIn everywhere, \
    with _racing_st_offset (30, -3), \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowActiveBlockBehavior(
    speed_offset_mph=-5.0,
    min_speed_mph=20.0,
    max_speed_mph=70.0,
    max_lat_speed_mps=3.0,
    max_lat_offset_m=5.0,
    deadband_m=0.4,
)
