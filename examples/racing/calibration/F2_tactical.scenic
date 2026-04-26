"""F2 variant with tactical_planner_enabled=True for RC-5/RC-6 validation.

Same setup as examples/racing/f_shared/F2_fellow_ahead_optimal_slower.scenic
(slower fellow 35 m ahead on optimal line, ego target 60 m/s) but with the
tactical planner ON. This exercises:
  - tactical state machine (FREE_RUN -> FOLLOW -> SETUP_PASS_* -> COMMIT_PASS_*)
  - RC-5 commit_abort_enabled=True default (planner can now enter COMMIT mode)
  - RC-5 assessment_enabled=True default (planner gets gap_ok / corridor_open inputs)
  - RC-5 fixed gate at behaviors.scenic:901 (assessment block runs when tactical is on)
  - RC-6 stability_guard_handle_ttl_switch planner_state bypass (only fires if guard is on)

Run:
    scenic examples/racing/calibration/F2_tactical.scenic --2d \
        --model scenic.simulators.dspace.racing_model --simulate --count 1 --time 12000

What to look for in the log:
  - [Phase3Tactical] lines: planner mode transitions
  - [Commit] / COMMIT_PASS_LEFT / COMMIT_PASS_RIGHT in any log line: commit_abort actually firing
  - [Assessment] lines: per-tick (RC-5)
  - [CtrlTrace] ttl=...: switching from 'optimal' to 'left'/'right' during pass attempt
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
param scenic_control = True
param fellowHarnessLog = True
model scenic.simulators.dspace.racing_model

ego = new RacingCar at (-72.78951200758087, -61.6425846392769), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

# Tactical planner ON; prediction/assessment/commit_abort fall through to RC-5 defaults (all True).
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=60,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    tactical_planner_enabled=True,
)

opponent = new RacingCar with _racing_st_offset ('ahead', 35), \
    with regionContainedIn everywhere, \
    with raceNumber 2, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')

opponent.behavior = FellowFollowTTLGeometricBehavior(speed_mph=20)
