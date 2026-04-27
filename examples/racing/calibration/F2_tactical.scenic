"""F2_tactical: slower fellow 35 m ahead on optimal line, ego target 60 m/s,
tactical_planner_enabled=True.

EXPECTED BEHAVIOR (post SD-9, 2026-04-27):
  This is a "NO SAFE PASS AVAILABLE" scenario by design. The right TTL on
  this section of LGS provides only ~2.08 m centerline-to-centerline from
  optimal — below the SD-9 racing safety threshold (2.5 m centerline =
  ~0.5 m body buffer). Left TTL similar. Therefore:

    - SETUP_PASS_* should NOT fire (pass_window_check correctly rejects).
    - Ego stays in FOLLOW behind the slower fellow indefinitely.
    - decision_reason expected: pass_window_unsafe_both_sides (or similar).
    - collision should be False, off_track False, lap_time longer than F0
      (because ego is rate-limited to fellow's speed + follow_margin = 11.5 m/s).

  This is the CORRECT racing behavior: when the geometry doesn't permit
  a safe pass, the right move is to wait. F2 was previously the canonical
  "ego must overtake" target but the geometry doesn't actually support it
  at this starting position.

  For test cases that DO exercise a successful overtake, see:
    F3L_fellow_ahead_left_cruise.scenic   — fellow on LEFT TTL, ego passes RIGHT
    F3R_fellow_ahead_right_cruise.scenic  — fellow on RIGHT TTL, ego passes LEFT
  Both have ~5 m lateral separation between side TTLs (well above 2.5 m).

State machine being exercised (mostly the FOLLOW path now):
  - tactical state machine (FREE_RUN <-> FOLLOW)
  - SD-3a/c pass_window_check correctly rejecting both sides
  - SD-4 brake-trigger gating (predicted_collision should be 0 throughout)
  - SD-9 raised lateral safety threshold

Run:
    scenic examples/racing/calibration/F2_tactical.scenic --2d \\
        --model scenic.simulators.dspace.racing_model --simulate --count 1 --time 3000

What to look for in the log:
  - [Tactical] lines: should stay in FOLLOW most of the time
  - [PathPredict] lines: predicted_collision=0, min_clear ~2.0 m on both sides
  - decision_reason=pass_window_unsafe_both_sides (or similar refusal)
  - collision=False, contact_recovery_hold=0
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
