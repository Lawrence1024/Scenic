"""Drive constant-d fellows around R1 (Pit) and R2 (Lap) to measure LGS_v1 centerlines.

Spawns 5 fellows on R2 at d in {-4, -2, 0, +2, +4} m and 3 fellows on R1 at d in
{-1, 0, +1} m. Each fellow drives at constant d via FellowFixedDBehavior, which
writes Const_v_Fellows_External / Const_d_Fellows_External directly. Per-step
RD-frame readback is logged to:

    tools/frames/data/lgs_v1_centerline_drive.csv

After the run, post-process with:
    python tools/frames/measure_centerline_from_drive.py

Single command (CoSim VEOS):
    scenic examples/racing/calibration/measure_lgs_v1_centerline.scenic --2d \
        --model scenic.simulators.dspace.racing_model --simulate --count 1 --time 20000

(--time is sim steps; 20000 * 0.01s = 200 sim seconds, ~1 full Lap at 50 mph.
 --count 1 enforces a single sample scene per cosim startup.)
"""
param map = localPath('../../../assets/maps/dSPACE/LGS_v1.xodr')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
param launch_veos_ipc_client = True
param scenic_control = True
param fellowHarnessLog = False
param prediction_enabled = False

model scenic.simulators.dspace.racing_model

import sys as _sys
_sys.path.insert(0, localPath('.'))
from centerline_logger import init_logger, log_step

init_logger('tools/frames/data/lgs_v1_centerline_drive.csv')


# Drives the fellow at constant speed and constant d (lateral offset from centerline).
# Bypasses placement-time t: behavior overrides _fellow_plant_state.d_m every step.
behavior FellowFixedDBehavior(speed_mph, d_setpoint):
    self._fellow_vd_plant_enabled = True
    while True:
        v_kmh = float(speed_mph) * 1.609344
        take SetFellowPlantAction(v_kmh, float(d_setpoint))
        wait


# --- Ego: placeholder; runs MPC at moderate speed to keep the racing stack happy. ---
# Same start as F0_ego_alone.scenic (known-good Lap placement on LGS_v1).
ego = new RacingCar at (-72.78951200758087, -61.6425846392769), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
#ego.behavior = FollowRacingLineMPCBehavior(
#    target_speed=50, manage_gears=True, use_waypoints=True, mpc_config_path=None,
#    prediction_enabled=globalParameters.prediction_enabled,
#)


# --- 5 R2 (Lap) fellows: ego-anchored, staggered by ds, at d in {-4,-2,0,+2,+4}. ---
# raceNumber encodes the d setpoint: 100 + (d * 10) so d=-4 -> 60, d=0 -> 100, d=+4 -> 140.
r2_speed_mph = 50

f_r2_dn4 = new RacingCar with _racing_st_offset (50, 0), \
    with regionContainedIn everywhere, with raceNumber 60, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r2_dn4.behavior = FellowFixedDBehavior(speed_mph=r2_speed_mph, d_setpoint=-4.0)

f_r2_dn2 = new RacingCar with _racing_st_offset (75, 0), \
    with regionContainedIn everywhere, with raceNumber 80, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r2_dn2.behavior = FellowFixedDBehavior(speed_mph=r2_speed_mph, d_setpoint=-2.0)

f_r2_d0 = new RacingCar with _racing_st_offset (100, 0), \
    with regionContainedIn everywhere, with raceNumber 100, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r2_d0.behavior = FellowFixedDBehavior(speed_mph=r2_speed_mph, d_setpoint=0.0)

f_r2_dp2 = new RacingCar with _racing_st_offset (125, 0), \
    with regionContainedIn everywhere, with raceNumber 120, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r2_dp2.behavior = FellowFixedDBehavior(speed_mph=r2_speed_mph, d_setpoint=+2.0)

f_r2_dp4 = new RacingCar with _racing_st_offset (150, 0), \
    with regionContainedIn everywhere, with raceNumber 140, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r2_dp4.behavior = FellowFixedDBehavior(speed_mph=r2_speed_mph, d_setpoint=+4.0)


# --- 3 R1 (Pit) fellows: explicit XODR placement near pit lane (no ego anchor),
# so route detection picks Pit. d in {-1, 0, +1}. raceNumber encodes d as 200+d*10. ---
# Placement points = early pit lane samples from ttl_pitlane.csv (RD frame), translated to
# XODR via RD = XODR + (-6.101, -50.761), i.e. XODR = RD + (6.101, 50.761).
#   line  50: RD ( 136.15,  87.72) -> XODR (142.25, 138.48)
#   line 150: RD (  81.55,  88.87) -> XODR ( 87.65, 139.63)
#   line 300: RD (  -7.05, -32.17) -> XODR ( -0.95,  17.04)
r1_speed_mph = 25

f_r1_dn1 = new RacingCar at (142.25, 138.48), \
    with regionContainedIn everywhere, with raceNumber 190, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r1_dn1.behavior = FellowFixedDBehavior(speed_mph=r1_speed_mph, d_setpoint=-1.0)

f_r1_d0 = new RacingCar at (87.65, 139.63), \
    with regionContainedIn everywhere, with raceNumber 200, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r1_d0.behavior = FellowFixedDBehavior(speed_mph=r1_speed_mph, d_setpoint=0.0)

f_r1_dp1 = new RacingCar at (-0.95, 17.04), \
    with regionContainedIn everywhere, with raceNumber 210, \
    with ttlFolder localPath('../../../assets/ttls/LS_ENU_TTL_CSV')
f_r1_dp1.behavior = FellowFixedDBehavior(speed_mph=r1_speed_mph, d_setpoint=+1.0)


# --- Per-step CSV logger ---
monitor CenterlineDriveLogger():
    while True:
        log_step(simulation())
        wait

require monitor CenterlineDriveLogger()

terminate after 200 seconds
