# Example: Empirical acceleration/deceleration calibration
#
# Run full throttle for x seconds, then full brake for y seconds, and log
# measured acceleration/deceleration. Use the printed values to set
# max_acceleration, max_deceleration, or brake_decel_scale in vehicle_mpc.yaml
# so the MPC matches the sim (avoids over-brake or under-brake).

# --- Map and timing ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param time_step = 0.05

# --- Calibration parameters ---
param warmup_seconds = 3.0    # Wait for simulator to initialize before starting
param throttle_seconds = 3.0   # Full throttle duration (s)
param brake_seconds = 10.0     # Full brake duration (s)
# Optional: set to a path string to write (t, speed, phase) CSV for analysis
param calibration_output = None

# --- Driving + racing domain ---
model scenic.domains.driving.model
model scenic.domains.racing.model

# --- Ego: single car running calibration ---
ego = new RacingCar at (55.766137, 88.269387), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_racing_line_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
    with ttlDX 0.0, \
    with ttlDY 0.0

# Run calibration: throttle then brake; results printed to console (and optionally to file)
ego.behavior = EmpiricalAccelDecelCalibrationBehavior(
    warmup_seconds=globalParameters.warmup_seconds,
    throttle_seconds=globalParameters.throttle_seconds,
    brake_seconds=globalParameters.brake_seconds,
    output_path=globalParameters.calibration_output
)
