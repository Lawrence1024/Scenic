# Example: Ego vehicle using MPC controller for racing line following
#
# This example demonstrates how to use the MPC-based behavior for improved
# racing performance compared to PID controllers.

# --- Map and timing (must come before the model line) ---
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
# 100 Hz simulation step, 20 Hz control and readback (0.05 s period)
param time_step = 0.01
param control_period = 0.05
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param scenic_control = False
param launch_veos_ipc_client = True

# --- dSPACE racing model (RacingCar, behaviors, 100 Hz step / 20 Hz control & readback) ---
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
# Pitlane Start
ego = new RacingCar at (79.766382000,97.055717000), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_right_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

# Use ART wrapper behavior 
ego.behavior = ARTStackControlBehavior()














