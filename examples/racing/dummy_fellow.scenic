param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
generateStartingGrid = False
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)
# Pitlane Start
# ego = new RacingCar at (79.766382000,97.055717000), \
# Pitlane End
# ego = new RacingCar at (196.952588000,9.974341000), \
# Main End
# ego = new RacingCar at (134.131413,125.953041),\  
# Weird Curve
# ego = new RacingCar at (614.659946,-302.782016),\  
# Werid Curve
# ego = new RacingCar at (-110.956171,-151.841778,8.331000),\ 
# Main Start
ego = new RacingCar at (60.3811498,103.7450019), \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_main_road.csv', \
    # with ttlFileName 'ttl_pitlane.csv', \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    # with ttlFileName 'ttl_right_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV'), \
    with ttlDX 0.0, \
    with ttlDY 0.0



ego.behavior = ARTStackControlBehavior()

# Dummy fellow: no behavior; driven by External_Signals (Velocity/Lateral deviation = Extern in dSPACE)
param fellow_dummy_centerline = True
param fellow_dummy_velocity_kmh = 50
fellow0 = new RacingCar at (49.3895,87.9318), with regionContainedIn everywhere, with raceNumber 2
# fellow1 = new RacingCar on mainTrack
# fellow2 = new RacingCar on mainTrack
# fellow3 = new RacingCar on mainTrack
# fellow4 = new RacingCar on mainTrack
# fellow5 = new RacingCar on mainTrack
# fellow6 = new RacingCar on mainTrack
# fellow7 = new RacingCar on mainTrack
# fellow8 = new RacingCar on mainTrack
# fellow9 = new RacingCar on mainTrack
# fellow10 = new RacingCar on pitTrack
# fellow11 = new RacingCar on pitTrack
# fellow12 = new RacingCar on pitTrack
# fellow13 = new RacingCar on pitTrack
# fellow14 = new RacingCar on pitTrack
# fellow15 = new RacingCar on pitTrack
# fellow16 = new RacingCar on pitTrack
# fellow17 = new RacingCar on pitTrack
# fellow18 = new RacingCar on pitTrack
# fellow19 = new RacingCar on pitTrack
# fellow20 = new RacingCar on pitTrack
# fellow21 = new RacingCar on pitTrack













