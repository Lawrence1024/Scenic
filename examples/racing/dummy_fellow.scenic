param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
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
# Facing roadDirection aligns ego with the track so "ahead of ego by 20" / "right of ego by 5" give sensible (s,t).
# ego = new RacingCar at (60.8199003,92.948660), \
ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFolder localPath('../../assets/ttls/LS_ENU_TTL_CSV')

ego.behavior = ARTStackControlBehavior()


# fellow1 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('left', 3)
# fellow2 = new RacingCar with regionContainedIn everywhere, with _racing_st_offset ('behind', 20)




# Dummy fellow: no behavior; driven by External_Signals (Velocity/Lateral deviation = Extern in dSPACE)
# param fellow_dummy_centerline = True
# param fellow_dummy_velocity_kmh = 50
# fellow0 = new RacingCar at (49.3895,87.9318), with regionContainedIn everywhere, with raceNumber 2

# fellow3 = new RacingCar on mainTrack
# fellow4 = new RacingCar on pitTrack













