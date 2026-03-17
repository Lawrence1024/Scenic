param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')
param use2DMap = True
param time_step = 0.01
param control_period = 0.05
param scenic_control = False
model scenic.simulators.dspace.racing_model

# --- Ego car with MPC behavior ---
# Using main racing road centerline TTL (excluding pitlane)
# PitTrack Start
# ego = new RacingCar at (-40.77355326377025,-79.02085385354275), \
# MainTrack Start
# ego = new RacingCar at (61.96155539333135,94.59380989753669), \
# MidPit
ego = new RacingCar at (-120.04546962222386,-218.69212142577055), \
# ego = new RacingCar on ttl, \
    facing roadDirection, \
    with regionContainedIn everywhere, \
    with raceNumber 1, \
    # with ttlFileName 'ttl_optimal_xodr.csv', \
    with ttlFileName 'ttl_pit_xodr.csv', \
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













