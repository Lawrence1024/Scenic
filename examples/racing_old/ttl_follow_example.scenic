param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

# TTL selection and transform (global for all usable TTLs)
param ttlIndex = 17        # choose among: 2, 3, 9, 15, 16, 17
param ttlDX = -53.6
param ttlDY = -15.7
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV')

model scenic.simulators.dspace.model

# Ego car on the main racing road; simulator assigns ttl automatically from ttlNumber
ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Follow the TTL using the racing line behavior
ego.behavior = FollowRacingLineBehavior(target_speed=30)
fellow1 = new RacingCar on road
fellow1.raceNumber = 2
fellow2 = new RacingCar on road
fellow2.raceNumber = 3


