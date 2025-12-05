param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

model scenic.simulators.dspace.model

param ttlIndex = 17
param ttlDX = -53.6
param ttlDY = -15.7
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV/usable')
param timestep = 2

testTTL = [
    (0, 0),
    (50, 0),
    (100, 0),
    (150, 0),
    (200, 0)
]

fellow1 = new FellowCar at (-115.049129, -360.688095, 0.000000)
fellow1.isFellow = True
fellow1.waypoints = testTTL
fellow1.ttl = testTTL
fellow1.behavior = FollowRacingLineBehavior(
    target_speed = 10,
)

terminate when simulation().currentTime > 30

