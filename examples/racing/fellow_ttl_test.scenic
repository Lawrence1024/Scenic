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
     (587, -260), (575, -257), (563, -244), (558, -219),
    (556, -202), (556, -172), (558, -149), (556, -118), 
    (549, -89), (533, -62)
]

fellow1 = new RacingCar at (-115.049129, -360.688095, 0.000000),
    with waypoints testTTL,
    with behavior FollowRacingLineBehavior(target_speed=25, use_waypoints=True)



terminate when simulation().currentTime > 30

