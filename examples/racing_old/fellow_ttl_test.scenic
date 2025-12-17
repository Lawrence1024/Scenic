param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

model scenic.simulators.dspace.model

param test_waypoints = [
    (587, -260), (575, -257), (563, -244), (558, -219),
    (556, -202), (556, -172), (558, -149), (556, -118), 
    (549, -89), (533, -62)
]

ego = new RacingCar at 601.772 @ -258.196

fellow1 = new RacingCar at 587 @ -260,
    with raceNumber 1,
    with waypoints globalParameters.test_waypoints,
    with behavior FollowRacingLineBehavior(target_speed=5, use_waypoints=True, lookahead=20)

terminate when simulation().currentTime > 100