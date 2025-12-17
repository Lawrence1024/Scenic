param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.simulators.dspace.racing_model

ego = new RacingCar on mainRacingRoad, with raceNumber 1
fellow1 = new RacingCar on mainRacingRoad, with raceNumber 2, with behavior SimpleRacingBehavior
fellow2 = new RacingCar on pitLaneRoad, with raceNumber 3, with behavior SimplePitBehavior