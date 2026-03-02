# Example: Relative placement (ahead/behind) on main racing road
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.simulators.dspace.racing_model

# ego = new RacingCar on mainRacingRoad
fellow1 = new RacingCar on mainRacingRoad
fellow2 = new RacingCar ahead of fellow1 by 10
fellow3 = new RacingCar behind fellow1 by 5

