param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param carla_map = 'Town01'
param use2DMap = True
model scenic.simulators.dspace.model

ego = new Car on pitLaneRoad
fellow1 = new Car on road
fellow2 = new Car left of fellow1 by 8
fellow3 = new Car behind fellow2 by 20

