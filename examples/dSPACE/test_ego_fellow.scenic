param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param carla_map = 'Town01'
param use2DMap = True
model scenic.simulators.dspace.model

ego = new Car on road
fellow1 = new Car behind ego by 10
fellow2 = new Car ahead of ego by 30

