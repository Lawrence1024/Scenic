param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.simulators.dspace.racing_model

# Racing cars on the racing line (this tests the racing domain!)
ego = new RacingCar on racingLine
opponent1 = new RacingCar on racingLine
opponent2 = new RacingCar on racingLine