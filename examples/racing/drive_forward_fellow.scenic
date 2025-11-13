param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 1

# Use the dSPACE racing model
model scenic.simulators.dspace.model

behavior forwardControlTask():
    # Accelerate straight continuously
    while True:
        take SetThrottleAction(0.5)
    

ego = new RacingCar on road
fellow1 = new RacingCar on road, with behavior forwardControlTask()

