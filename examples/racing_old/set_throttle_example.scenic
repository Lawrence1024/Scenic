# Simple dSPACE racing example to test SetThrottleAction

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 0.1

# Use the dSPACE racing model (imports dSPACE actions and RacingCar)
model scenic.simulators.dspace.model

# Define a simple behavior: apply throttle continuously
behavior TestThrottle():
    # Apply 50% throttle continuously
    # In Scenic, actions are applied once per timestep, so we loop
    while True:
        take SetThrottleAction(0.5)

# Place ego on pit lane
ego = new RacingCar on pitLaneRoad, with behavior TestThrottle()
