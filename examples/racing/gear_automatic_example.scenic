# Normal gear shifting (no clutch needed)
#
# Use SetGearAction for all gear changes while moving.
# Clutch is only needed when starting from neutral (gear 0 → 1).

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 0.1

# Use the dSPACE racing model
model scenic.simulators.dspace.model

# Behavior: Normal gear shifting (already in 1st gear)
behavior NormalGearShifting():
    # Assume already in 1st gear and moving
    
    # Upshift to 2nd
    take SetGearAction(2)
    wait
    wait
    
    # Upshift to 3rd
    take SetGearAction(3)
    wait
    wait
    
    # Upshift to 4th
    take SetGearAction(4)
    wait
    wait
    
    # Downshift to 2nd
    take SetGearAction(2)

# Place ego on pit lane
ego = new RacingCar on pitLaneRoad, with behavior NormalGearShifting()

