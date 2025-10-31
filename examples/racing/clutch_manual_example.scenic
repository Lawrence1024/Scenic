# Starting from neutral with clutch control
#
# Use PressClutchAction and ReleaseClutchAction when starting from neutral (gear 0 → 1).
# After that, use SetGearAction directly for normal gear changes.

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 0.1

# Use the dSPACE racing model
model scenic.simulators.dspace.model

# Behavior: Start from neutral, then shift normally
behavior StartAndShift():
    # Start from neutral (gear 0)
    take PressClutchAction()       # Press clutch
    wait
    take SetGearAction(1)           # Engage 1st gear
    wait
    take ReleaseClutchAction()      # Release clutch to start moving
    wait
    wait
    
    # Now shift normally (no clutch needed for these)
    take SetGearAction(2)           # 2nd gear
    wait
    wait
    
    take SetGearAction(3)           # 3rd gear
    wait
    wait
    
    take SetGearAction(4)           # 4th gear
    wait
    wait
    
    take SetGearAction(2)           # Downshift to 2nd

# Place ego on pit lane
ego = new RacingCar on pitLaneRoad, with behavior StartAndShift()

