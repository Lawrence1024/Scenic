# Test steering control in dSPACE

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 0.1

# Use the dSPACE racing model
model scenic.simulators.dspace.model

# Behavior: Test steering with throttle
behavior TestSteering():
    # Apply throttle and steer left
    take SetThrottleAction(0.3)
    take SetSteerAction(-0.5)  # Steer left (-1 to 1 range)
    wait
    wait
    
    # Steer right
    take SetThrottleAction(0.3)
    take SetSteerAction(0.5)   # Steer right
    wait
    wait
    
    # Return to center
    take SetThrottleAction(0.3)
    take SetSteerAction(0.0)   # Center
    wait
    wait
    
    # Full left
    take SetThrottleAction(0.3)
    take SetSteerAction(-1.0)  # Full left
    wait
    wait
    
    # Full right
    take SetThrottleAction(0.3)
    take SetSteerAction(1.0)   # Full right
    wait
    wait
    
    # Back to center and reduce throttle
    take SetThrottleAction(0.2)
    take SetSteerAction(0.0)

# Place ego on pit lane
ego = new RacingCar on pitLaneRoad, with behavior TestSteering()

