# Comprehensive test of all dSPACE vehicle controls
#
# Tests: throttle, brake, steering, and gear control

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False
param timestep = 0.1

# Use the dSPACE racing model
model scenic.simulators.dspace.model

# Behavior: Test all controls
behavior FullControlTest():
    # Start from neutral with clutch
    take PressClutchAction()
    wait
    take SetGearAction(1)
    wait
    take ReleaseClutchAction()
    wait
    
    # Accelerate straight
    take SetThrottleAction(0.5)
    take SetSteerAction(0.0)
    wait
    wait
    
    # Shift to 2nd while steering left
    take SetGearAction(2)
    take SetThrottleAction(0.6)
    take SetSteerAction(-0.3)
    wait
    wait
    
    # Shift to 3rd while steering right
    take SetGearAction(3)
    take SetThrottleAction(0.7)
    take SetSteerAction(0.4)
    wait
    wait
    
    # Apply brakes while centering steering
    take SetBrakeAction(0.5)
    take SetThrottleAction(0.0)
    take SetSteerAction(0.0)
    wait
    wait
    
    # Downshift to 2nd
    take SetGearAction(2)
    take SetBrakeAction(0.0)
    take SetThrottleAction(0.4)
    wait
    wait
    
    # Final: moderate throttle, slight left steer
    take SetThrottleAction(0.5)
    take SetSteerAction(-0.2)

# Place ego on pit lane
ego = new RacingCar on pitLaneRoad, with behavior FullControlTest()

