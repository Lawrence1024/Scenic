param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

# TTL (Target Trajectory Line) Configuration
param ttlIndex = 17
param ttlDX = -53.6
param ttlDY = -15.7
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV/usable')

# Load the dSPACE World Model
model scenic.simulators.dspace.model

# Create the Ego Car
ego = new RacingCar on mainRacingRoad, 
    with raceNumber 1,
    with maxSpeed 60 # m/s

# Assign the Behavior
# Use the driving-domain lane-following behavior. When a TTL is attached by the
# dSPACE simulator, FollowLaneBehavior will use the TTL's signedDistanceTo for
# lateral control; otherwise it follows the lane centerline.
ego.behavior = FollowLaneBehavior(target_speed=30)

# End Condition
terminate when simulation().currentTime > 30