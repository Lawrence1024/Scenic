"""
Laguna Seca Racing Scenario

This scenario demonstrates the racing domain features:
- Racing track setup with proper direction
- Starting grid positions
- Multiple racing cars
- Racing behaviors

The scenario places cars on the starting grid at Laguna Seca
and can be run with the dSPACE simulator.
"""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'  # Laguna Seca runs counterclockwise
param generateStartingGrid = True
param startingGridPositions = 10  # 10 grid positions
model scenic.simulators.dspace.racing_model

# Create the ego car (pole position)
ego = new RacingCar at startingGrid[0], \
    with raceNumber 1, \
    with team "Team Ego", \
    with color [1, 0, 0]  # Red

# Create opponent cars on the starting grid
opponent1 = new RacingCar at startingGrid[1], \
    with raceNumber 2, \
    with team "Team Blue", \
    with color [0, 0, 1]  # Blue

opponent2 = new RacingCar at startingGrid[2], \
    with raceNumber 3, \
    with team "Team Green", \
    with color [0, 1, 0]  # Green

opponent3 = new RacingCar at startingGrid[3], \
    with raceNumber 4, \
    with team "Team Yellow", \
    with color [1, 1, 0]  # Yellow

# You can add behaviors for a dynamic scenario:
# ego with behavior FollowRacingLineBehavior(target_speed=30)
# opponent1 with behavior FollowRacingLineBehavior(target_speed=28)
# opponent2 with behavior FollowRacingLineBehavior(target_speed=29)

