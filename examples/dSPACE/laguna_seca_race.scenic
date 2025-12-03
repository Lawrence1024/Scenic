"""
Laguna Seca Racing Scenario

This scenario sets up a racing scenario at Laguna Seca with cars positioned
on the starting grid. The ego car is placed at pole position (grid slot 0).
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = True
param startingGridPositions = 20
param startingGridSpacing = 8.0

# Use dSPACE racing model
model scenic.simulators.dspace.racing_model

# Ego car at pole position (starting grid position 0)
ego = new RacingCar at startingGrid[0], 
    with raceNumber 1,
    team "Team Scenic",
    fuelLevel 1.0,
    tireWear 0.0

# Opponent cars on starting grid
opponent1 = new RacingCar at startingGrid[1],
    with raceNumber 2,
    team "Team Alpha",
    fuelLevel 1.0,
    tireWear 0.0

opponent2 = new RacingCar at startingGrid[2],
    with raceNumber 3,
    team "Team Beta",
    fuelLevel 1.0,
    tireWear 0.0

opponent3 = new RacingCar at startingGrid[3],
    with raceNumber 4,
    team "Team Gamma",
    fuelLevel 1.0,
    tireWear 0.0

