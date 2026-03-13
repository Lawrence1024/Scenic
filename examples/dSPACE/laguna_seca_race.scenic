"""
Laguna Seca Racing Scenario

This scenario sets up a racing scenario at Laguna Seca with cars placed on
the main track. Ego and opponents are sampled from mainTrack.
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

# Use dSPACE racing model
model scenic.simulators.dspace.racing_model

# Ego and opponents on main track
ego = new RacingCar on mainTrack,
    with raceNumber 1,
    team "Team Scenic",
    fuelLevel 1.0,
    tireWear 0.0

opponent1 = new RacingCar on mainTrack,
    with raceNumber 2,
    team "Team Alpha",
    fuelLevel 1.0,
    tireWear 0.0

opponent2 = new RacingCar on mainTrack,
    with raceNumber 3,
    team "Team Beta",
    fuelLevel 1.0,
    tireWear 0.0

opponent3 = new RacingCar on mainTrack,
    with raceNumber 4,
    team "Team Gamma",
    fuelLevel 1.0,
    tireWear 0.0

