"""
Example: Waypoint-based Autonomous Racing with Dallara AV-24

This example demonstrates the new waypoint-based behaviors for autonomous racing
using the simplified RacingCar class with Dallara AV-24 specifications.

Features demonstrated:
- Single configurable RacingCar class (no more FormulaCar/GTCar/PrototypeCar)
- Waypoint-based behaviors using Scenic's PID controllers
- Configurable car properties (maxSpeed, aggressiveness, etc.)
- Pit lane vs main racing road distinction
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.simulators.dspace.racing_model

# Example 1: Basic Dallara AV-24 with default settings
ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Example 2: Aggressive racing car with higher speed
aggressiveCar = new RacingCar on mainRacingRoad, 
    with raceNumber 2,
    maxSpeed 35,  # Higher top speed
    controllerAggressiveness 0.8,  # More aggressive driving
    waypointTolerance 2.0  # Tighter waypoint following

# Example 3: Conservative pit lane car
pitCar = new RacingCar on pitLaneRoad,
    with raceNumber 3,
    maxSpeed 15,  # Pit lane speed limit
    controllerAggressiveness 0.2,  # Conservative driving
    waypointTolerance 4.0  # More forgiving waypoint following

# Example 4: Custom configured car
customCar = new RacingCar on mainRacingRoad,
    with raceNumber 4,
    carType "Custom Racing Car",
    team "Team Alpha",
    maxSpeed 28,
    acceleration 9.0,
    braking -13.0,
    controllerAggressiveness 0.6

# Example 5: Multiple cars with different configurations
#opponent1 = new RacingCar on mainRacingRoad, with raceNumber 5
#opponent2 = new RacingCar on mainRacingRoad, with raceNumber 6
#opponent3 = new RacingCar on pitLaneRoad, with raceNumber 7

# Note: To use waypoint behaviors, you would add:
# do FollowWaypointsBehavior(waypoints, targetSpeed=25)
# do RacingLineBehavior(targetSpeed=30)
# do PitLaneBehavior(targetSpeed=15)
# do AdaptiveRacingBehavior(baseSpeed=25, aggression=0.7)
