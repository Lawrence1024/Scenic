"""Example scenario demonstrating decision tree behaviors.

This example shows how to use the new decision tree behaviors and actions
for race decision engine integration.
"""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

model scenic.simulators.dspace.model

# Create racing cars
ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Example 1: Flag-based speed behavior
# ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0)

# Example 2: Lane selection behavior
# ego.behavior = LaneSelectionBehavior(ttl_selection="race")

# Example 3: Stop behavior
# ego.behavior = StopBehavior(stop_type="safe")

# Example 4: Follow mode behavior (requires opponent)
# opponent = new RacingCar ahead of ego by 50, with raceNumber 2
# ego.behavior = FollowModeBehavior(target_car=opponent, target_gap=31.0)

# Example 5: Pit lane behavior
# ego.behavior = PitLaneBehavior()

# Default: Use flag-based speed with green flag
ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0)

