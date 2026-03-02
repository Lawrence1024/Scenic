# Example: Decision-tree behaviors for race decision engine integration
#
# Demonstrates FlagBasedSpeedBehavior, LaneSelectionBehavior, StopBehavior,
# and FollowModeBehavior (see behaviors.scenic).

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

model scenic.simulators.dspace.racing_model

ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Uncomment one:
# ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0)
# ego.behavior = LaneSelectionBehavior(ttl_selection="race")
# ego.behavior = StopBehavior(stop_type="safe")
# ego.behavior = FollowModeBehavior(target_car=opponent, target_gap=31.0)

ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0)
