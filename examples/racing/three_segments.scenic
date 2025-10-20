"""
Two Mutually Exclusive Track Segments

This scenario demonstrates the racing domain's two-segment architecture
for the Laguna Seca track.

Track Segments (auto-identified from OpenDRIVE):
1. mainRacingRoad: Union of all racing roads
   - The Corkscrew1 (2484.58m)
   - Andretti Hairpin1_3 (988.05m)
   - Any other parallel racing roads
2. pitLaneRoad: Pit lane only
   - Pit Lane1_2 (883.46m)

Architecture:
- Two mutually exclusive segments (no overlap)
- Comprehensive (cover the entire track)
- Simulator-agnostic (work with any simulator)

Route Assignment (dSPACE):
- Automatic detection based on placement:
  * Objects on pitLaneRoad → ModelDesk "Pit" route
  * Objects on mainRacingRoad → ModelDesk "Lap" route
- No manual route specification needed!

PREREQUISITE (dSPACE): Create routes in ModelDesk Road Generator:
- Route0 (for pit lane vehicles)
- Route1 (for main lap vehicles)

Run with:
    scenic examples/racing/two_segments.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1
"""

# Configure Laguna Seca track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
param generateStartingGrid = False

model scenic.simulators.dspace.racing_model

# Vehicles on track segments - routes auto-detected from projection road ID
ego = new RacingCar on pitLaneRoad
opponent1 = new RacingCar ahead of ego by 20
# opponent2 = new RacingCar on pitLaneRoad
# opponent3 = new RacingCar on pitLaneRoad
# opponent4 = new RacingCar on pitLaneRoad
# opponent5 = new RacingCar on pitLaneRoad
# opponent6 = new RacingCar on pitLaneRoad

