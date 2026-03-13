"""
Two Mutually Exclusive Track Segments

This scenario demonstrates the racing domain's two-segment architecture
for the Laguna Seca track.

Track Segments (segment centerlines with buffer; from OpenDRIVE or TTL):
1. mainTrack (alias mainRacingRoad): Main road centerline + 5 m each side
   - Corkscrew, Andretti Hairpin, junction links
2. pitTrack (alias pitLaneRoad): Pit lane centerline + 2 m each side
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

model scenic.simulators.dspace.racing_model

# Vehicles on track segments - use mainTrack/pitTrack (or mainRacingRoad/pitLaneRoad)
# Routes auto-detected from placement (dSPACE)
ego = new RacingCar on pitTrack
fellow1 = new RacingCar on mainTrack
fellow2 = new RacingCar on mainTrack
fellow3 = new RacingCar on mainTrack
fellow4 = new RacingCar on pitTrack
fellow5 = new RacingCar on pitTrack
fellow6 = new RacingCar on pitTrack

