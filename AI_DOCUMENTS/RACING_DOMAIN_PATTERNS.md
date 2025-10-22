# Racing Domain: Practical Patterns & Examples

This guide provides **practical examples** of how to write racing scenarios using the proper domain architecture patterns learned from the driving domain.

---

## Table of Contents

1. [Basic Racing Scenario Template](#basic-racing-scenario-template)
2. [Common Patterns](#common-patterns)
3. [Track Segment Usage](#track-segment-usage)
4. [Object Placement Patterns](#object-placement-patterns)
5. [Behavior Patterns](#behavior-patterns)
6. [Multi-Simulator Scenarios](#multi-simulator-scenarios)
7. [Debugging Tips](#debugging-tips)

---

## Basic Racing Scenario Template

### Minimal Racing Scenario

```scenic
"""Minimal racing scenario template."""

# 1. CONFIGURE TRACK
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

# 2. IMPORT MODEL (this loads track, creates regions)
model scenic.domains.racing.model

# 3. CREATE OBJECTS (using racing regions)
ego = new RacingCar on racingLine

# 4. ADD BEHAVIORS (optional)
ego with behavior FollowRacingLineBehavior()
```

**What happens when model loads**:
1. Loads OpenDRIVE map into `Network`
2. Creates `RacingTrack` wrapper around network
3. Identifies `mainRacingRoad` and `pitLaneRoad` segments
4. Creates regions: `road`, `racingLine`, `pitLane`, `mainRacingRoad`, `pitLaneRoad`
5. Generates `startingGrid` positions (if `generateStartingGrid = True`)

---

## Common Patterns

### Pattern 1: Starting Grid Formation

```scenic
"""Racing scenario with starting grid."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param generateStartingGrid = True
param startingGridPositions = 20  # Generate 20 grid positions
param startingGridSpacing = 8.0   # 8 meters between positions

model scenic.domains.racing.model

# Place cars on starting grid
ego = new RacingCar at startingGrid[0], with raceNumber 1
opponent1 = new RacingCar at startingGrid[1], with raceNumber 2
opponent2 = new RacingCar at startingGrid[2], with raceNumber 3

# Formation lap behavior
ego with behavior FormationLapBehavior(leader=None)
opponent1 with behavior FormationLapBehavior(leader=ego)
opponent2 with behavior FormationLapBehavior(leader=opponent1)
```

### Pattern 2: Main Track vs Pit Lane Split

```scenic
"""Scenario with cars on both main track and pit lane."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

model scenic.domains.racing.model

# Cars on main racing circuit
ego = new RacingCar on mainRacingRoad, with raceNumber 1
opponent1 = new RacingCar on mainRacingRoad, with raceNumber 2

# Car in pit lane
pit_car = new RacingCar on pitLaneRoad, with raceNumber 99

# Behaviors
ego with behavior FollowRacingLineBehavior()
opponent1 with behavior FollowRacingLineBehavior()
pit_car with behavior PitLaneBehavior(speedLimit=20)
```

**Why this works**:
- `mainRacingRoad` and `pitLaneRoad` are **mutually exclusive** regions
- Simulators can detect which road an object is on
- dSPACE automatically assigns routes based on road segment

### Pattern 3: Relative Positioning on Track

```scenic
"""Using relative positioning based on road network."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Ego on main track
ego = new RacingCar on mainRacingRoad

# Opponent ahead by distance (follows road)
opponent1 = new RacingCar ahead of ego by Range(20, 50)

# Opponent behind by distance
opponent2 = new RacingCar behind ego by Range(15, 30)

# Opponent to the left (lateral offset)
opponent3 = new RacingCar following roadDirection from ego for Range(-2, -1),
                        left of ego by Range(1.5, 2.0)
```

**Key concept**: `ahead of` and `behind` follow the **road network**, not straight-line distance.

### Pattern 4: Pit Stop Scenario

```scenic
"""Scenario involving pit stops."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Racing car that will pit
ego = new RacingCar on mainRacingRoad,
    with fuelLevel 0.2,  # Low fuel, needs pit
    with tireWear 0.8    # Worn tires

# Other cars continuing to race
opponent1 = new RacingCar ahead of ego by 100
opponent2 = new RacingCar behind ego by 80

# Behaviors
ego with behavior RaceWithPitStopBehavior(
    pitLap=3,
    pitBox=track.pitLane.pitBoxes[0] if track.pitLane else None
)

opponent1 with behavior FollowRacingLineBehavior()
opponent2 with behavior OvertakingBehavior(target=ego)
```

### Pattern 5: Overtaking Scenario

```scenic
"""Two cars in overtaking situation."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Leader (slower car)
leader = new RacingCar on racingLine,
    with maxSpeed 25

# Attacker (faster car, close behind)
attacker = new RacingCar behind leader by Range(5, 15),
    with maxSpeed 35

# Behaviors
leader with behavior DefensiveDrivingBehavior()
attacker with behavior OvertakingBehavior(target=leader, aggressive=True)

# Require specific track section for overtaking
require attacker in track.sectors[1].region  # Overtake in sector 2
```

---

## Track Segment Usage

### Understanding the Two-Segment Architecture

After importing `scenic.domains.racing.model`, you have these **mutually exclusive** regions:

1. **`mainRacingRoad`**: Union of all non-pit roads
   - Main racing line
   - Parallel roads/tracks
   - Alternative routes (if any)

2. **`pitLaneRoad`**: Pit lane only
   - Separate from main circuit
   - Has speed limits
   - Entry/exit points

### Choosing the Right Region

```scenic
# For general racing (use most often)
car1 = new RacingCar on racingLine       # ✅ Optimal: on main track, not pit
car2 = new RacingCar on road            # ✅ OK: on any drivable road
car3 = new RacingCar on mainRacingRoad  # ✅ Explicit: on main racing roads

# For pit lane scenarios
pit_car = new RacingCar on pitLane      # ✅ In pit lane region
pit_car2 = new RacingCar on pitLaneRoad # ✅ Explicit: on pit lane road

# For starting grid
grid_car = new RacingCar at startingGrid[0]  # ✅ At specific grid position

# Generic (less specific)
any_car = new RacingCar                 # ⚠️  Position undefined, will be random on 'road'
```

### Region Hierarchy

```
road (from driving domain)
├── racingLine = road - pitLane
│   └── mainRacingRoad (explicit road segments)
└── pitLane (if exists)
    └── pitLaneRoad (explicit pit road)
```

**Best Practice**: Use most specific region appropriate for your scenario.

---

## Object Placement Patterns

### Pattern: Placement with Constraints

```scenic
"""Advanced placement using constraints."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Ego in a specific sector
ego = new RacingCar on mainRacingRoad

# Opponent in same sector, ahead
opponent = new RacingCar on mainRacingRoad,
    ahead of ego by Range(20, 100)

# Require both in sector 2 (middle sector)
require ego in track.sectors[1].region
require opponent in track.sectors[1].region

# Require reasonable separation
require (distance from ego to opponent) > 15
```

### Pattern: Track Position Sampling

```scenic
"""Sample positions along track based on distance."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Sample position at specific distance along track
ego = new RacingCar on mainRacingRoad

# Get position 500m ahead along track
# (This would require track.positionAtDistance() method)
checkpoint_pos = track.positionAtDistance(500)

# Place object at that position
checkpoint = new RacingCar at checkpoint_pos
```

### Pattern: Lane-Based Placement

```scenic
"""Place cars in specific lanes (for multi-lane tracks)."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Get specific lane
main_lane = network.lanes[0]

# Place on specific lane
ego = new RacingCar on main_lane

# Place on adjacent lane (if multi-lane track)
if len(network.lanes) > 1:
    opponent = new RacingCar on network.lanes[1]
```

---

## Behavior Patterns

### Pattern: Sequential Behaviors

```scenic
"""Car executes sequence of behaviors during race."""

behavior RaceLapBehavior(lapNumber):
    """Execute one racing lap."""
    
    # Start lap
    print(f"Starting lap {lapNumber}")
    
    # Race the lap
    do FollowRacingLineBehavior(target_speed=30) until (
        distanceToSectorEnd(self) < 10 and
        self in track.sectors[-1].region  # Last sector
    )
    
    print(f"Completed lap {lapNumber}")

behavior MultiLapRaceBehavior(totalLaps=5):
    """Execute multi-lap race."""
    
    for lap in range(1, totalLaps + 1):
        do RaceLapBehavior(lap)
    
    # Race finished, slow down
    take SetBrakeAction(0.5)
```

### Pattern: Interrupt-Based Behaviors

```scenic
"""Behavior with interrupts for racing events."""

behavior RaceWithAwarenessBehavior():
    """Race with awareness of other cars and track conditions."""
    
    try:
        # Main racing behavior
        do FollowRacingLineBehavior(target_speed=30)
        
    interrupt when self.fuelLevel < 0.2:
        # Low fuel, need pit stop
        do ExecutePitStopBehavior()
        
    interrupt when self.distanceToClosest(RacingCar) < 3:
        # Car very close, defensive driving
        do DefensiveDrivingBehavior() for 10 seconds
        
    interrupt when self in pitLane:
        # Entered pit lane, obey speed limit
        do PitLaneBehavior(speedLimit=20)
```

### Pattern: Competitive Behaviors

```scenic
"""Behaviors for competitive racing."""

behavior OvertakingBehavior(target, aggressive=False):
    """Attempt to overtake target car."""
    
    # Close the gap
    while (distance from self to target) > 5:
        do FollowBehavior(target, followDistance=3)
    
    # Execute overtake
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)
        take DRSAction(activate=True)
    
    # Move to side and accelerate
    do LateralMoveBehavior(offset=2.0)  # 2m to the left
    take SetThrottleAction(1.0)
    
    # Complete overtake
    wait until (distance from self to target) > 10
    
    # Return to racing line
    do ReturnToRacingLineBehavior()

behavior DefensiveDrivingBehavior():
    """Defend position from cars behind."""
    
    # Monitor car behind
    cars_behind = [car for car in simulation().objects 
                   if isinstance(car, RacingCar) and 
                   carIsBehind(self, car)]
    
    if len(cars_behind) == 0:
        # No one behind, normal racing
        do FollowRacingLineBehavior()
    else:
        closest = min(cars_behind, key=lambda c: distance from self to c)
        
        # Take defensive line
        do DefensiveLineBehavior(threat=closest)
```

---

## Multi-Simulator Scenarios

### Pattern: Simulator-Agnostic Scenario

```scenic
"""Scenario that works in any racing simulator."""

# Generic parameters (work everywhere)
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

# Import GENERIC racing model (not simulator-specific)
model scenic.domains.racing.model

# Use only standard racing features
ego = new RacingCar on racingLine
opponent = new RacingCar ahead of ego by 50

# Use only standard actions
ego with behavior FollowRacingLineBehavior()
```

**Run in different simulators**:
```bash
# Visualizer only
scenic scenario.scenic

# dSPACE ModelDesk
scenic scenario.scenic --2d --model scenic.simulators.dspace.racing_model --simulate

# CARLA (hypothetically)
scenic scenario.scenic --model scenic.simulators.carla.racing_model --simulate

# Newtonian simulator
scenic scenario.scenic --model scenic.simulators.newtonian.racing_model --simulate
```

### Pattern: Simulator-Specific Extensions

```scenic
"""Scenario with optional simulator-specific features."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

# Import specific simulator model
model scenic.simulators.dspace.racing_model

# Use standard racing features
ego = new RacingCar on racingLine

# Check if running in simulator
if simulation() is not None:
    # Use simulator-specific features
    ego with behavior DSPACEWaypointFollowingBehavior(waypoints=myWaypoints)
else:
    # Fallback for visualization
    ego with behavior FollowRacingLineBehavior()
```

---

## Debugging Tips

### 1. Visualize Regions

```scenic
"""Debug scenario to visualize track regions."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Place markers at region boundaries
main_marker = new RacingCar on mainRacingRoad, with color [1, 0, 0]  # Red
pit_marker = new RacingCar on pitLaneRoad, with color [0, 0, 1]      # Blue

# Place markers on starting grid
for i, pos in enumerate(startingGrid[:5]):
    new RacingCar at pos, with raceNumber (i+1), with color [0, 1, 0]  # Green
```

Run without simulation to see visualization:
```bash
scenic debug_regions.scenic --2d
```

### 2. Print Track Information

```scenic
"""Debug track feature identification."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Print track info (happens during model load)
# See console output from track._identifyRacingFeatures()

# Create object to verify
ego = new RacingCar on mainRacingRoad

# Verify placement
print(f"Ego position: {ego.position}")
print(f"Ego road: {ego.road}")
print(f"Track has pit lane: {track.pitLane is not None}")
print(f"Number of sectors: {len(track.sectors)}")
```

### 3. Test Region Membership

```scenic
"""Test whether objects are in expected regions."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

# Place objects
ego = new RacingCar on mainRacingRoad
pit_car = new RacingCar on pitLaneRoad

# Test membership (these will reject if false)
require ego in mainRacingRoad
require ego in racingLine
require ego not in pitLane

require pit_car in pitLaneRoad
require pit_car in pitLane
require pit_car not in racingLine

print("✅ All region membership tests passed!")
```

### 4. Distance and Sector Checks

```scenic
"""Debug distance calculations and sector identification."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True

model scenic.domains.racing.model

ego = new RacingCar on racingLine
opponent = new RacingCar ahead of ego by 50

# Check distances
straight_distance = distance from ego to opponent
# Road distance would require track.distanceAlongTrack() method

# Check sectors (if implemented)
ego_sector = track.getSectorAt(ego.position)
if ego_sector:
    print(f"Ego in sector {ego_sector.number}")
    
    sector_end_dist = distanceToSectorEnd(ego)
    if sector_end_dist:
        print(f"Distance to sector end: {sector_end_dist}m")
```

### 5. Validate Track Features

```scenic
"""Validate that track features were correctly identified."""

param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param pitLaneRoadId = "1545702203"  # Explicit pit lane
param mainLineRoadId = "2117817291"  # Explicit main line

model scenic.domains.racing.model

# Verify track features
assert track.pitLane is not None, "❌ Pit lane not found!"
assert track.pitLaneRoad is not None, "❌ Pit lane road not identified!"
assert track.mainRacingRoad is not None, "❌ Main racing road not identified!"

print("✅ Track feature validation passed!")

# Create test objects
ego = new RacingCar on mainRacingRoad
print(f"✅ Ego successfully placed on main racing road")
```

---

## Advanced Patterns

### Pattern: Dynamic Racing Strategy

```scenic
"""Racing strategy that adapts based on position and fuel."""

behavior AdaptiveRacingBehavior():
    """Adapt strategy based on race state."""
    
    lap = 1
    
    while True:
        # Determine strategy based on state
        if self.fuelLevel < 0.15:
            # Critical fuel, must pit
            print("Critical fuel - executing pit stop")
            do ExecutePitStopBehavior()
            
        elif self.tireWear > 0.75:
            # High tire wear, defensive driving
            print("High tire wear - conservative driving")
            do FollowRacingLineBehavior(target_speed=25)
            
        elif len(carsAheadInSector(self)) > 0:
            # Cars ahead, try to overtake
            print("Cars ahead - attacking")
            target = carsAheadInSector(self)[0]
            do OvertakingBehavior(target=target)
            
        else:
            # Clear track, push hard
            print("Clear track - pushing")
            do FollowRacingLineBehavior(target_speed=35)
```

### Pattern: Formation Lap to Race Start

```scenic
"""Complete race start sequence."""

behavior RaceStartSequenceBehavior(gridPosition):
    """Execute formation lap and race start."""
    
    # Formation lap - hold position
    print(f"Formation lap - position {gridPosition}")
    do FormationLapBehavior(position=gridPosition) for 120 seconds
    
    # Return to grid
    do ReturnToGridBehavior(gridPosition=gridPosition)
    
    # Wait for start
    wait
    
    # Race start!
    print("🏁 Green flag!")
    do RaceStartBehavior()
    
    # Continue racing
    do MultiLapRaceBehavior(totalLaps=10)
```

---

## Summary: Key Takeaways

1. **Always start with the standard template**: Map, params, model import, objects
2. **Use appropriate regions**: `racingLine` for racing, `pitLaneRoad` for pit lane
3. **Leverage relative positioning**: `ahead of`, `behind`, `left of` work with road network
4. **Extend, don't replace**: Use `RacingCar` (extends `Car`), not a completely new class
5. **Compose behaviors**: Build complex behaviors from simpler ones
6. **Stay simulator-agnostic**: Use standard actions/behaviors when possible
7. **Test incrementally**: Start with visualization, then add simulation
8. **Debug with markers**: Use colored cars to visualize regions and positions

---

## Quick Reference: Common Regions

| Region | Description | Use When |
|--------|-------------|----------|
| `road` | All drivable surfaces | Generic placement |
| `racingLine` | Main track minus pit lane | Normal racing |
| `mainRacingRoad` | Main racing road segments | Explicit main track |
| `pitLane` | Pit lane region | Pit stop scenarios |
| `pitLaneRoad` | Pit lane road segment | Explicit pit lane |
| `startingGrid[i]` | Grid position i | Race starts |
| `track.sectors[i].region` | Sector i region | Sector-specific scenarios |

---

## Next Steps

1. **Study existing examples**: Look at `examples/racing/*.scenic`
2. **Experiment with regions**: Create debug scenarios to visualize
3. **Build incrementally**: Start simple, add complexity
4. **Test across simulators**: Ensure portability
5. **Contribute back**: Share useful patterns with the community

---

*Happy Racing! 🏎️*

