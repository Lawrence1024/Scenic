## Racing Domain for Scenic

The racing domain extends the [driving domain](../driving) with racing-specific features for closed-circuit race tracks.

### Features

#### 🏁 Racing Tracks
- **Track direction enforcement** - One-way circuits (clockwise/counterclockwise)
- **Pit lanes** - Special lanes with speed limits for pit stops
- **Sectors** - Track divisions for timing (typically 3 sectors per lap)
- **Starting grid** - Automatic generation of staggered grid positions
- **Racing line** - Optimal path through the circuit

#### 🏎️ Racing Objects
- **RacingCar** - Base racing vehicle with racing-specific properties
  - `raceNumber` - Car number for identification
  - `team` - Team affiliation
  - `fuelLevel` - Current fuel (0.0 to 1.0)
  - `tireWear` - Tire condition (0.0 = new, 1.0 = worn)
- **FormulaCar** - Open-wheel racing car
- **GTCar** - GT racing car
- **PrototypeCar** - Prototype racing car (LMP, DPi)
- **PitCrew** - Pit crew members
- **TrackMarshal** - Safety officials

#### 🎯 Racing Behaviors
- `FollowRacingLineBehavior` - Follow optimal racing line
- `OvertakingBehavior` - Attempt to pass another car
- `DefensiveDrivingBehavior` - Defend position from overtakes
- `PitStopBehavior` - Execute complete pit stop sequence
- `QualifyingLapBehavior` - Fast single-lap performance
- `FormationLapBehavior` - Maintain grid order before race start
- `RaceStartBehavior` - Launch from stationary grid position
- `ConserveFuelBehavior` - Fuel-saving mode for endurance
- `TrafficManagementBehavior` - Smart racing with traffic

#### ⚡ Racing Actions
- `DRSAction` - Drag Reduction System activation
- `ERSDeployAction` - Energy Recovery System deployment
- `TractionControlAction` - TC adjustment
- `BrakeBiasAction` - Brake balance control
- `PitLimiterAction` - Pit lane speed limiter
- `OvertakeAction` - Overtaking maneuver
- `DefendPositionAction` - Defensive positioning
- `SlipstreamAction` - Drafting behind another car

### Usage

#### Basic Racing Scenario

```scenic
param map = localPath('laguna_seca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.domains.racing.model

# Cars are placed on the starting grid automatically
ego = new RacingCar at startingGrid[0]
opponent1 = new RacingCar at startingGrid[1]
opponent2 = new RacingCar at startingGrid[2]
```

#### With Behaviors

```scenic
param map = localPath('laguna_seca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.domains.racing.model

ego = new RacingCar at startingGrid[0], \
    with behavior FollowRacingLineBehavior(target_speed=30)

opponent = new RacingCar at startingGrid[1], \
    with behavior TrafficManagementBehavior(target_speed=28)
```

#### Pit Stop Example

```scenic
param map = localPath('laguna_seca.xodr')
param use2DMap = True
model scenic.domains.racing.model

ego = new RacingCar on road, \
    with fuelLevel 0.2  # Low fuel

# After some laps, execute pit stop
ego with behavior PitStopBehavior(duration=25)
```

### Supported Simulators

The racing domain works with any simulator supporting the driving domain:

- **dSPACE ModelDesk** - Use `scenic.simulators.dspace.racing_model`
- **CARLA** - Use `scenic.simulators.carla.model` with racing maps
- **LGSVL** - Use `scenic.simulators.lgsvl.model` with racing circuits
- **Newtonian** - Use `scenic.simulators.newtonian.driving_model`

### Parameters

Global parameters for racing scenarios:

```python
param trackDirection = 'counterclockwise'  # or 'clockwise'
param generateStartingGrid = True  # Auto-generate grid positions
param startingGridPositions = 20  # Number of grid slots
param startingGridSpacing = 8.0  # Meters between grid positions
```

### Track Features

#### Sectors

Tracks are automatically divided into sectors for timing:

```python
sector = track.getSectorAt(car.position)
print(f"Car is in {sector.name}")
distance_to_end = distanceToSectorEnd(car)
```

#### Pit Lane

If the track has a pit lane:

```python
if track.isOnPitLane(car.position):
    print("Car is in pit lane")

# Pit lane has speed limit
print(f"Pit speed limit: {track.pitLane.speedLimit} m/s")
```

#### Starting Grid

```python
# Get grid positions
positions = track.startingGrid

# Place cars on grid
for i, pos in enumerate(positions[:10]):
    car = new RacingCar at pos, with raceNumber (i+1)
```

### Architecture

```
scenic.domains.racing/
├── __init__.py          # Domain documentation
├── tracks.py            # RacingTrack, PitLane, Sector classes
├── model.scenic         # Racing objects and regions
├── behaviors.scenic     # Racing behaviors
├── actions.py           # Racing actions
└── README.md            # This file
```

The racing domain extends:
```
scenic.domains.driving/
├── roads.py            # Network, Road, Lane (base classes)
├── model.scenic        # Vehicle, Pedestrian (base classes)
├── behaviors.scenic    # FollowLaneBehavior (extended)
└── actions.py          # Driving actions (extended)
```

### Examples

See `examples/dSPACE/laguna_seca_race.scenic` for a complete example.

### Future Enhancements

Planned features:
- [ ] DRS zones (specific track regions where DRS can be activated)
- [ ] Safety car behavior
- [ ] Track limits detection (automatic rejection if car exceeds limits)
- [ ] Tire temperature simulation
- [ ] Weather conditions (wet/dry track)
- [ ] Flag system (yellow, red, blue flags)
- [ ] Automatic pit lane detection from OpenDRIVE lane types
- [ ] Racing line calculation from track geometry
- [ ] Optimal braking points
- [ ] Overtaking zone identification

### Contributing

When adding new racing features:
1. Extend `tracks.py` for track-level features
2. Add object types to `model.scenic`
3. Implement behaviors in `behaviors.scenic`
4. Create reusable actions in `actions.py`
5. Update this README with examples

