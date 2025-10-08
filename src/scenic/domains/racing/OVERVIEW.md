# Racing Domain - Implementation Overview

## Summary

The racing domain has been successfully created as an extension of the driving domain, adding racing-specific concepts and features for closed-circuit race tracks like Laguna Seca.

## Key Features Implemented

### 1. **Track Direction Enforcement** ✅
- Tracks have a defined direction (clockwise/counterclockwise)
- `RacingTrack.enforceTrackDirection()` validates vehicle headings
- Rejects scenarios where cars face the wrong way

### 2. **Pit Lane Support** ✅
- `PitLane` class with speed limits
- Entry/exit points
- Pit box regions
- `PitStopBehavior` for complete pit stop sequences

### 3. **Sectors** ✅
- Tracks automatically divided into 3 sectors
- `Sector` class with timing regions
- `track.getSectorAt(position)` for location queries
- Helper functions like `distanceToSectorEnd(car)`

### 4. **Starting Grid** ✅
- Automatic generation of staggered grid positions
- `track.generateStartingGrid(numPositions, spacing)`
- Pre-populated `startingGrid` list in model
- Easy car placement: `new RacingCar at startingGrid[0]`

### 5. **Racing-Specific Objects** ✅

**Vehicles:**
- `RacingCar` - Base racing vehicle
- `FormulaCar` - Open-wheel cars (F1, IndyCar)
- `GTCar` - GT racing
- `PrototypeCar` - LMP/DPi prototypes

**Personnel:**
- `PitCrew` - Pit crew members
- `TrackMarshal` - Safety officials

**Properties:**
- `raceNumber` - Car identification
- `team` - Team affiliation
- `fuelLevel` - Fuel state (0.0-1.0)
- `tireWear` - Tire condition (0.0-1.0)

### 6. **Racing Behaviors** ✅

Implemented 10 racing-specific behaviors:

1. **FollowRacingLineBehavior** - Optimal line following
2. **OvertakingBehavior** - Multi-phase overtaking
3. **DefensiveDrivingBehavior** - Position defense
4. **PitStopBehavior** - Complete pit stop sequence
5. **QualifyingLapBehavior** - Fast single-lap mode
6. **FormationLapBehavior** - Pre-race formation
7. **RaceStartBehavior** - Launch from grid
8. **ConserveFuelBehavior** - Fuel-saving mode
9. **TrafficManagementBehavior** - Smart traffic handling
10. **RaceStartBehavior** - Race start procedure

### 7. **Racing Actions** ✅

Implemented 11 racing-specific actions:

1. **DRSAction** - Drag Reduction System
2. **ERSDeployAction** - Energy Recovery deployment
3. **TractionControlAction** - TC adjustment
4. **BrakeBiasAction** - Brake balance
5. **DifferentialAction** - Diff settings
6. **PitLimiterAction** - Speed limiter
7. **FormationHoldAction** - Grid spacing
8. **OvertakeAction** - Overtaking maneuver
9. **DefendPositionAction** - Defensive positioning
10. **SlipstreamAction** - Drafting

## File Structure

```
src/scenic/domains/racing/
├── __init__.py              # Domain documentation & initialization
├── tracks.py                # RacingTrack, PitLane, Sector classes
├── model.scenic             # Racing objects, regions, utilities
├── behaviors.scenic         # Racing behaviors
├── actions.py               # Racing actions
├── README.md                # User documentation
└── OVERVIEW.md              # This file

src/scenic/simulators/dspace/
└── racing_model.scenic      # dSPACE+Racing integration

examples/dSPACE/
├── laguna_seca_race.scenic  # Example racing scenario
└── test_racing_domain.py    # Test script
```

## Architecture

### Inheritance Hierarchy

```
Scenic Core
    ↓
Driving Domain (roads, vehicles, driving behaviors)
    ↓
Racing Domain (tracks, racing cars, racing behaviors)
    ↓
dSPACE Racing Model (simulator integration)
```

### Class Relationships

```
Network (driving)
    ↓
RacingTrack (racing)
    ├── PitLane
    ├── Sector (multiple)
    └── RacingLine

Vehicle (driving)
    ↓
RacingCar (racing)
    ├── FormulaCar
    ├── GTCar
    └── PrototypeCar
```

## Usage Examples

### Basic Grid Start

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.simulators.dspace.racing_model

# Cars automatically placed on grid
ego = new RacingCar at startingGrid[0]
opponent1 = new RacingCar at startingGrid[1]
opponent2 = new RacingCar at startingGrid[2]
```

### With Behaviors

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
model scenic.simulators.dspace.racing_model

# Ego on grid with racing behavior
ego = new RacingCar at startingGrid[0], \
    with behavior FollowRacingLineBehavior(target_speed=30)

# Opponent with traffic management
opponent = new RacingCar at startingGrid[1], \
    with behavior TrafficManagementBehavior()
```

### Pit Stop Scenario

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
model scenic.simulators.dspace.racing_model

# Car on track with low fuel
ego = new RacingCar on road, \
    with fuelLevel 0.15, \
    with behavior PitStopBehavior(duration=25)
```

## Integration with dSPACE

The racing domain integrates seamlessly with dSPACE:

1. **Model Import**: Use `scenic.simulators.dspace.racing_model`
2. **Coordinate System**: Inherits driving domain's (x,y) → (s,t) transformation
3. **Ego/Fellow Handling**: Racing cars properly split between Maneuver and Fellows
4. **Track Direction**: Ensures cars face correct direction (VehicleOrientation=0)

## Testing

Run the test script:

```bash
python examples/dSPACE/test_racing_domain.py
```

Expected output:
- ✓ Scenario compiles
- ✓ Racing cars generated on grid
- ✓ Racing properties verified
- ✓ dSPACE simulation configured

## Differences from Driving Domain

| Feature | Driving Domain | Racing Domain |
|---------|----------------|---------------|
| **Traffic** | Bidirectional, intersections | One-way circuit |
| **Focus** | Safety, navigation | Speed, performance |
| **Lanes** | Regular roads | Racing line + pit lane |
| **Start** | Anywhere on road | Starting grid |
| **Properties** | Basic vehicle | Fuel, tires, race number |
| **Behaviors** | Lane following | Overtaking, pit stops |

## Implementation Notes

### What Works Now

- ✅ Track direction enforcement
- ✅ Starting grid generation
- ✅ Racing car objects with proper properties
- ✅ Comprehensive behavior library
- ✅ Racing-specific actions
- ✅ dSPACE integration
- ✅ Sector-based organization

### Future Enhancements

These features are documented but not yet fully implemented:

- 🔄 Automatic pit lane detection from OpenDRIVE
- 🔄 Racing line calculation from geometry
- 🔄 DRS zones (specific track regions)
- 🔄 Track limits detection
- 🔄 Tire temperature simulation
- 🔄 Weather conditions
- 🔄 Safety car behavior
- 🔄 Flag system (yellow, red, blue)

## API Reference

### Track Methods

```python
track.distanceAlongTrack(position) → float
track.getSectorAt(position) → Sector
track.isOnPitLane(position) → bool
track.enforceTrackDirection(heading, position) → bool
track.generateStartingGrid(numPositions, spacing) → List[Vector]
```

### Utility Functions

```python
carsInFormation(positions, carType) → List[Car]
isOnRacingLine(car, tolerance) → bool
distanceToSectorEnd(car) → float
carsAheadInSector(car, sector) → List[Car]
```

### Global Parameters

```python
param trackDirection = 'counterclockwise'  # or 'clockwise'
param generateStartingGrid = True
param startingGridPositions = 20
param startingGridSpacing = 8.0  # meters
```

## Conclusion

The racing domain successfully extends the driving domain with comprehensive racing features while maintaining compatibility with the dSPACE simulator and other Scenic simulators. The implementation provides a solid foundation for creating realistic racing scenarios with proper track structure, starting grids, pit stops, and racing behaviors.

