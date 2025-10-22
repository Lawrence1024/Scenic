# Racing Domain Library Reference

## Overview

The Scenic Racing Domain (`@racing/`) extends the Driving Domain (`@driving/`) with racing-specific objects, behaviors, and actions. This document provides a comprehensive reference of all racing-specific components that are **additional** to the driving domain.

## Table of Contents

1. [Racing Objects](#racing-objects)
2. [Racing Actions](#racing-actions)
3. [Racing Behaviors](#racing-behaviors)
4. [Racing Regions](#racing-regions)
5. [Racing Protocols](#racing-protocols)
6. [Racing Track Features](#racing-track-features)
7. [Global Parameters](#global-parameters)
8. [File Structure](#file-structure)

---

## Racing Objects

### `RacingCar`
**Location**: `src/scenic/domains/racing/model.scenic`

**Inheritance**: `Car` (from driving domain)

**Additional Properties (defaults)**:
- `carType`: "Racing Car"
- `position`: on `mainRacingRoad`
- `ttl`: default `racingLine` (target line to drive on)
- `maxSpeed`: numeric (m/s)

**Minimal Racing API** (implemented by simulators or stored as properties):
```python
def setMaxSpeed(self, max_speed: float) -> None: ...
def setTTL(self, ttl) -> None: ...  # ttl is a Region-like line with signedDistanceTo
```

### `RacingTrack`
**Location**: `src/scenic/domains/racing/model.scenic`

**Purpose**: Central racing track object that manages all track features

**Key Properties**:
- `network`: The underlying road network
- `pitLane`: Region representing pit lane
- `racingLine`: Region representing optimal racing line
- `mainRacingRoad`: Region representing main racing track
- `pitLaneRoad`: Region representing pit lane road
- `sectors`: List of track sectors
- `startingGrid`: Optional starting grid positions

---

## Racing Actions

**Location**: `src/scenic/domains/racing/actions.py`

The racing action surface is intentionally minimal:

### `SetMaxSpeedAction`
Set the maximum allowed speed (m/s) for a racing car.

```python
class SetMaxSpeedAction(Action):
    def __init__(self, max_speed: float): ...
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setMaxSpeed'): obj.setMaxSpeed(max_speed)
        else: obj.maxSpeed = max_speed
```

Usage: `take SetMaxSpeedAction(35)`

### `SetTTLAction`
Set the car's TTL (target line to drive on). The TTL is a Region-like object
supporting `signedDistanceTo`, e.g., a lane centerline or racing line.

```python
class SetTTLAction(Action):
    def __init__(self, ttl): ...
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setTTL'): obj.setTTL(ttl)
        else: obj.ttl = ttl
```

Usage: `take SetTTLAction(racingLine)`

---

## dSPACE Example (Implementation Notes)

**Location**: `src/scenic/simulators/dspace/racing_model.scenic`, `simulator.py`

- `DSPACERacingCar` implements:
  - `setMaxSpeed(max_speed)`: stores and forwards to control API
  - `setTTL(ttl)`: stores TTL for controllers/behaviors
- Route assignment (ModelDesk):
  - Detects segment via projection and compares to `pitLaneRoadIds` / `mainRacingRoadIds`
  - Maps segment to route preference: Pit → Route0, Lap → Route1
  - Activates the matching route (with fallbacks if COM index/name activation fails)
- Ego uses the same route selection path as fellows (via `_set_fellow_route_via_sequence`).

---

## Racing Behaviors

**Location**: `src/scenic/domains/racing/behaviors.scenic`

### `FollowRacingLineBehavior`
**Purpose**: Follow the car's TTL (target line) using controllers, respecting max speed

```scenic
behavior FollowRacingLineBehavior(target_speed=30):
    # Ensure TTL/max speed set; default TTL is racingLine or mainRacingRoad
    if not hasattr(self, 'ttl') or self.ttl is None:
        take SetTTLAction(racingLine)
    take SetMaxSpeedAction(target_speed)
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    past_steer_angle = 0
    while True:
        current_speed = (self.speed if self.speed is not None else 0)
        line = (self.ttl if hasattr(self, 'ttl') and self.ttl is not None else racingLine)
        cte = line.signedDistanceTo(self.position)
        speed_error = min(self.maxSpeed, target_speed) - current_speed
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer
```

**Usage**: `do FollowRacingLineBehavior(target_speed=35)`

### `PitStopBehavior`
**Purpose**: Execute a complete pit stop sequence

```scenic
behavior PitStopBehavior():
    """Execute a pit stop using racing-specific systems."""
    
    # Enter pit lane with speed limiter
    take PitLimiterAction(activate=True)
    do FollowRacingLineBehavior(target_speed=20)
    
    # Stop for pit stop
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take PitLimiterAction(activate=False)
```

**Usage**: `do PitStopBehavior()`

### `OvertakingBehavior`
**Purpose**: Execute overtaking maneuvers using racing systems

```scenic
behavior OvertakingBehavior(target_car, aggressive=False):
    """Attempt to overtake target car using racing systems.
    
    Args:
        target_car: The car to overtake
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    # Close the gap
    while (distance from self to target_car) > 5:
        do FollowRacingLineBehavior(target_speed=35)
    
    # Execute overtake with racing systems
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)
        take DRSAction(activate=True)
    
    # Move to side and accelerate
    take SetThrottleAction(1.0)
    
    # Complete overtake
    do FollowRacingLineBehavior() until (distance from self to target_car) > 10
    
    # Return to racing line
    do FollowRacingLineBehavior()
```

**Usage**: `do OvertakingBehavior(opponent_car, aggressive=True)`

### `DefensiveBehavior`
**Purpose**: Defend position using racing-specific systems

```scenic
behavior DefensiveBehavior():
    """Defend position using racing-specific systems."""
    
    # Adjust racing systems for defense
    take TractionControlAction(level=8)  # More conservative TC
    take BrakeBiasAction(bias=0.6)  # More front bias for stability
    
    # Follow racing line defensively
    do FollowRacingLineBehavior(target_speed=25)
```

**Usage**: `do DefensiveBehavior()`

---

## Racing Regions

**Location**: `src/scenic/domains/racing/model.scenic`

There are exactly three regions of interest:

- `road`: Entire drivable surface (from driving domain)
- `pitLaneRoad`: Region for pit lane lanes
- `mainRacingRoad`: Complement of `pitLaneRoad` in `road`

Invariant: `mainRacingRoad` and `pitLaneRoad` are mutually exclusive and
`mainRacingRoad ∪ pitLaneRoad = road`.

---

## Racing Track Features

**Location**: `src/scenic/domains/racing/tracks.py`

### `Sector`
**Purpose**: Represents track sectors for timing and analysis

```python
class Sector:
    def __init__(self, start_s: float, end_s: float, name: str = None):
        self.start_s = start_s
        self.end_s = end_s
        self.name = name
```

### `PitLane`
**Purpose**: Represents pit lane features

```python
class PitLane:
    def __init__(self, road: Road):
        self.road = road
        self.pit_boxes = []  # List of pit box regions
```

### `RacingLine`
**Purpose**: Represents the optimal racing line

```python
class RacingLine:
    def __init__(self, track: 'RacingTrack'):
        self.track = track
        self.points = []  # Racing line points
```

### `RacingTrack`
**Purpose**: Main track management class

```python
class RacingTrack:
    def __init__(self, network: Network, **kwargs):
        self.network = network
        self.pit_lane = None
        self.racing_line = None
        self.sectors = []
        self.starting_grid = None
    
    def calculate_track_length(self) -> float: ...
    def identify_track_features(self) -> None: ...
    def generate_starting_grid(self, num_cars: int) -> List[Point]: ...
```

---

## Global Parameters

**Location**: `src/scenic/domains/racing/model.scenic`

### Track Configuration
```scenic
param trackDirection = 'clockwise'  # or 'counterclockwise'
param generateStartingGrid = True
param pitLaneRoadId = None  # Auto-detected if None
param racingLineOffset = 0.0  # Offset from center line
```

### Track Analysis
```scenic
param sectorCount = 3  # Number of sectors to generate
param enableDRS = True  # Enable DRS zones
param enableERS = True  # Enable ERS deployment
```

---

## File Structure

```
src/scenic/domains/racing/
├── __init__.py              # Domain initialization
├── README.md               # Domain overview and usage
├── OVERVIEW.md             # Detailed domain overview
├── model.scenic            # Racing world model
├── actions.py              # Racing-specific actions
├── behaviors.scenic        # Racing-specific behaviors
├── tracks.py               # Track analysis and management
├── simulators.py           # Racing simulator interfaces
└── __pycache__/            # Python cache files
```

---

## Key Differences from Driving Domain

### Objects
- **Driving**: `Car`, `NPCCar`, `Pedestrian`
- **Racing**: `RacingCar` (extends `Car` with racing systems)

### Actions
- **Driving**: Basic vehicle control (`SetThrottleAction`, `SetSteerAction`, etc.)
- **Racing**: Racing-specific systems (`DRSAction`, `ERSDeployAction`, `PitLimiterAction`, etc.)

### Behaviors
- **Driving**: `FollowLaneBehavior`, `DriveAvoidingCollisions`, etc.
- **Racing**: `FollowRacingLineBehavior`, `PitStopBehavior`, `OvertakingBehavior`, etc.

### Regions
- **Driving**: `road`, `curb`, `sidewalk`, `intersection`
- **Racing**: `pitLane`, `racingLine`, `mainRacingRoad`, `pitLaneRoad`

### Track Features
- **Driving**: Basic road network
- **Racing**: Sectors, pit lanes, racing lines, starting grids

---

## Usage Examples

### Basic Racing Scenario
```scenic
model scenic.domains.racing.model

# Create racing cars
ego = new RacingCar on mainRacingRoad
opponent = new RacingCar ahead of ego by 50

# Assign racing behaviors
ego.behavior = FollowRacingLineBehavior(target_speed=35)
opponent.behavior = DefensiveBehavior()
```

### Pit Stop Scenario
```scenic
# Create car that needs to pit
racing_car = new RacingCar on mainRacingRoad
racing_car.behavior = PitStopBehavior()
```

### Overtaking Scenario
```scenic
# Create cars for overtaking
leader = new RacingCar on mainRacingRoad
chaser = new RacingCar behind leader by 20

# Assign overtaking behavior
chaser.behavior = OvertakingBehavior(leader, aggressive=True)
```

---

## Simulator Implementation Requirements

To implement the racing domain, simulators must:

1. **Implement `RacingSteers` Protocol**: Provide concrete implementations for all racing system methods
2. **Provide Racing Controllers**: Implement `getRacingControllers()`, `getRacingLineControllers()`, `getPitLaneControllers()`
3. **Support Track Detection**: Implement `detectTrackSegment()` and `assignRoute()`
4. **Handle Racing Actions**: Process all racing-specific actions correctly

See `src/scenic/simulators/dspace/racing_model.scenic` for a complete implementation example.

---

## Summary

The Racing Domain provides a comprehensive set of racing-specific components that extend the Driving Domain:

- **1 New Object Type**: `RacingCar` with racing systems
- **5 New Action Types**: DRS, ERS, Pit Limiter, Traction Control, Brake Bias
- **4 New Behaviors**: Racing Line Following, Pit Stops, Overtaking, Defense
- **4 New Regions**: Pit Lane, Racing Line, Main Racing Road, Pit Lane Road
- **Multiple Track Features**: Sectors, Pit Lanes, Racing Lines, Starting Grids

This architecture enables simulator-independent racing scenarios while providing rich racing-specific functionality through abstract protocols that simulators must implement.
