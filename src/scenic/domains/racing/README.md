# Racing Domain - Complete Reference

## Overview

The Scenic Racing Domain (`@racing/`) extends the Driving Domain (`@driving/`) with racing-specific objects, behaviors, and actions for closed-circuit race tracks. This document provides a comprehensive reference to all racing-specific components that are **additional** to the driving domain.

### Key Features

- **Closed-loop circuits** with defined direction (clockwise/counterclockwise)
- **Pit lanes** separate from racing lanes with automatic detection
- **Racing controllers:** PID (driving domain) or **MPC** (MPCC lateral + longitudinal) via `getRacingControllers(agent, use_mpc=True)`
- **Waypoint-based racing line** with segment maps and TTL loading (`segments/`, `mpc/`)
- **Minimal but extensible API** that simulators can implement

## Table of Contents

1. [Quick Start](#quick-start)
2. [Racing Objects](#racing-objects)
3. [Racing Actions](#racing-actions)
4. [Racing Behaviors](#racing-behaviors)
5. [Racing Regions](#racing-regions)
6. [Racing Track Features](#racing-track-features)
7. [Global Parameters](#global-parameters)
8. [Architecture](#architecture)
9. [Usage Examples](#usage-examples)
10. [Implementation Status](#implementation-status)
11. [API Reference](#api-reference)
12. [Simulator Implementation](#simulator-implementation)
13. [Control contract](#control-contract)

---

## Quick Start

### Basic Setup

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.domains.racing.model

# Create cars on track
ego = new RacingCar on mainTrack
opponent = new RacingCar on mainTrack
```

### With Behaviors

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
model scenic.domains.racing.model

ego = new RacingCar on mainTrack, \
    with behavior FollowRacingLineMPCBehavior(target_speed=30)
```

---

## Racing Objects

### `RacingCar`

**Location**: `src/scenic/domains/racing/model.scenic`

**Inheritance**: `Car` (from driving domain)

**Additional Properties (defaults)**:
- `carType`: "Racing Car"
- `position`: on `mainRacingRoad`
- `ttl`: default `racingLine` (target line to drive on)
- `speed`: 25 m/s (higher default for racing)
- `maxSpeed`: 30.0 m/s (~108 km/h)

**Racing Identification**:
- `raceNumber`: Range(1, 999) - Car number for identification
- `team`: str | None - Team name or identifier
- `carType`: str - Vehicle type for reference

**Performance Characteristics** (configurable):
- `maxSpeed`: float - Maximum speed in m/s (default: 30.0)
- `acceleration`: float - Acceleration capability in m/s² (default: 8.0)
- `braking`: float - Braking capability in m/s² (default: -12.0)

**State Properties**:
- `fuelLevel`: Range(0.0, 1.0), default Range(0.5, 1.0)
- `tireWear`: Range(0.0, 1.0), default 0.0 (0=new, 1=worn)

**Autonomous Capabilities**:
- `waypointTolerance`: float - Distance tolerance for waypoint following (default: 2.5)
- `controllerAggressiveness`: Range(0.0, 1.0) - Controller aggressiveness (default: 0.5)

**Minimal Racing API** (implemented by simulators or stored as properties):
```python
def setMaxSpeed(self, max_speed: float) -> None: ...
def setTTL(self, ttl) -> None: ...  # ttl is a Region-like line with signedDistanceTo
```

**Note**: Specialized car types (FormulaCar, GTCar, PrototypeCar) are **not** implemented in the base domain. Simulators may extend `RacingCar` to provide these.

### `RacingTrack`

**Location**: `src/scenic/domains/racing/tracks.py`

**Purpose**: Central racing track object that manages all track features

**Key Properties**:
- `network`: The underlying road Network
- `direction`: 'clockwise' or 'counterclockwise'
- `pitLane`: Optional `PitLane` object (if pit lane detected)
- `pitLaneRoad`: Optional Road object for pit lane
- `mainRacingRoad`: Union of all non-pit roads
- `racingLine`: Optional `RacingLine` object (defaults to `mainRacingRoad`)
- `trackLength`: Total track length in meters

---

## Racing Actions

**Location**: `src/scenic/domains/racing/actions.py`

The racing action surface is intentionally minimal, focusing on core functionality:

### `SetMaxSpeedAction`

Set the maximum allowed speed (m/s) for a racing car.

```python
class SetMaxSpeedAction(Action):
    def __init__(self, max_speed: float): ...
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setMaxSpeed'): 
            obj.setMaxSpeed(max_speed)
        else: 
            obj.maxSpeed = max_speed
```

**Usage**: `take SetMaxSpeedAction(35)`

### `SetTTLAction`

Set the car's TTL (target line to drive on). The TTL is a Region-like object supporting `signedDistanceTo`, such as a lane centerline or racing line.

```python
class SetTTLAction(Action):
    def __init__(self, ttl): ...
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setTTL'): 
            obj.setTTL(ttl)
        else: 
            obj.ttl = ttl
```

**Usage**: `take SetTTLAction(racingLine)`

### `SetGearAction`

Set gear to a specific value (0-6). Racing domain action for manual transmission control.

```python
class SetGearAction(Action):
    def __init__(self, gear: int): ...
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setGear'): 
            obj.setGear(gear)
        else: 
            obj.gear = gear
```

- `gear`: 0 = Neutral, 1-6 = Gears
- **Note**: For starting from neutral (gear 0 → gear 1), use `PressClutchAction` first

**Usage**: `take SetGearAction(2)`

### `PressClutchAction`

Press clutch pedal (one-shot action).

**Primary use case**: Starting from neutral (gear 0 → gear 1)
- Press clutch when in neutral
- Use `SetGearAction(1)` to engage 1st gear
- Release clutch to start moving

**Usage**: `take PressClutchAction()`

### `ReleaseClutchAction`

Release clutch pedal (one-shot action).

**Primary use case**: Completing the start from neutral
- After pressing clutch and engaging 1st gear
- Release clutch to begin moving

**Usage**: `take ReleaseClutchAction()`

### `HasManualTransmission` Protocol

Mixin protocol for agents with manual transmission control. Simulators should implement these methods:

```python
class HasManualTransmission:
    def setGear(self, gear: int) -> None: ...
    def setClutch(self, clutch: float) -> None: ...  # 0.0=released, 1.0=pressed
```

### Actions Referenced But Not Implemented

The following actions are referenced in behaviors but are **not** implemented in `actions.py`. Simulators may need to provide their own implementations:

- `PitLimiterAction` - Speed limiter for pit lane
- `DRSAction` - Drag Reduction System
- `ERSDeployAction` - Energy Recovery System deployment
- `TractionControlAction` - Traction control adjustment
- `BrakeBiasAction` - Brake balance adjustment
- `DifferentialAction` - Differential settings

---

## Racing Behaviors

**Location**: `src/scenic/domains/racing/behaviors.scenic`

### `FollowRacingLineMPCBehavior`

**Purpose**: Primary racing-line behavior: follow the car's TTL using **MPC** for lateral control (MPCC) and longitudinal control, with an opponent-aware tactical intelligence pipeline that chooses among `optimal`, `left`, and `right` TTLs.

**Implementation**: Uses `getRacingControllers(self, use_mpc=True, mpc_config_path=...)` to obtain `MPCLateralController` and `MPCLongitudinalController`. CTE and reference are computed from waypoints. Supports gear management and optional custom MPC config path.

**Signature**:
```scenic
FollowRacingLineMPCBehavior(
    target_speed=30,
    manage_gears=True,
    use_waypoints=True,
    mpc_config_path=None,
    planner_enabled=False,          # Phase 1: scripted TTL schedule
    ttl_schedule=None,
    target_speed_cap=None,
    tactical_planner_enabled=False, # enables 4-layer intelligence pipeline
    prediction_enabled=False,         # log [Prediction] per cycle
    assessment_enabled=False,         # log [Assessment] per cycle
    stability_guard_enabled=False,    # stability guard + [Guard] telemetry
    commit_abort_enabled=False,       # COMMIT_PASS / ABORT_PASS states
    segment_aware_enabled=False,      # segment-conditioned commit gating
)
```

**Behavior parameters can also be set via `simulation().scene.params`** using the semantic keys `assessment_enabled`, `stability_guard_enabled`, `commit_abort_enabled`, `segment_aware_enabled`.

**Details**: See `mpc/README.md` for formulation, configuration, and integration.

### `PitStopBehavior`

**Purpose**: Execute a pit stop sequence.

**Implementation**:
```scenic
behavior PitStopBehavior():
    """Execute a pit stop using racing-specific systems."""
    
    # Enter pit lane with speed limiter
    take PitLimiterAction(activate=True)  # ⚠️ Not implemented - simulator must provide
    do FollowRacingLineMPCBehavior(target_speed=20)
    
    # Stop for pit stop
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take PitLimiterAction(activate=False)  # ⚠️ Not implemented - simulator must provide
```

**Usage**: `do PitStopBehavior()`

**Note**: This behavior references `PitLimiterAction` which is not implemented in the base domain. Simulators must provide their own implementation.

### `OvertakingBehavior`

**Purpose**: Execute overtaking maneuvers using racing systems.

**Implementation**:
```scenic
behavior OvertakingBehavior(target_car, aggressive=False):
    """Attempt to overtake target car using racing systems.
    
    Args:
        target_car: The car to overtake
        aggressive: If True, use all available systems (DRS, ERS)
    """
    
    # Close the gap
    while (distance from self to target_car) > 5:
        do FollowRacingLineMPCBehavior(target_speed=35)
    
    # Execute overtake with racing systems
    if aggressive:
        take ERSDeployAction(mode='overtake', amount=1.0)  # ⚠️ Not implemented
        take DRSAction(activate=True)  # ⚠️ Not implemented
    
    # Move to side and accelerate
    take SetThrottleAction(1.0)
    
    # Complete overtake
    do FollowRacingLineMPCBehavior() until (distance from self to target_car) > 10
    
    # Return to racing line
    do FollowRacingLineMPCBehavior()
```

**Usage**: `do OvertakingBehavior(opponent_car, aggressive=True)`

**Note**: This behavior references `ERSDeployAction` and `DRSAction` which are not implemented in the base domain.

### `DefensiveBehavior`

**Purpose**: Defend position using racing-specific systems.

**Implementation**:
```scenic
behavior DefensiveBehavior():
    """Defend position using racing-specific systems."""
    
    # Adjust racing systems for defense
    take TractionControlAction(level=8)  # ⚠️ Not implemented
    take BrakeBiasAction(bias=0.6)  # ⚠️ Not implemented
    
    # Follow racing line defensively
    do FollowRacingLineMPCBehavior(target_speed=25)
```

**Usage**: `do DefensiveBehavior()`

**Note**: This behavior references `TractionControlAction` and `BrakeBiasAction` which are not implemented in the base domain.

---

## Racing Regions

**Location**: `src/scenic/domains/racing/model.scenic`

There are exactly three regions of interest:

- `road`: Entire drivable surface (from driving domain)
- `pitLaneRoad`: Region for pit lane lanes (if pit lane detected)
- `mainRacingRoad`: Complement of `pitLaneRoad` in `road`

**Invariant**: `mainRacingRoad` and `pitLaneRoad` are mutually exclusive and `mainRacingRoad ∪ pitLaneRoad = road`.

The `racingLine` region defaults to `mainRacingRoad` if no explicit racing line is defined.

---

## Racing Track Features

**Location**: `src/scenic/domains/racing/tracks.py`

### `PitLane`

Represents pit lane features with speed limits and pit boxes.

```python
@attr.s(auto_attribs=True, kw_only=True, eq=False)
class PitLane:
    lane: Lane
    speedLimit: float = 22.0  # ~80 km/h default
    entryPoint: Optional[Vector] = None
    exitPoint: Optional[Vector] = None
    pitBoxes: List[PolygonalRegion] = []
    
    @property
    def region(self) -> PolygonalRegion:
        return self.lane.polygon
```

### `RacingLine`

Represents the optimal racing line through a section of track.

```python
@attr.s(auto_attribs=True, kw_only=True, eq=False)
class RacingLine:
    path: PolylineRegion
    section: str = "general"
    speedProfile: List[Tuple[float, float]] = []
```

### `RacingTrack`

Main track management class extending the driving domain's Network.

**Key Methods**:
- `isOnPitLane(position) → bool` - Check if position is on pit lane
- `enforceTrackDirection(heading, position) → bool` - Validate heading matches track direction
**Initialization**:
```python
track = createRacingTrack(
    mapFile: str,
    direction: str = 'counterclockwise',
    pitLaneRoadId: Optional[str] = None,
    pitLaneRoadName: Optional[str] = None,
    mainLineRoadId: Optional[str] = None,
    **map_options
)
```

---

## Global Parameters

**Location**: `src/scenic/domains/racing/model.scenic`

### Track Configuration

```scenic
param trackDirection = 'counterclockwise'  # or 'clockwise'
```

### Track Segment Identification (Optional)

```scenic
param pitLaneRoadId = None  # OpenDRIVE road ID (e.g., "1545702203" for Laguna Seca)
param pitLaneRoadName = "pit"  # Pattern to match pit lane name
param mainLineRoadId = None  # OpenDRIVE road ID (e.g., "2117817291" for Laguna Seca)
```

---

## Architecture

### Inheritance Hierarchy

```
Scenic Core
    ↓
Driving Domain (roads, vehicles, driving behaviors)
    ↓
Racing Domain (tracks, racing cars, racing behaviors)
    ↓
Simulator-specific implementations (e.g., dSPACE racing model)
```

### Class Relationships

```
Network (driving)
    ↓
RacingTrack (racing)
    ├── PitLane
    └── RacingLine (optional, defaults to mainRacingRoad)

Vehicle (driving)
    ↓
RacingCar (racing)
    └── Simulator-specific extensions (e.g., DSPACERacingCar)
```

### Intelligence Pipeline (4 Layers)

When `tactical_planner_enabled=True`, `FollowRacingLineMPCBehavior` runs a 4-layer opponent-aware planning stack each control cycle:

```
Layer 1 — Perception   : situation_assessment.py
                          assess_nearest_opponent() → OpponentSituation
                          Log tag: [Phase2]

Layer 2 — Assessment   : assessment/race_situation.py
                          assess_race_situation()   → RaceSituationAssessment
                          Log tag: [Assessment]

Layer 3 — Planning     : tactical_planner.py
                          tactical_planner_step_v1() → (mode, ttl_key, speed_cap, reason)
                          Modes: FREE_RUN / FOLLOW / SETUP_LEFT / SETUP_RIGHT /
                                 COMMIT_PASS_LEFT / COMMIT_PASS_RIGHT / ABORT_PASS
                          Log tags: [Planner], [Commit], [Hazard]

Layer 4 — Safety       : safety/stability_guard.py
                          stability_guard_step()    → StabilityGuardDecision
                          Log tag: [Guard]
```

Prediction (Layer 0.5): `prediction/fellow_predictor.py` feeds `FellowPredictor` step results into the assessment layer. Log tag: `[Prediction]`.

### Design Principles

1. **Simulator Independence**: Abstract protocols that simulators implement
2. **Extensibility**: Easy to add simulator-specific features
3. **Compatibility**: Inherits all driving domain functionality
4. **Minimal Surface Area**: Core actions and behaviors only

---

## Usage Examples

### Basic Grid Start

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.domains.racing.model

# Cars automatically placed on grid
ego = new RacingCar on mainTrack
opponent1 = new RacingCar on mainTrack
opponent2 = new RacingCar on mainTrack
```

### With Behaviors

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
model scenic.domains.racing.model

# Ego on grid with racing behavior
ego = new RacingCar on mainTrack, \
    with behavior FollowRacingLineMPCBehavior(target_speed=30)

# Opponent with defensive behavior
opponent = new RacingCar on mainTrack, \
    with behavior DefensiveBehavior()
```

### Pit Stop Scenario

```scenic
param map = localPath('LagunaSeca.xodr')
param use2DMap = True
model scenic.domains.racing.model

# Car on track with low fuel
ego = new RacingCar on mainRacingRoad, \
    with fuelLevel 0.15, \
    with behavior PitStopBehavior()
```

**Note**: `PitStopBehavior` references `PitLimiterAction` which is not yet implemented in the base domain.

### Manual Transmission Control

```scenic
# Start from neutral
take PressClutchAction()
take SetGearAction(1)
take ReleaseClutchAction()

# Change gears while moving (no clutch needed)
take SetGearAction(2)
take SetGearAction(3)
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

## Implementation Status

### ✅ Fully Implemented

- Track direction enforcement
- Starting grid generation
- Racing car objects with proper properties
- Core racing behaviors: `FollowRacingLineMPCBehavior` (TTL line follow), `PitStopBehavior`, `OvertakingBehavior`, `DefensiveBehavior`, plus dSPACE fellow (v, d) plants and decision-tree helpers in `behaviors.scenic`
- 5 racing actions (max speed, TTL, gear/clutch)
- dSPACE integration (via `racing_model.scenic`)
- Pit lane identification (via road ID or name pattern)
- Racing simulator interface (`RacingSimulator`, `RacingSimulation`)
- Manual transmission protocol (`HasManualTransmission`)
- Racing controllers: **MPC** (lateral MPCC + longitudinal) when `getRacingControllers(agent, use_mpc=True)`; otherwise optimized PID from driving domain
- MPC module: MPCC lateral controller, longitudinal MPC, reference builder, speed profile, result_data analysis (see `mpc/README.md`)
- **Opponent-aware 4-layer intelligence pipeline**: Perception (`situation_assessment.py`) → Assessment (`assessment/race_situation.py`) → Planning (`tactical_planner.py`) → Safety (`safety/stability_guard.py`). Enabled via `tactical_planner_enabled=True`. Tactical modes: FREE_RUN / FOLLOW / SETUP_LEFT / SETUP_RIGHT / COMMIT_PASS_LEFT / COMMIT_PASS_RIGHT / ABORT_PASS. Asymmetric-opening detection correctly suppresses safety pressure when the fellow is on a parallel TTL (fixes braking in F3L/F3R scenarios). Status: `plans/README.md`.

### ⚠️ Partially Implemented

- **Behaviors reference missing actions**: `PitStopBehavior`, `OvertakingBehavior`, and `DefensiveBehavior` reference actions that are not implemented in `actions.py`:
  - `PitLimiterAction`
  - `DRSAction`
  - `ERSDeployAction`
  - `TractionControlAction`
  - `BrakeBiasAction`
  
  These behaviors will work only if simulators provide these actions.

- **Racing line**: Defaults to `mainRacingRoad` but explicit racing line calculation is not implemented.

### ❌ Not Implemented

These features are referenced in documentation or behaviors but are **not** in the codebase:

- Specialized car types: `FormulaCar`, `GTCar`, `PrototypeCar`
- Personnel objects: `PitCrew`, `TrackMarshal`
- Additional behaviors: `QualifyingLapBehavior`, `FormationLapBehavior`, `RaceStartBehavior`, `ConserveFuelBehavior`, `TrafficManagementBehavior`
- Racing system actions: `DRSAction`, `ERSDeployAction`, `TractionControlAction`, `BrakeBiasAction`, `DifferentialAction`, `PitLimiterAction`
- Advanced racing actions: `FormationHoldAction`, `OvertakeAction`, `DefendPositionAction`, `SlipstreamAction`
- Automatic DRS zones
- Track limits detection
- Tire temperature simulation
- Weather conditions
- Safety car behavior
- Flag system (yellow, red, blue)

---

## API Reference

### Track Methods

```python
track.isOnPitLane(position: Vector) -> bool
track.enforceTrackDirection(heading: float, position: Vector) -> bool
```

### Utility Functions

```python
carsInFormation(positions: List) -> List[RacingCar]
```

### Racing Car Properties

```python
RacingCar:
    # Identification
    raceNumber: Range(1, 999)
    team: str | None
    carType: str  # default "Racing Car"
    
    # Performance
    maxSpeed: float  # m/s, default 30.0
    acceleration: float  # m/s², default 8.0
    braking: float  # m/s², default -12.0
    
    # State
    fuelLevel: Range(0.0, 1.0)  # default Range(0.5, 1.0)
    tireWear: Range(0.0, 1.0)  # default 0.0
    
    # Racing API
    ttl: Region  # Target line, default racingLine
    setMaxSpeed(max_speed: float) -> None
    setTTL(ttl: Region) -> None
```

### Racing Actions

```python
SetMaxSpeedAction(max_speed: float)
SetTTLAction(ttl: Region)
SetGearAction(gear: int)  # 0-6
PressClutchAction()
ReleaseClutchAction()
```

### Racing Behaviors

```scenic
FollowRacingLineMPCBehavior(
    target_speed=30, manage_gears=True, use_waypoints=True, mpc_config_path=None,
    tactical_planner_enabled=False,       # enables 4-layer intelligence pipeline
    prediction_enabled=False,             # [Prediction] logs per cycle
    assessment_enabled=False,             # [Assessment] logs per cycle
    stability_guard_enabled=False,        # stability guard + [Guard] logs
    commit_abort_enabled=False,           # COMMIT_PASS / ABORT_PASS states
    segment_aware_enabled=False,          # segment-conditioned commit gating
)
PitStopBehavior()  # May require simulator-specific PitLimiterAction
OvertakingBehavior(target_car, aggressive=False)  # May require simulator-specific actions
DefensiveBehavior()  # May require simulator-specific actions
```

### Racing Controllers (from `RacingSimulation`)

```python
getRacingControllers(agent, use_mpc=False, mpc_config_path=None)
#   -> Tuple[LongitudinalController, LateralController]
#   If use_mpc=True: (MPCLongitudinalController, MPCLateralController)
#   If use_mpc=False: (PIDLongitudinalController, PIDLateralController) from driving domain

getRacingLineControllers(agent) -> Tuple[PIDLongitudinalController, PIDLateralController]
getPitLaneControllers(agent) -> Tuple[PIDLongitudinalController, PIDLateralController]
getOvertakingControllers(agent) -> Tuple[PIDLongitudinalController, PIDLateralController]
```

---

## Simulator Implementation

To implement the racing domain, simulators must:

### 1. Extend Base Classes

```python
from scenic.domains.racing.simulators import RacingSimulator, RacingSimulation

class MyRacingSimulator(RacingSimulator):
    def createSimulation(self, ...):
        return MyRacingSimulation(...)

class MyRacingSimulation(RacingSimulation):
    # Implement required methods
    pass
```

### 2. Implement Racing Car Protocol

Extend `RacingCar` and implement required methods:

```python
class MyRacingCar(RacingCar):
    def setMaxSpeed(self, max_speed: float):
        # Store and forward to control API
        self.maxSpeed = max_speed
        self.simulator.setMaxSpeed(self, max_speed)
    
    def setTTL(self, ttl):
        # Store TTL for controllers/behaviors
        self.ttl = ttl
        # Optionally forward to simulator
```

### 3. Implement Manual Transmission (Optional)

If supporting gear/clutch actions:

```python
class MyRacingCar(RacingCar, HasManualTransmission):
    def setGear(self, gear: int):
        self.gear = gear
        self.simulator.setGear(self, gear)
    
    def setClutch(self, clutch: float):
        self.clutch = clutch
        self.simulator.setClutch(self, clutch)
```

### 4. Provide Racing Controllers

Implement or override controller methods:

```python
def getRacingControllers(self, agent, use_mpc=False, mpc_config_path=None):
    # If use_mpc=True: return (MPCLongitudinalController, MPCLateralController)
    # Else: return optimized PID controllers for racing from driving domain
    return lon_controller, lat_controller
```

### 5. Track Segment Detection and Route Assignment

Implement track segment detection and route mapping:

```python
def detectTrackSegment(self, position):
    # Detect if position is on pit lane or main racing road
    return 'pitLane' or 'mainRacing'

def assignRoute(self, agent, track_segment):
    # Map track segments to simulator routes
    if track_segment == 'pitLane':
        return 'Pit'  # or simulator-specific route name
    elif track_segment == 'mainRacing':
        return 'Lap'
```

### 6. Implement Missing Actions (Optional)

If behaviors need them, implement missing actions:

```python
# In simulator-specific code
class PitLimiterAction(Action):
    def applyTo(self, obj, sim):
        obj.pitLimiter = self.activate
        sim.setPitLimiter(obj, self.activate)
```

### dSPACE Example

See `src/scenic/simulators/dspace/racing_model.scenic` and `simulator.py` for a complete implementation:

- `DSPACERacingCar` implements `setMaxSpeed()` and `setTTL()`
- Route assignment via `pitLaneRoadIds` / `mainRacingRoadIds`
- Maps segments to routes: Pit → Route0, Lap → Route1
- Ego uses same route selection as fellows

---

## Control contract

Single contract for steering (and throttle/brake) across behavior, MPC, and dSPACE.

- **Steering units:** PID path: normalized [-1, 1]. MPC path: **road wheel angle in radians**. Simulator interprets `_control_state['steering']` using `agent._racing_steer_units` (set by `getRacingControllers(agent, use_mpc=...)`): `'rad'` or `'normalized'`.
- **Constants:** All limits in `scenic.domains.racing.constants` (`DELTA_MAX_RAD`, `THETA_SW_MAX_DEG`, `R`). Do not hardcode 0.2816 or 240 elsewhere.
- **dSPACE:** Rad → steering wheel deg only in `simulators/dspace/steer_io.py` via `road_rad_to_dspace_value`. Fellow physics expects [-1, 1]; when MPC, convert rad → normalized before physics.
- **SetSteerAction:** For racing MPC the behavior passes radians; simulators must use `_racing_steer_units`, not assume [-1, 1] only.

---

## File Structure

```
src/scenic/domains/racing/
├── __init__.py              # Domain documentation & initialization
├── tracks.py                # RacingTrack, PitLane, RacingLine classes
├── model.scenic             # Racing objects, regions, utilities
├── behaviors.scenic         # Racing behaviors (incl. FollowRacingLineMPCBehavior)
├── actions.py               # Racing actions
├── simulators.py            # Racing simulator interfaces (getRacingControllers with use_mpc)
├── situation_assessment.py  # Layer 1: assess_nearest_opponent() → OpponentSituation
├── tactical_planner.py      # Layer 3: tactical_planner_step_v1() → (mode, ttl, cap, reason)
│                            #   TacticalPlannerConfig, TacticalPlannerState, CommitPlannerState
├── README.md                # This file (complete reference)
├── assessment/              # Layer 2: race situation assessment
│   ├── race_situation.py    #   assess_race_situation() → RaceSituationAssessment
│   └── __init__.py
├── prediction/              # Fellow next-step pose predictor
│   ├── fellow_predictor.py  #   FellowPredictor, format_prediction_log_line
│   └── __init__.py
├── safety/                  # Layer 4: stability guard
│   ├── stability_guard.py   #   stability_guard_step() → StabilityGuardDecision
│   └── __init__.py
├── benchmarks/              # Benchmark runners and log analysis
│   ├── phase_run_common.py  #   Log parser (RE_PLANNER, RE_ASSESSMENT, RE_GUARD, RE_COMMIT …)
│   ├── f_scenario_bank.py   #   Scenario name banks per runner
│   ├── phase7_runner.py … phase12_runner.py
│   ├── parse_commit_metrics.py, analyze_racing_log.py
│   └── phase_run_common.py
├── mpc/                     # MPC/MPCC lateral + longitudinal controllers
│   ├── config.py, reference_builder.py, mpc_lateral.py, mpc_longitudinal.py
│   ├── speed_profile.py, io_adapter.py, utils.py, calibration.py
│   ├── vehicle_mpc.yaml, README.md
│   ├── result_data/         # Log analysis (README.md)
│   └── testing/             # Unit and integration tests (README.md)
└── segments/                # Segment map and racing-line utilities (README.md)

src/scenic/simulators/dspace/
└── racing_model.scenic      # dSPACE+Racing integration
```

---

## Key Differences from Driving Domain

| Feature | Driving Domain | Racing Domain |
|---------|----------------|---------------|
| **Traffic** | Bidirectional, intersections | One-way circuit |
| **Focus** | Safety, navigation | Speed, performance |
| **Lanes** | Regular roads | Racing line + pit lane |
| **Start** | Anywhere on road | Starting grid |
| **Properties** | Basic vehicle | Fuel, tires, race number |
| **Behaviors** | Lane following | Racing line, pit stops |
| **Actions** | Basic control | Max speed, TTL, gear/clutch |
| **Controllers** | Standard PID | Racing-optimized PID |

---

## Summary

The Racing Domain provides a **minimal but functional** foundation for racing scenarios:

- **1 Object Type**: `RacingCar` with racing systems
- **5 Actions**: Max speed, TTL, gear, clutch (press/release)
- **5 Behaviors**: Racing line following (PID), **racing line following with MPC**, pit stops, overtaking, defense
- **3 Regions**: Main racing road, pit lane road, racing line
- **Multiple Track Features**: Pit lanes, racing lines
- **MPC submodule**: MPCC lateral + longitudinal MPC, waypoint reference, speed profile, log analysis (`mpc/`, `segments/`)

The implementation is intentionally lean, focusing on core racing functionality that simulators can build upon. The domain supports both PID and MPC-based racing line following; see `mpc/README.md` for MPC details.

For questions or contributions, see the main Scenic documentation or contact the development team.
