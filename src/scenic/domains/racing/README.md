# Racing Domain - Complete Reference

> **Architecture / change log**
> - `docs/cleanup_inventory.md` — full deletion + rename map for the CC-* cycle.
> - `docs/frames.md` — coordinate frames, track elevation, frame calibration history.
> - `docs/racing_smart_driving.md` — opponent-aware planner improvements (SD-* cycle).
> - `docs/falsification_pipeline.md` — verifai-runner pipeline.
>
> **Canonical scenarios:** `examples/racing/f_shared/F0–F14` (17 scenarios incl. `F3L`, `F3R`, `F13c` variants) is the complete test set.
> All other historical scenario directories were removed in CC-2.
>
> **Phase numbering removed:** the old "Phase 0–12" tags throughout the codebase
> were renamed to descriptive prefixes in CC-3 (e.g. `_phase7_*` → `_prediction_*`,
> `_phase11_*` → `_commit_*`). See `docs/cleanup_inventory.md` for the full map.

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
param map = localPath('LGS_v1.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'
model scenic.domains.racing.model

# Create cars on track
ego = new RacingCar on mainTrack
opponent = new RacingCar on mainTrack
```

### With Behaviors

```scenic
param map = localPath('LGS_v1.xodr')
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

### `RacingSteers` Protocol

Extended protocol for simulator-facing racing controls (used by dSPACE decision-tree integration). Simulators implement these methods to handle `RacingAction` subclasses:

```python
class RacingSteers:
    def setSpeedLimit(self, speed_limit: float) -> None: ...
    def setTTLSelection(self, selection: str) -> None: ...  # "left"|"right"|"race"|"optimal"|"pit"
    def setTargetGap(self, gap: float) -> None: ...
    def setStrategy(self, strategy_type: str) -> None: ...  # "cruise_control"|"follow_mode"
    def setPowertrainMode(self, mode: str) -> None: ...  # "pit_lane"|"quiet"|"nominal"|"race"|"overboost"
    def setScaleFactor(self, scale_factor: float) -> None: ...  # 0.0–1.0
    def setPush2Pass(self, active: bool) -> None: ...
```

Actions backed by this protocol: `SetSpeedLimitAction`, `SetTTLSelectionAction`, `SetTargetGapAction`, `SetStrategyAction`, `SetPowertrainModeAction`, `SetScaleFactorAction`, `SetPush2PassAction`, `StopCarAction`.

### `HasFellowPlant` Protocol

Mixin protocol for traffic agents driven by route-relative speed and lateral offset (used for dSPACE fellow traffic control via `External_Signals`):

```python
class HasFellowPlant:
    def setFellowPlant(self, v_kmh: float, d_m: float) -> None: ...
    # v_kmh: longitudinal speed in km/h; d_m: lateral offset (Frenet t) in meters
```

Action: `SetFellowPlantAction(v_kmh, d_m)`. Plant state is mirrored in `agent._fellow_plant_state`.

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

### Other supported behaviors

In addition to `FollowRacingLineMPCBehavior`, the domain ships the following behaviors used by the F-bank and demo scenarios:

- **Fellow plant behaviors**: `FellowConstantSpeedTrackOffsetBehavior`, `FellowFollowTTLGeometricBehavior`, `FellowSuddenStopIntervalBehavior`, `FellowSwerveOutOfControlBehavior`, `FellowAlwaysFasterThanEgoBehavior`, `FellowActiveBlockBehavior` — drive non-ego traffic via route-relative `(v, d)` plant inputs.
- **Decision-tree behaviors**: `FlagBasedSpeedBehavior`, `LaneSelectionBehavior`, `StopBehavior`, `FollowModeBehavior`, `SimpleRaceBehavior`, `PitLaneBehavior`.
- **ART integration**: `ARTStackControlBehavior` hands ego control off to the external ART driving stack (for the S2-falsify ART-ego comparison).

See `behaviors.scenic` for full signatures.

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

### Intelligence Pipeline (post-SD-13: trajectory-prediction-driven)

When `tactical_planner_enabled=True` AND `prediction_enabled=True`,
`FollowRacingLineMPCBehavior` runs an opponent-aware planning stack each
control cycle. Post-SD-13 the planner is **strategy-driven** rather than
snapshot-driven — see `docs/racing_smart_driving.md` for the full SD-11/12/13
cycle history.

```
Layer 1 — Perception   : situation_assessment.py
                          assess_nearest_opponent() → OpponentSituation
                          (sit.delta_s_m / sit.lateral_m / sit.ahead now feed
                           ONLY the lifecycle, not entry decisions)
                          Log tag: [Phase2]

Layer 2 — Assessment   : assessment/race_situation.py
                          assess_race_situation()   → RaceSituationAssessment
                          Log tag: [Assessment]

Layer 3a — Trajectory   : prediction/fellow_predictor.py FellowPredictor.trajectory()
           Prediction      → multi-step CV-extrapolated fellow trajectory
                          Log tag: [Prediction]

Layer 3b — Strategy    : prediction/strategy_simulator.py simulate_strategy()
           Selection       × {stay_optimal, follow_fellow, pass_left, pass_right}
                          → 4 StrategyOutcomes (reachable_progress, min_clearance, ...)
                          Then planner/strategy_selector.py select_strategy()
                          → SelectedStrategy (the chosen plan + diagnostics)
                          Log tag: [Strategy]

Layer 3c — Planning    : tactical_planner.py tactical_planner_step_v1()
                          Strategy authority owns the entry decision
                          (FREE_RUN / FOLLOW / COMMIT_PASS_*); the lifecycle
                          (COMMIT_PASS_* → HOLD_PASS_* → FREE_RUN, with ABORT
                          on hard hazards) executes the chosen plan.
                          Modes: FREE_RUN / FOLLOW / COMMIT_PASS_{LEFT,RIGHT} /
                                 HOLD_PASS_{LEFT,RIGHT} / ABORT_PASS
                          (SETUP_LEFT / SETUP_RIGHT constants survive as
                          legacy aliases for _canonical_mode but are no
                          longer entered by any code path post-SD-13.)
                          Log tags: [Planner], [Commit], [Hazard]

Layer 4 — Safety       : safety/stability_guard.py
                          stability_guard_step()    → StabilityGuardDecision
                          Independent SD-4 emergency-brake authority via
                          path_collision_predicted (1.5s horizon). Two-key
                          safety: strategy commits, SD-4 vetoes.
                          Log tag: [Guard]
```

### Design Principles

1. **Simulator Independence**: Abstract protocols that simulators implement
2. **Extensibility**: Easy to add simulator-specific features
3. **Compatibility**: Inherits all driving domain functionality
4. **Minimal Surface Area**: Core actions and behaviors only

---

## Usage Examples

### Basic Grid Start

```scenic
param map = localPath('LGS_v1.xodr')
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
param map = localPath('LGS_v1.xodr')
param use2DMap = True
model scenic.domains.racing.model

# Ego on grid with racing behavior
ego = new RacingCar on mainTrack, \
    with behavior FollowRacingLineMPCBehavior(target_speed=30)

# Opponent with cruise-like behavior on the optimal TTL
opponent = new RacingCar on mainTrack, \
    with behavior FellowConstantSpeedTrackOffsetBehavior(speed_mph=130)
```

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

For opponent-aware overtaking, use `FollowRacingLineMPCBehavior` with the tactical pipeline enabled — it picks among `optimal`/`left`/`right` TTLs at runtime based on the predicted fellow trajectory:

```scenic
# Create cars for overtaking
leader = new RacingCar on mainRacingRoad, \
    with behavior FellowConstantSpeedTrackOffsetBehavior(speed_mph=130)
chaser = new RacingCar behind leader by 20, \
    with behavior FollowRacingLineMPCBehavior(
        target_speed=40,
        tactical_planner_enabled=True,
        prediction_enabled=True,
        assessment_enabled=True,
        commit_abort_enabled=True,
    )
```

See `examples/racing/f_shared/F1`..`F14` for the canonical overtaking scenarios used in the SD-44 regression baseline.

---

## Implementation Status

### ✅ Fully Implemented

- Track direction enforcement
- Starting grid generation
- Racing car objects with proper properties
- Core racing behavior: `FollowRacingLineMPCBehavior` (TTL line follow + 4-layer intelligence pipeline), plus dSPACE fellow (v, d) plants and decision-tree helpers in `behaviors.scenic`
- 5 behavior-facing racing actions (max speed, TTL, gear/clutch) + 9 simulator-protocol actions via `RacingSteers` and `HasFellowPlant`
- dSPACE integration (via `racing_model.scenic`)
- Pit lane identification (via road ID or name pattern)
- Racing simulator interface (`RacingSimulator`, `RacingSimulation`)
- Manual transmission protocol (`HasManualTransmission`)
- Racing controllers: **MPC** (lateral MPCC + longitudinal) when `getRacingControllers(agent, use_mpc=True)`; otherwise optimized PID from driving domain
- MPC module: MPCC lateral controller, longitudinal MPC, reference builder, speed profile, result_data analysis (see `mpc/README.md`)
- **Opponent-aware trajectory-prediction-driven planning** (post-SD-13): Perception (`situation_assessment.py`) → Assessment (`assessment/race_situation.py`) → Trajectory Prediction (`prediction/fellow_predictor.py`) → Strategy Selection (`prediction/strategy_simulator.py` + `planner/strategy_selector.py`) → Planning (`tactical_planner.py`) → Safety (`safety/stability_guard.py`). Enabled via `tactical_planner_enabled=True` + `prediction_enabled=True`. Tactical modes: FREE_RUN / FOLLOW / COMMIT_PASS_{LEFT,RIGHT} / HOLD_PASS_{LEFT,RIGHT} / ABORT_PASS. The strategy authority simulates each candidate (stay_optimal / follow_fellow / pass_left / pass_right) over a 10s horizon and picks the fastest safe one — replacing the legacy snapshot-driven SETUP entry chain. Two-key safety: strategy commits a plan, SD-4's 1.5s `path_collision_predicted` vetoes mid-flight if needed. Full cycle history: `docs/racing_smart_driving.md`.

### ❌ Not in supported surface

The following names appear in older Scenic-racing documentation but are **not part of this fork's supported API**. They were removed in the Phase E production-readiness cleanup (see commit history) because they were either stubs that referenced non-existent action classes, or simply documentation aspirational items never implemented:

- **Specialized car types**: `FormulaCar`, `GTCar`, `PrototypeCar`
- **Personnel objects**: `PitCrew`, `TrackMarshal`
- **Stub behaviors (removed)**: `PitStopBehavior`, `OvertakingBehavior`, `DefensiveBehavior` — for opponent-aware overtaking, use `FollowRacingLineMPCBehavior` with `tactical_planner_enabled=True` instead.
- **Racing-system actions (never implemented)**: `DRSAction`, `ERSDeployAction`, `TractionControlAction`, `BrakeBiasAction`, `DifferentialAction`, `PitLimiterAction`, `FormationHoldAction`, `OvertakeAction`, `DefendPositionAction`, `SlipstreamAction`
- **Aspirational behaviors**: `QualifyingLapBehavior`, `FormationLapBehavior`, `RaceStartBehavior`, `ConserveFuelBehavior`, `TrafficManagementBehavior`
- **Race-state features**: automatic DRS zones, track-limits detection, tire-temperature simulation, weather, safety car, flag system

If you need any of the above, treat them as new feature work — they have no existing skeleton in this repo.

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

**Behavior-facing (core):**
```python
SetMaxSpeedAction(max_speed: float)
SetTTLAction(ttl: Region)
SetGearAction(gear: int)  # 0-6
PressClutchAction()
ReleaseClutchAction()
```

**Simulator-protocol (`RacingSteers`, dSPACE decision-tree integration):**
```python
SetSpeedLimitAction(speed_limit: float, speed_type: str = None)
SetTTLSelectionAction(selection: str)   # "left"|"right"|"race"|"optimal"|"pit"
SetTargetGapAction(gap: float, gap_type: str = None)
SetStrategyAction(strategy_type: str)   # "cruise_control"|"follow_mode"
SetPowertrainModeAction(mode: str)      # "pit_lane"|"quiet"|"nominal"|"race"|"overboost"
SetScaleFactorAction(scale_factor: float)
SetPush2PassAction(active: bool)
StopCarAction(stop_type: str)           # "emergency"|"immediate"|"safe"
```

**Fellow plant (`HasFellowPlant`, dSPACE traffic control):**
```python
SetFellowPlantAction(v_kmh: float, d_m: float)
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

# Fellow (non-ego) plant-driven behaviors
FellowConstantSpeedTrackOffsetBehavior(speed_mph=31)
FellowFollowTTLGeometricBehavior(speed_mph=31)
FellowSuddenStopIntervalBehavior(speed_mph=150, interval=20.0, duration=3.0)
FellowSwerveOutOfControlBehavior(...)
FellowAlwaysFasterThanEgoBehavior(speed_offset_mph=10, ...)
FellowActiveBlockBehavior(speed_offset_mph=-5.0, ...)

# Decision-tree behaviors (race-state oriented)
FlagBasedSpeedBehavior(speed_type="green", speed_limit=None)
LaneSelectionBehavior(ttl_selection="race")
StopBehavior(stop_type="safe")
FollowModeBehavior(target_car, target_gap=31.0)
PitLaneBehavior(manage_gears=True)
SimpleRaceBehavior(...)

# ART (external stack) hand-off
ARTStackControlBehavior()
```

### Racing Controllers (from `RacingSimulation`)

```python
getRacingControllers(agent, use_mpc=False, mpc_config_path=None)
#   -> Tuple[LongitudinalController, LateralController]
#   If use_mpc=True: (MPCLongitudinalController, MPCLateralController)
#   If use_mpc=False: (PIDLongitudinalController, PIDLateralController) from driving domain

getRacingLineControllers(agent) -> Tuple[PIDLongitudinalController, PIDLateralController]
getPitLaneControllers(agent) -> Tuple[PIDLongitudinalController, PIDLateralController]
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

### 6. Add Custom Actions (Optional)

If a downstream backend needs additional racing actions beyond the supported set (max speed, TTL, gear/clutch, fellow plant), implement them as `Action` subclasses in simulator-specific code:

```python
# In simulator-specific code
class CustomBackendAction(Action):
    def __init__(self, value):
        self.value = value
    def applyTo(self, obj, sim):
        sim.setBackendSpecificField(obj, self.value)
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
├── tactical_planner.py      # Layer 3c: tactical_planner_step_v1() → (mode, ttl, cap, reason)
│                            #   TacticalPlannerConfig, TacticalPlannerState, CommitPlannerState
│                            #   Strategy authority + COMMIT/HOLD/ABORT lifecycle
├── README.md                # This file (complete reference)
├── assessment/              # Layer 2: race situation assessment + pass geometry
│   ├── race_situation.py    #   assess_race_situation() → RaceSituationAssessment
│   ├── pass_geometry.py     #   path_collision_predicted (SD-4 emergency brake),
│   │                        #   pass_window_check, _xy_at_arclength (cached)
│   └── __init__.py
├── prediction/              # Layer 3a: trajectory prediction + Layer 3b: strategy simulator
│   ├── fellow_predictor.py  #   FellowPredictor.step + .trajectory (CV multi-step)
│   ├── strategy_simulator.py#   simulate_strategy() per candidate over 10s horizon
│   └── __init__.py
├── planner/                 # Layer 3b: strategy selector
│   ├── strategy_selector.py #   select_strategy() — pure ranking function
│   └── __init__.py
├── safety/                  # Layer 4: stability guard
│   ├── stability_guard.py   #   stability_guard_step() → StabilityGuardDecision
│   └── __init__.py
├── benchmarks/              # Benchmark runners and log analysis
│   ├── phase_run_common.py  #   Shared framework: log parser, runner spec, digest emitter
│   ├── f_scenario_bank.py   #   F-scenario registry (F0..F14 + variants)
│   ├── metrics.py           #   SampleMetrics (used by verifai_runner)
│   ├── monitors.py          #   Robustness monitors (used by verifai_runner)
│   ├── full_stack_runner.py #   F-bank regression: all F-scenarios with the full smart-ego stack
│   └── verifai_runner.py    #   In-process falsification driver (Halton / CE / random samplers)
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
| **Behaviors** | Lane following | Racing line (PID + MPC + tactical pipeline), fellow plant |
| **Actions** | Basic control | Max speed, TTL, gear/clutch + `RacingSteers`/`HasFellowPlant` protocols |
| **Controllers** | Standard PID | Racing-optimized PID |

---

## Summary

The Racing Domain provides a **minimal but functional** foundation for racing scenarios:

- **1 Object Type**: `RacingCar` with racing systems
- **14 Actions**: 5 behavior-facing (max speed, TTL, gear, clutch press/release) + 8 `RacingSteers` protocol + 1 `HasFellowPlant` protocol
- **Behaviors**: racing-line following (PID and MPC with the 4-layer tactical pipeline), Fellow plant behaviors (Fellow{ConstantSpeedTrackOffset, FollowTTLGeometric, SuddenStopInterval, SwerveOutOfControl, AlwaysFasterThanEgo, ActiveBlock}), decision-tree helpers (FlagBasedSpeed, LaneSelection, StopBehavior, FollowMode, SimpleRaceBehavior, PitLaneBehavior), and ART hand-off (ARTStackControlBehavior)
- **3 Regions**: Main racing road, pit lane road, racing line
- **Multiple Track Features**: Pit lanes, racing lines
- **MPC submodule**: MPCC lateral + longitudinal MPC, waypoint reference, speed profile, log analysis (`mpc/`, `segments/`)

The implementation is intentionally lean, focusing on core racing functionality that simulators can build upon. The domain supports both PID and MPC-based racing line following; see `mpc/README.md` for MPC details.

For questions or contributions, see the main Scenic documentation or contact the development team.
