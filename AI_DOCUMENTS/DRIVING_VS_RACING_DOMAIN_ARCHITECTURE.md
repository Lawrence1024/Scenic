# Driving vs Racing Domain Architecture Guide

## Overview

This document explains how the **driving domain** (`scenic.domains.driving`) is structured, and how the **racing domain** (`scenic.domains.racing`) extends it. Understanding this architecture is essential for properly writing racing scenarios.

---

## Table of Contents

1. [Core Architecture Principles](#core-architecture-principles)
2. [File Structure Comparison](#file-structure-comparison)
3. [Key Components Deep Dive](#key-components-deep-dive)
4. [How Racing Extends Driving](#how-racing-extends-driving)
5. [Best Practices](#best-practices)

---

## Core Architecture Principles

### Driving Domain Philosophy

The driving domain is designed around these principles:

1. **Simulator Independence**: Scenarios written for the driving domain should work in ANY simulator (CARLA, LGSVL, MetaDrive, Newtonian)
2. **Road Network Foundation**: Everything is built on top of a `Network` object (from OpenDRIVE files)
3. **Semantic Regions**: Provides meaningful regions like `road`, `sidewalk`, `intersection`, `curb`
4. **Reusable Behaviors**: Common driving behaviors (lane following, turning) work across simulators
5. **Action Protocols**: Abstract actions (steering, throttling) that simulators implement

### Racing Domain Philosophy

The racing domain **extends** these principles:

1. **Inherits Everything**: All driving domain features are available
2. **Racing-Specific Extensions**: Adds track-specific concepts (pit lanes, racing lines, sectors)
3. **Maintains Compatibility**: Racing scenarios should work in any racing-capable simulator
4. **Track-Aware**: Understands closed-loop circuits, track direction, starting grids

---

## File Structure Comparison

### Driving Domain Structure

```
scenic/domains/driving/
├── __init__.py              # Domain documentation and description
├── model.scenic             # Main world model (objects, regions, behaviors)
├── roads.py                 # Road network infrastructure (Network, Road, Lane, etc.)
├── actions.py               # Driving-specific actions (Steers, Walks protocols)
├── behaviors.scenic         # Driving behaviors (FollowLaneBehavior, etc.)
├── controllers.py           # PID controllers for lane following
├── workspace.py             # Visualization workspace
└── simulators.py            # Base driving simulator interface
```

### Racing Domain Structure (Following Same Pattern)

```
scenic/domains/racing/
├── __init__.py              # Racing domain documentation
├── model.scenic             # Racing world model (extends driving.model)
├── tracks.py                # Racing track infrastructure (extends roads.py)
├── actions.py               # Racing-specific actions (DRS, ERS, etc.)
├── behaviors.scenic         # Racing behaviors (extends driving.behaviors)
└── simulators.py            # Base racing simulator interface
```

**Key Observation**: The structure mirrors the driving domain, making it easy to understand where functionality belongs.

---

## Key Components Deep Dive

### 1. `__init__.py` - Domain Description

**Purpose**: Document what the domain does and how to use it

**Driving Domain** (`driving/__init__.py`):
```python
"""Domain for driving scenarios.

This domain must currently be used in `2D compatibility mode`.

The world model defines Scenic classes for cars, pedestrians, etc.
Scenarios for the driving domain should import the model as follows::

    model scenic.domains.driving.model

Scenarios written for the driving domain should work without changes in:
    * MetaDrive
    * CARLA
    * LGSVL
    * Built-in Newtonian simulator
"""
```

**Racing Domain** (`racing/__init__.py`):
```python
"""Domain for racing scenarios on closed-circuit race tracks.

The racing domain extends the driving domain with racing-specific features:
    * Racing tracks - Closed-loop circuits
    * Pit lanes - Special lanes for pit stops
    * Sectors - Track divisions for timing
    * Racing lines - Optimal paths
    * Starting grid - Formation positions

Scenarios for the racing domain should import::

    model scenic.domains.racing.model

Racing scenarios inherit all features from the driving domain.
"""
```

**Key Takeaway**: The `__init__.py` serves as the "landing page" documentation for the domain.

---

### 2. `model.scenic` - The Heart of the Domain

This is where the domain's **world model** is defined. It includes:
- Global parameters
- Regions (road, sidewalk, etc.)
- Object classes (Car, Pedestrian, etc.)
- Utility functions

#### Driving Domain Model Structure

```scenic
"""Scenic world model for driving scenarios.

Map must be specified via 'map' parameter:
    param map = localPath('mymap.xodr')
    model scenic.domains.driving.model
"""

# 1. IMPORTS - Bring in infrastructure
from scenic.domains.driving.roads import Network, Road, Lane, ...
from scenic.domains.driving.actions import *
from scenic.domains.driving.behaviors import *

# 2. PARAMETERS - Configuration
param use2DMap = True
param map_options = {}

# 3. NETWORK - Load road network from map file
network: Network = Network.fromFile(globalParameters.map, **globalParameters.map_options)

# 4. WORKSPACE - For visualization
workspace = DrivingWorkspace(network)

# 5. REGIONS - Semantic regions derived from network
road: Region = network.drivableRegion
curb: Region = network.curbRegion
sidewalk: Region = network.sidewalkRegion
shoulder: Region = network.shoulderRegion
intersection: Region = network.intersectionRegion
roadDirection: VectorField = network.roadDirection

# 6. OBJECT CLASSES - Domain-specific objects
class DrivingObject:
    """Base class for all driving objects"""
    elevation[dynamic]: None if is2DMode() else float(self.position.z)
    
    @property
    def lane(self) -> Lane:
        return network.laneAt(self.position, reject='not in lane')
    
    @property
    def road(self) -> Road:
        return network.roadAt(self.position, reject='not on road')

class Vehicle(DrivingObject):
    """Vehicles which drive"""
    regionContainedIn: roadOrShoulder
    position: new Point on road
    parentOrientation: (roadDirection at self.position) + self.roadDeviation
    width: 2
    length: 4.5

class Car(Vehicle):
    """A car"""
    pass

class Pedestrian(DrivingObject):
    """A pedestrian"""
    regionContainedIn: network.walkableRegion
    position: new Point on network.walkableRegion
    width: 0.75
    length: 0.75

# 7. UTILITY FUNCTIONS
def withinDistanceToAnyCars(car, thresholdDistance):
    """Check if car is near other cars"""
    ...
```

#### Racing Domain Model Structure (Extending Driving)

```scenic
"""Scenic world model for racing scenarios.

Extends scenic.domains.driving.model with racing features.
"""

# 1. IMPORT EVERYTHING FROM DRIVING
from scenic.domains.driving.model import *
from scenic.domains.racing.tracks import RacingTrack, createRacingTrack
from scenic.domains.racing.behaviors import *

# 2. RACING-SPECIFIC PARAMETERS
param trackDirection = 'counterclockwise'
param generateStartingGrid = True
param pitLaneRoadId = None

# 3. RACING TRACK (extends Network)
track: RacingTrack = createRacingTrack(
    globalParameters.map,
    direction=globalParameters.trackDirection,
    pitLaneRoadId=globalParameters.pitLaneRoadId,
    ...
)

# Replace network with track's network
network = track.network

# 4. RACING-SPECIFIC REGIONS (extend driving regions)
pitLane: Region = track.pitLane.region if track.pitLane else nowhere
racingLine: Region = road.difference(pitLane)
mainRacingRoad: Region = track.mainRacingRoad
pitLaneRoad: Region = ...  # Pit lane as separate region

# 5. STARTING GRID
if globalParameters.generateStartingGrid:
    startingGrid = track.generateStartingGrid(...)

# 6. RACING-SPECIFIC OBJECT CLASSES (extend driving classes)
class RacingCar(Car):
    """A racing car (extends Car from driving domain)"""
    speed: 25  # Higher default speed
    position: new Point on racingLine  # Use racing line, not general road
    
    # Racing-specific properties
    raceNumber: Range(1, 999)
    maxSpeed: 30.0
    fuelLevel: Range(0.5, 1.0)
    tireWear: 0.0

# 7. RACING-SPECIFIC UTILITY FUNCTIONS
def carsInFormation(positions):
    """Create formation of racing cars"""
    ...

def distanceToSectorEnd(car):
    """Get distance to end of current sector"""
    ...
```

**Key Pattern**: Racing domain **imports everything** from driving domain with `from scenic.domains.driving.model import *`, then **adds** racing-specific features on top.

---

### 3. Infrastructure Layer (`roads.py` vs `tracks.py`)

#### Driving: `roads.py`

Provides the foundational infrastructure for road networks:

```python
"""Library for representing road network geometry.

A road network is represented by a Network class, created from map files.
"""

class Network:
    """A road network loaded from OpenDRIVE or similar format"""
    
    def __init__(self, roads, intersections, ...):
        self.roads = roads
        self.intersections = intersections
        self.drivableRegion = ...
        self.sidewalkRegion = ...
    
    @staticmethod
    def fromFile(path, **options):
        """Load network from file (OpenDRIVE, etc.)"""
        ...
    
    def laneAt(self, position, reject=None):
        """Get lane at given position"""
        ...
    
    def roadAt(self, position, reject=None):
        """Get road at given position"""
        ...

class Road:
    """A road in the network"""
    sections: List[LaneSection]
    lanes: List[Lane]
    
class Lane:
    """A lane within a road"""
    centerline: PolylineRegion
    polygon: PolygonalRegion
    
class Intersection:
    """An intersection between roads"""
    maneuvers: List[Maneuver]

class Maneuver:
    """A possible path through an intersection"""
    type: ManeuverType  # STRAIGHT, LEFT_TURN, RIGHT_TURN
    startLane: Lane
    endLane: Lane
    connectingLane: Optional[Lane]
```

#### Racing: `tracks.py` (Extends `roads.py`)

Adds racing-specific infrastructure:

```python
"""Racing track representation extending driving domain's road network."""

from scenic.domains.driving.roads import Network, Road, Lane, ...

class RacingTrack:
    """A racing track (closed-loop circuit).
    
    Extends Network with racing features:
    - Track direction (one-way)
    - Pit lane identification  
    - Sectors
    - Starting grid
    """
    
    def __init__(self, network: Network, direction='clockwise', ...):
        self.network = network  # Uses driving domain's Network!
        self.direction = direction
        self.pitLane: Optional[PitLane] = None
        self.sectors: List[Sector] = []
        self.startingGrid: List[Vector] = []
        
    def _identifyRacingFeatures(self):
        """Identify pit lanes, sectors from network"""
        # Analyzes the driving domain's Network to find racing features
        ...

class PitLane:
    """A pit lane on a racing track"""
    lane: Lane  # Uses driving domain's Lane!
    speedLimit: float = 22.0
    entryPoint: Optional[Vector] = None
    exitPoint: Optional[Vector] = None

class Sector:
    """A sector of a racing track for timing"""
    number: int
    startDistance: float
    endDistance: float
    region: PolygonalRegion

class RacingLine:
    """The optimal racing line through track"""
    path: PolylineRegion
    speedProfile: List[Tuple[float, float]]
```

**Key Pattern**: `RacingTrack` **wraps** the driving domain's `Network` and adds racing-specific analysis and features on top of it.

---

### 4. Actions (`actions.py`)

Actions define what agents can **do**. They use **protocol classes** (mixins) to define capabilities.

#### Driving Domain Actions

```python
"""Actions for dynamic agents in driving domain."""

from scenic.core.simulators import Action

# PROTOCOL CLASSES (Define capabilities)
class Steers:
    """Mixin for agents that can steer"""
    def setThrottle(self, throttle):
        raise NotImplementedError
    
    def setSteering(self, steering):
        raise NotImplementedError
    
    def setBraking(self, braking):
        raise NotImplementedError

class Walks:
    """Mixin for agents that can walk"""
    def setWalkingDirection(self, heading):
        ...

# ACTIONS (Use protocols)
class SteeringAction(Action):
    """Base class for steering actions"""
    def canBeTakenBy(self, agent):
        return isinstance(agent, Steers)

class SetThrottleAction(SteeringAction):
    def __init__(self, throttle: float):
        self.throttle = throttle
    
    def applyTo(self, obj, sim):
        obj.setThrottle(self.throttle)

class SetSteerAction(SteeringAction):
    ...

class SetBrakeAction(SteeringAction):
    ...

class WalkingAction(Action):
    """Base class for walking actions"""
    def canBeTakenBy(self, agent):
        return isinstance(agent, Walks)
```

#### Racing Domain Actions (Extends Driving)

```python
"""Racing-specific actions extending driving actions."""

from scenic.domains.driving.actions import *  # Import all driving actions

# Add racing-specific actions
class DRSAction(Action):
    """Activate/deactivate DRS (Drag Reduction System)"""
    def __init__(self, activate: bool = True):
        self.activate = activate
    
    def applyTo(self, obj, sim):
        if hasattr(obj, 'setDRS'):
            obj.setDRS(self.activate)

class ERSDeployAction(Action):
    """Deploy ERS power boost"""
    ...

class PitLimiterAction(Action):
    """Activate pit speed limiter"""
    ...

class OvertakeAction(Action):
    """Attempt overtaking maneuver"""
    ...
```

**Key Pattern**: Racing domain imports all driving actions, then adds racing-specific ones.

---

### 5. Behaviors (`behaviors.scenic`)

Behaviors are **time-extended strategies** using actions.

#### Driving Domain Behaviors

```scenic
"""Library of useful behaviors for driving scenarios."""

import scenic.domains.driving.model as _model
from scenic.domains.driving.actions import *

behavior FollowLaneBehavior(target_speed=10, laneToFollow=None):
    """Follow the current lane until reaching intersection"""
    
    # Setup
    current_lane = self.lane if laneToFollow is None else laneToFollow
    _lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)
    past_steer_angle = 0
    
    while True:
        # Compute cross-track error
        cte = current_lane.centerline.signedDistanceTo(self.position)
        speed_error = target_speed - self.speed
        
        # Run controllers
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        
        # Take action
        take RegulatedControlAction(throttle, steer, past_steer_angle)
        past_steer_angle = steer

behavior TurnBehavior(trajectory, target_speed=6):
    """Turn behavior for intersections"""
    _lon_controller, _lat_controller = simulation().getTurningControllers(self)
    ...

behavior LaneChangeBehavior(laneSectionToSwitch, target_speed=10):
    """Change lanes"""
    ...

behavior DriveAvoidingCollisions(target_speed=25, avoidance_threshold=10):
    """Drive and avoid collisions"""
    try:
        do FollowLaneBehavior(target_speed=target_speed)
    interrupt when self.distanceToClosest(_model.Vehicle) <= avoidance_threshold:
        take SetBrakeAction(1)
```

#### Racing Domain Behaviors (Should Extend Driving)

```scenic
"""Racing-specific behaviors extending driving behaviors."""

from scenic.domains.driving.behaviors import *
import scenic.domains.racing.model as _racing

behavior FollowRacingLineBehavior(target_speed=30):
    """Follow the optimal racing line"""
    # Similar to FollowLaneBehavior but optimized for racing
    ...

behavior PitStopBehavior(pitBox):
    """Execute a pit stop"""
    # Enter pit lane
    do EnterPitLaneBehavior()
    
    # Navigate to pit box
    do NavigateToPitBoxBehavior(pitBox)
    
    # Stop in box
    take SetBrakeAction(1.0)
    wait
    
    # Exit pit lane
    do ExitPitLaneBehavior()

behavior OvertakingBehavior(target_car):
    """Attempt to overtake target car"""
    ...

behavior DefensiveBehavior():
    """Defend position from cars behind"""
    ...
```

**Key Pattern**: Racing behaviors extend driving behaviors with racing-specific strategies.

---

### 6. Workspace (`workspace.py`)

Controls how scenarios are visualized.

#### Driving Domain

```python
"""Workspaces for the driving domain."""

from scenic.core.workspaces import Workspace

class DrivingWorkspace(Workspace):
    """Workspace created from a road Network."""
    
    def __init__(self, network):
        self.network = network
        super().__init__()
    
    def show2D(self, plt):
        self.network.show()  # Show road network
    
    @property
    def minimumZoomSize(self):
        return 20
```

#### Racing Domain (Could Add)

```python
"""Workspaces for the racing domain."""

from scenic.domains.driving.workspace import DrivingWorkspace

class RacingWorkspace(DrivingWorkspace):
    """Workspace for racing scenarios."""
    
    def __init__(self, track):
        self.track = track
        super().__init__(track.network)
    
    def show2D(self, plt):
        super().show2D(plt)
        # Add racing-specific visualization
        self.track.showStartingGrid(plt)
        self.track.showSectors(plt)
```

---

### 7. Simulators (`simulators.py`)

Defines base interfaces for simulators to implement.

#### Driving Domain

```python
"""Base simulator interface for the driving domain."""

from scenic.core.simulators import Simulator

class DrivingSimulator(Simulator):
    """Abstract interface for driving simulators.
    
    Simulators must implement:
    - Vehicle control (throttle, steering, braking)
    - Pedestrian control (walking)
    - Lane-following controllers
    """
    
    def getLaneFollowingControllers(self, agent):
        """Return (longitudinal, lateral) controllers for lane following"""
        raise NotImplementedError
    
    def getTurningControllers(self, agent):
        """Return controllers tuned for turning"""
        raise NotImplementedError
```

#### Racing Domain

```python
"""Base simulator interface for racing domain."""

from scenic.domains.driving.simulators import DrivingSimulator

class RacingSimulator(DrivingSimulator):
    """Abstract interface for racing simulators.
    
    Extends DrivingSimulator with racing features:
    - DRS/ERS systems
    - Pit stop mechanics
    - Lap timing
    - Sector timing
    """
    
    def setDRS(self, vehicle, activate):
        """Activate/deactivate DRS"""
        raise NotImplementedError
    
    def executePitStop(self, vehicle, pitBox):
        """Execute pit stop"""
        raise NotImplementedError
    
    def getCurrentLap(self, vehicle):
        """Get current lap number"""
        raise NotImplementedError
```

---

## How Racing Extends Driving

### Extension Strategy

The racing domain uses **inheritance and composition**:

1. **Imports Everything**: `from scenic.domains.driving.model import *`
2. **Replaces Network**: `network = track.network` (track wraps original network)
3. **Extends Classes**: `class RacingCar(Car)` extends driving's `Car`
4. **Adds New Concepts**: Pit lanes, sectors, racing lines
5. **Maintains Compatibility**: All driving features still work

### What Racing Inherits from Driving

✅ **Infrastructure**:
- `Network`, `Road`, `Lane` classes
- OpenDRIVE file loading
- Lane geometry and topology

✅ **Regions**:
- `road`, `sidewalk`, `intersection` regions
- `roadDirection` vector field

✅ **Object Classes**:
- `DrivingObject` base class with `.lane`, `.road` properties
- `Vehicle`, `Car` classes

✅ **Actions**:
- `SetThrottleAction`, `SetSteerAction`, `SetBrakeAction`
- `RegulatedControlAction`
- All driving action protocols

✅ **Behaviors**:
- `FollowLaneBehavior`, `TurnBehavior`, `LaneChangeBehavior`
- Controller infrastructure

### What Racing Adds

➕ **Track Infrastructure** (`tracks.py`):
- `RacingTrack` class
- `PitLane`, `Sector`, `RacingLine` classes
- Track direction awareness
- Starting grid generation

➕ **Racing Regions**:
- `pitLane`, `racingLine`
- `mainRacingRoad`, `pitLaneRoad` (mutually exclusive segments)

➕ **Racing Objects**:
- `RacingCar` (extends `Car`)
- Racing-specific properties (fuel, tires, race number)

➕ **Racing Actions**:
- `DRSAction`, `ERSDeployAction`
- `PitLimiterAction`, `OvertakeAction`
- Formation and defensive actions

➕ **Racing Behaviors**:
- Racing line following
- Pit stop behaviors
- Overtaking strategies

---

## Best Practices

### 1. Follow the Layered Architecture

```
Domain Layer:          racing/model.scenic, racing/tracks.py
  ↓ (uses)
Driving Layer:         driving/model.scenic, driving/roads.py
  ↓ (uses)
Core Scenic:           scenic.core.*
```

### 2. Import Pattern

**In `racing/model.scenic`**:
```scenic
# Import EVERYTHING from driving first
from scenic.domains.driving.model import *

# Then import racing-specific additions
from scenic.domains.racing.tracks import RacingTrack
from scenic.domains.racing.behaviors import *
```

**In `racing/tracks.py`**:
```python
# Import specific driving components you extend
from scenic.domains.driving.roads import Network, Road, Lane, ...

# Don't import with * in Python files (only in .scenic files)
```

### 3. Class Extension Pattern

Always extend, never replace:

```python
# ✅ GOOD: Extend existing class
class RacingCar(Car):
    """Extends Car with racing features"""
    raceNumber: Range(1, 999)
    fuelLevel: Range(0.5, 1.0)

# ❌ BAD: Define completely new class
class RacingVehicle(DrivingObject):
    """Duplicate Car functionality"""
    ...
```

### 4. Region Derivation Pattern

Derive racing regions from driving regions:

```scenic
# ✅ GOOD: Derive from existing regions
racingLine: Region = road.difference(pitLane)  # Uses driving's 'road'

# ❌ BAD: Create completely separate regions
racingLine: Region = track.createRacingLineRegion()  # Disconnected from 'road'
```

### 5. Network Wrapper Pattern

Wrap, don't replace the network:

```python
# ✅ GOOD: Wrap the network
class RacingTrack:
    def __init__(self, network: Network, ...):
        self.network = network  # Keep original network
        self._identifyRacingFeatures()  # Analyze it

# In model.scenic:
track = RacingTrack(network, ...)
network = track.network  # Use wrapped network

# ❌ BAD: Create separate network
class RacingTrack:
    def __init__(self, map_file):
        self.racing_network = self.loadRacingNetwork(map_file)  # Separate!
```

### 6. Behavior Composition Pattern

Compose behaviors, don't duplicate:

```scenic
# ✅ GOOD: Reuse driving behaviors
behavior FollowRacingLineBehavior(target_speed=30):
    # Similar to FollowLaneBehavior but with racing optimizations
    do FollowLaneBehavior(target_speed=target_speed, laneToFollow=racingLine)

# ❌ BAD: Copy-paste and modify
behavior FollowRacingLineBehavior(target_speed=30):
    # [100 lines of duplicated FollowLaneBehavior code with tweaks]
```

### 7. Parameter Inheritance Pattern

Extend parameters, maintain compatibility:

```scenic
# Driving parameters (still work in racing)
param map = localPath('map.xodr')
param use2DMap = True
param map_options = {}

# Racing-specific parameters (optional, have defaults)
param trackDirection = 'counterclockwise'
param generateStartingGrid = True
param pitLaneRoadId = None
```

### 8. Documentation Pattern

Document the extension relationship:

```python
"""Racing domain description.

**Extends**: scenic.domains.driving

**Adds**:
- Pit lanes and pit stops
- Track sectors
- Starting grids
- Racing lines

**Inherits** from driving domain:
- Road network infrastructure
- Vehicle objects
- Lane-following behaviors
- All driving actions
"""
```

---

## Summary: The Extension Checklist

When extending a domain, follow this checklist:

- [ ] **Import everything** from parent domain (`from parent.model import *`)
- [ ] **Extend, don't replace** classes (`class RacingCar(Car)`)
- [ ] **Wrap infrastructure** (`track.network = network`)
- [ ] **Derive regions** from parent regions (`racingLine = road.difference(pitLane)`)
- [ ] **Add new concepts** without breaking existing ones
- [ ] **Maintain parameter compatibility** (existing params still work)
- [ ] **Compose behaviors** instead of duplicating
- [ ] **Document the relationship** clearly
- [ ] **Mirror file structure** for consistency
- [ ] **Test compatibility** with parent domain scenarios

---

## Quick Reference: Key Differences

| Aspect | Driving Domain | Racing Domain |
|--------|---------------|---------------|
| **Primary Use** | General road scenarios | Closed-circuit racing |
| **Network Type** | `Network` (general roads) | `RacingTrack` (wraps Network) |
| **Direction** | Two-way (opposing traffic) | One-way (track direction) |
| **Special Lanes** | Shoulders, bike lanes | Pit lanes |
| **Regions** | `road`, `sidewalk`, `intersection` | `racingLine`, `pitLane`, `mainRacingRoad` |
| **Objects** | `Car`, `Pedestrian` | `RacingCar` (extends Car) |
| **Behaviors** | Lane following, turning | Racing line, pit stops, overtaking |
| **Actions** | Throttle, brake, steer | + DRS, ERS, pit limiter |
| **Simulator Examples** | CARLA, LGSVL, MetaDrive | dSPACE ModelDesk, CARLA (racing) |

---

## Additional Resources

- **Driving Domain Source**: `src/scenic/domains/driving/`
- **Racing Domain Source**: `src/scenic/domains/racing/`
- **Core Scenic Docs**: Scenic language reference
- **OpenDRIVE Spec**: Understanding map files

---

*This document serves as a comprehensive guide to understanding how Scenic domains are structured and how to properly extend them.*

