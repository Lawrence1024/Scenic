# Scenic Domain Architecture: Complete Implementation Guide

## Table of Contents

1. [Overview](#overview)
2. [Core Architecture Principles](#core-architecture-principles)
3. [Domain Layer Structure](#domain-layer-structure)
4. [The CARLA Pattern (Gold Standard)](#the-carla-pattern-gold-standard)
5. [Driving Domain Architecture](#driving-domain-architecture)
6. [Racing Domain Architecture](#racing-domain-architecture)
7. [Simulator Implementation Pattern](#simulator-implementation-pattern)
8. [dSPACE Implementation Guide](#dspace-implementation-guide)
9. [Common Patterns and Best Practices](#common-patterns-and-best-practices)
10. [Practical Examples](#practical-examples)
11. [Debugging and Testing](#debugging-and-testing)
12. [Quick Reference](#quick-reference)

---

## Overview

This document provides a complete guide to understanding and implementing Scenic domains and simulator integrations. It combines architectural principles, implementation patterns, and practical examples based on the successful CARLA implementation.

### Key Insight

**Domains are abstract, simulators are concrete.** Domains define *what* can be done (protocols, actions, behaviors), while simulators implement *how* it's done (concrete methods, simulator-specific APIs).

---

## Core Architecture Principles

### The Layered Architecture

```
┌─────────────────────────────────────────────────────────────┐
│                  SCENARIOS (.scenic files)                   │
│           User-written scenarios using domain API            │
└──────────────────────────┬──────────────────────────────────┘
                           │ imports
┌──────────────────────────▼──────────────────────────────────┐
│               RACING DOMAIN (abstract)                       │
│  • RacingCar, RacingTrack, PitLane                          │
│  • Actions: DRS, ERS, PitLimiter                            │
│  • Behaviors: FollowRacingLine, PitStop, Overtaking         │
│  • Extends driving domain                                    │
└──────────────────────────┬──────────────────────────────────┘
                           │ extends
┌──────────────────────────▼──────────────────────────────────┐
│               DRIVING DOMAIN (abstract)                      │
│  • Network, Road, Lane                                       │
│  • Car, Pedestrian                                           │
│  • Actions: SetThrottle, SetBrake, SetSteer                 │
│  • Behaviors: FollowLane, Turn, LaneChange                  │
│  • Protocols: Steers, Walks                                 │
└──────────────────────────┬──────────────────────────────────┘
                           │ uses
┌──────────────────────────▼──────────────────────────────────┐
│                  CORE SCENIC                                 │
│  • Object, Region, VectorField                              │
│  • Distributions, Specifiers                                │
│  • Simulator interface                                       │
└─────────────────────────────────────────────────────────────┘
                           │ implements
┌──────────────────────────▼──────────────────────────────────┐
│              SIMULATOR LAYER (concrete)                      │
│  ┌────────────────┐  ┌────────────────┐                    │
│  │ CARLA          │  │ dSPACE         │                    │
│  │ Implements:    │  │ Implements:    │                    │
│  │ - Steers       │  │ - Steers       │                    │
│  │ - Walks        │  │ - RacingSteers │                    │
│  │ - Controllers  │  │ - Controllers  │                    │
│  └────────────────┘  └────────────────┘                    │
└─────────────────────────────────────────────────────────────┘
```

### Golden Rules

1. **Extend, Don't Replace**: Racing extends driving, simulators extend domains
2. **Protocols Are Key**: Abstract protocols enable simulator independence
3. **Import Everything**: Use `from parent.model import *` in domains
4. **Marker Classes**: Use mixins like `_CarlaVehicle`, `_DSpaceVehicle` for type checking
5. **No Duplication**: Never redefine domain actions in simulators

---

## Domain Layer Structure

### Standard File Structure

```
scenic/domains/{domain_name}/
├── __init__.py              # Domain documentation
├── model.scenic             # World model (objects, regions, behaviors)
├── {infrastructure}.py      # Infrastructure (roads.py, tracks.py)
├── actions.py               # Domain-specific actions
├── behaviors.scenic         # Domain-specific behaviors
├── controllers.py           # Control algorithms (optional)
├── workspace.py             # Visualization (optional)
└── simulators.py            # Base simulator interface
```

### File Responsibilities

| File | Purpose | Contains |
|------|---------|----------|
| `__init__.py` | Documentation | Domain description, usage examples |
| `model.scenic` | World model | Parameters, regions, objects, utilities |
| `*.py` (infrastructure) | Geometry/topology | Network, roads, tracks, lanes |
| `actions.py` | What agents do | Action classes, protocols |
| `behaviors.scenic` | Time-extended strategies | Behavior definitions |
| `controllers.py` | Control algorithms | PID controllers, etc. |
| `simulators.py` | Simulator interface | Abstract methods simulators implement |

---

## The CARLA Pattern (Gold Standard)

### Why CARLA Is the Template

CARLA successfully implements the driving domain with clean separation of concerns. All other simulators should follow this pattern.

### CARLA Actions Pattern

```python
# carla/actions.py
from scenic.domains.driving.actions import *  # ← Import ALL driving actions

# CARLA marker mixin
class _CarlaVehicle:
    """Mixin identifying CARLA vehicles."""
    pass

# ONLY define CARLA-specific actions (not in driving domain)
class VehicleAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _CarlaVehicle)

class SetAutopilotAction(VehicleAction):
    """CARLA-specific autopilot."""
    def applyTo(self, obj, sim):
        obj.carlaActor.set_autopilot(self.enabled, sim.tm.get_port())

class SetManualGearShiftAction(VehicleAction):
    """CARLA-specific gear control."""
    def applyTo(self, obj, sim):
        ctrl = obj.carlaActor.get_control()
        ctrl.manual_gear_shift = self.manualGearShift
        obj.carlaActor.apply_control(ctrl)
```

**Key Observation**: CARLA does NOT redefine `SetThrottleAction`, `SetBrakeAction`, or `SetSteerAction`. These come from the driving domain.

### CARLA Model Pattern

```scenic
# carla/model.scenic
from scenic.domains.driving.model import *  # ← Import EVERYTHING

# CARLA-specific base class
class CarlaActor(DrivingObject):
    carlaActor: None  # Link to CARLA's internal actor
    blueprint: None   # CARLA blueprint ID
    
    def setPosition(self, pos, elevation):
        self.carlaActor.set_location(_utils.scenicToCarlaLocation(pos, elevation))

# Implement Steers protocol
class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    """CARLA vehicle implementation."""
    
    def setThrottle(self, throttle):
        self.control.throttle = throttle  # ← Implements Steers.setThrottle()
    
    def setSteering(self, steering):
        self.control.steer = steering  # ← Implements Steers.setSteering()
    
    def setBraking(self, braking):
        self.control.brake = braking  # ← Implements Steers.setBraking()
```

### CARLA Simulator Pattern

```python
# carla/simulator.py
from scenic.domains.driving.simulators import DrivingSimulator, DrivingSimulation

class CarlaSimulator(DrivingSimulator):
    def __init__(self, carla_map, ...):
        super().__init__()
        self.client = carla.Client(address, port)
        self.world = self.client.load_world(carla_map)

class CarlaSimulation(DrivingSimulation):
    def createObjectInSimulator(self, obj):
        # Create CARLA actor
        blueprint = self.blueprintLib.find(obj.blueprint)
        transform = _utils.scenicToCarlaTransform(obj.position, obj.heading)
        carlaActor = self.world.try_spawn_actor(blueprint, transform)
        obj.carlaActor = carlaActor  # ← Link Scenic object to CARLA actor
        return carlaActor
    
    def executeActions(self, allActions):
        super().executeActions(allActions)
        # Apply accumulated control updates
        for obj in self.agents:
            if obj._control is not None:
                obj.carlaActor.apply_control(obj._control)
                obj._control = None
```

---

## Driving Domain Architecture

### Components

```
scenic/domains/driving/
├── model.scenic         # Car, Pedestrian, road regions
├── roads.py            # Network, Road, Lane, Intersection
├── actions.py          # Steers, Walks protocols + actions
├── behaviors.scenic    # FollowLane, Turn, LaneChange
├── controllers.py      # PID controllers
└── simulators.py       # DrivingSimulator interface
```

### Key Classes

#### Protocols (in `actions.py`)

```python
class Steers:
    """Mixin protocol for agents which can steer."""
    def setThrottle(self, throttle):
        raise NotImplementedError
    def setSteering(self, steering):
        raise NotImplementedError
    def setBraking(self, braking):
        raise NotImplementedError
    def setHandbrake(self, handbrake):
        raise NotImplementedError
    def setReverse(self, reverse):
        raise NotImplementedError

class Walks:
    """Mixin protocol for agents which can walk."""
    def setWalkingDirection(self, heading):
        raise NotImplementedError
    def setWalkingSpeed(self, speed):
        raise NotImplementedError
```

#### Actions (in `actions.py`)

```python
class SteeringAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, Steers)

class SetThrottleAction(SteeringAction):
    def __init__(self, throttle: float):
        self.throttle = throttle
    def applyTo(self, obj, sim):
        obj.setThrottle(self.throttle)  # ← Calls protocol method
```

#### Objects (in `model.scenic`)

```scenic
class DrivingObject:
    """Base class for all driving objects."""
    @property
    def lane(self) -> Lane:
        return network.laneAt(self.position)
    
    @property
    def road(self) -> Road:
        return network.roadAt(self.position)

class Vehicle(DrivingObject):
    """Vehicles which drive."""
    width: 2
    length: 4.5

class Car(Vehicle):
    """A car."""
    pass
```

---

## Racing Domain Architecture

### Extension Strategy

Racing domain **extends** driving domain, adding racing-specific features while maintaining full compatibility.

```python
# racing/model.scenic
from scenic.domains.driving.model import *  # ← Import EVERYTHING

# Wrap the network
track: RacingTrack = createRacingTrack(network, ...)
network = track.network  # Replace with wrapped version

# Add racing regions
pitLaneRoad: Region = track.pitLaneRoad
mainRacingRoad: Region = road.difference(pitLaneRoad)
racingLine: Region = mainRacingRoad

# Extend Car
class RacingCar(Car):
    """Racing car with racing-specific properties."""
    raceNumber: Range(1, 999)
    maxSpeed: 30.0
    fuelLevel: Range(0.5, 1.0)
    tireWear: 0.0
    ttl = racingLine  # Target line to drive on
```

### What Racing Inherits

✅ **From Driving Domain**:
- `Network`, `Road`, `Lane` infrastructure
- `SetThrottleAction`, `SetBrakeAction`, `SetSteerAction`
- `FollowLaneBehavior`, `TurnBehavior`
- `Steers` protocol

### What Racing Adds

➕ **Racing-Specific**:
- `RacingTrack`, `PitLane`, `Sector`, `RacingLine`
- `RacingCar` (extends `Car`)
- `SetMaxSpeedAction`, `SetTTLAction`
- `FollowRacingLineBehavior`, `PitStopBehavior`, `OvertakingBehavior`

### The Two-Segment Architecture

Racing tracks have exactly two mutually exclusive segments:

```scenic
# mainRacingRoad ∪ pitLaneRoad = road
# mainRacingRoad ∩ pitLaneRoad = ∅

mainRacingRoad: Region  # Main racing circuit
pitLaneRoad: Region     # Pit lane only

# racingLine = main track (no pit lane)
racingLine: Region = road.difference(pitLane)
```

---

## Simulator Implementation Pattern

### Step-by-Step Guide

#### 1. Create Simulator Structure

```
scenic/simulators/{simulator_name}/
├── __init__.py
├── simulator.py       # Simulator and Simulation classes
├── model.scenic       # Domain + simulator integration
├── actions.py         # Simulator-specific actions only
├── behaviors.scenic   # Simulator-specific behaviors (optional)
├── blueprints.py      # Simulator assets (optional)
└── utils/            # Utilities (coordinate conversion, etc.)
```

#### 2. Implement Actions (actions.py)

```python
# Import domain actions (DON'T redefine them!)
from scenic.domains.{domain}.actions import *

# Define marker mixin
class _{Simulator}Vehicle:
    """Mixin identifying {simulator} vehicles."""
    pass

# ONLY define simulator-specific actions
class {Simulator}SpecificAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _{Simulator}Vehicle)
    
    def applyTo(self, obj, sim):
        # Simulator-specific implementation
        obj.simulatorActor.{simulator_method}(...)
```

#### 3. Implement Model (model.scenic)

```scenic
# Import domain model
from scenic.domains.{domain}.model import *

# Import simulator components
from scenic.simulators.{simulator}.actions import _{Simulator}Vehicle
from scenic.domains.driving.actions import Steers

# Simulator-specific base class
class {Simulator}Actor({DomainObject}):
    simulatorActor: None  # Link to simulator's internal object
    
    def setPosition(self, pos, elevation):
        # Convert and apply to simulator
        self.simulatorActor.set_position(...)

# Implement protocols
class Vehicle(Vehicle, {Simulator}Actor, Steers, _{Simulator}Vehicle):
    """Simulator vehicle implementation."""
    
    def setThrottle(self, throttle):
        # Store for later application
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['throttle'] = throttle
    
    def setSteering(self, steering):
        self._control_state['steering'] = steering
    
    def setBraking(self, braking):
        self._control_state['braking'] = braking
```

#### 4. Implement Simulator (simulator.py)

```python
from scenic.domains.{domain}.simulators import {Domain}Simulator, {Domain}Simulation

class {Simulator}Simulator({Domain}Simulator):
    def __init__(self, **kwargs):
        super().__init__()
        # Simulator-specific setup
        self.connection = self.connect_to_simulator(...)
    
    def createSimulation(self, scene, **kwargs):
        return {Simulator}Simulation(scene, self, **kwargs)

class {Simulator}Simulation({Domain}Simulation):
    def createObjectInSimulator(self, obj):
        # Create simulator's internal representation
        simulatorActor = self.spawn_object(obj.position, obj.heading, ...)
        obj.simulatorActor = simulatorActor  # ← Link
        return simulatorActor
    
    def step(self):
        # Apply any pending control state
        for obj in self.scene.objects:
            if hasattr(obj, '_control_state') and obj._control_state:
                self.applyControlToSimulator(obj, obj._control_state)
        
        # Step simulator
        self.simulator_connection.step()
```

---

## dSPACE Implementation Guide

### Current Architecture

```
scenic/simulators/dspace/
├── __init__.py
├── simulator.py          # DSpaceSimulator, DSpaceSimulation
├── model.scenic          # Racing + dSPACE integration
├── racing_model.scenic   # Alias for model.scenic
├── actions.py            # dSPACE-specific actions
├── controldesk.py        # ControlDesk COM API wrapper
└── utils.py             # Utilities
```

### Correct Implementation

#### 1. actions.py (Fixed)

```python
"""dSPACE-specific actions.

This module provides ONLY dSPACE-specific actions. Standard driving and racing
actions are inherited from their respective domains.
"""

from scenic.core.simulators import Action

# Marker mixin
class _DSpaceVehicle:
    """Mixin identifying dSPACE vehicles."""
    pass

# Combined control action (dSPACE-specific convenience)
class SetVehicleControl(Action):
    """Set multiple control inputs simultaneously.
    
    This is a dSPACE convenience action. For individual controls,
    use standard driving actions (SetThrottleAction, etc.).
    """
    
    def __init__(self, throttle=0.0, brake=0.0, steer=0.0, velocity=None):
        self.throttle = max(0.0, min(1.0, throttle))
        self.brake = max(0.0, min(1.0, brake))
        self.steer = max(-1.0, min(1.0, steer))
        self.velocity = velocity
    
    def canBeTakenBy(self, agent):
        return isinstance(agent, _DSpaceVehicle)
    
    def applyTo(self, obj, sim):
        if not hasattr(obj, '_control_state'):
            obj._control_state = {}
        obj._control_state.update({
            'throttle': self.throttle,
            'braking': self.brake,
            'steering': self.steer
        })
        if self.velocity is not None:
            obj._control_state['velocity'] = self.velocity
```

#### 2. model.scenic (Fixed)

```scenic
"""dSPACE-specific racing model."""

# Import racing domain (which imports driving domain)
from scenic.domains.racing.model import *
from scenic.domains.racing.actions import SetMaxSpeedAction, SetTTLAction

# Import dSPACE-specific components
import scenic.simulators.dspace as dspace
from scenic.simulators.dspace.actions import _DSpaceVehicle
from scenic.domains.driving.actions import Steers

# dSPACE ModelDesk parameters
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
param timestep = 0.1

# Configure the dSPACE simulator
simulator dspace.DSpaceSimulator(
    scenario_src=globalParameters.scenario_src,
    scenario_name=globalParameters.scenario_name,
    timestep=globalParameters.timestep,
)

# dSPACE-specific racing car implementation
class DSPACERacingCar(RacingCar, _DSpaceVehicle, Steers):
    """dSPACE implementation of racing car."""
    
    # dSPACE-specific properties
    dspaceActor: None
    routeId: None
    
    # Racing-specific methods
    def setMaxSpeed(self, max_speed):
        self.maxSpeed = max_speed
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.set_control({'max_speed': float(max_speed)})
    
    def setTTL(self, ttl):
        self.ttl = ttl
        if hasattr(self, 'dspaceActor') and self.dspaceActor:
            self.dspaceActor.set_control({'ttl_set': True})
    
    # Steers protocol implementation (for driving domain actions)
    def setThrottle(self, throttle):
        """Set throttle using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['throttle'] = float(throttle)
    
    def setSteering(self, steering):
        """Set steering using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['steering'] = float(steering)
    
    def setBraking(self, braking):
        """Set braking using driving domain protocol."""
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['braking'] = float(braking)
    
    def setHandbrake(self, handbrake):
        """Set handbrake (not implemented in dSPACE yet)."""
        pass
    
    def setReverse(self, reverse):
        """Set reverse gear (not implemented in dSPACE yet)."""
        pass

# Replace the abstract RacingCar with dSPACE implementation
RacingCar = DSPACERacingCar
```

#### 3. Key ControlDesk Integration

The simulator's `step()` method applies stored control state:

```python
# In simulator.py
def step(self):
    # Apply any pending control state from driving domain actions
    if self._cd:
        for obj in self.scene.objects:
            if hasattr(obj, '_control_state') and obj._control_state:
                control = obj._control_state
                
                # Determine vehicle name
                vehicle_name = self._getVehicleName(obj)
                
                # Apply via ControlDesk
                self.setVehicleControl(
                    vehicle_name=vehicle_name,
                    throttle=control.get('throttle'),
                    brake=control.get('braking'),
                    steering=control.get('steering')
                )
    
    time.sleep(self.timestep)
```

---

## Common Patterns and Best Practices

### Pattern 1: Multiple Inheritance

```scenic
class SimulatorVehicle(DomainCar, SimulatorActor, Steers, _SimulatorVehicle):
    #                   ↑           ↑              ↑      ↑
    #                 Domain    Simulator      Protocol  Marker
```

- **Domain class first**: Inherit domain functionality
- **Simulator base**: Add simulator-specific properties
- **Protocol**: Implement required interface
- **Marker**: Enable type checking for actions

### Pattern 2: Protocol Implementation

```scenic
# In simulator model.scenic
class Vehicle(Vehicle, SimulatorActor, Steers):
    def setThrottle(self, throttle):
        # Store in _control_state for later application
        if not hasattr(self, '_control_state'):
            self._control_state = {}
        self._control_state['throttle'] = throttle
```

```python
# In simulator.py
def step(self):
    # Apply accumulated control state
    for obj in self.scene.objects:
        if hasattr(obj, '_control_state'):
            self.applyControl(obj, obj._control_state)
```

### Pattern 3: Network Wrapping

```python
# In racing domain
class RacingTrack:
    def __init__(self, network: Network):
        self.network = network  # Wrap, don't replace
        self._identifyRacingFeatures()

# In model.scenic
network = Network.fromFile(...)
track = RacingTrack(network)
network = track.network  # Use wrapped version
```

### Pattern 4: Region Derivation

```scenic
# Derive racing regions from driving regions
mainRacingRoad: Region = road.difference(pitLaneRoad)
racingLine: Region = mainRacingRoad

# DON'T create disconnected regions
```

### Pattern 5: Behavior Composition

```scenic
behavior RaceWithPitStop():
    # Reuse existing behaviors
    do FollowRacingLineBehavior() for 3 laps
    do PitStopBehavior()
    do FollowRacingLineBehavior()

# DON'T copy-paste and modify
```

---

## Practical Examples

### Example 1: Basic Racing Scenario

```scenic
"""Simple racing scenario following best practices."""

# 1. Configure track
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param trackDirection = 'counterclockwise'

# 2. Import model (gets ALL domain features)
model scenic.simulators.dspace.model

# 3. Create objects using domain features
ego = new RacingCar on mainRacingRoad, with raceNumber 1
opponent = new RacingCar ahead of ego by 50, with raceNumber 2

# 4. Use domain actions/behaviors
behavior Race():
    while True:
        take SetThrottleAction(0.5)  # ← Driving domain action!

ego.behavior = Race()
```

### Example 2: Starting Grid

```scenic
param map = localPath('../../assets/maps/dSPACE/LagunaSeca.xodr')
param use2DMap = True
param generateStartingGrid = True
param startingGridPositions = 10

model scenic.simulators.dspace.model

# Place cars on grid
ego = new RacingCar at startingGrid[0], with raceNumber 1
for i in range(1, 5):
    new RacingCar at startingGrid[i], with raceNumber (i+1)
```

### Example 3: Using Standard Driving Actions

```scenic
model scenic.simulators.dspace.model

ego = new RacingCar on mainRacingRoad

behavior TestControls():
    # These are driving domain actions, work everywhere!
    take SetThrottleAction(0.5)
    wait for 3 seconds
    take SetBrakeAction(0.3)
    wait for 2 seconds
    take SetSteerAction(0.2)

ego.behavior = TestControls()
```

---

## Debugging and Testing

### Verify Domain Import Chain

```scenic
# Test that domain features are available
param map = localPath('map.xodr')
param use2DMap = True

model scenic.simulators.dspace.model

# These should all work (from driving domain):
test_car = new Car on road
test_vehicle = new Vehicle on road

# These should work (from racing domain):
racing_car = new RacingCar on mainRacingRoad

# These regions should exist:
print(f"road exists: {road is not None}")
print(f"mainRacingRoad exists: {mainRacingRoad is not None}")
print(f"pitLaneRoad exists: {pitLaneRoad is not None}")
```

### Verify Action Availability

```scenic
model scenic.simulators.dspace.model

ego = new RacingCar on mainRacingRoad

behavior TestActions():
    # Driving domain actions
    take SetThrottleAction(0.5)
    take SetBrakeAction(0.3)
    take SetSteerAction(0.1)
    
    # Racing domain actions
    take SetMaxSpeedAction(30)
    take SetTTLAction(racingLine)

ego.behavior = TestActions()
```

### Verify Protocol Implementation

```python
# In simulator test
from scenic.simulators.dspace.model import DSPACERacingCar
from scenic.domains.driving.actions import Steers

# Check inheritance
assert issubclass(DSPACERacingCar, Steers)

# Check methods exist
car = DSPACERacingCar(...)
assert hasattr(car, 'setThrottle')
assert hasattr(car, 'setSteering')
assert hasattr(car, 'setBraking')
```

---

## Quick Reference

### Import Patterns

```python
# In domain .scenic files
from scenic.domains.parent.model import *  # Import parent domain

# In domain .py files  
from scenic.domains.parent.specific import SpecificClass  # Specific imports

# In simulator actions.py
from scenic.domains.{domain}.actions import *  # Import domain actions

# In simulator model.scenic
from scenic.domains.{domain}.model import *  # Import domain model
from scenic.simulators.{sim}.actions import _SimVehicle  # Import marker
from scenic.domains.driving.actions import Steers  # Import protocols
```

### Class Definition Patterns

```scenic
# Domain object (abstract)
class RacingCar(Car):
    """Abstract racing car."""
    def setDRS(self, activate):
        raise NotImplementedError

# Simulator object (concrete)
class SimRacingCar(RacingCar, SimActor, Steers, _SimVehicle):
    """Concrete implementation."""
    def setThrottle(self, throttle):
        self._control_state['throttle'] = throttle
    
    def setDRS(self, activate):
        self.simActor.set_drs(activate)
```

### Action Patterns

```python
# Domain action (abstract)
class SetThrottleAction(SteeringAction):
    def applyTo(self, obj, sim):
        obj.setThrottle(self.throttle)  # Calls protocol method

# Simulator-specific action (concrete)
class SimSpecificAction(Action):
    def canBeTakenBy(self, agent):
        return isinstance(agent, _SimVehicle)
    
    def applyTo(self, obj, sim):
        obj.simActor.sim_specific_method()
```

### Region Patterns

```scenic
# Domain regions
road: Region = network.drivableRegion  # From driving domain
mainRacingRoad: Region = ...           # Racing-specific

# Object placement
ego = new Car on road                   # Driving domain
racer = new RacingCar on mainRacingRoad # Racing domain
```

---

## Common Mistakes to Avoid

### ❌ Mistake 1: Redefining Domain Actions

```python
# WRONG - in simulator/actions.py
class SetThrottleAction(Action):  # ← Shadows driving domain action!
    def applyTo(self, obj, sim):
        obj.simActor.set_throttle(...)
```

```python
# CORRECT - import from domain
from scenic.domains.driving.actions import *  # ← Get SetThrottleAction from here
# Only define simulator-specific actions
```

### ❌ Mistake 2: Not Implementing Protocols

```scenic
# WRONG
class SimVehicle(Vehicle):
    # Missing setThrottle(), setSteering(), setBraking()
    pass
```

```scenic
# CORRECT
class SimVehicle(Vehicle, Steers):
    def setThrottle(self, throttle):
        self._control_state['throttle'] = throttle
    
    def setSteering(self, steering):
        self._control_state['steering'] = steering
    
    def setBraking(self, braking):
        self._control_state['braking'] = braking
```

### ❌ Mistake 3: Creating Disconnected Regions

```scenic
# WRONG
racingLine = track.createCustomRegion()  # Disconnected from 'road'
```

```scenic
# CORRECT
racingLine = road.difference(pitLane)  # Derived from 'road'
```

### ❌ Mistake 4: Replacing Instead of Wrapping

```python
# WRONG
class RacingTrack:
    def __init__(self, map_file):
        self.racing_network = self.loadNew(map_file)  # ← Separate network!
```

```python
# CORRECT
class RacingTrack:
    def __init__(self, network: Network):
        self.network = network  # ← Wrap existing network
```

---

## Summary: Implementation Checklist

When implementing a new simulator or domain:

### Domain Implementation
- [ ] Follow file structure pattern
- [ ] Import everything from parent domain
- [ ] Define abstract protocols for capabilities
- [ ] Extend parent classes, don't replace
- [ ] Derive regions from parent regions
- [ ] Document the extension relationship

### Simulator Implementation
- [ ] Import domain actions (don't redefine)
- [ ] Define marker mixin (`_SimulatorVehicle`)
- [ ] Implement all required protocols (`Steers`, etc.)
- [ ] Link Scenic objects to simulator actors
- [ ] Apply control state in `step()` method
- [ ] Only add simulator-specific actions/features
- [ ] Test with domain scenarios

### Verification
- [ ] Domain scenarios work unchanged in simulator
- [ ] All domain actions available in simulator
- [ ] Protocol methods implemented correctly
- [ ] Regions properly inherited/derived
- [ ] No shadowing of domain actions
- [ ] Multiple inheritance chain correct

---

## Conclusion

The key to successful Scenic domain and simulator implementation is understanding the **separation of concerns**:

- **Domains define abstractions** (what can be done)
- **Simulators provide implementations** (how it's done)
- **Protocols enable portability** (scenarios work anywhere)
- **CARLA shows the way** (follow its pattern)

By following these patterns, you can:
- Write scenarios once, run anywhere
- Add new simulators easily
- Extend domains cleanly
- Maintain clear architecture
- Avoid common pitfalls

**Remember**: Extend, don't replace. Import, don't redefine. Implement protocols, don't skip them.

---

*This guide combines insights from the CARLA implementation, driving/racing domain architecture, and practical dSPACE integration experience. Last updated: 2024.*

