# CARLA Simulator: Implementation of Driving Domain

This document analyzes how the **CARLA simulator** (`scenic.simulators.carla`) implements the abstract interfaces defined by the **driving domain** (`scenic.domains.driving`). This serves as a perfect example of how racing simulators should implement the racing domain.

---

## Table of Contents

1. [Architecture Overview](#architecture-overview)
2. [Domain Implementation Pattern](#domain-implementation-pattern)
3. [File-by-File Implementation](#file-by-file-implementation)
4. [Protocol Implementation](#protocol-implementation)
5. [Controller Implementation](#controller-implementation)
6. [Key Implementation Patterns](#key-implementation-patterns)
7. [Lessons for Racing Simulators](#lessons-for-racing-simulators)

---

## Architecture Overview

### The Implementation Chain

```
SCENARIO (.scenic)
    ↓ imports
CARLA MODEL (scenic.simulators.carla.model)
    ↓ imports
DRIVING DOMAIN (scenic.domains.driving.model)
    ↓ implements
CARLA SIMULATOR (scenic.simulators.carla.simulator)
    ↓ uses
CARLA ACTIONS (scenic.simulators.carla.actions)
    ↓ implements
DRIVING PROTOCOLS (Steers, Walks)
```

### Key Insight: **CARLA extends driving domain, doesn't replace it**

```scenic
# In carla/model.scenic:
from scenic.domains.driving.model import *  # ← Import EVERYTHING from driving

# Then add CARLA-specific features:
class CarlaActor(DrivingObject):  # ← Extends driving's DrivingObject
    carlaActor: None
    blueprint: None

class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):  # ← Multiple inheritance!
    def setThrottle(self, throttle):
        self.control.throttle = throttle  # ← Implements Steers protocol
```

---

## Domain Implementation Pattern

### 1. **Model Extension** (`carla/model.scenic`)

CARLA **extends** the driving domain model rather than replacing it:

```scenic
"""Scenic world model for traffic scenarios in CARLA.

The model currently supports vehicles, pedestrians, and props. It implements the
basic Car and Pedestrian classes from the scenic.domains.driving domain,
while also providing convenience classes for specific types of objects...
"""

# 1. IMPORT EVERYTHING FROM DRIVING DOMAIN
from scenic.domains.driving.model import *

# 2. IMPORT CARLA-SPECIFIC MODULES
import scenic.simulators.carla.blueprints as blueprints
from scenic.simulators.carla.behaviors import *
from scenic.simulators.carla.actions import *

# 3. ADD CARLA-SPECIFIC PARAMETERS
param carla_map = map_town
param address = '127.0.0.1'
param port = 2000
param timestep = 0.1
param weather = Uniform('ClearNoon', 'CloudyNoon', ...)

# 4. CREATE CARLA SIMULATOR INSTANCE
simulator CarlaSimulator(
    carla_map=globalParameters.carla_map,
    map_path=globalParameters.map,
    address=globalParameters.address,
    port=int(globalParameters.port),
    timestep=float(globalParameters.timestep)
)

# 5. EXTEND DRIVING OBJECTS WITH CARLA FEATURES
class CarlaActor(DrivingObject):
    """Abstract class for CARLA objects."""
    carlaActor: None  # ← CARLA-specific: reference to carla.Actor
    blueprint: None   # ← CARLA-specific: blueprint identifier
    rolename: None    # ← CARLA-specific: role name
    physics: True     # ← CARLA-specific: physics enabled
    snapToGround: globalParameters.snapToGroundDefault

    def setPosition(self, pos, elevation):
        # ← Implements DrivingObject.setPosition()
        self.carlaActor.set_location(_utils.scenicToCarlaLocation(pos, elevation))

    def setVelocity(self, vel):
        # ← Implements DrivingObject.setVelocity()
        cvel = _utils.scenicToCarlaVector3D(*vel)
        self.carlaActor.set_target_velocity(cvel)

# 6. IMPLEMENT PROTOCOLS WITH CARLA-SPECIFIC METHODS
class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    """Abstract class for steerable vehicles."""
    
    def setThrottle(self, throttle):
        # ← Implements Steers.setThrottle()
        self.control.throttle = throttle

    def setSteering(self, steering):
        # ← Implements Steers.setSteering()
        self.control.steer = steering

    def setBraking(self, braking):
        # ← Implements Steers.setBraking()
        self.control.brake = braking

    def setHandbrake(self, handbrake):
        # ← Implements Steers.setHandbrake()
        self.control.hand_brake = handbrake

    def setReverse(self, reverse):
        # ← Implements Steers.setReverse()
        self.control.reverse = reverse

# 7. CONCRETE OBJECT CLASSES WITH CARLA BLUEPRINTS
class Car(Vehicle):
    """A car."""
    blueprint: Uniform(*blueprints.carModels)  # ← CARLA-specific blueprints

class Pedestrian(Pedestrian, CarlaActor, Walks, _CarlaPedestrian):
    """A pedestrian."""
    blueprint: Uniform(*blueprints.walkerModels)
    
    def setWalkingDirection(self, heading):
        # ← Implements Walks.setWalkingDirection()
        direction = Vector(0, 1, 0).rotatedBy(heading)
        self.control.direction = _utils.scenicToCarlaVector3D(*direction)

    def setWalkingSpeed(self, speed):
        # ← Implements Walks.setWalkingSpeed()
        self.control.speed = speed
```

**Key Pattern**: CARLA **inherits** all driving domain functionality and **adds** CARLA-specific features on top.

---

## File-by-File Implementation

### 1. **`__init__.py`** - Domain Description

```python
"""Interface to the CARLA driving simulator.

This interface must currently be used in `2D compatibility mode`.

This interface has been tested with CARLA versions 0.9.9, 0.9.10, and 0.9.11.
It supports dynamic scenarios involving vehicles, pedestrians, and props.

The interface implements the scenic.domains.driving abstract domain, so any
object types, behaviors, utility functions, etc. from that domain may be used freely.
"""

# Only import CarlaSimulator if the carla package is installed
carla = None
try:
    import carla
except ImportError:
    pass
if carla:
    from .simulator import CarlaSimulator
del carla
```

**Key Points**:
- Documents that it **implements** the driving domain
- Handles optional dependency gracefully
- Only imports simulator if CARLA is available

### 2. **`simulator.py`** - Core Simulator Implementation

```python
"""Simulator interface for CARLA."""

from scenic.domains.driving.simulators import DrivingSimulation, DrivingSimulator

class CarlaSimulator(DrivingSimulator):
    """Implementation of Simulator for CARLA."""
    
    def __init__(self, carla_map, map_path, address="127.0.0.1", port=2000, ...):
        super().__init__()  # ← Call driving domain's simulator init
        
        # CARLA-specific setup
        self.client = carla.Client(address, port)
        self.world = self.client.load_world(carla_map)
        self.tm = self.client.get_trafficmanager(traffic_manager_port)
        
        # Configure CARLA for synchronous mode
        settings = self.world.get_settings()
        settings.synchronous_mode = True
        settings.fixed_delta_seconds = timestep
        self.world.apply_settings(settings)

    def createSimulation(self, scene, *, timestep, **kwargs):
        return CarlaSimulation(scene, self.client, self.tm, ...)

class CarlaSimulation(DrivingSimulation):
    """CARLA-specific simulation implementation."""
    
    def __init__(self, scene, client, tm, render, record, scenario_number, **kwargs):
        self.client = client
        self.world = self.client.get_world()
        self.map = self.world.get_map()
        self.blueprintLib = self.world.get_blueprint_library()
        self.tm = tm
        
        super().__init__(scene, **kwargs)  # ← Call driving domain's simulation init

    def createObjectInSimulator(self, obj):
        """Create a Scenic object in CARLA."""
        # Extract blueprint
        blueprint = self.blueprintLib.find(obj.blueprint)
        
        # Set up transform
        loc = utils.scenicToCarlaLocation(obj.position, ...)
        rot = utils.scenicToCarlaRotation(obj.orientation)
        transform = carla.Transform(loc, rot)
        
        # Create CARLA actor
        carlaActor = self.world.try_spawn_actor(blueprint, transform)
        obj.carlaActor = carlaActor  # ← Link Scenic object to CARLA actor
        
        # Configure physics
        carlaActor.set_simulate_physics(obj.physics)
        
        # Vehicle-specific setup
        if isinstance(carlaActor, carla.Vehicle):
            carlaActor.apply_control(carla.VehicleControl(manual_gear_shift=True, gear=1))
        
        return carlaActor

    def executeActions(self, allActions):
        super().executeActions(allActions)  # ← Call driving domain's action execution
        
        # Apply CARLA control updates
        for obj in self.agents:
            ctrl = obj._control
            if ctrl is not None:
                obj.carlaActor.apply_control(ctrl)  # ← Apply to CARLA actor
                obj._control = None

    def step(self):
        """Run simulation for one timestep."""
        self.current_frame = self.world.tick()  # ← CARLA-specific step
        
        # Wait for sensors
        for obj in self.objects:
            if obj.sensors:
                for sensor in obj.sensors.values():
                    while sensor.frame != self.current_frame:
                        pass

    def getProperties(self, obj, properties):
        """Extract properties from CARLA actor."""
        carlaActor = obj.carlaActor
        currTransform = carlaActor.get_transform()
        currLoc = currTransform.location
        currVel = carlaActor.get_velocity()
        
        # Convert CARLA properties to Scenic properties
        position = utils.carlaToScenicPosition(currLoc)
        velocity = utils.carlaToScenicPosition(currVel)
        speed = math.hypot(*velocity)
        
        return dict(
            position=position,
            velocity=velocity,
            speed=speed,
            # ... other properties
        )
```

**Key Pattern**: CARLA simulator **extends** `DrivingSimulator` and **implements** all abstract methods with CARLA-specific code.

### 3. **`actions.py`** - Action Implementation

```python
"""Actions for dynamic agents in CARLA scenarios."""

from scenic.domains.driving.actions import *  # ← Import all driving actions

# CARLA-SPECIFIC ACTIONS (extend driving actions)
class SetAngularVelocityAction(Action):
    """CARLA-specific action for setting angular velocity."""
    def applyTo(self, obj, sim):
        newAngularVel = _utils.scalarToCarlaVector3D(xAngularVel, yAngularVel)
        obj.carlaActor.set_angular_velocity(newAngularVel)

class SetTransformAction(Action):
    """CARLA-specific action for setting transform."""
    def applyTo(self, obj, sim):
        loc = _utils.scenicToCarlaLocation(self.pos, z=obj.elevation)
        rot = _utils.scenicToCarlaRotation(self.heading)
        transform = _carla.Transform(loc, rot)
        obj.carlaActor.set_transform(transform)

# VEHICLE-SPECIFIC ACTIONS
class _CarlaVehicle:
    """Mixin identifying CARLA vehicles."""
    pass

class VehicleAction(Action):
    """Base class for vehicle actions."""
    def canBeTakenBy(self, agent):
        return isinstance(agent, _CarlaVehicle)

class SetManualGearShiftAction(VehicleAction):
    """CARLA-specific gear control."""
    def applyTo(self, obj, sim):
        vehicle = obj.carlaActor
        ctrl = vehicle.get_control()
        ctrl.manual_gear_shift = self.manualGearShift
        vehicle.apply_control(ctrl)

class SetAutopilotAction(VehicleAction):
    """CARLA's autopilot system."""
    def applyTo(self, obj, sim):
        vehicle = obj.carlaActor
        vehicle.set_autopilot(self.enabled, sim.tm.get_port())
        
        if self.path:
            sim.tm.set_route(vehicle, self.path)
        if self.speed:
            sim.tm.set_desired_speed(vehicle, 3.6 * self.speed)

# PEDESTRIAN-SPECIFIC ACTIONS
class _CarlaPedestrian:
    """Mixin identifying CARLA pedestrians."""
    pass

class PedestrianAction(Action):
    """Base class for pedestrian actions."""
    def canBeTakenBy(self, agent):
        return isinstance(agent, _CarlaPedestrian)

class SetJumpAction(PedestrianAction):
    """CARLA-specific pedestrian jumping."""
    def applyTo(self, obj, sim):
        walker = obj.carlaActor
        ctrl = walker.get_control()
        ctrl.jump = self.jump
        walker.apply_control(ctrl)
```

**Key Pattern**: CARLA actions **import** all driving actions and **add** CARLA-specific ones. They use **protocol mixins** (`_CarlaVehicle`, `_CarlaPedestrian`) to identify which objects can use which actions.

### 4. **`behaviors.scenic`** - Behavior Implementation

```scenic
"""Behaviors for dynamic agents in CARLA scenarios."""

from scenic.domains.driving.behaviors import *  # ← Import all driving behaviors

try:
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    pass  # ignore; error will be caught later

# CARLA-SPECIFIC BEHAVIORS (extend driving behaviors)
behavior AutopilotBehavior(enabled = True, **kwargs):
    """Behavior causing a vehicle to use CARLA's built-in autopilot."""
    take SetAutopilotAction(enabled=enabled, **kwargs)

behavior WalkForwardBehavior(speed=0.5):
    """CARLA-specific walking behavior."""
    take SetWalkingDirectionAction(self.heading), SetWalkingSpeedAction(speed)

behavior WalkBehavior(maxSpeed=1.4):
    """CARLA's AI walker behavior."""
    take SetWalkAction(True, maxSpeed)

behavior CrossingBehavior(reference_actor, min_speed=1, threshold=10, final_speed=None):
    """CARLA-specific crossing behavior with synchronization."""
    
    while (distance from self to reference_actor) > threshold:
        wait

    while True:
        distance_vec = self.position - reference_actor.position
        rotated_vec = distance_vec.rotatedBy(-reference_actor.heading)
        
        ref_dist = rotated_vec.y
        if ref_dist < 0:
            break  # Reference actor has passed
        
        actor_dist = rotated_vec.x
        ref_speed = reference_actor.speed
        ref_time = ref_speed / ref_dist
        actor_speed = actor_dist * ref_time
        
        if isinstance(self, Walks):
            do WalkForwardBehavior(actor_speed)
        elif isinstance(self, Steers):
            take SetSpeedAction(actor_speed)
```

**Key Pattern**: CARLA behaviors **import** all driving behaviors and **add** CARLA-specific ones. They use **protocol checking** (`isinstance(self, Walks)`) to determine which actions to use.

### 5. **`blueprints.py`** - CARLA-Specific Assets

```python
"""CARLA blueprints for cars, pedestrians, etc."""

# CARLA BLUEPRINT DEFINITIONS
carModels = [
    "vehicle.audi.a2",
    "vehicle.audi.etron",
    "vehicle.bmw.grandtourer",
    "vehicle.chevrolet.impala",
    # ... more car models
]

walkerModels = [
    "walker.pedestrian.0001",
    "walker.pedestrian.0002",
    # ... more pedestrian models
]

# BLUEPRINT COMPATIBILITY
oldBlueprintNames = {
    "vehicle.dodge.charger_police": ("vehicle.dodge_charger.police",),
    "vehicle.lincoln.mkz_2017": ("vehicle.lincoln.mkz2017",),
    # ... handle CARLA version changes
}
```

**Key Pattern**: CARLA provides **concrete asset definitions** (blueprints) that map to CARLA's 3D models. This is simulator-specific and wouldn't exist in the domain layer.

---

## Protocol Implementation

### How CARLA Implements Driving Domain Protocols

The driving domain defines **abstract protocols** (mixins) that simulators must implement:

#### 1. **`Steers` Protocol** (from `driving/actions.py`)

```python
# In driving/actions.py:
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
```

#### 2. **CARLA Implementation** (in `carla/model.scenic`)

```scenic
class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    """Abstract class for steerable vehicles."""
    
    def setThrottle(self, throttle):
        # ← Implements Steers.setThrottle()
        self.control.throttle = throttle

    def setSteering(self, steering):
        # ← Implements Steers.setSteering()
        self.control.steer = steering

    def setBraking(self, braking):
        # ← Implements Steers.setBraking()
        self.control.brake = braking

    def setHandbrake(self, handbrake):
        # ← Implements Steers.setHandbrake()
        self.control.hand_brake = handbrake

    def setReverse(self, reverse):
        # ← Implements Steers.setReverse()
        self.control.reverse = reverse
```

#### 3. **`Walks` Protocol** Implementation

```python
# In driving/actions.py:
class Walks:
    """Mixin protocol for agents which can walk."""
    def setWalkingDirection(self, heading):
        velocity = Vector(0, self.speed).rotatedBy(heading)
        self.setVelocity(velocity)
    
    def setWalkingSpeed(self, speed):
        velocity = speed * self.velocity.normalized()
        self.setVelocity(velocity)
```

```scenic
# In carla/model.scenic:
class Pedestrian(Pedestrian, CarlaActor, Walks, _CarlaPedestrian):
    """A pedestrian."""
    
    def setWalkingDirection(self, heading):
        # ← Implements Walks.setWalkingDirection()
        direction = Vector(0, 1, 0).rotatedBy(heading)
        self.control.direction = _utils.scenicToCarlaVector3D(*direction)

    def setWalkingSpeed(self, speed):
        # ← Implements Walks.setWalkingSpeed()
        self.control.speed = speed
```

**Key Pattern**: CARLA **implements** the abstract protocol methods with **CARLA-specific control mechanisms**.

---

## Controller Implementation

### How CARLA Provides Controllers

The driving domain expects simulators to provide **PID controllers** for behaviors:

#### 1. **Expected Interface** (from `driving/behaviors.scenic`)

```scenic
# In FollowLaneBehavior:
_lon_controller, _lat_controller = simulation().getLaneFollowingControllers(self)

# In TurnBehavior:
_lon_controller, _lat_controller = simulation().getTurningControllers(self)

# In LaneChangeBehavior:
_lon_controller, _lat_controller = simulation().getLaneChangingControllers(self)
```

#### 2. **CARLA Implementation** (in `carla/simulator.py`)

```python
class CarlaSimulator(DrivingSimulator):
    """Implementation of Simulator for CARLA."""
    
    def getLaneFollowingControllers(self, agent):
        """Return controllers for lane following."""
        # CARLA could provide its own PID controllers
        # or use the default ones from driving domain
        from scenic.domains.driving.controllers import PIDLateralController, PIDLongitudinalController
        
        lateral = PIDLateralController()
        longitudinal = PIDLongitudinalController()
        return lateral, longitudinal
    
    def getTurningControllers(self, agent):
        """Return controllers tuned for turning."""
        # Different tuning for turning vs lane following
        lateral = PIDLateralController(kp=1.0, ki=0.0, kd=0.0)  # More aggressive
        longitudinal = PIDLongitudinalController(kp=0.5, ki=0.0, kd=0.0)  # Slower
        return lateral, longitudinal
```

**Key Pattern**: CARLA **provides** the controller interface expected by driving behaviors, either by implementing its own controllers or using the default ones from the driving domain.

---

## Key Implementation Patterns

### 1. **Multiple Inheritance Pattern**

CARLA uses **multiple inheritance** to combine domain functionality with simulator-specific features:

```scenic
class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    #     ↑        ↑         ↑       ↑
    #   Domain  Simulator Protocol  Type
```

- **`Vehicle`** (first): From driving domain - provides basic vehicle functionality
- **`CarlaActor`**: CARLA-specific base class - provides `carlaActor`, `blueprint`, etc.
- **`Steers`**: Protocol from driving domain - defines interface for steering
- **`_CarlaVehicle`**: CARLA-specific mixin - identifies this as a CARLA vehicle

### 2. **Protocol Implementation Pattern**

CARLA **implements** abstract protocols with **concrete simulator methods**:

```python
# Abstract protocol (driving domain):
class Steers:
    def setThrottle(self, throttle):
        raise NotImplementedError

# Concrete implementation (CARLA):
class Vehicle(Vehicle, CarlaActor, Steers, _CarlaVehicle):
    def setThrottle(self, throttle):
        self.control.throttle = throttle  # ← CARLA-specific implementation
```

### 3. **Object Linking Pattern**

CARLA **links** Scenic objects to simulator actors:

```python
# In createObjectInSimulator():
carlaActor = self.world.try_spawn_actor(blueprint, transform)
obj.carlaActor = carlaActor  # ← Link Scenic object to CARLA actor

# Later, actions can access the CARLA actor:
def setThrottle(self, throttle):
    self.control.throttle = throttle  # self.control accesses carlaActor.get_control()
```

### 4. **Action Accumulation Pattern**

CARLA **accumulates** control updates and applies them in batch:

```python
# In actions:
def setThrottle(self, throttle):
    self.control.throttle = throttle  # ← Modifies control object

# In simulator:
def executeActions(self, allActions):
    super().executeActions(allActions)
    
    # Apply accumulated control updates
    for obj in self.agents:
        ctrl = obj._control
        if ctrl is not None:
            obj.carlaActor.apply_control(ctrl)  # ← Apply to CARLA actor
            obj._control = None
```

### 5. **Coordinate Conversion Pattern**

CARLA **converts** between Scenic coordinates and CARLA coordinates:

```python
# Scenic → CARLA
def scenicToCarlaLocation(pos, elevation):
    return carla.Location(x=pos.x, y=pos.y, z=elevation)

# CARLA → Scenic  
def carlaToScenicPosition(loc):
    return Vector(loc.x, loc.y)
```

### 6. **Graceful Degradation Pattern**

CARLA **handles** missing dependencies gracefully:

```python
# In model.scenic:
try:
    from scenic.simulators.carla.simulator import CarlaSimulator
    from scenic.simulators.carla.actions import *
except ModuleNotFoundError:
    # Provide dummy implementations for compilation without CARLA
    def CarlaSimulator(*args, **kwargs):
        raise RuntimeError('the "carla" package is required to run simulations')
```

---

## Lessons for Racing Simulators

### How Racing Simulators Should Follow CARLA's Pattern

Based on CARLA's implementation, here's how racing simulators should implement the racing domain:

#### 1. **Racing Simulator Structure**

```
scenic/simulators/[simulator_name]/
├── __init__.py              # Domain description + optional imports
├── simulator.py              # RacingSimulator extends RacingSimulator
├── model.scenic             # Extends racing.model with simulator features
├── actions.py               # Implements racing actions + simulator-specific
├── behaviors.scenic         # Implements racing behaviors + simulator-specific
├── blueprints.py            # Simulator-specific assets/models
└── utils/                   # Coordinate conversion, utilities
```

#### 2. **Model Extension Pattern** (for racing simulators)

```scenic
# In simulator/model.scenic:
from scenic.domains.racing.model import *  # ← Import EVERYTHING from racing domain

# Add simulator-specific parameters
param simulator_specific_param = "value"

# Create simulator instance
simulator MyRacingSimulator(...)

# Extend racing objects with simulator features
class SimulatorRacingActor(RacingObject):
    """Base class for simulator-specific racing objects."""
    simulatorActor: None  # ← Link to simulator's internal representation
    model: None          # ← Simulator-specific model identifier
    
    def setPosition(self, pos, elevation):
        # ← Implements RacingObject.setPosition()
        self.simulatorActor.set_position(self._convertPosition(pos, elevation))

# Implement racing protocols with simulator-specific methods
class RacingCar(RacingCar, SimulatorRacingActor, RacingSteers, _SimulatorRacingCar):
    """Racing car with simulator-specific implementation."""
    
    def setDRS(self, activate):
        # ← Implements racing domain's DRS action
        self.simulatorActor.set_drs(activate)
    
    def deployERS(self, mode, amount):
        # ← Implements racing domain's ERS action
        self.simulatorActor.deploy_ers(mode, amount)
    
    def setPitLimiter(self, activate):
        # ← Implements racing domain's pit limiter
        self.simulatorActor.set_pit_limiter(activate)
```

#### 3. **Simulator Implementation Pattern**

```python
# In simulator/simulator.py:
from scenic.domains.racing.simulators import RacingSimulator, RacingSimulation

class MyRacingSimulator(RacingSimulator):
    """Implementation of RacingSimulator for [Simulator Name]."""
    
    def __init__(self, track_file, **kwargs):
        super().__init__()
        
        # Simulator-specific setup
        self.track = self.loadTrack(track_file)
        self.physics_engine = self.initializePhysics()
        
    def createSimulation(self, scene, **kwargs):
        return MyRacingSimulation(scene, self.track, self.physics_engine, **kwargs)
    
    def getRacingControllers(self, agent):
        """Return controllers optimized for racing."""
        # Provide racing-specific PID tuning
        lateral = PIDLateralController(kp=2.0, ki=0.1, kd=0.5)  # Aggressive for racing
        longitudinal = PIDLongitudinalController(kp=1.5, ki=0.2, kd=0.3)
        return lateral, longitudinal

class MyRacingSimulation(RacingSimulation):
    """Racing simulation implementation."""
    
    def createObjectInSimulator(self, obj):
        """Create racing object in simulator."""
        # Create simulator's internal representation
        simulatorActor = self.track.spawn_vehicle(obj.model, obj.position)
        obj.simulatorActor = simulatorActor
        
        # Configure racing-specific properties
        if hasattr(obj, 'raceNumber'):
            simulatorActor.set_race_number(obj.raceNumber)
        
        return simulatorActor
    
    def executeActions(self, allActions):
        super().executeActions(allActions)
        
        # Apply racing-specific actions
        for obj in self.agents:
            if hasattr(obj, '_racing_control'):
                obj.simulatorActor.apply_racing_control(obj._racing_control)
```

#### 4. **Action Implementation Pattern**

```python
# In simulator/actions.py:
from scenic.domains.racing.actions import *  # ← Import all racing actions

class _SimulatorRacingCar:
    """Mixin identifying simulator racing cars."""
    pass

class RacingAction(Action):
    """Base class for racing actions."""
    def canBeTakenBy(self, agent):
        return isinstance(agent, _SimulatorRacingCar)

class SimulatorDRSAction(RacingAction):
    """Simulator-specific DRS implementation."""
    def applyTo(self, obj, sim):
        obj.simulatorActor.set_drs(self.activate)

class SimulatorERSDeployAction(RacingAction):
    """Simulator-specific ERS implementation."""
    def applyTo(self, obj, sim):
        obj.simulatorActor.deploy_ers(self.mode, self.amount)

class SimulatorPitLimiterAction(RacingAction):
    """Simulator-specific pit limiter implementation."""
    def applyTo(self, obj, sim):
        obj.simulatorActor.set_pit_limiter(self.activate)
```

#### 5. **Behavior Implementation Pattern**

```scenic
# In simulator/behaviors.scenic:
from scenic.domains.racing.behaviors import *  # ← Import all racing behaviors

# Simulator-specific racing behaviors
behavior SimulatorFollowRacingLineBehavior(target_speed=30):
    """Follow racing line using simulator's controllers."""
    
    # Get racing-specific controllers
    _lon_controller, _lat_controller = simulation().getRacingControllers(self)
    
    while True:
        # Use racing line from track
        cte = track.racingLine.signedDistanceTo(self.position)
        speed_error = target_speed - self.speed
        
        throttle = _lon_controller.run_step(speed_error)
        steer = _lat_controller.run_step(cte)
        
        take RegulatedControlAction(throttle, steer, past_steer)

behavior SimulatorPitStopBehavior(pitBox):
    """Execute pit stop using simulator's pit mechanics."""
    
    # Enter pit lane
    take SimulatorPitLimiterAction(activate=True)
    do FollowRacingLineBehavior(target_speed=20) until self in pitBox
    
    # Stop in pit box
    take SetBrakeAction(1.0)
    wait  # Simulate pit stop time
    
    # Exit pit lane
    take SimulatorPitLimiterAction(activate=False)
```

---

## Summary: The CARLA Implementation Template

### For Any Racing Simulator

1. **Extend, Don't Replace**: Import everything from racing domain (`from scenic.domains.racing.model import *`)

2. **Implement Protocols**: Provide concrete implementations of racing domain protocols (DRS, ERS, pit limiter, etc.)

3. **Link Objects**: Connect Scenic racing objects to simulator's internal representations

4. **Provide Controllers**: Implement `getRacingControllers()` method with racing-optimized PID tuning

5. **Handle Coordinates**: Convert between Scenic coordinates and simulator coordinates

6. **Graceful Degradation**: Handle missing dependencies gracefully for compilation without simulator

7. **Multiple Inheritance**: Use multiple inheritance to combine domain functionality with simulator features

8. **Action Accumulation**: Accumulate control updates and apply them efficiently

### The Key Insight

**CARLA doesn't reinvent the driving domain - it implements it.** Similarly, racing simulators shouldn't reinvent the racing domain - they should implement it.

The racing domain provides the **abstraction** (protocols, behaviors, objects), and racing simulators provide the **implementation** (concrete methods, simulator-specific features, coordinate conversion).

This separation allows:
- **Domain scenarios** to work with any racing simulator
- **Simulator-specific features** to be added without breaking compatibility
- **Reusable behaviors** across different racing simulators
- **Clean architecture** with clear separation of concerns

---

## Quick Reference: CARLA Implementation Checklist

For racing simulators implementing the racing domain:

- [ ] **Extend racing domain**: `from scenic.domains.racing.model import *`
- [ ] **Implement protocols**: DRS, ERS, pit limiter, etc.
- [ ] **Provide controllers**: `getRacingControllers()` method
- [ ] **Link objects**: Connect Scenic objects to simulator actors
- [ ] **Convert coordinates**: Scenic ↔ Simulator coordinate systems
- [ ] **Handle dependencies**: Graceful degradation when simulator not available
- [ ] **Use multiple inheritance**: Combine domain + simulator + protocols
- [ ] **Accumulate actions**: Batch control updates for efficiency
- [ ] **Add simulator features**: Blueprints, models, simulator-specific actions
- [ ] **Test compatibility**: Ensure racing domain scenarios work

---

*CARLA's implementation of the driving domain serves as the gold standard for how simulators should implement Scenic domains. Racing simulators should follow this exact pattern when implementing the racing domain.*

