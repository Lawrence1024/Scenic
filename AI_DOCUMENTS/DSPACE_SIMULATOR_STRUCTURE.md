# dSPACE Simulator Architecture Documentation

## Overview

The dSPACE simulator integration provides a comprehensive interface between Scenic scenarios and the dSPACE simulation environment (ModelDesk, ControlDesk, VEOS, and Aurelion). This document provides a complete structural overview of the entire dSPACE simulator package.

**Location:** `src/scenic/simulators/dspace/`

## Directory Structure

```
src/scenic/simulators/dspace/
├── __init__.py                 # Main exports
├── simulator.py               # Core simulator implementation (1613 lines)
├── utils.py                   # Legacy compatibility utilities
├── actions.py                 # dSPACE-specific actions
├── blueprints.py              # Vehicle model definitions
├── model.scenic               # Base dSPACE model
├── racing_model.scenic        # Racing-specific model
├── controldesk/               # Runtime control interface
│   ├── __init__.py
│   ├── connection.py          # ControlDesk COM wrapper
│   └── per_tick_control.py    # Per-tick control implementation
├── geometry/                  # Coordinate system handling
│   ├── __init__.py
│   ├── coordinate_transform.py # XODR→RD transformation
│   ├── projection.py          # World-to-road projection
│   ├── rd_parser.py           # RD geometry parser
│   ├── utils.py               # COM helper utilities
│   └── xodr_parser.py         # XODR geometry parser
├── modeldesk/                 # Scenario authoring
│   ├── __init__.py
│   ├── connection.py          # ModelDesk COM connection
│   ├── scenario.py            # Scenario management
│   └── vehicle_placement.py   # Vehicle positioning logic
├── converters/                # Format conversion utilities
│   ├── __init__.py
│   ├── rd_to_xodr.py          # RD→XODR converter
│   └── set_ego_position.py    # Ego positioning utilities
└── sensors/                   # Sensor integration
    ├── __init__.py
    └── aurelion.py            # Aurelion sensor hub
```

## Core Components

### 1. Simulator (`simulator.py`)

The main simulator implementation that orchestrates the entire dSPACE integration.

#### Key Classes

**`DSpaceSimulator`**
- Main simulator class extending `RacingSimulator`
- Configuration: scenario source, name, timestep
- Creates simulation instances

**`DSpaceSimulation`**
- Core simulation implementation extending `RacingSimulation`
- Manages two-phase architecture:
  - **Phase 1:** ModelDesk scenario authoring (static positioning)
  - **Phase 2:** ControlDesk runtime control (dynamic behaviors)

#### Key Responsibilities

1. **Initialization (`setup()` method):**
   - Connect to ModelDesk COM application
   - Save-as scenario copy for isolation
   - Build road geometry index from XODR/RD files
   - Apply coordinate transformations (XODR→RD)
   - Create ego and fellow vehicles
   - Author scenario in ModelDesk if dynamic control needed
   - Connect to ControlDesk for runtime control
   - Initialize VesiInterface manual control

2. **Vehicle Creation:**
   - `createObjectInSimulator()`: Routes to ego/fellow creation
   - `createEgoInSimulator()`: Configures ego via Maneuver API
   - `createFellowInSimulator()`: Creates fellows via Fellows API

3. **Dynamic Control:**
   - `setVehicleControl()`: Apply throttle/brake/steering via VesiInterface
   - `setVehicleGear()`: Set gear (one-shot action)
   - `setVehicleClutch()`: Set clutch pedal
   - `getVehicleState()`: Retrieve vehicle state

4. **Route Management:**
   - `detectTrackSegment()`: Identify pit lane vs main track
   - `assignRoute()`: Map track segments to dSPACE routes
   - `_set_fellow_route_via_sequence()`: Configure routes in ModelDesk

5. **VesiInterface Integration:**
   - `_initializeVesiInterface()`: Setup manual control interface
   - Configure master switches, race control, and channel enables

### 2. Actions (`actions.py`)

dSPACE-specific action definitions.

**`_DSpaceVehicle`**
- Marker mixin to identify dSPACE-backed vehicles
- Used for action gating

**`SetVehicleControl`**
- Combined control action (throttle + brake + steering)
- dSPACE-specific convenience action
- Note: Individual controls use driving domain actions (SetThrottleAction, etc.)

### 3. Models

**`model.scenic`**
- Base dSPACE model that imports racing domain
- Minimal implementation, delegates to racing_model

**`racing_model.scenic`**
- Full racing-specific implementation
- Defines `DSPACERacingCar` class implementing:
  - `RacingCar` protocol
  - `_DSpaceVehicle` marker
  - `Steers` protocol
  - `HasManualTransmission` protocol

#### DSPACERacingCar Protocol Methods

- `setMaxSpeed(max_speed)`: Set maximum speed
- `setTTL(ttl)`: Set time-to-live
- `setThrottle(throttle)`: Throttle control (0-1)
- `setSteering(steering)`: Steering control (-1 to 1)
- `setBraking(braking)`: Brake control (0-1)
- `setHandbrake()`: Not implemented
- `setReverse()`: Not implemented
- `setGear(gear)`: Gear selection (one-shot)
- `setClutch(clutch)`: Clutch control (0-1, one-shot)

### 4. ControlDesk Module

Runtime control interface for dynamic vehicle behaviors.

#### `connection.py` - ControlDeskApp

Lightweight COM automation wrapper for ControlDesk.

**Methods:**
- `connect()`: Initialize COM and connect to application
- `go_online()`: Start online calibration
- `go_offline()`: Stop online calibration
- `start_measurement()`: Begin data acquisition
- `stop_measurement()`: End data acquisition
- `get_var(path)`: Read variable value
- `set_var(path, value)`: Write variable value

#### `per_tick_control.py` - PerTickController

Per-tick control implementation (10ms timing, dt=0.01).

**Methods:**
- `connectControlDesk()`: Connect with CTun verification
- `getControlVariables()`: Retrieve vehicle control variables
- `setVehicleControl()`: Apply control inputs per tick
- `startPerTickControl()`: Launch control loop thread
- `stopPerTickControl()`: Terminate control loop

#### `per_tick_control.py` - ExternalControlManager

Manager for external control flags via ASM_Maneuver.py script.

**Methods:**
- `enableExternalControlViaScript()`: Enable control via Docker/VEOS script

### 5. Geometry Module

Coordinate system transformation and projection utilities.

#### `coordinate_transform.py`

Automatic XODR→RD coordinate transformation builder.

**Key Functions:**
- `build_coordinate_transform()`: Create transform from sample points
- `apply_coordinate_transform()`: Apply transformation to coordinates
- `save_transform()`: Cache transformation to JSON
- `load_transform()`: Load cached transformation

**Algorithm:**
1. Parse XODR and RD geometries
2. Sample points at regular s-intervals
3. Compute affine transformation via least squares
4. Validate transformation accuracy

#### `projection.py`

World-to-road coordinate projection.

**Key Functions:**
- `project_world_to_st()`: Project (x,y) → (s,t)
- `find_road_id_for_position()`: Identify road for position

**Features:**
- Multi-road projection with distance sorting
- Scaling for t-coordinate calibration
- Robust handling of off-road projections

#### `xodr_parser.py`

XODR geometry parser.

**Key Functions:**
- `build_xodr_sec_points()`: Build road index with independent s-coordinates

**Features:**
- Filters to main roads only (not junctions)
- Creates independent road segments
- Samples lines and arcs
- Compatible with `project_world_to_st()`

#### `rd_parser.py`

RD geometry parser (native dSPACE format).

**Key Functions:**
- `parse_rd_geometry()`: Parse RD file into road segments
- `build_rd_road_index()`: Build road index compatible with projection

**Features:**
- Parses cubic polynomial segments
- Independent s-coordinate systems per road
- Native Aurelion coordinate system

#### `utils.py`

COM helper utilities for ModelDesk automation.

**Key Functions:**
- `clear_collection()`: Clear COM collection
- `ensure_two_segments()`: Ensure sequence has 2 segments
- `activate_type()`: Activate typed object element
- `set_activity_constant()`: Set constant value
- `make_endless_transition()`: Configure endless transition
- `configure_seg0_absolute_pose()`: Set absolute pose (s,t)
- `configure_seg1_motion()`: Configure velocity motion

**Constants:**
- `MAIN_ROAD_NAMES`: Main road identifiers for Laguna Seca

### 6. ModelDesk Module

Scenario authoring and vehicle placement.

#### `connection.py` - ModelDeskConnection

COM connection wrapper.

**Methods:**
- `connect()`: Connect to ModelDesk application
- `get_traffic_scenario()`: Access active TrafficScenario

#### `scenario.py` - ScenarioManager

Scenario lifecycle management.

**Methods:**
- `save_as_scenario()`: Save-as scenario copy
- `clear_fellows()`: Clear existing fellows
- `start_simulation()`: Save, download, reset, start

#### `vehicle_placement.py`

Vehicle positioning logic.

**Key Functions:**
- `project_scenic_to_st()`: Project Scenic position to (s,t)
- `create_fellow_vehicle()`: Create fellow with positioning
- `create_ego_vehicle()`: Configure ego maneuver

**Features:**
- Applies coordinate transformations
- Configures absolute pose segments
- Sets up velocity motion segments
- Configures routes

### 7. Converters Module

Format conversion and utility scripts.

#### `rd_to_xodr.py`

Comprehensive RD→XODR converter for Laguna Seca.

**Features:**
- Multiple modes:
  - `--comprehensive`: Full circuit with junctions
  - `--outer-loop`: Main track only
  - `--pit-lane`: Corkscrew + pit lane
- Junction detection and connection
- Proper lane linking
- Self-loop handling for closed roads

**Usage:**
```bash
python rd_to_xodr.py --rd Laguna_Seca.rd --xodr output.xodr --comprehensive
```

#### `set_ego_position.py`

Script to set ego vehicle starting position.

**Parameters:**
- `--s`: Longitudinal position (meters)
- `--t`: Lateral offset (meters)
- `--velocity`: Initial velocity (m/s)
- `--orientation`: Orientation angle (degrees)
- `--lane`: Lane index
- `--height`: Z-position (meters)

**Usage:**
```bash
python set_ego_position.py --s 800 --velocity 15
```

### 8. Sensors Module

Sensor integration framework.

#### `aurelion.py` - AurelionSensorHub

REST client for Aurelion sensor data.

**Methods:**
- `read_sim_time()`: Read simulation time
- `wait_until_time()`: Wait for target time
- `barrier_flush()`: Frame synchronization hook

**Constants:**
- `SIM_TIME_KEY`: Simulation time key
- `FRAME_KEY`: Frame index key

### 9. Utilities (`utils.py`)

Legacy compatibility layer.

**Re-exports:**
- Geometry functions from `geometry/`
- ModelDesk functions from `modeldesk/`
- ControlDesk functions from `controldesk/`

**Purpose:**
- Backward compatibility
- Migration path from old code
- Single import convenience

### 10. Blueprints (`blueprints.py`)

Vehicle model definitions.

**Constants:**
- `carModels`: Vehicle model names
- `walkerModels`: Pedestrian models (empty)
- `propModels`: Prop models (empty)

## Architecture Overview

### Two-Phase Design

**Phase 1: ModelDesk Scenario Authoring**
- Static vehicle positioning
- Route configuration
- Initial state setup
- Automated via COM

**Phase 2: ControlDesk Runtime Control**
- Dynamic control inputs
- Per-tick updates (10ms)
- VesiInterface manual control
- Behavior execution

### Coordinate System Transformation

**Challenge:** Scenic uses XODR coordinates, Aurelion uses RD coordinates
**Solution:** Automatic transformation via sampled point calibration

**Pipeline:**
1. Parse XODR and RD geometries
2. Sample at regular s-intervals
3. Compute affine transformation
4. Cache for performance
5. Apply during vehicle positioning

### Route Management

**Detection:**
- Auto-detect track segment from road projection
- Map segment to dSPACE route preference
- Fallback to explicit attributes

**Configuration:**
- Use FellowSequence.Route API
- Enumerate AvailableElements
- Smart matching (exact, substring, regex)
- Direction and external control flags

### VesiInterface Integration

**Purpose:** Unified manual control interface

**Configuration:**
- Master switches (CLIF, Manual Overwrite)
- Race control (System state, track/vehicle flags)
- Channel enables (throttle, brake, steering, gear)

**Control Path:**
- Scenic actions → Simulator methods → ControlDesk variables
- Scaling: Scenic 0-1 → VesiInterface 0-100
- One-shot actions: Gear, Clutch

## Integration Points

### With Scenic Core

1. **Simulation Interface:** Extends `RacingSimulation`
2. **Action Protocol:** Implements `Steers`, `HasManualTransmission`
3. **Agent Creation:** `createObjectInSimulator()` hook
4. **State Updates:** `step()`, `getProperties()`

### With Racing Domain

1. **Track Segments:** pitLane vs mainRacing
2. **Route Assignment:** Lap vs Pit
3. **Racing Behaviors:** Integration with racing behaviors
4. **Protocol Methods:** setMaxSpeed, setTTL

### With dSPACE Environment

1. **ModelDesk:** COM automation for authoring
2. **ControlDesk:** COM automation for runtime control
3. **VEOS:** Simulator runtime
4. **Aurelion:** Sensor data and rendering
5. **CTun:** Communication tunnel

## Configuration

### Scenario Parameters

```scenic
param scenario_src = "LagunaSeca_ExternalControl"
param scenario_name = None
param timestep = 0.1
param map = localPath('path/to/LagunaSeca.xodr')
```

### Simulator Configuration

```scenic
simulator dspace.DSpaceSimulator(
    scenario_src=globalParameters.scenario_src,
    scenario_name=globalParameters.scenario_name,
    timestep=globalParameters.timestep,
)
```

### Road Network Setup

- XODR file for Scenic positioning
- RD file for Aurelion rendering
- Coordinate transformation cache
- Main road filtering

## Error Handling

### Common Issues

1. **COM Connection Failures:**
   - Verify ModelDesk/ControlDesk running
   - Check COM registration

2. **Coordinate Mismatches:**
   - Ensure both XODR and RD files present
   - Rebuild coordinate transformation

3. **Route Configuration:**
   - Verify AvailableElements accessible
   - Check route names match

4. **ControlDesk Connectivity:**
   - Ensure CTun running
   - Verify VEOS registration

## Testing and Debugging

### Debug Outputs

- Geometric projections with coordinates
- Route detection and assignment
- Control value scaling and application
- COM operation results

### Verification Points

1. Vehicle positions in ModelDesk
2. Coordinate transformations
3. Route assignments
4. ControlDesk variable access
5. VesiInterface configuration

## Future Enhancements

### Potential Improvements

1. **Sensor Integration:**
   - Full Aurelion RGB pipeline
   - Lidar/radar support
   - Multi-camera setups

2. **Advanced Control:**
   - Trajectory following
   - Adaptive cruise control
   - Lane keeping assistance

3. **Coordinated Multi-Vehicle:**
   - Formation driving
   - Collective maneuvers
   - Traffic orchestration

4. **Real-time Debugging:**
   - Live visualization
   - Control value monitoring
   - State trajectory playback

## Related Documentation

- `DSPACE_CONTROL_INTERFACES.md`: Control system details
- `VEHICLE_CONTROL_IMPLEMENTATION.md`: Vehicle control specifics
- `SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md`: Overall architecture

## Summary

The dSPACE simulator provides a comprehensive, production-ready integration between Scenic scenarios and the dSPACE simulation environment. Its two-phase architecture cleanly separates static authoring from dynamic runtime control, while sophisticated coordinate transformation handles the XODR→RD mapping challenge. The modular design enables easy extension for new features while maintaining backward compatibility.

