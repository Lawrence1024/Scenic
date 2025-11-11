# dSPACE Simulator Comprehensive Guide

**Complete technical documentation for the Scenic-dSPACE simulator integration**

## Table of Contents

1. [Overview & Architecture](#overview--architecture)
2. [Directory Structure](#directory-structure)
3. [Core Components](#core-components)
4. [Vehicle Control Module](#vehicle-control-module)
5. [Control Interfaces](#control-interfaces)
6. [Coordinate Transformation Pipeline](#coordinate-transformation-pipeline)
7. [Integration Points](#integration-points)
8. [Configuration](#configuration)
9. [Troubleshooting](#troubleshooting)

---

## Overview & Architecture

The dSPACE simulator integration provides a comprehensive interface between Scenic scenarios and the dSPACE simulation environment (ModelDesk, ControlDesk, VEOS, and Aurelion).

**Location:** `src/scenic/simulators/dspace/`

### Two-Phase Architecture

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

### Key Challenge: Coordinate Systems

**Problem:** Scenic uses XODR coordinates, but Aurelion uses RD coordinates  
**Solution:** Automatic transformation via sampled point calibration

---

## Directory Structure

```
src/scenic/simulators/dspace/
├── __init__.py                 # Main exports
├── simulator.py               # Core simulator (1673 lines, 78.7 KB)
├── utils.py                   # Legacy compatibility utilities
├── actions.py                 # dSPACE-specific actions
├── blueprints.py              # Vehicle model definitions
├── model.scenic               # Base dSPACE model
├── racing_model.scenic        # Racing-specific model
│
├── vehicle/                   # Vehicle control module ⭐ NEW
│   ├── __init__.py
│   ├── physics.py             # VehiclePhysicsState (131 lines)
│   └── controller.py          # VehicleController (222 lines)
│
├── controldesk/               # Runtime control interface
│   ├── __init__.py
│   ├── connection.py          # ControlDesk COM wrapper
│   └── per_tick_control.py    # Per-tick control implementation
│
├── geometry/                  # Coordinate system handling
│   ├── __init__.py
│   ├── coordinate_transform.py # XODR→RD transformation
│   ├── projection.py          # World-to-road projection
│   ├── rd_parser.py           # RD geometry parser
│   ├── utils.py               # COM helper utilities
│   └── xodr_parser.py         # XODR geometry parser
│
├── modeldesk/                 # Scenario authoring
│   ├── __init__.py
│   ├── connection.py          # ModelDesk COM connection
│   ├── scenario.py            # Scenario management
│   └── vehicle_placement.py   # Vehicle positioning logic
│
├── converters/                # Format conversion utilities
│   ├── __init__.py
│   ├── rd_to_xodr.py          # RD→XODR converter
│   └── set_ego_position.py    # Ego positioning utilities
│
└── sensors/                   # Sensor integration
    ├── __init__.py
    └── aurelion.py            # Aurelion sensor hub
```

---

## Core Components

### 1. Simulator (`simulator.py`)

Main simulator implementation orchestrating the entire dSPACE integration.

#### Key Classes

**`DSpaceSimulator`**
- Extends `RacingSimulator`
- Configuration: scenario source, name, timestep
- Creates simulation instances

**`DSpaceSimulation`**
- Core simulation extending `RacingSimulation`
- Manages two-phase architecture
- Handles coordinate transformations

**`DSpaceVehicleActor`**
- Internal representation of vehicles
- Contains physics state for fellow vehicles
- Stores position, velocity, heading

#### Key Responsibilities

**Initialization (`setup()` method):**
1. Connect to ModelDesk COM application
2. Save-as scenario copy for isolation
3. Build road geometry index from XODR/RD files
4. Apply coordinate transformations (XODR→RD)
5. Create ego and fellow vehicles
6. Author scenario in ModelDesk if dynamic control needed
7. Connect to ControlDesk for runtime control
8. Initialize VesiInterface manual control

**Vehicle Creation:**
- `createObjectInSimulator()`: Routes to ego/fellow creation
- `createEgoInSimulator()`: Configures ego via Maneuver API
- `createFellowInSimulator()`: Creates fellows via Fellows API

**Dynamic Control:**
- `executeActions()`: Apply accumulated control state
- Routes to `VehicleController` for ego/fellow control
- Clears control state after applying

**Route Management:**
- `detectTrackSegment()`: Identify pit lane vs main track
- `assignRoute()`: Map track segments to dSPACE routes
- `_set_fellow_route_via_sequence()`: Configure routes in ModelDesk

### 2. Actions (`actions.py`)

**`_DSpaceVehicle`**
- Marker mixin to identify dSPACE-backed vehicles
- Used for action gating

**`SetVehicleControl`**
- Combined control action (throttle + brake + steering)
- dSPACE-specific convenience action

### 3. Models

**`racing_model.scenic`** - `DSPACERacingCar` class

Implements protocols:
- `RacingCar`: Racing domain car
- `_DSpaceVehicle`: Marker for dSPACE
- `Steers`: Standard driving actions
- `HasManualTransmission`: Gear and clutch

**Protocol Methods:**
- `setMaxSpeed(max_speed)`: Set maximum speed
- `setTTL(ttl)`: Set time-to-live
- `setThrottle(throttle)`: Throttle control (0-1)
- `setSteering(steering)`: Steering control (-1 to 1)
- `setBraking(braking)`: Brake control (0-1)
- `setGear(gear)`: Gear selection (one-shot)
- `setClutch(clutch)`: Clutch control (0-1, one-shot)

---

## Vehicle Control Module

**Location:** `src/scenic/simulators/dspace/vehicle/`

A refactored module containing vehicle physics simulation and control logic, extracted from the main simulator file for better organization.

### Module Structure

```
vehicle/
├── __init__.py          # Module exports
├── physics.py           # VehiclePhysicsState - Physics simulation
└── controller.py        # VehicleController - Control application
```

### VehiclePhysicsState (`physics.py`)

Simple physics model that converts control inputs (throttle, brake, steering) into motion outputs (velocity, lateral deviation) for kinematic control of fellow vehicles.

**Key Features:**
- Longitudinal dynamics: throttle/brake → acceleration → velocity
- Lateral dynamics: steering → lateral velocity → deviation
- Tunable parameters for realistic behavior
- Euler integration with configurable timestep

**Physics Parameters:**
```python
max_acceleration = 10.0  # m/s² (0-100 km/h in ~3s)
max_deceleration = 15.0  # m/s² (emergency braking)
max_velocity = 100.0     # m/s (~360 km/h)
min_velocity = 0.0       # m/s

max_lateral_velocity = 5.0  # m/s lateral movement
steering_sensitivity = 2.0  # meters per second per steering unit
```

**Usage:**
```python
from scenic.simulators.dspace.vehicle import VehiclePhysicsState

# Create physics state
physics = VehiclePhysicsState(initial_velocity=0.0, initial_deviation=0.0)

# Update with control inputs
new_velocity, new_deviation = physics.update(
    throttle=0.5,
    brake=0.0,
    steering=0.2,
    dt=0.1
)

# Tune parameters
physics.set_parameters(max_acceleration=12.0, steering_sensitivity=2.5)
```

### VehicleController (`controller.py`)

Handles the application of control commands to vehicles in the dSPACE simulation environment.

**Control Strategies:**

1. **Ego Vehicle**: VesiInterface physics-based control
   - Writes to VesiInterface manual control variables
   - Direct control: throttle/brake/steering → physics engine
   - Handles gear and clutch commands

2. **Fellow Vehicles**: Kinematic control via external signals
   - Uses VehiclePhysicsState to compute motion
   - Control flow: throttle/brake/steering → physics model → velocity/deviation → ControlDesk

**Key Methods:**
- `apply_ego_control(obj)` - Apply control to ego vehicle
- `apply_fellow_control(obj)` - Apply control to fellow vehicle
- `get_fellow_index(obj)` - Convert raceNumber to array index
- `read_fellow_state(obj)` - Read fellow state from ControlDesk

**Indexing Convention:**
Fellow vehicles use 0-based array indexing:
- F1 (raceNumber=1) → index 0 → `Value[0]`
- F2 (raceNumber=2) → index 1 → `Value[1]`
- F3 (raceNumber=3) → index 2 → `Value[2]`

**Usage:**
```python
from scenic.simulators.dspace.vehicle import VehicleController

# Create controller
controller = VehicleController(simulation)

# Apply controls
controller.apply_ego_control(ego_obj)
controller.apply_fellow_control(fellow_obj)

# Read fellow state
state = controller.read_fellow_state(fellow_obj)
# Returns: {'velocity': 25.3, 'deviation': 1.2, 'fellow_index': 0}
```

### Integration with Simulator

**Initialization** (in `setup()`):
```python
self._vehicle_controller = VehicleController(self)
```

**Actor Creation** (in `DSpaceVehicleActor.__init__()`):
```python
self.physics = VehiclePhysicsState(initial_velocity=0.0, initial_deviation=0.0)
```

**Control Application** (in `executeActions()`):
```python
if is_ego:
    self._vehicle_controller.apply_ego_control(obj)
else:
    self._vehicle_controller.apply_fellow_control(obj)
```

### Refactoring Benefits

**Before:**
- `simulator.py`: 1898 lines, 88 KB
- All logic embedded in main file

**After:**
- `simulator.py`: 1673 lines, 78.7 KB (**-12% reduction**)
- `vehicle/physics.py`: 131 lines, 5.4 KB
- `vehicle/controller.py`: 222 lines, 10 KB
- Clear separation of concerns
- Easier to test and maintain

---

## Control Interfaces

dSPACE provides **two different interfaces** for programmatic vehicle control in ControlDesk.

### Interface Comparison

| Feature | ExternalUserData | VesiInterface Manual |
|---------|------------------|---------------------|
| **Throttle** | `Pos_AccPedal[%]/Value` | `Const_throttle_cmd/Value` |
| **Brake** | Unified `Pos_BrakePedal[%]/Value` | Separate front/rear |
| **Steering** | `Angle_SteeringWheel[deg]/Value` | `Const_steering_cmd/Value` |
| **Gear** | `Gear[]/Value` (0-6) | `Const_gear_cmd/Value` (0-6) |
| **Clutch** | `Pos_ClutchPedal[%]/Value` | Not available |
| **Initialization** | Simple | **Required** (critical) |
| **Enable Flags** | Not required | Required per control type |
| **Complexity** | Low | High |

**Recommendation**: Use VesiInterface for precise control with separate front/rear braking.

### VesiInterface Manual Control (Current Implementation)

**Critical Discovery**: Without proper initialization, control commands are silently ignored.

#### Required Initialization

**Must be set at simulation start:**

**1. VesiInterface Master Switches**

| Variable Path | Value | Description |
|--------------|-------|-------------|
| `Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0\|1]/Value` | `0.0` | Deactivates CLIF interface |
| `Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0\|1]/Value` | `1.0` | **CRITICAL**: Enables manual control |

**2. Race Control Configuration**

| Variable Path | Value | Description |
|--------------|-------|-------------|
| `Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern\|1Extern\|2Orchestrator]/Value` | `0.0` | Internal mode (required) |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value` | `9` | **CRITICAL** system state |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value` | `1` | Track flag |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value` | `0` | Vehicle flag |

**3. Enable Flags**

| Variable Path | Value | Description |
|--------------|-------|-------------|
| `Const_enable_brake_cmd/Value` | `1` | Enable brake commands |
| `Const_enable_gear_cmd/Value` | `1` | Enable gear commands |
| `Const_enable_steering_cmd/Value` | `1` | Enable steering commands |
| `Const_enable_throttle_cmd/Value` | `1` | Enable throttle commands |

#### Control Command Variables

**Ego Vehicle (VesiInterface):**
```python
# Throttle (0-100 command range)
"Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"

# Brake - Separate front/rear (0-100 command range)
"Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
"Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"

# Steering (-70 to +70 command range)
"Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"

# Gear (0-6 integer)
"Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"

# Clutch (0-100 percentage)
"Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
```

**Fellow Vehicles (External Signals):**
```python
base = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"

# Velocity (km/h) - uses array indexing
f"{base}/Const_v_Fellows_External[km|h]/Value[{fellow_index}]"

# Lateral deviation (meters) - uses array indexing
f"{base}/Const_d_Fellows_External[m]/Value[{fellow_index}]"
```

### Unit Conversions

| Quantity | Scenic/Physics | ControlDesk | Conversion |
|----------|---------------|-------------|------------|
| Throttle | 0-1 | 0-100 | scenic × 100 |
| Brake | 0-1 | 0-100 | scenic × 100 |
| Steering | -1 to 1 | -70 to +70 | -scenic × 70 |
| Velocity | m/s | km/h | m/s × 3.6 |
| Deviation | meters | meters | 1:1 |
| Gear | 0-6 | 0-6 | 1:1 |
| Clutch | 0-1 | 0-100 | scenic × 100 |

---

## Coordinate Transformation Pipeline

### Overview

Complete transformation from Scenic world coordinates to dSPACE ModelDesk (s,t) road coordinates.

### Transformation Flow

```
Scenic Scene (world coordinates)
    ↓  obj.position = (scenic_x, scenic_y)
    ↓  obj.heading = heading_angle
  
[Optional: Coordinate Transform]
    ↓  XODR coordinates → RD coordinates
    ↓  (transformed_x, transformed_y)
  
[Geometric Projection]
    ↓  World (x,y) → Road (s,t)
    ↓  s_val = longitudinal along reference line
    ↓  t_val = lateral deviation (scaled × 0.3)
  
[Orientation Adjustment]
    ↓  Scenic heading → dSPACE yaw
    ↓  VehicleOrientation = heading - π/2
  
ModelDesk COM API
    ↓  seq.StartPosition = s_val
    ↓  seg0.LateralType.Constant = t_val
    ↓  seq.VehicleOrientation = dspace_orientation
```

### Phase 1: Setup & Initialization

**Location:** `simulator.py` lines 111-168

**Decision Tree:**
```
param map = 'LagunaSeca.xodr'
Expected: 'Laguna_Seca.rd' (check if exists)

IF RD file exists:
    → Build XODR → RD transformation
    → Use RD geometry for projection
    → Full accuracy pipeline
    
ELSE:
    → Use XODR-only geometry
    → Fallback mode (may have positioning errors up to 34m)
```

**Coordinate Transformation Building:**

**Location:** `geometry/coordinate_transform.py`

1. **Parse Geometries**
   - RD: `parse_rd_geometry(rd_path, step=0.5m)`
   - XODR: `build_xodr_sec_points(xodr_path, step=2.0m)`

2. **Sample Calibration Points**
   - Sample 100 points at equal s-intervals
   - Get `(xodr_x, xodr_y)` and `(rd_x, rd_y)` for each

3. **Compute Transformation**
   ```python
   IF std_offset < 5m:
       Transform: 'translation'
       offset = mean_offset
   ELSE:
       Transform: 'affine'
       [rd_x, rd_y]^T = A × [xodr_x, xodr_y]^T + b
   ```

4. **Validate & Cache**
   - Mean error should be < 2m
   - Cache to `'_transform.json'`

### Phase 2: XODR → RD Transform

**Location:** `geometry/coordinate_transform.py` lines 196-223

```python
scenic_x, scenic_y = obj.position.x, obj.position.y

IF self._coordinate_transform is not None:
    transformed_x, transformed_y = apply_coordinate_transform(
        self._coordinate_transform, (scenic_x, scenic_y)
    )
ELSE:
    transformed_x, transformed_y = scenic_x, scenic_y
```

**Transformation Types:**

- **Translation**: Simple offset `(x + dx, y + dy)`
- **Affine**: Matrix multiplication `A @ [x, y] + b`

**Example:**
```
Scenic coords (-101.920, -457.520) → RD coords (-98.123, -453.245)
```

### Phase 3: Geometric Projection to (s,t)

**Location:** `geometry/projection.py` lines 64-148

**Algorithm:**

**Step 1: Find Closest Point on Segment**

For each segment: `point_0(x0,y0,s0) → point_1(x1,y1,s1)`

```python
# Segment direction vector
vx, vy = x1 - x0, y1 - y0
seg_len2 = vx*vx + vy*vy

# Vector from segment start to point
wx, wy = px - x0, py - y0

# Projection parameter (0 = start, 1 = end)
u = (wx*vx + wy*vy) / seg_len2
u = clamp(u, 0.0, 1.0)

# Closest point on segment
qx = x0 + u*vx
qy = y0 + u*vy

# Distance to segment
dist2 = (px - qx)² + (py - qy)²
```

**Step 2: Compute s-Coordinate (Longitudinal)**

```python
s_proj = s0 + u × (s1 - s0)
```

**Step 3: Compute t-Coordinate (Lateral Deviation)**

```python
# Left normal vector (perpendicular, pointing left)
nx_left = -vy/seg_len
ny_left =  vx/seg_len

# Raw lateral distance
raw_t = dx × nx_left + dy × ny_left

# Apply calibration scale (CRITICAL)
t_val = raw_t × 0.3
```

**Notes:**
- **Positive t**: Left of reference line
- **Negative t**: Right of reference line
- **Scale factor 0.3**: Calibrated to match lane width

**Step 4: Select Closest Projection**

```python
all_projections.sort(key=lambda x: x[0])  # Sort by distance
best = all_projections[0]
return (s_proj, t_val)
```

**Example:**
```
World coordinates (-98.123, -453.245) → Road coordinates (s=1234.5, t=0.045)
```

### Phase 4: Orientation Conversion

**Location:** `simulator.py` lines 297-301, 415-423

**Coordinate System Differences:**

| System | Zero Angle Points | Rotation |
|--------|------------------|----------|
| **Scenic** | North (+Y axis) | CCW |
| **dSPACE** | East (+X axis) | CCW |

**Conversion:**

```python
dspace_orientation = obj.heading - π/2
seq.VehicleOrientation = dspace_orientation
```

**Example:**
```
Scenic heading: 45° → dSPACE orientation: -45°
Scenic heading: 90° → dSPACE orientation: 0°
Scenic heading: 180° → dSPACE orientation: 90°
```

### Phase 5: Application to ModelDesk

**Location:** `simulator.py`, `geometry/utils.py`

**EGO Vehicle Configuration:**

```python
# Access ego maneuver
seq = self.ts.Maneuver.Item(0).Sequences.Item(0)

# Set longitudinal position
seq.StartPosition = float(s_val)

# Set orientation
seq.VehicleOrientation = dspace_orientation

# Set lateral deviation (if significant)
IF |t_val| > 0.1:
    seg0 = seq.Segments.Item(0)
    seg0.LateralType.Activate("Deviation")
    dep = seg0.LateralType.ActiveElement.DependencyType
    dep.Activate("Absolute")
    seg0.LateralType.ActiveElement.SourceType.ActiveElement.Constant = t_val
```

**FELLOW Vehicle Configuration:**

```python
# Create fellow
F = self.ts.Fellows.Add()

# Configure seg0: absolute position
configure_seg0_absolute_pose(segs, s=s_val, t=t_val)

# Configure seg1: motion
configure_seg1_motion(segs, v=0.0, t=t_val)
make_endless_transition(segs)
```

### Key Calibration Parameters

**t-Coordinate Scale Factor: 0.3×**

**Location:** `geometry/projection.py` line 118

```python
t_val = raw_t × 0.3
```

**Purpose:** Transform raw lateral distance to ModelDesk-compatible units.

**Calibration:**
- Typical lane width: 3-4 meters
- Without scaling: t ≈ ±1.5 to ±2.0 meters
- With 0.3× scaling: t ≈ ±0.45 to ±0.60 meters

**Independent s-Coordinates**

Each road segment has its own s-coordinate system:
- Main road: s ∈ [0, 2484.6]
- Pit lane: s ∈ [0, 883.4]

Prevents coordinate clustering when multiple vehicles on same road.

**Sampling Intervals**

- **XODR**: 2.0 meters (balance accuracy/performance)
- **RD**: 0.5 meters (higher precision)

**Orientation Offset: π/2 radians**

```python
dspace_orientation = obj.heading - math.pi / 2
```

Fixed constant accounting for coordinate system difference.

### Coordinate System Differences

**World Coordinate Systems:**

| Feature | Scenic/Domain | XODR | RD (dSPACE) |
|---------|--------------|------|-------------|
| **Based on** | OpenDRIVE | OpenDRIVE standard | dSPACE native |
| **Units** | Meters | Meters | Meters |
| **Used by** | Scenic scenarios | Road parsing | Aurelion/dSPACE |

**Orientation Systems:**

| Feature | Scenic | dSPACE | Conversion |
|---------|--------|--------|------------|
| **Zero direction** | North (+Y) | East (+X) | Subtract π/2 |
| **Rotation** | CCW | CCW | Same |
| **Range** | [0, 2π) | [0, 2π) | Additive offset |

**Road Coordinate System (s,t):**

- **s-coordinate (Longitudinal)**: Distance along reference line from start
- **t-coordinate (Lateral)**: Signed distance perpendicular to reference line
  - **Positive**: Left of reference line
  - **Negative**: Right of reference line

---

## Integration Points

### With Scenic Core

1. **Simulation Interface**: Extends `RacingSimulation`
2. **Action Protocol**: Implements `Steers`, `HasManualTransmission`
3. **Agent Creation**: `createObjectInSimulator()` hook
4. **State Updates**: `step()`, `getProperties()`

### With Racing Domain

1. **Track Segments**: pitLane vs mainRacing
2. **Route Assignment**: Lap vs Pit
3. **Racing Behaviors**: Integration with racing behaviors
4. **Protocol Methods**: setMaxSpeed, setTTL

### With dSPACE Environment

1. **ModelDesk**: COM automation for authoring
2. **ControlDesk**: COM automation for runtime control
3. **VEOS**: Simulator runtime
4. **Aurelion**: Sensor data and rendering
5. **CTun**: Communication tunnel

---

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

---

## Troubleshooting

### Vehicles Appear in Wrong Location

**Checklist:**

1. **Verify coordinate transformation**
   ```
   Look for: "Scenic coords (...) -> RD coords (...)"
   Check: self._coordinate_transform exists
   ```

2. **Check s-coordinate wrapping**
   ```
   Look for: "World coordinates (...) -> Road coordinates (s=..., t=...)"
   Verify: s is within [0, road_length]
   ```

3. **Verify lateral scaling**
   ```
   If laterally offset: check t-coordinate scale factor (0.3)
   Raw t should be ±0.5 to ±1.5 for typical lanes
   ```

### Positioning Errors up to 34 meters

**Cause:** Using XODR-only mode without RD transformation.

**Solution:**
1. Ensure `Laguna_Seca.rd` exists next to `LagunaSeca.xodr`
2. Check logs for: `"⚠️  No RD file found - coordinate mismatches possible"`
3. Transform builds automatically on first run

### Control Commands Not Working

**Checklist:**

1. **Check Master Switch**: `Sw_Manual_VESI_Overwrite = 1.0`
2. **Check Race Control**: `Sw_RaceControl = 0.0` (Intern mode)
3. **Check System State**: `Const_sys_state = 9`
4. **Check Enable Flags**: All `Const_enable_*_cmd` flags set to 1
5. **Verify Platform**: Accessing variables on `Platform_2`
6. **Check Online Calibration**: ControlDesk in online calibration mode

### Vehicles Clustering at Same Position

**Cause:** Cumulative s-coordinates causing overlaps.

**Solution:**
- Ensure independent s-coordinates per road
- Each road should start at s=0
- Verify `build_xodr_sec_points` and `build_rd_road_index`

### Orientation Misalignment

**Cause:** Incorrect orientation conversion.

**Solution:**
- Check: `"Set orientation: ... degrees (from Scenic heading ...)"`
- Verify formula: `dspace_orientation = heading - π/2`
- Ensure heading is in radians, not degrees

### High Validation Errors in Transform

**Check Logs:**
```
Mean error: X.XXm
Max error: X.XXm
```

**Thresholds:**
- `< 2m`: ✅ Good transform
- `2-5m`: ⚠️  Moderate errors
- `> 5m`: ❌ High errors

**Solutions:**
1. Verify XODR and RD represent same track
2. Check for file corruption
3. Consider manual calibration points
4. Fall back to XODR-only mode

### Debug Logging

**Setup Logs:**
```
"[Transform] Building automatic XODR→RD coordinate transformation..."
"[Geometry] Using RD geometry for accurate (s,t) projection"
"[Status] ✅ Full coordinate transformation pipeline active"
```

**Object Creation Logs:**
```
"Scenic coords (...)-> RD coords (...)"
"World coordinates (...) -> Road coordinates (s=..., t=...)"
"Set orientation: ... degrees (from Scenic heading ...)"
```

**Key Files for Debugging:**
- `simulator.py`: Main transformation logic
- `geometry/projection.py`: (s,t) projection algorithm
- `geometry/coordinate_transform.py`: XODR→RD transformation
- `geometry/utils.py`: ModelDesk COM helpers

---

## Summary

The dSPACE simulator provides a comprehensive, production-ready integration between Scenic scenarios and the dSPACE simulation environment:

✅ **Two-phase architecture** cleanly separates static authoring from dynamic control  
✅ **Automatic coordinate transformation** handles XODR→RD mapping  
✅ **Modular vehicle control** with physics simulation for fellows  
✅ **VesiInterface integration** for precise ego vehicle control  
✅ **Proper fellow indexing** for external signal control  
✅ **Comprehensive error handling** and debugging support  

**Success Indicators:**
- Mean transform error < 2m
- Vehicles placed accurately on track
- Correct orientation alignment
- No coordinate clustering
- Control commands responding properly

---

## 2025‑11 Update Highlights (External Control & Initialization)

This release refines fellow vehicle control and startup robustness:

- **Warm‑Up Gating of Behaviors**  
  `executeActions` now defers behavior execution until the plant’s fellow pose arrays (`FellowMovement/FELLOW_POS_VEL/FellowTrailer/x|y|yaw_deg_out`) are present and contain non‑zero values. This avoids writing controls before the fellow is actually spawned in the plant.

- **Bulk External Signals Write with Auto‑Probe**  
  Instead of relying on element addressing (which can be 0‑based or 1‑based, or not exposed via COM in some projects), the simulator writes to External Signals in bulk:  
  `Environment/Traffic/PlantModel/FellowMovement/External_Signals/Const_v_Fellows_External[km|h]/Value` and `.../Const_d_Fellows_External[m]/Value`.  
  A one‑time probe determines the correct velocity unit token (`km/h` vs `km|h`) and array base; the control then updates the relevant element in the bulk array and writes the whole vector back, followed by a readback of the same slot for verification.

- **Segment Configuration for External Velocity/Deviation**  
  In `createFellowInSimulator`, the second segment’s `Activity.LongitudinalType` and `Activity.LateralType` are set to `"Continue"` and the segment is marked `Endless`, allowing external velocity/deviation to take effect. Ensure `Route.UseExternal = True` is enabled in the ModelDesk route so the plant consumes External Signals.

- **Reduced Startup Noise**  
  Initialization no longer spams “index out of bounds” logs; the simulator silently single‑steps until the bulk arrays are present. Only a single readiness log is printed.

See also:

- `SIMULATION_LOOP_FLOW.md` → “Warm‑Up Gating & External Signals Path”
- `VEHICLE_CONTROL_IMPLEMENTATION.md` → “2025‑11 Updates: External Control & Warm‑Up”


