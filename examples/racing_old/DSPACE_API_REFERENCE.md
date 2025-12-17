# dSPACE Integration API Reference

This document contains important API discoveries and patterns for dSPACE ModelDesk and ControlDesk integration with Scenic.

## COM Interface Names

### ControlDesk
- **Correct COM Interface**: `ControlDeskNG.Application`
- **Version Tested**: 23.1
- **Note**: `ControlDesk.Application` does NOT work - must use `ControlDeskNG.Application`

### ModelDesk  
- **COM Interface**: Uses existing connection (already working)

## ControlDesk API Structure

### Object Hierarchy
```
ControlDeskNG.Application
  └─ ActiveProject
      └─ ActiveExperiment
          ├─ Platforms (collection)
          │   └─ Platform
          │       ├─ Connect()
          │       ├─ Disconnect()
          │       ├─ StartOnlineCalibration()
          │       ├─ StopOnlineCalibration()
          │       ├─ ConnectionState (property)
          │       ├─ CalibrationState (property)
          │       └─ ConnectionSettings
          │           ├─ BoardName
          │           └─ IPAddress
          └─ GetVariable(path)
```

### Key Methods

#### Platform Connection
```python
from win32com.client import Dispatch

cd = Dispatch("ControlDeskNG.Application")
proj = cd.ActiveProject
exp = proj.ActiveExperiment
platform = exp.Platforms.Item(0)  # 0-based indexing

# Connect to VEOS
platform.Connect()

# Go Online (start calibration mode)
if platform.CanStartOnlineCalibration():
    platform.StartOnlineCalibration()
```

#### Variable Access
```python
# Must be online to access variables
var = exp.GetVariable("Environment.Vehicle.F1.Driver.Throttle")
value = var.Value
var.Value = 0.5  # Write value
```

### Connection States
- **0** = Disconnected
- **1** = Connecting  
- **2** = Connected

### Calibration States
- **0** = Offline
- **1** = Online (calibration active)

## Variable Paths

### Driver Control Variables
```
Environment.Vehicle.<Fx>.Driver.Throttle           # 0.0 to 1.0
Environment.Vehicle.<Fx>.Driver.Brake              # 0.0 to 1.0
Environment.Vehicle.<Fx>.Driver.SteeringWheelAngle # degrees
```

Where `<Fx>` is:
- `Ego` - Ego vehicle
- `F1`, `F2`, `F3`, ... - Fellow vehicles (numbered by raceNumber)

## Two-Phase Architecture

### Phase 1: ModelDesk (Scenario Authoring)
- **Purpose**: One-time setup, create scenario structure
- **Operations**:
  - Create ego and fellow vehicles
  - Set routes (Pit vs Lap)
  - Set initial positions (s, t coordinates)
  - Download scenario to VEOS
- **Key Methods**:
  - `ts.Fellows.Create()` - Create fellow vehicle
  - `ts.Save()` - Save scenario
  - `ts.Download()` - Download to VEOS

### Phase 2: ControlDesk (Runtime Control)
- **Purpose**: Per-tick control during simulation
- **Operations**:
  - Write throttle/brake/steering every tick
  - Read vehicle state
- **Timing**: 10ms (dt = 0.01) per manual4.md
- **Requirement**: Vehicles must have external control flag enabled

## External Control Flags

### Purpose
Allows ControlDesk to override internal behavior and directly control vehicle inputs.

### Methods to Enable

#### Method 1: ModelDesk COM (preferred)
```python
# In fellow configuration
route_sel = fellow.Sequences.Item(1).Route
route_sel.UseExternal = True
```

#### Method 2: Docker Script
```bash
docker exec -it veos python3 /home/dspace/scripts/ASM_Maneuver.py vehicleflag_3 trackflag_4
```

## Coordinate System

### World Coordinates (x, y, z)
- Scenic uses standard Cartesian coordinates
- Origin and units defined by map

### Road Coordinates (s, t)
- **s**: Longitudinal position along road (meters from road start)
- **t**: Lateral offset from road reference line (positive = left, negative = right)
- Each road has **independent s-coordinate system** starting from 0

### Key Roads (Laguna Seca Example)
- **Main Racing Roads**: "The Corkscrew1", "Andretti Hairpin1_3"
- **Pit Lane Road**: "Pit Lane1_2"

### Transformation Pipeline
```
Scenic (x,y) → RD Geometry → (s,t) for dSPACE
```

### Important Functions
- `dutils.project_world_to_st(road_index, (x, y))` - Convert world to road coords
- `dutils.find_road_id_for_position(road_index, x, y)` - Find which road a position belongs to

## Scenic Racing Domain

### Key Classes
- `RacingCar` - Main vehicle class (extends Car)
  - `raceNumber` - Used to generate dSPACE vehicle name (F<raceNumber>)
  - Default car type: "Dallara AV-24"

### Key Regions
- `mainRacingRoad` - Union of main circuit roads
- `pitLaneRoad` - Pit lane section

### Behaviors
- `SimpleRacingBehavior` - Basic throttle/steering control for racing
- `SimplePitBehavior` - Reduced speed behavior for pit lane

### Actions
- `DSPACESetThrottleAction(value)` - Set throttle (0.0 to 1.0)
- `DSPACESetBrakeAction(value)` - Set brake (0.0 to 1.0)
- `DSPACESetSteerAction(value)` - Set steering (-1.0 to 1.0)

## Infrastructure Requirements

### For Per-Tick Control to Work
1. **VEOS**: Docker container running
2. **CTun**: Client connected to VEOS IP (typically 10.6.0.2 or 192.168.100.101)
3. **ModelDesk**: Scenario authored and downloaded
4. **ControlDesk**: 
   - Application running
   - Project loaded
   - Connected to VEOS (`platform.Connect()`)
   - Online/Calibration mode active (`platform.StartOnlineCalibration()`)
5. **External Control Flags**: Enabled for fellow vehicles

### Verification Steps
```python
# Check connection state
cd = Dispatch("ControlDeskNG.Application")
platform = cd.ActiveProject.ActiveExperiment.Platforms.Item(0)
print(f"Connection: {platform.ConnectionState}")  # Should be 2
print(f"Calibration: {platform.CalibrationState}") # Should be 1

# Try variable access
var = cd.ActiveProject.ActiveExperiment.GetVariable("Environment.Vehicle.F1.Driver.Throttle")
print(f"Value: {var.Value}")  # Should not error
```

## Route Assignment

### Route Types
- **"Pit"** (Route0) - For pit lane vehicles
- **"Lap"** (Route1) - For main circuit vehicles

### Auto-Detection Logic
```python
# Based on road segment placement in Scenic
if placed_on(pitLaneRoad):
    assign "Pit" route
elif placed_on(mainRacingRoad):
    assign "Lap" route
```

## Common Issues & Solutions

### Issue: `'charmap' codec can't encode character`
- **Cause**: Unicode characters (emojis, special symbols) in print statements
- **Solution**: Use ASCII-only characters in logging

### Issue: `Invalid class string` when connecting to ControlDesk
- **Cause**: Wrong COM interface name
- **Solution**: Use `ControlDeskNG.Application` not `ControlDesk.Application`

### Issue: `<unknown>.GetVariable` error
- **Cause**: ControlDesk not online or not connected to VEOS
- **Solution**: Call `platform.Connect()` and `platform.StartOnlineCalibration()`

### Issue: Variables not accessible even after going online
- **Cause**: ControlDesk can't reach VEOS (network/CTun issue)
- **Solution**: Verify VEOS is running, CTun is connected, IP is reachable

### Issue: Cars overlap in (s, t) coordinates
- **Cause**: Multiple roads share s-coordinates (cumulative system)
- **Solution**: Use independent road segments with per-road s-coordinates starting at 0

### Issue: Large t-values (cars off-road)
- **Cause**: Scale mismatch between coordinate systems
- **Solution**: Apply scaling factor (e.g., `t_signed = raw_t * 0.3`)

## File Locations

### Core Implementation
- `src/scenic/simulators/dspace/simulator.py` - Main simulator logic
- `src/scenic/simulators/dspace/per_tick_control.py` - ControlDesk per-tick control
- `src/scenic/simulators/dspace/actions.py` - dSPACE-specific actions
- `src/scenic/simulators/dspace/utils.py` - Coordinate transformation utilities
- `src/scenic/simulators/dspace/rd_geometry.py` - RD geometry parsing

### Racing Domain
- `src/scenic/domains/racing/model.scenic` - Racing objects and regions
- `src/scenic/domains/racing/behaviors.scenic` - Racing behaviors

### Examples
- `examples/racing/per_tick_control_example.scenic` - Minimal working example
- `examples/racing/per_tick_control_script.py` - Standalone Python example
- `examples/racing/three_segments.scenic` - Multi-segment racing scenario

## Notes for Future Development

1. **ControlDesk must be fully connected** for per-tick control to work
2. **External control flags are critical** - without them, fellows ignore ControlDesk inputs
3. **Timing matters** - use 10ms (0.01s) intervals to match simulation step
4. **Road coordinate systems are independent** - don't assume cumulative s-values
5. **Platform object** is the key to ControlDesk connection management
6. **GetVariable returns a reference** - access `.Value` property to read/write

## Version Information

- **ControlDesk**: 23.1
- **Python**: win32com library required
- **dSPACE Platform Type**: 26 (VEOS)
- **Tested with**: Laguna Seca track (LagunaSeca.xodr)

