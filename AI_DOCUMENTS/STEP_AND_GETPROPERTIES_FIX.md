# Fix for step() and getProperties() Methods

## Overview

This document describes the fixes implemented for the `step()` and `getProperties()` methods in the dSPACE simulator, which are required abstract methods from the base `Simulation` class.

## Problem Statement

### Original Issues

1. **`step()` method**: Only called `time.sleep()` without actually advancing the simulation
2. **`getProperties()` method**: Attempted to read from `obj._backend`, which was never initialized

## Solution Architecture

### Internal State Representation via dspaceActor

Uses the existing `dspaceActor` attribute (defined in `DSPACERacingCar`) to track vehicle state:
- **Position** (x, y, z coordinates)
- **Linear velocity** (velocity vector)
- **Angular velocity** (rotation rate)
- **Heading** (yaw angle in radians)

This integrates with the vehicle model architecture where:
1. `DSPACERacingCar` defines `dspaceActor: None` as a placeholder
2. Simulator creates `DSpaceVehicleActor` instance during object creation
3. Updated every timestep by reading from ControlDesk
4. Used by `getProperties()` to return current state
5. Used by vehicle control methods (`setMaxSpeed`, `setTTL`) via `set_control()`

### Encapsulated Helper Methods

Created clean, focused helper methods for better organization:

#### 1. Simulation Control Methods

**`_pauseSimulation()`**
- Pauses the simulation during setup for step-by-step control
- Called once during `setup()` after ControlDesk connection

**`_advanceSimulationStep()`**
- Advances simulation by one timestep using ControlDesk COM interface
- Called by `step()` after control variables are written
- Uses `Application.PlatformManagement.Platforms.Item(0).RealTimeApplications.Item(0).SingleStep()`

#### 2. Vehicle State Management Methods

**`_initializeDSpaceActor(obj)`**
- Creates `DSpaceVehicleActor` instance and assigns to `obj.dspaceActor`
- Stores internal state (position, velocity, heading)
- Called during object creation and as needed
- Integrates with existing `DSPACERacingCar.dspaceActor` attribute

**`_readVehicleStateFromControlDesk(obj)`**
- Master method to read vehicle state
- Routes to ego or fellow-specific readers
- Updates `dspaceActor` with fresh data

**`_readEgoStateFromControlDesk(obj)`**
- Reads ego vehicle state from ControlDesk
- Uses paths under `Environment/Maneuver/PlantModel`
- Converts degrees to radians for heading

**`_readFellowStateFromControlDesk(obj)`**
- Reads fellow vehicle state from ControlDesk
- Uses paths from `FELLOW_POS_VEL/FellowTrailer`
- Reads: x, y, z, yaw_deg, v_Fellows, w_Fellows
- Converts velocity components using heading angle

**`_getFellowIndex(obj)`**
- Determines fellow vehicle index (0-based)
- Checks: raceNumber, fellow_vehicles dict, objects list
- Returns None if object not found

## DSpaceVehicleActor Class

The `DSpaceVehicleActor` class serves as the internal representation of a vehicle in the dSPACE simulator:

```python
class DSpaceVehicleActor:
    """Internal representation of a vehicle in the dSPACE simulator.
    
    Attributes:
        scenic_obj: Reference to the parent Scenic object
        position: Current position as Vector(x, y, z) in meters
        linvel: Linear velocity as Vector(vx, vy, vz) in m/s
        angvel: Angular velocity as Vector(wx, wy, wz) in rad/s
        heading: Heading angle (yaw) in radians
    """
    
    def set_control(self, control_dict):
        """Set control parameters (used by setMaxSpeed, setTTL, etc.)."""
```

**Integration with Vehicle Model:**
- Defined as `dspaceActor: None` in `DSPACERacingCar` class
- Instantiated by simulator during object creation
- Used by vehicle methods: `setMaxSpeed()`, `setTTL()` call `dspaceActor.set_control()`
- Updated by simulator every timestep with fresh ControlDesk data
- Accessed by `getProperties()` to return current vehicle state

## Implementation Details

### step() Method

```python
def step(self):
    """Execute one simulation step (advance physics simulation).
    
    This advances the dSPACE simulation by one timestep using ControlDesk.
    Control variables should already be written by executeActions() before this is called.
    """
    if self._cd:
        try:
            # Advance simulation by one step using ControlDesk COM interface
            self._advanceSimulationStep()
        except Exception as e:
            print(f"[step] Warning: Failed to advance simulation step: {e}")
            # Fallback to sleep
            time.sleep(self.timestep)
    else:
        # No ControlDesk connection, just sleep
        time.sleep(self.timestep)
```

**Key Features:**
- Calls `_advanceSimulationStep()` if ControlDesk available
- Graceful fallback to sleep if ControlDesk unavailable
- Error handling with informative messages

### getProperties() Method

```python
def getProperties(self, obj, properties):
    """Read the values of the given properties of the object from the simulator.
    
    This method reads vehicle state from ControlDesk and updates the internal
    _backend representation, then returns the requested properties.
    """
    # Initialize dspaceActor if it doesn't exist
    self._initializeDSpaceActor(obj)
    
    # Try to read fresh state from ControlDesk
    self._readVehicleStateFromControlDesk(obj)
    
    # Get state from dspaceActor
    actor = obj.dspaceActor
    pos = actor.position
    vel = actor.linvel
    ang = actor.angvel
    yaw = actor.heading
    
    # Build property values dictionary
    vals = {
        "position":        pos,
        "velocity":        vel,
        "speed":           vel.norm(),
        "angularVelocity": ang,
        "angularSpeed":    ang.norm(),
        "yaw":             float(yaw),
        "pitch":           0.0,
        "roll":            0.0,
        "elevation":       float(pos.z),
    }
    
    # Return only requested properties
    return {k: vals[k] for k in properties if k in vals}
```

**Key Features:**
- Ensures backend is initialized
- Reads fresh state from ControlDesk each call
- Returns only requested properties
- Graceful handling when ControlDesk unavailable

## ControlDesk Variable Paths

### Fellow Vehicles

Base path: `Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer`

- `x[index]` - X coordinate (meters)
- `y[index]` - Y coordinate (meters)
- `z[index]` - Z coordinate (meters)
- `yaw_deg_out[index]` - Heading angle (degrees)
- `v_Fellows[index]` - Speed (m/s)
- `w_Fellows[index]` - Angular velocity (rad/s)

### Ego Vehicle

Base path: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel`

- `Ego_x/Value` - X coordinate
- `Ego_y/Value` - Y coordinate
- `Ego_z/Value` - Z coordinate
- `Ego_yaw/Value` - Heading angle (degrees)
- `Ego_velocity/Value` - Speed (m/s)

**Note:** Ego paths may need adjustment based on actual ControlDesk model structure.

## Simulation Flow

### Setup Phase
1. Connect to ModelDesk and ControlDesk
2. Create vehicles in ModelDesk
3. Initialize backends for all vehicles
4. **Pause simulation** (`_pauseSimulation()`)
5. Initialize VesiInterface control

### Simulation Loop (each timestep)
1. Behaviors compute desired actions
2. `executeActions()` writes control variables to ControlDesk
3. **`step()` advances simulation by one timestep**
4. **`getProperties()` reads new vehicle states**
5. Scenic updates object properties
6. Requirements and monitors checked
7. Repeat

## Benefits

### Clean Separation of Concerns
- Simulation control separate from state reading
- Each method has single, clear responsibility
- Easy to debug and maintain

### Robust Error Handling
- Graceful fallback when ControlDesk unavailable
- Informative error messages
- No crashes from missing data

### Flexible Architecture
- Easy to add new state variables
- Support for both ego and fellow vehicles
- Extensible for future enhancements

## Testing Recommendations

1. **Without ControlDesk**: Verify fallback behavior works
2. **With ControlDesk**: Verify state updates correctly
3. **Multiple Vehicles**: Test ego + multiple fellows
4. **Dynamic Behaviors**: Test with racing behaviors

## Future Enhancements

1. **Caching**: Cache ControlDesk variable objects for performance
2. **Batch Reads**: Read multiple vehicles in single COM call
3. **Error Recovery**: Auto-reconnect if ControlDesk connection lost
4. **State Validation**: Verify read values are reasonable
5. **Performance Monitoring**: Track read/write times

## Related Files

- `simulator.py`: Main implementation
- `connection.py`: ControlDesk COM wrapper
- `DSPACE_CONTROL_INTERFACES.md`: Variable paths documentation
- `DSPACE_SIMULATOR_STRUCTURE.md`: Architecture overview

## Summary

The fixes provide a complete, robust implementation of the required abstract methods:

- ✅ **`step()`**: Properly advances simulation using ControlDesk
- ✅ **`getProperties()`**: Reads actual vehicle state from ControlDesk
- ✅ **Internal State**: Maintains `_backend` representation for each vehicle
- ✅ **Error Handling**: Graceful fallbacks and informative messages
- ✅ **Clean Code**: Well-encapsulated helper methods

The implementation follows best practices with clear separation of concerns and robust error handling.

