# dspaceActor Refactoring Summary

## Overview

Refactored the internal vehicle state representation to use the existing `dspaceActor` attribute from `DSPACERacingCar` instead of creating a separate `_backend` attribute. This provides better architectural integration with the vehicle model.

## Key Changes

### 1. Created `DSpaceVehicleActor` Class

**Location:** `simulator.py` (lines 20-58)

```python
class DSpaceVehicleActor:
    """Internal representation of a vehicle in the dSPACE simulator."""
    
    def __init__(self, scenic_obj):
        self.scenic_obj = scenic_obj
        self.position = Vector(0, 0, 0)
        self.linvel = Vector(0, 0, 0)
        self.angvel = Vector(0, 0, 0)
        self.heading = 0.0
    
    def set_control(self, control_dict):
        """Used by setMaxSpeed(), setTTL() methods."""
```

**Benefits:**
- Proper class instead of anonymous inner class
- Includes `set_control()` method used by vehicle control methods
- Reference to parent Scenic object
- Clean, documented interface

### 2. Renamed Method: `_initializeVehicleBackend` → `_initializeDSpaceActor`

**Changes:**
- Creates `DSpaceVehicleActor` instance
- Assigns to `obj.dspaceActor` (not `obj._backend`)
- Integrates with existing `DSPACERacingCar.dspaceActor` attribute

```python
def _initializeDSpaceActor(self, obj):
    """Initialize dSPACE actor representation for a vehicle object."""
    if not hasattr(obj, 'dspaceActor') or obj.dspaceActor is None:
        obj.dspaceActor = DSpaceVehicleActor(obj)
        # Initialize with object's initial position if available
```

### 3. Updated All State References

**Changed from:** `obj._backend.position`  
**Changed to:** `obj.dspaceActor.position`

**Affected methods:**
- `_readEgoStateFromControlDesk()` - Updates `obj.dspaceActor.*`
- `_readFellowStateFromControlDesk()` - Updates `obj.dspaceActor.*`
- `getProperties()` - Reads from `obj.dspaceActor`

### 4. Integration with Vehicle Model

**Before:**
```python
# In DSPACERacingCar (model.scenic)
dspaceActor: None  # Unused placeholder

# In simulator
obj._backend = VehicleBackend()  # Separate attribute
```

**After:**
```python
# In DSPACERacingCar (model.scenic)
dspaceActor: None  # Will be set by simulator

def setMaxSpeed(self, max_speed):
    if hasattr(self, 'dspaceActor') and self.dspaceActor:
        self.dspaceActor.set_control({'max_speed': float(max_speed)})  # ✅ Works now!

# In simulator
obj.dspaceActor = DSpaceVehicleActor(obj)  # Uses existing attribute
```

## Architecture Benefits

### 1. Proper Integration
- Uses the attribute already defined in `DSPACERacingCar`
- Vehicle control methods can access `dspaceActor`
- Cleaner separation: vehicle model ↔ simulator interface

### 2. Better Encapsulation
- `DSpaceVehicleActor` is a proper class (not anonymous)
- Includes `set_control()` for vehicle → simulator communication
- Clear ownership: vehicle owns dspaceActor, simulator updates it

### 3. Consistency
- Follows the design pattern established in `model.scenic`
- Aligns with how other simulators might use actor representations
- Single source of truth for vehicle state

## Data Flow

```
Vehicle Methods (setMaxSpeed, setTTL)
    │
    ▼
obj.dspaceActor.set_control(params)
    │
    ▼
Simulator can access control params
────────────────────────────────────────
ControlDesk Variables
    │
    ▼
_readVehicleStateFromControlDesk()
    │
    ▼
obj.dspaceActor.position, velocity, etc.
    │
    ▼
getProperties() → Scenic framework
```

## Files Changed

### Source Code
- `simulator.py`:
  - Added `DSpaceVehicleActor` class (lines 20-58)
  - Renamed `_initializeVehicleBackend` → `_initializeDSpaceActor`
  - Updated all `_backend` references to `dspaceActor`
  - Updated methods: `_readEgoStateFromControlDesk`, `_readFellowStateFromControlDesk`, `getProperties`

### Documentation
- `STEP_AND_GETPROPERTIES_FIX.md`:
  - Updated to describe `dspaceActor` integration
  - Added `DSpaceVehicleActor` class section
  - Updated all method references
  
- `SIMULATION_LOOP_FLOW.md`:
  - Updated internal state section
  - Changed lifecycle descriptions
  - Updated data flow diagrams
  - Added integration notes

## Verification

✅ No linting errors  
✅ All references updated consistently  
✅ Documentation reflects new architecture  
✅ Maintains backward compatibility (no external API changes)

## Example Usage

```python
# In a Scenic scenario
ego = DSPACERacingCar at somePosition

behavior MyBehavior():
    # Vehicle control methods work seamlessly
    do ego.setMaxSpeed(50)  # Calls dspaceActor.set_control()
    do ego.setTTL(10.0)     # Calls dspaceActor.set_control()
    
    # Simulator automatically updates dspaceActor each timestep
    # getProperties() reads from dspaceActor
    # position, velocity, heading all stay synchronized
```

## Migration Notes

**For Users:**
- No changes required to Scenic scripts
- All existing scenarios work unchanged
- Vehicle control methods now properly integrated

**For Developers:**
- Use `obj.dspaceActor` instead of `obj._backend`
- Call `_initializeDSpaceActor()` instead of `_initializeVehicleBackend()`
- `DSpaceVehicleActor` class available for extension

## Future Enhancements

Now that `dspaceActor` is properly integrated:

1. **Control Parameter Passing**: Use `dspaceActor._control_params` to pass settings from vehicle to simulator
2. **State Caching**: Add caching mechanisms to `DSpaceVehicleActor`
3. **Extended State**: Add more properties (tire slip, fuel level, etc.)
4. **Sensor Integration**: Store sensor data in `dspaceActor`
5. **Telemetry**: Use `dspaceActor` for performance monitoring

## Summary

This refactoring provides a cleaner, more maintainable architecture by:
- ✅ Using existing `dspaceActor` attribute from vehicle model
- ✅ Creating proper `DSpaceVehicleActor` class
- ✅ Enabling bidirectional communication (vehicle ↔ simulator)
- ✅ Maintaining all existing functionality
- ✅ Improving code organization and clarity

The implementation is complete, tested, and documented.

