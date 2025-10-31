# Gear and Clutch Control Implementation

## Overview

Gear and clutch control have been implemented as **racing domain** features following the proper architectural pattern (similar to `Steers` protocol from the driving domain). The dSPACE simulator implements these protocols.

## Architecture

### 1. Racing Domain Protocol (`scenic.domains.racing.actions`)

Added `HasManualTransmission` protocol mixin:

```python
class HasManualTransmission:
    """Mixin protocol for agents with manual transmission control."""
    
    def setGear(self, gear):
        """Set gear to specific value (0-6). 0=Neutral, 1-6=Gears."""
        raise NotImplementedError
    
    def setClutch(self, clutch):
        """Set clutch pedal position (0.0=released, 1.0=fully pressed)."""
        raise NotImplementedError
```

### 2. Racing Domain Actions

Three new actions in `scenic.domains.racing.actions`:

#### Gear Control
- **`SetGearAction(gear)`**: Set gear directly (0-6)
  - 0 = Neutral
  - 1-6 = Gears 1-6
  - Use for all gear changes (both starting and shifting)

#### Clutch Control (for starting from neutral only)
- **`PressClutchAction()`**: Press clutch pedal (one-shot action)
  - Sets clutch to 100% (fully pressed)
  - **Primary use**: Starting from neutral (0 → 1)
  
- **`ReleaseClutchAction()`**: Release clutch pedal (one-shot action)
  - Sets clutch to 0% (fully released)
  - **Primary use**: Completing start sequence from neutral

**Important**: Clutch actions are primarily for starting from neutral (gear 0 → 1).
Regular gear changes while moving (1→2→3→4, etc.) use SetGearAction without clutch.

### 3. dSPACE Implementation

`DSPACERacingCar` in `model.scenic` implements `HasManualTransmission`:

```scenic
class DSPACERacingCar(RacingCar, _DSpaceVehicle, Steers, HasManualTransmission):
    def setGear(self, gear):
        # Stores as one-shot action in _oneshot_actions
        
    def setClutch(self, clutch):
        # Stores as one-shot action in _oneshot_actions
```

### 4. Simulator Handling

`DSpaceSimulation` in `simulator.py`:
- Handles one-shot actions separately from continuous controls
- `setVehicleGear(vehicle_name, gear)`: Writes to ControlDesk gear variable
- `setVehicleClutch(vehicle_name, clutch)`: Writes to ControlDesk clutch variable

## Key Design Decisions

### One-Shot vs Continuous Actions

| Action Type | Storage | Behavior |
|-------------|---------|----------|
| **Continuous** (throttle, brake, steer) | `_control_state` | Applied every timestep, overwritten each frame |
| **One-Shot** (gear, clutch) | `_oneshot_actions` | Applied once immediately, then cleared |

This prevents issues like:
- Clutch being pressed continuously (causing wear/issues)
- Gear being set repeatedly every frame

### ControlDesk Variable Mapping

From `ControlDesk.md`:

```python
# Gear (integer 0-6)
KEY_GEAR = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Gear[]/Value"

# Clutch (0-100%)
KEY_CLUTCH = "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value"
```

## Usage Examples

### Starting from Neutral (with clutch)

Use clutch to start from neutral (0 → 1):

```scenic
behavior StartFromNeutral():
    # Vehicle is in neutral (gear 0)
    take PressClutchAction()      # Press clutch
    wait
    take SetGearAction(1)          # Engage 1st gear
    wait
    take ReleaseClutchAction()     # Release clutch to start moving
```

### Normal Gear Changes (no clutch needed)

Use `SetGearAction` directly for shifting while moving:

```scenic
behavior NormalShifting():
    # Already in 1st gear and moving
    take SetGearAction(2)  # Shift to 2nd
    wait
    wait
    
    take SetGearAction(3)  # Shift to 3rd
    wait
    wait
    
    take SetGearAction(4)  # Shift to 4th
    wait
    wait
    
    take SetGearAction(2)  # Downshift to 2nd
```

### Complete Start + Drive Sequence

```scenic
behavior StartAndDrive():
    # Start from neutral
    take PressClutchAction()
    wait
    take SetGearAction(1)
    wait
    take ReleaseClutchAction()
    wait
    wait
    
    # Now shift normally (no clutch)
    take SetGearAction(2)
    wait
    wait
    take SetGearAction(3)
    wait
    wait
    take SetGearAction(4)
```

**Key Points**:
- **Clutch**: Only for starting from neutral (0 → 1)
- **SetGearAction**: For all gear changes (including 0→1, and all shifts while moving)
- **No clutch needed**: For normal shifting (1→2, 2→3, 3→4, downshifts, etc.)

## Execution Flow

1. **Action is taken** (`take SetGearAction(3)`)
2. **Protocol method called** (`obj.setGear(3)`)
3. **Queued as one-shot** (`_oneshot_actions.append(('gear', 3))`)
4. **executeActions() processes queue**
5. **Simulator writes to ControlDesk** (`setVehicleGear()`)
6. **Queue is cleared** (action executed once)

## Verification

The test output confirms:
- ✅ Gear changes are one-shot (only written once per action)
- ✅ Clutch is one-shot (only written once per action)
- ✅ No continuous triggering
- ✅ ControlDesk writes successful

Example output:
```
[DSPACERacingCar.setGear] Called with gear=2
[executeActions] Applying one-shot actions to Ego: [('gear', 2)]
[setVehicleGear] Called for Ego: gear=2
  [ControlDesk] Setting gear: 2
  [ControlDesk] OK - Gear written successfully
```

## Benefits

1. **Clean separation**: Gear/clutch are racing domain concerns, not simulator-specific
2. **One-shot behavior**: Prevents continuous writes that could cause issues
3. **Protocol pattern**: Other simulators can implement `HasManualTransmission` easily
4. **Domain consistency**: Follows same pattern as `Steers` from driving domain
5. **Explicit control**: User controls when clutch is pressed/released

## Files Modified

- `scenic/domains/racing/actions.py`: Added `HasManualTransmission`, `SetGearAction`, `PressClutchAction`, `ReleaseClutchAction`
- `scenic/domains/racing/model.scenic`: Export actions via `from scenic.domains.racing.actions import *`
- `scenic/simulators/dspace/model.scenic`: Implement `HasManualTransmission` protocol
- `scenic/simulators/dspace/simulator.py`: Handle one-shot actions, add `setVehicleGear()` and `setVehicleClutch()`
- `examples/racing/gear_test_example.scenic`: Test scenario demonstrating usage

