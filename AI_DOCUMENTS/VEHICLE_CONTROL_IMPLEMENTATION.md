# Vehicle Control Implementation Guide

## Overview

This document describes the complete implementation of vehicle control in Scenic for the dSPACE simulator. All controls (throttle, brake, steering, gear, clutch) are fully implemented and tested.

**Status**: ✅ All controls implemented and working

---

## Control Inputs Available

| Control | Action | Input Range | ControlDesk Range | Status |
|---------|--------|-------------|-------------------|--------|
| **Throttle** | `SetThrottleAction(value)` | 0.0 - 1.0 | 0-100% | ✅ Working |
| **Brake** | `SetBrakeAction(value)` | 0.0 - 1.0 | 0-100% | ✅ Working |
| **Steering** | `SetSteerAction(value)` | -1.0 - 1.0 | ±25° | ✅ Working |
| **Gear** | `SetGearAction(gear)` | 0-6 (int) | 0-6 (int) | ✅ Working |
| **Clutch** | `PressClutchAction()` / `ReleaseClutchAction()` | N/A | 0-100% | ✅ Working |

---

## Architecture

### Control Types

Controls are divided into two categories based on how they're applied:

#### Continuous Controls (Throttle, Brake, Steering)
- **Storage**: `_control_state` dictionary on vehicle objects
- **Applied**: Every timestep via `executeActions()`
- **Cleared**: After each application
- **Protocol**: `Steers` (from driving domain)
- **Behavior**: Values persist and are reapplied each frame until explicitly changed

#### One-Shot Controls (Gear, Clutch)
- **Storage**: `_oneshot_actions` list on vehicle objects
- **Applied**: Once immediately via `executeActions()`
- **Cleared**: After single application
- **Protocol**: `HasManualTransmission` (from racing domain)
- **Behavior**: Executed once per `take` statement, not continuously

**Why this separation?**
- Prevents clutch from being pressed continuously (causing wear/issues)
- Prevents gear from being set repeatedly every frame
- Matches real-world behavior (clutch/gear are discrete actions, not continuous)

### Domain Architecture

Controls follow Scenic's layered domain architecture:

```
┌─────────────────────────────────────┐
│     SCENARIOS (.scenic files)       │
│  User-written behaviors using API   │
└──────────────┬──────────────────────┘
               │
┌──────────────▼──────────────────────┐
│      RACING DOMAIN (abstract)       │
│  • RacingCar                         │
│  • HasManualTransmission protocol    │
│  • Actions: SetGearAction,           │
│            PressClutchAction, etc.   │
└──────────────┬──────────────────────┘
               │ extends
┌──────────────▼──────────────────────┐
│      DRIVING DOMAIN (abstract)       │
│  • Car, Network, Road, Lane          │
│  • Steers protocol                   │
│  • Actions: SetThrottleAction,      │
│            SetBrakeAction,           │
│            SetSteerAction            │
└──────────────┬──────────────────────┘
               │ implements
┌──────────────▼──────────────────────┐
│   dSPACE SIMULATOR (concrete)        │
│  • DSPACERacingCar class             │
│  • setThrottle(), setSteering(),    │
│    setBraking() methods              │
│  • setGear(), setClutch() methods   │
│  • Writes to ControlDesk             │
└──────────────────────────────────────┘
```

### Protocols

#### `Steers` Protocol (Driving Domain)

```python
class Steers:
    """Mixin protocol for agents that can steer, throttle, and brake."""
    
    def setThrottle(self, throttle):
        """Set throttle position (0.0-1.0)."""
        raise NotImplementedError
    
    def setSteering(self, steering):
        """Set steering angle (-1.0 to 1.0)."""
        raise NotImplementedError
    
    def setBraking(self, braking):
        """Set brake position (0.0-1.0)."""
        raise NotImplementedError
```

#### `HasManualTransmission` Protocol (Racing Domain)

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

### Actions

#### Driving Domain Actions

Located in `scenic.domains.driving.actions`:

- **`SetThrottleAction(value)`**: Set throttle (0.0-1.0)
- **`SetBrakeAction(value)`**: Set brake (0.0-1.0)
- **`SetSteerAction(value)`**: Set steering (-1.0 to 1.0)

#### Racing Domain Actions

Located in `scenic.domains.racing.actions`:

- **`SetGearAction(gear)`**: Set gear directly (0-6)
  - 0 = Neutral
  - 1-6 = Gear positions
  - Use for all gear changes (both starting and shifting)

- **`PressClutchAction()`**: Press clutch pedal (one-shot)
  - Sets clutch to 100% (fully pressed)
  - **Primary use**: Starting from neutral (0 → 1)

- **`ReleaseClutchAction()`**: Release clutch pedal (one-shot)
  - Sets clutch to 0% (fully released)
  - **Primary use**: Completing start sequence from neutral

### dSPACE Implementation

#### `DSPACERacingCar` (model.scenic)

```scenic
class DSPACERacingCar(RacingCar, _DSpaceVehicle, Steers, HasManualTransmission):
    """dSPACE-specific racing car implementation."""
    
    def setThrottle(self, throttle):
        # Store in _control_state for continuous application
        self._control_state['throttle'] = throttle
    
    def setSteering(self, steering):
        # Store in _control_state for continuous application
        self._control_state['steering'] = steering
    
    def setBraking(self, braking):
        # Store in _control_state for continuous application
        self._control_state['braking'] = braking
    
    def setGear(self, gear):
        # Store as one-shot action
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        self._oneshot_actions.append(('gear', gear))
    
    def setClutch(self, clutch):
        # Store as one-shot action
        if not hasattr(self, '_oneshot_actions'):
            self._oneshot_actions = []
        self._oneshot_actions.append(('clutch', clutch))
```

#### `DSpaceSimulation` (simulator.py)

The simulator's `executeActions()` method:

1. Calls `super().executeActions()` to process Scenic actions
2. Applies continuous controls from `_control_state` via `setVehicleControl()`
3. Applies one-shot actions from `_oneshot_actions` via `setVehicleGear()` and `setVehicleClutch()`
4. Clears `_control_state` and `_oneshot_actions` after application

### ControlDesk Variable Mappings

All controls write to `Platform_2` under `ExternalUserData`:

```python
# Continuous controls (applied every timestep)
KEY_THROTTLE = "Platform()://ASM_Traffic/.../ExternalUserData/Pos_AccPedal[%]/Value"
KEY_BRAKE = "Platform()://ASM_Traffic/.../ExternalUserData/Pos_BrakePedal[%]/Value"
KEY_STEER = "Platform()://ASM_Traffic/.../ExternalUserData/Angle_SteeringWheel[deg]/Value"

# One-shot controls (applied once)
KEY_GEAR = "Platform()://ASM_Traffic/.../ExternalUserData/Gear[]/Value"
KEY_CLUTCH = "Platform()://ASM_Traffic/.../ExternalUserData/Pos_ClutchPedal[%]/Value"
```

**Value Conversions**:
- Throttle: 0.0-1.0 → 0-100%
- Brake: 0.0-1.0 → 0-100%
- Steering: -1.0 to 1.0 → ±25 degrees
- Gear: 0-6 (passed through as integer)
- Clutch: 0.0-1.0 → 0-100%

---

## Usage Examples

### Simple Throttle + Steering

```scenic
behavior Drive():
    take SetThrottleAction(0.5)  # 50% throttle
    take SetSteerAction(-0.3)     # Steer left
    wait
    take SetSteerAction(0.3)      # Steer right
    wait
    take SetSteerAction(0.0)      # Center steering
```

### Complete Start Sequence (from Neutral)

**Key Concept**: Clutch is only needed when starting from neutral (gear 0 → 1).

```scenic
behavior StartFromNeutral():
    # Vehicle is in neutral (gear 0)
    take PressClutchAction()      # Press clutch
    wait
    take SetGearAction(1)          # Engage 1st gear
    wait
    take ReleaseClutchAction()     # Release clutch to start moving
    wait
    wait
    # Now drive
    take SetThrottleAction(0.5)
    take SetSteerAction(0.0)
```

### Normal Gear Shifting (No Clutch)

For normal shifting while moving, use `SetGearAction` directly:

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
    
    # Apply throttle
    take SetThrottleAction(0.6)
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

### Braking

```scenic
behavior Brake():
    take SetBrakeAction(0.6)      # 60% brake
    take SetThrottleAction(0.0)    # Release throttle
    take SetSteerAction(0.0)       # Center steering
```

### Progressive Acceleration

```scenic
behavior ProgressiveAcceleration():
    # Start from neutral
    take PressClutchAction()
    wait
    take SetGearAction(1)
    wait
    take ReleaseClutchAction()
    wait
    wait
    
    # Apply constant throttle
    take SetThrottleAction(0.6)     # 60% throttle
    wait
    
    # Progressive gear shifting
    take SetGearAction(2)
    wait
    wait
    take SetGearAction(3)
    wait
    wait
    take SetGearAction(4)
    wait
    wait
    take SetGearAction(5)
    wait
    wait
    take SetGearAction(6)
```

---

## Gear and Clutch: Key Concepts

### When to Use Clutch

| Situation | Clutch Needed? | Actions |
|-----------|----------------|---------|
| **Starting from neutral** | ✅ Yes | `PressClutchAction()` → `SetGearAction(1)` → `ReleaseClutchAction()` |
| **Normal shifting (1→2, 2→3, etc.)** | ❌ No | `SetGearAction(N)` only |
| **Downshifting (4→3, 3→2, etc.)** | ❌ No | `SetGearAction(N)` only |
| **Upshifting while moving** | ❌ No | `SetGearAction(N)` only |

**Rule of thumb**: Clutch is **only** for starting from neutral (gear 0 → 1). All other gear changes use `SetGearAction` directly.

### Execution Flow

1. **Action is taken** (`take SetGearAction(3)`)
2. **Protocol method called** (`obj.setGear(3)`)
3. **Queued as one-shot** (`_oneshot_actions.append(('gear', 3))`)
4. **`executeActions()` processes queue**
5. **Simulator writes to ControlDesk** (`setVehicleGear()`)
6. **Queue is cleared** (action executed once)

---

## Test Results

From `full_control_test.scenic`:

```
✅ Clutch: 1.0 -> 100.0%
✅ Gear: 1
✅ Clutch: 0.0 -> 0.0%
✅ Throttle: 0.5 -> 50.0%
✅ Steering: 0.0 -> 0.0 deg
✅ Gear: 2
✅ Throttle: 0.6 -> 60.0%
✅ Steering: -0.3 -> -7.5 deg
✅ Gear: 3
✅ Throttle: 0.7 -> 70.0%
✅ Steering: 0.4 -> 10.0 deg
✅ Brake: 0.5 -> 50.0%
✅ Throttle: 0.0 -> 0.0%
✅ Steering: 0.0 -> 0.0 deg
```

**Verification confirms**:
- ✅ Gear changes are one-shot (only written once per action)
- ✅ Clutch is one-shot (only written once per action)
- ✅ No continuous triggering
- ✅ ControlDesk writes successful
- ✅ Value conversions correct (0.5 → 50%, -0.3 → -7.5°)

### 2025‑11 Updates: External Control & Warm‑Up

To improve robustness and align with dSPACE’s External Signals model:

- ✅ Switched fellow control writes to a safe, model‑agnostic pattern:
  - We now perform a one‑time probe to determine the correct External Signals path (`km/h` vs `km|h`) and whether arrays are 0‑ or 1‑based.
  - Continuous controls for fellows (velocity and lateral deviation) are written via bulk array updates to `Environment/Traffic/PlantModel/FellowMovement/External_Signals/*`:
    - `Const_v_Fellows_External[km|h]/Value[<idx>]` (km/h)
    - `Const_d_Fellows_External[m]/Value[<idx>]` (meters)
  - We perform a readback from the same ExternalSignals arrays and log `v` and `d` to verify writes immediately.

- ✅ Robust fellow state reads:
  - Fellow positions and kinematics are read from `FellowMovement/FELLOW_POS_VEL/FellowTrailer/*` using bulk array gets (e.g., `x[ ]`, `y[ ]`, `yaw_deg_out[ ]`, `v_Fellows[ ]`, `w_Fellows[ ]`). This avoids indexing issues with uninitialized or 1‑based arrays.
  - We gate behavior execution until these bulk arrays return non‑zero values to ensure the fellow is spawned and the plant has initialized.

- ✅ Segment configuration for external control:
  - In `createFellowInDbSpace`, we now set the second segment’s `Activity.LongitudinalType` and `Activity.LateralType` to `"Continue"` and mark the segment `Endless`, so external velocity/deviation can drive motion without being overridden by a fixed `Const_*` profile.
  - Ensure `Route.UseExternal = True` in ModelDesk for each fellow so External Signals are consumed by the plant (this is enabled during ModelDesk authoring).

These changes reduce coupling to specific ASM enum names/indices and improve resilience when ControlDesk arrays aren’t immediately initialized.

---

## Implementation Files

### Domain Files (Abstract)
- **Actions**: `scenic/domains/driving/actions.py` (throttle, brake, steer)
- **Racing Actions**: `scenic/domains/racing/actions.py` (gear, clutch)
- **Protocol**: `scenic/domains/driving/actions.py` (`Steers`)
- **Racing Protocol**: `scenic/domains/racing/actions.py` (`HasManualTransmission`)
- **Racing Model**: `scenic/domains/racing/model.scenic` (exports actions)

### Simulator Files (Concrete)
- **Implementation**: `scenic/simulators/dspace/model.scenic` (`DSPACERacingCar`)
- **Simulator**: `scenic/simulators/dspace/simulator.py` (`setVehicleControl`, `setVehicleGear`, `setVehicleClutch`)
- **ControlDesk Wrapper**: `scenic/simulators/dspace/controldesk.py` (COM interface)

### Test Examples
- `set_throttle_example.scenic` - Throttle only
- `steering_test_example.scenic` - Steering + throttle
- `gear_automatic_example.scenic` - Normal gear shifting
- `clutch_manual_example.scenic` - Starting with clutch
- `full_control_test.scenic` - All controls together
- `progressive_acceleration.scenic` - Constant throttle with gear progression

---

## Key Design Decisions

1. **Domain-correct architecture**: Controls are racing domain features, not simulator-specific
2. **Protocol-based**: Uses `Steers` and `HasManualTransmission` protocols for consistency
3. **Continuous vs One-shot**: Proper separation prevents clutch/gear spam and matches real-world behavior
4. **Normalized inputs**: Scenic uses 0-1 for throttle/brake, -1 to 1 for steering (consistent across simulators)
5. **Automatic scaling**: Simulator converts to ControlDesk ranges (%, degrees) automatically
6. **Action accumulation**: Actions are stored and applied in batch during `executeActions()` for efficiency
7. **Clutch usage**: Explicit design choice that clutch is only for starting from neutral, not normal shifting

---

## Benefits

1. **Clean separation**: Controls are domain concerns, not simulator-specific
2. **One-shot behavior**: Prevents continuous writes that could cause issues
3. **Protocol pattern**: Other simulators can implement `Steers` and `HasManualTransmission` easily
4. **Domain consistency**: Follows same pattern across driving and racing domains
5. **Explicit control**: User controls when clutch is pressed/released
6. **Testability**: Clear examples demonstrate all control patterns
7. **Maintainability**: Well-organized code with clear responsibilities

---

## Related Documentation

- **dSPACE Control Interfaces**: See `DSPACE_CONTROL_INTERFACES.md` for ControlDesk variable paths and initialization requirements
- **Domain Architecture**: See `SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md` for architectural details

