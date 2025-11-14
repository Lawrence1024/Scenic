# ControlDesk Joystick Integration

## Overview

This document describes the ControlDesk instrument script that maps physical joystick input to dSPACE VesiInterface manual control variables. The script runs within ControlDesk's instrument environment and converts raw joystick axis values to the appropriate ControlDesk variable ranges.

## Purpose

The instrument script bridges the gap between physical joystick controllers and dSPACE's VesiInterface manual control system. It:
- Receives raw joystick axis position changes via COM events
- Converts raw values to ControlDesk command ranges
- Writes directly to VesiInterface control variables
- Enables real-time manual control of the vehicle during simulation

## Implementation

### Function Signature

```python
def OnAxisPositionChanged(self, sender, value, axisType, axis):
    """
    Callback for joystick axis position changes in ControlDesk instrument.
    
    Args:
        sender: The sender object (joystick instrument)
        value: Raw axis value from joystick (range depends on axisType)
        axisType: Integer identifier for axis type
            0 = Steering axis
            1 = Throttle/Brake axis
        axis: Axis object reference
    """
```

### Axis Type 0: Steering

**Input Range:** `0` to `65535` (16-bit unsigned integer)

**Output Range:** `-70` to `+70` (ControlDesk steering command)

**Mapping Formula:**
```python
normalized_value = ((value / 65535.0) - 0.5) * 2 * 70
```

**Conversion Breakdown:**
1. Normalize to 0.0-1.0: `value / 65535.0`
2. Center at zero: `- 0.5` → Range: `-0.5` to `+0.5`
3. Expand to ±1.0: `* 2` → Range: `-1.0` to `+1.0`
4. Scale to ControlDesk range: `* 70` → Range: `-70` to `+70`

**ControlDesk Variable Path:**
```
Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value
```

**Behavior:**
- Center position (value ≈ 32767) → `0.0` (straight)
- Left turn (value → 0) → Approaches `+70` (maximum left)
- Right turn (value → 65535) → Approaches `-70` (maximum right)

### Axis Type 1: Throttle/Brake

**Input Range:** `0` to `32511` (approximately 16-bit, device-specific)

**Output Range:** 
- Throttle: `0` to `100` (when `normalized_value >= 0`)
- Brake: `0` to `100` (when `normalized_value <= 0`)

**Mapping Formula:**
```python
normalized_value = 100 - (value / 32511.0) * 100
```

**Conversion Breakdown:**
1. Normalize to 0.0-1.0: `value / 32511.0`
2. Scale to 0-100: `* 100`
3. Invert: `100 - ...` → High value = low output, low value = high output

**Result:**
- When `value ≈ 0` → `normalized_value ≈ 100` (full throttle)
- When `value ≈ 32511` → `normalized_value ≈ 0` (no throttle/brake threshold)
- When `value > 32511` → `normalized_value < 0` (brake zone)

**ControlDesk Variable Paths:**

*Throttle (when normalized_value >= 0):*
```
Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value
```

*Brake Front (when normalized_value <= 0):*
```
Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value
```

*Brake Rear (when normalized_value <= 0):*
```
Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value
```

**Behavior:**
- Released (value → 0) → Full throttle (`100`)
- Partially pressed (value → middle) → Reduced throttle
- Fully pressed (value → 32511) → No throttle (`0`)
- Over-pressed (value > 32511) → Negative value triggers brake application
  - Brake value = `-normalized_value` (converts negative to positive brake command)
  - Both front and rear brakes receive the same value

## Complete Implementation

```python
def OnAxisPositionChanged(self, sender, value, axisType, axis):
    """
    Handle joystick axis position changes and write to ControlDesk variables.
    
    This function is called automatically by ControlDesk when a connected
    joystick axis value changes.
    """
    if axisType == 0:
        # Steering axis: Map 0-65535 to -70 to +70
        normalized_value = ((value / 65535.0) - 0.5) * 2 * 70
        
        try:
            vars_obj = Application.ActiveExperiment.Platforms.Item("Platform").Platforms.Item("Platform_2").ActiveVariableDescription.Variables
            steering_path = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value"
            vars_obj[steering_path].ValueConverted = normalized_value
        except Exception as e:
            pass
    
    elif axisType == 1:
        # Throttle/Brake axis: Map 0-32511 to 100-0, negative = brake
        normalized_value = 100 - (value / 32511.0) * 100
        
        if normalized_value >= 0:
            # Throttle zone
            try:
                vars_obj = Application.ActiveExperiment.Platforms.Item("Platform").Platforms.Item("Platform_2").ActiveVariableDescription.Variables
                throttle_path = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"
                vars_obj[throttle_path].ValueConverted = normalized_value
            except Exception as e:
                pass
        
        if normalized_value <= 0:
            # Brake zone (applied when over-pressed)
            try:
                vars_obj = Application.ActiveExperiment.Platforms.Item("Platform").Platforms.Item("Platform_2").ActiveVariableDescription.Variables
                break_front_path = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"
                break_rear_path = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"
                vars_obj[break_front_path].ValueConverted = -normalized_value  # Convert negative to positive
                vars_obj[break_rear_path].ValueConverted = -normalized_value
            except Exception as e:
                pass
```

## ControlDesk Platform Navigation

The script navigates the ControlDesk COM object hierarchy to access variables:

```python
Application
  └─ ActiveExperiment
      └─ Platforms
          └─ Item("Platform")           # Outer platform
              └─ Platforms
                  └─ Item("Platform_2")  # Inner platform (simulation target)
                      └─ ActiveVariableDescription
                          └─ Variables   # Dictionary-like access to variables
```

## Variable Ranges Reference

| Control | Input Range | Output Range | Notes |
|---------|-------------|--------------|-------|
| Steering | 0-65535 | -70 to +70 | Center = 32767 → 0.0 |
| Throttle | 0-32511 | 100 to 0 | 0 = full throttle, 32511 = no throttle |
| Brake | >32511 | 0 to 100 | Applied when throttle axis over-pressed |

## Integration Points

### Related Documentation
- **[DSPACE_CONTROL_INTERFACES.md](./DSPACE_CONTROL_INTERFACES.md)**: COM automation API details
- **[VEHICLE_CONTROL_IMPLEMENTATION.md](./VEHICLE_CONTROL_IMPLEMENTATION.md)**: Scenic-side control protocols
- **[DSPACE_SIMULATOR_STRUCTURE.md](./DSPACE_SIMULATOR_STRUCTURE.md)**: Simulator implementation

### Code Locations
- ControlDesk variable paths match those used in:
  - `Scenic/src/scenic/simulators/dspace/simulator.py::setVehicleControl()`
  - `Scenic/src/scenic/simulators/dspace/simulator.py::_readAndPrintControlDeskValues()`
  - `Scenic/src/scenic/simulators/dspace/simulator.py::_initializeVesiInterface()`

## Troubleshooting

### Issue: Values appear as extremely large numbers (E+37 range)

**Cause:** Variable path mismatch or uninitialized memory

**Solution:**
1. Verify the exact ControlDesk variable path in the UI
2. Ensure `Platform_2` is the correct inner platform name
3. Check that VesiInterface manual control is enabled
4. Verify the variable exists and is accessible

### Issue: Steering direction reversed

**Solution:** Flip the sign in the steering calculation:
```python
# Instead of: * 70
normalized_value = -((value / 65535.0) - 0.5) * 2 * 70
```

### Issue: Throttle/Brake not responding correctly

**Possible Causes:**
1. Wrong maximum value (32511 may need adjustment for your joystick)
2. Inverted throttle/brake logic
3. Missing brake path configuration

**Solution:** Test and calibrate the `32511.0` divisor based on your joystick's actual maximum value.

### Issue: ControlDesk errors when writing

**Common Causes:**
- Variable path typo
- Wrong platform hierarchy
- Variable not available (model not loaded)
- VesiInterface not initialized

**Debug:** Add error logging:
```python
except Exception as e:
    print(f"Error writing to ControlDesk: {e}")
```

## Calibration Notes

### Steering Calibration
- Center point: Adjust if joystick center doesn't map to `value ≈ 32767`
- Dead zone: Can be added to ignore small movements around center
- Range: `70` can be adjusted for more/less sensitive steering

### Throttle/Brake Calibration
- Maximum value: `32511` may need adjustment based on joystick model
- Threshold: The point where throttle transitions to brake can be tuned
- Sensitivity: Adjust `100` multiplier for more/less sensitive control

## Best Practices

1. **Error Handling**: Always wrap COM calls in try-except blocks
2. **Value Clamping**: Consider adding explicit bounds checking:
   ```python
   normalized_value = max(-70.0, min(70.0, normalized_value))
   ```
3. **Dead Zones**: Consider implementing dead zones for analog joystick drift
4. **Testing**: Test all axes separately to verify correct mapping
5. **Documentation**: Update calibration values if joystick hardware changes

## Future Enhancements

Potential improvements:
- Gear shift support via button events
- Clutch control via separate axis
- Dead zone configuration
- Sensitivity adjustment parameters
- Multiple joystick support
- Calibration wizard

## Notes

- The script runs in ControlDesk's Python environment, not Scenic's Python
- COM objects must be accessed through the `Application` global variable
- Variable paths use ControlDesk's path syntax with `Platform()://` prefix
- The script executes in real-time as joystick values change
- Errors are silently caught to prevent instrument crashes












