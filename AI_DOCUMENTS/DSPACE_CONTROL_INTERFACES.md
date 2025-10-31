# dSPACE ControlDesk Control Interfaces

## Overview

dSPACE provides **two different interfaces** for programmatic vehicle control in ControlDesk:

1. **ExternalUserData Interface** - Simple, unified controls (currently used by Scenic)
2. **VesiInterface Manual Control** - Advanced interface with separate front/rear brake control

This document describes both interfaces, their initialization requirements, and variable paths.

---

## Interface Comparison

| Feature | ExternalUserData | VesiInterface Manual |
|---------|------------------|---------------------|
| **Throttle** | `Pos_AccPedal[%]/Value` | `Const_throttle_cmd/Value` |
| **Brake** | Unified `Pos_BrakePedal[%]/Value` | Separate `Const_brake_cmd_front` and `Const_brake_cmd_rear` |
| **Steering** | `Angle_SteeringWheel[deg]/Value` | `Const_steering_cmd/Value` |
| **Gear** | `Gear[]/Value` (0-6) | `Const_gear_cmd/Value` (0-6) |
| **Clutch** | `Pos_ClutchPedal[%]/Value` | Not available |
| **Initialization** | Simple (direct control) | **Required** (master switches, race control) |
| **Enable Flags** | Not required | Required per control type |
| **Complexity** | Low | High |

**Recommendation**: Use ExternalUserData for most scenarios. Use VesiInterface only if you need separate front/rear brake control.

---

## Interface 1: ExternalUserData

### Overview

The ExternalUserData interface provides simple, direct control over vehicle inputs. All variables are writable without initialization requirements.

### Variable Paths

All variables are located under:
```
Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData
```

On `Platform_2`.

### Confirmed Writable Variables

#### Steering Control
- **Path**: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Angle_SteeringWheel[deg]/Value`
- **Type**: Float (degrees)
- **Range**: Typically ±25 degrees
- **Scenic Mapping**: -1.0 to 1.0 → ±25 degrees

#### Throttle Control
- **Path**: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_AccPedal[%]/Value`
- **Type**: Float (percentage)
- **Range**: 0-100%
- **Scenic Mapping**: 0.0 to 1.0 → 0-100%

#### Brake Control (Unified)
- **Path**: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_BrakePedal[%]/Value`
- **Type**: Float (percentage)
- **Range**: 0-100%
- **Scenic Mapping**: 0.0 to 1.0 → 0-100%
- **Note**: Unified brake pedal (applies to both front and rear wheels)

#### Clutch Control
- **Path**: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value`
- **Type**: Float (percentage)
- **Range**: 0-100%
- **Scenic Mapping**: 0.0 to 1.0 → 0-100%
- **Note**: Model logic may change gear on clutch up/down

#### Gear Control
- **Path**: `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Gear[]/Value`
- **Type**: Integer
- **Range**: 0-6 (0 = Neutral, 1-6 = Gear positions)
- **Important**: Must be integer, not boolean

#### Additional Variables

These variables are also available but may not be actively used:

- **Lane Index Reference**: `LaneIdx_Ref[]/Value`
- **Steering Torque**: `Trq_Steering[Nm]/Value`
- **Vehicle Reference Speed**: `v_Vehicle_Ref[m|s]/Value`
- **Distance Reference**: `d_Ref[m]/Value`

### Access Pattern

```python
from win32com.client import Dispatch
import pythoncom

pythoncom.CoInitialize()
app = Dispatch("ControlDeskNG.Application")
exp = app.ActiveExperiment
plat2 = exp.Platforms.Item("Platform").Platforms.Item("Platform_2")
vars_obj = plat2.ActiveVariableDescription.Variables

# Read
throttle = vars_obj["Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_AccPedal[%]/Value"].ValueConverted

# Write
vars_obj["Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_AccPedal[%]/Value"].ValueConverted = 50.0
vars_obj["Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Gear[]/Value"].ValueConverted = 3
```

**Note**: Use `ValueConverted` property for both reading and writing. Use exact key strings including `Platform()://.../Value`.

---

## Interface 2: VesiInterface Manual Control

### Overview

The VesiInterface Manual Control provides advanced control capabilities, including **separate front and rear brake control**. However, it requires specific initialization variables to be set **before** control commands will be accepted.

**Critical Discovery**: Without proper initialization, control commands are silently ignored even if individual enable flags are set.

### Initialization Required

The following variables **must be set at the beginning of the simulation** (before sending control commands):

#### 1. VesiInterface Master Switches

| Variable Path | Required Value | Description |
|--------------|----------------|-------------|
| `Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0\|1]/Value` | `0.0` | Deactivates CLIF interface |
| `Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0\|1]/Value` | `1.0` | **CRITICAL**: Enables manual VESI control interface |

**Note**: `Sw_Manual_VESI_Overwrite` is the master switch. Without this set to `1.0`, all manual control commands are ignored.

#### 2. Race Control Configuration

| Variable Path | Required Value | Description |
|--------------|----------------|-------------|
| `Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern\|1Extern\|2Orchestrator]/Value` | `0.0` | Sets to **Internal** mode (required for manual control) |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value` | `9` | System state constant (unsigned integer) - **CRITICAL** |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value` | `1` | Track flag (unsigned integer) |
| `Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value` | `0` | Vehicle flag (unsigned integer) |

**Critical Variables**: `Sw_RaceControl` (set to `0.0` for Intern mode) and `Const_sys_state` (set to `9`) were the key variables that made manual control functional.

#### 3. Enable Flags

Enable individual control channels:

| Variable Path | Required Value | Description |
|--------------|----------------|-------------|
| `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_brake_cmd/Value` | `1` | Enable brake commands |
| `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_gear_cmd/Value` | `1` | Enable gear commands |
| `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_steering_cmd/Value` | `1` | Enable steering commands |
| `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_throttle_cmd/Value` | `1` | Enable throttle commands |

### Control Command Variables

Once initialized, control commands can be sent to:

#### Throttle Control
- **Path**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value`
- **Type**: Float
- **Range**: Typically 0-100+ (depends on model configuration)

#### Brake Control (Front/Rear Separate)
- **Front**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value`
- **Rear**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value`
- **Type**: Float
- **Range**: 0.0 (no brake) to maximum brake value
- **Advantage**: Allows independent front/rear brake control

#### Steering Control
- **Path**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value`
- **Type**: Float
- **Range**: Depends on model (typically degrees or normalized -1 to 1)

#### Gear Control
- **Path**: `Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value`
- **Type**: Integer
- **Range**: 0-6 (0 = neutral, 1-6 = gear positions)

### Complete Initialization Example

```python
from win32com.client import Dispatch
import pythoncom

pythoncom.CoInitialize()
app = Dispatch("ControlDeskNG.Application")
exp = app.ActiveExperiment
plat2 = exp.Platforms.Item("Platform").Platforms.Item("Platform_2")
vars_obj = plat2.ActiveVariableDescription.Variables

# Step 1: VesiInterface Master Switches
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value"].ValueConverted = 0.0
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value"].ValueConverted = 1.0  # CRITICAL

# Step 2: Race Control Configuration
vars_obj["Platform()://ASM_Traffic/Model Root/RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value"].ValueConverted = 0.0  # Intern mode
vars_obj["Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_sys_state/Value"].ValueConverted = 9  # CRITICAL
vars_obj["Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_track_flag/Value"].ValueConverted = 1
vars_obj["Platform()://ASM_Traffic/Model Root/RaceControl/race_control/Const_veh_flag/Value"].ValueConverted = 0

# Step 3: Enable Individual Control Channels
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_brake_cmd/Value"].ValueConverted = 1
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_gear_cmd/Value"].ValueConverted = 1
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_steering_cmd/Value"].ValueConverted = 1
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_enable_throttle_cmd/Value"].ValueConverted = 1

# Step 4: Now you can send control commands
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value"].ValueConverted = 20.0
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value"].ValueConverted = 1
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value"].ValueConverted = 0.0
vars_obj["Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value"].ValueConverted = 0.0
```

### Troubleshooting

If commands are not reflecting:

1. **Check Master Switch**: Verify `Sw_Manual_VESI_Overwrite = 1.0`
2. **Check Race Control**: Verify `Sw_RaceControl = 0.0` (Intern mode)
3. **Check System State**: Verify `Const_sys_state = 9`
4. **Check Enable Flags**: Verify all relevant `Const_enable_*_cmd` flags are set to 1
5. **Verify Platform**: Ensure you're accessing variables on `Platform_2`
6. **Check Online Calibration**: Ensure ControlDesk is in online calibration mode

---

## Platform Information

- **Platform Structure**: `Platform` (outer) → `Platform_2`, `Platform_3`, `Platform_4` (nested)
- **Variables Location**: All variables are on `Platform_2`
- **Key Path Format**: `Platform()://ASM_Traffic/Model Root/...`
- **Variables Object**: `exp.Platforms.Item("Platform").Platforms.Item("Platform_2").ActiveVariableDescription.Variables`

---

## References

- This documentation was created through iterative testing with `probe_controldesk.py`
- Based on dSPACE ControlDesk NG Application COM interface
- Tested with ControlDesk version that supports `ControlDeskNG.Application` ProgID

