# ControlDesk variable access (Platform_2)

All keys below live under `Model Root/Environment/Maneuver/PlantModel/ExternalUserData` on `Platform_2`.

Access pattern via COM:
- Read: `Variables["<key>"].ValueConverted`
- Write: `Variables["<key>"].ValueConverted = <value>`
- Use exact key strings (include `Platform()://.../Value`). Prefer numeric values (e.g., `0.0`, `10.0`).

## Confirmed writable inputs
- Steering wheel angle (deg)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Angle_SteeringWheel[deg]/Value`
- Accelerator pedal position (%)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_AccPedal[%]/Value`
- Brake pedal position (%)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_BrakePedal[%]/Value`
- Clutch pedal position (%)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_ClutchPedal[%]/Value`
  - Note: Model logic may change gear on clutch up/down.
- Gear selector (integer)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Gear[]/Value`
  - Range: 0–6 (integers). Booleans are rejected.
- Lane index reference
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/LaneIdx_Ref[]/Value`
- Steering torque (Nm)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Trq_Steering[Nm]/Value`
- Vehicle reference speed (m/s)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/v_Vehicle_Ref[m|s]/Value`
- Distance reference (m)
  - `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/d_Ref[m]/Value`

## Example (Python)
```python
from win32com.client import Dispatch
import pythoncom
pythoncom.CoInitialize()
app = Dispatch("ControlDeskNG.Application")
exp = app.ActiveExperiment
plat2 = exp.Platforms.Item("Platform").Platforms.Item("Platform_2")
vars_obj = plat2.ActiveVariableDescription.Variables
# Throttle 10%
vars_obj["Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Pos_AccPedal[%]/Value"].ValueConverted = 10.0
# Gear = 3 (0–6)
vars_obj["Platform()://ASM_Traffic/Model Root/Environment/Maneuver/PlantModel/ExternalUserData/Gear[]/Value"].ValueConverted = 3
```
