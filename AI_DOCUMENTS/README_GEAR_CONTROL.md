# Gear and Clutch Control Examples

This directory contains examples demonstrating gear and clutch control in racing scenarios.

## Key Concept

**Clutch is only for starting from neutral (gear 0 → 1)**

- **Starting**: Use clutch + `SetGearAction(1)` to engage 1st gear from neutral
- **Normal shifting**: Use `SetGearAction` directly (no clutch needed for 1→2, 2→3, etc.)

---

## Example 1: Normal Gear Shifting (`gear_automatic_example.scenic`)

**Scenario**: Vehicle is already in 1st gear and moving

**Actions used**:
- `SetGearAction(gear)` - Direct gear selection for shifting

**Example**:
```scenic
behavior NormalShift():
    # Already moving in 1st gear
    take SetGearAction(2)  # Shift to 2nd
    wait
    take SetGearAction(3)  # Shift to 3rd
    wait
    take SetGearAction(4)  # Shift to 4th
```

**Use when**:
- Vehicle is already moving
- Normal upshifts and downshifts
- Most racing scenarios

---

## Example 2: Starting from Neutral (`clutch_manual_example.scenic`)

**Scenario**: Vehicle starts in neutral (gear 0), needs clutch to engage 1st gear

**Actions used**:
- `PressClutchAction()` - Press clutch
- `SetGearAction(1)` - Engage 1st gear
- `ReleaseClutchAction()` - Release clutch
- `SetGearAction(2/3/4...)` - Normal shifting after that

**Example**:
```scenic
behavior StartAndDrive():
    # Start from neutral
    take PressClutchAction()      # Press clutch
    wait
    take SetGearAction(1)          # Engage 1st gear
    wait  
    take ReleaseClutchAction()     # Release clutch, start moving
    wait
    
    # Normal shifting (no clutch)
    take SetGearAction(2)
    wait
    take SetGearAction(3)
```

**Use when**:
- Vehicle starts in neutral
- Beginning of race from standstill
- Pit stop scenarios

---

## Summary

| Situation | Clutch Needed? | Actions |
|-----------|----------------|---------|
| **Starting from neutral** | ✅ Yes | `PressClutch` → `SetGear(1)` → `ReleaseClutch` |
| **Normal shifting (1→2, 2→3, etc.)** | ❌ No | `SetGearAction(N)` only |
| **Downshifting (4→3, 3→2, etc.)** | ❌ No | `SetGearAction(N)` only |

## Implementation Details

Both approaches use the `HasManualTransmission` protocol from the racing domain:
- `setGear(gear)` - Set gear directly
- `setClutch(clutch)` - Set clutch position (0.0-1.0)

The dSPACE simulator implements these methods and writes to ControlDesk:
- Gear: `ExternalUserData/Gear[]/Value` (integer 0-6)
- Clutch: `ExternalUserData/Pos_ClutchPedal[%]/Value` (0-100%)

Both are **one-shot actions** - they execute once per `take`, not continuously.

