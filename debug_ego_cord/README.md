# Ego Vehicle T-Coordinate Analysis

## Purpose

This folder is dedicated to testing and analyzing whether **ego vehicles** exhibit the same t-coordinate (lateral deviation) behavior that was discovered for **Fellow vehicles**.

## Background

### Discovery in Fellow Vehicle Testing

Testing in `debug_route_code/` revealed a **critical issue** with Fellow vehicles:

**Finding**: dSPACE ModelDesk is **completely ignoring the t-coordinate (lateral deviation)** when placing Fellow vehicles, regardless of the value set.

**Evidence from Fellow Testing**:
- Test set t-coordinate values: 0.0, 0.6, 1.2 (corresponding to 0m, 2m, 4m lateral offsets)
- **All readback positions were IDENTICAL**: `(-89.244390, -149.987233)` regardless of t value
- When projecting readback positions back to (s,t), all show t ≈ 0.0 (centerline)
- Position errors equal the attempted lateral offsets (2m offset → 2m error, 4m offset → 4m error)

**Test Scripts Used** (in `debug_route_code/`):
- `test_t_coordinate_calibration.py` - Tested multiple scale factors (all produced identical errors)
- `test_t_coordinate_no_scaling.py` - Tested without scaling (same results)
- `test_t_coordinate_readback_analysis.py` - **Critical test** that revealed t-coordinate is being ignored

**Vehicle Type**: All tests were performed on **Fellow vehicles only** (NOT ego vehicles).

## Question to Answer

**Does the ego vehicle exhibit the same behavior?**

We need to determine if:
1. Ego vehicles also ignore t-coordinate settings
2. Ego vehicles correctly apply t-coordinate (lateral deviation)
3. There are differences in how ego vs Fellow vehicles handle t-coordinate

## Test Plan

### Test 1: Ego T-Coordinate Readback Analysis

**Purpose**: Replicate `test_t_coordinate_readback_analysis.py` but for ego vehicles instead of Fellow vehicles.

**What to test**:
- Set ego vehicle at known lateral offsets (0m, 2m, 4m from centerline)
- Set t-coordinate in ModelDesk for ego vehicle
- Read back actual position from ControlDesk
- Project readback position to see what t-coordinate it corresponds to
- Compare expected vs actual t-coordinate

**Expected Behavior** (if ego works correctly):
- Different t values should produce different readback positions
- Readback positions should project back to the correct t-coordinate values
- Position errors should be small (< 0.1m for centerline, < 1m for lateral offsets)

**Expected Behavior** (if ego has same issue as Fellow):
- All readback positions will be identical regardless of t value
- Readback positions will all project to t ≈ 0.0 (centerline)
- Position errors will equal the attempted lateral offsets

### Test 2: Ego T-Coordinate Calibration

**Purpose**: Test if scale factor matters for ego vehicles (if t-coordinate is being applied).

**What to test**:
- Test multiple lateral offsets (0m, 2m, 4m, 6m, 8m)
- Test multiple scale factors (0.2, 0.25, 0.3, 0.35, 0.4)
- Measure round-trip errors for each combination

**Note**: This test is only meaningful if Test 1 shows that ego vehicles DO apply t-coordinate. If ego also ignores t-coordinate, this test will show identical results for all scale factors (as seen in Fellow testing).

### Test 3: Ego vs Fellow Comparison

**Purpose**: Direct comparison of ego and Fellow behavior.

**What to test**:
- Set both ego and Fellow at same (s, t) coordinates
- Compare readback positions
- Determine if there are differences in how they handle t-coordinate

## Implementation Notes

### Ego Vehicle Configuration

Ego vehicles are configured differently than Fellow vehicles:

**Fellow Configuration** (from `configure_seg0_absolute_pose`):
```python
# Uses segments
segs = sequence.Segments
seg0 = segs[0]
lat0 = seg0.Activity.LateralType
activate_type(lat0, "Deviation")
set_activity_constant(lat0, t_val)
```

**Ego Configuration** (from `create_ego_vehicle`):
```python
# Uses sequence directly
seq = ego_maneuver.Sequences.Item(0)
seq.StartPosition = s_val
# Lateral position set via segments (similar to Fellow)
segments = seq.Segments
seg0 = segments.Item(0)
lat0 = seg0.Activity.LateralType
activate_type(lat0, "Deviation")
set_activity_constant(lat0, t_val)
```

**Key Difference**: Ego uses `seq.StartPosition` for s-coordinate, while Fellow uses segment activity. Both use segment activity for t-coordinate.

### Reading Ego Position from ControlDesk

Ego vehicle position is read from different ControlDesk variables than Fellow vehicles:

**Fellow Position Path**:
```
Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer/x
Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer/y
```

**Ego Position Path**:
```
Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1
Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1
```

**Key Difference**: Fellow uses arrays (`x_arr[0]`, `y_arr[0]`), while Ego uses single values (`get_var(path_x)`, `get_var(path_y)`).

## Test Scripts

### 1. `test_ego_t_coordinate_readback_analysis.py` ✅

**Purpose**: Analyze what t-coordinate dSPACE actually uses for EGO vehicles by projecting readback positions.

**Status**: ✅ **COMPLETED**

**Results**:
- EGO vehicles exhibit the **SAME behavior** as Fellow vehicles
- All readback positions are essentially identical (within 0.01m)
- Position errors equal lateral offsets (2m offset → 2m error, 4m offset → 4m error)
- Cannot activate "Deviation" mode (warning: "Could not activate Deviation mode")

**Usage**:
```bash
cd debug_ego_cord
python test_ego_t_coordinate_readback_analysis.py
```

## Status

**Status**: ✅ **TEST COMPLETED** - Same issue as Fellow vehicles identified

**Test Results** (from `test_ego_t_coordinate_readback_analysis.py`):

### Key Findings

1. **EGO vehicles have the SAME issue as Fellow vehicles**:
   - All readback positions are essentially identical (within 0.01m) regardless of t-coordinate setting
   - Position errors equal lateral offsets (2m offset → 2m error, 4m offset → 4m error)
   - All readback positions project to t ≈ 0.0 (centerline)

2. **Root Cause**: "Could not activate Deviation mode"
   - The test cannot activate "Deviation" mode for ego vehicle lateral positioning
   - This prevents setting the t-coordinate (lateral deviation)
   - Same issue affects both EGO and Fellow vehicles

3. **Test Results Summary**:
   - t_offset = 0.0m: Position error = 0.012m, t_readback = -0.0004m
   - t_offset = 2.0m: Position error = 2.001m, t_readback = -0.0004m (should be ~2.0m)
   - t_offset = 4.0m: Position error = 4.001m, t_readback = -0.0004m (should be ~4.0m)

### Conclusion

**Both EGO and Fellow vehicles ignore t-coordinate settings** due to a ModelDesk configuration issue: "Deviation" mode cannot be activated for lateral positioning. This is a **critical issue** that prevents precise vehicle placement, which is essential for:
- Accurate scenario generation
- Precise waypoint following
- Correct maneuver planning based on 2D/3D position

### Next Steps

1. **Contact dSPACE Support**: Email drafted (see `email_to_dspace_support.md`) asking for guidance on:
   - Whether this behavior is intended
   - How to properly activate Deviation mode for lateral positioning
   - Alternative methods for precise vehicle placement

2. **Investigate Alternative Approaches**:
   - Test "Lane selection" mode instead of "Deviation"
   - Check if route configuration affects lateral deviation support
   - Verify if there are different APIs for EGO vs Fellow lateral positioning

3. **Documentation**: Findings documented in both `debug_ego_cord` and `debug_route_code` READMEs
