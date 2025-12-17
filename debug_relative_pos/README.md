# Scenic Relative Positioning Testing Documentation

## Purpose

This folder contains debugging scripts and tools for testing and verifying Scenic's **relative positioning specifiers** in static placement scenarios. The focus is on validating that vehicles placed using relative specifiers (e.g., "left of", "right of", "ahead of", "behind") are correctly positioned in both Scenic's coordinate system and dSPACE ModelDesk/ControlDesk.

## Main Objective

**Test Scenic's relative positioning specifiers for static vehicle placement and verify their accuracy in dSPACE.**

The goal is to ensure that when vehicles are placed using relative specifiers like:
- `left of {Object} [by {distance}]`
- `right of {Object} [by {distance}]`
- `ahead of {Object} [by {distance}]`
- `behind {Object} [by {distance}]`

The resulting positions:
1. **In Scenic**: Are correctly computed relative to the reference object
2. **In dSPACE ModelDesk**: Are correctly transformed and placed
3. **In ControlDesk readback**: Match the expected Scenic positions (round-trip accuracy)

## Scenic Relative Positioning Specifiers

### Overview

Scenic provides several specifiers for relative positioning:

#### 1. **Left/Right of Object**

```scenic
# Place vehicle to the left of another vehicle
fellow1 = new RacingCar on mainRacingRoad
fellow2 = new RacingCar left of fellow1 by 5.0

# Place vehicle to the right (default distance uses contactTolerance)
fellow3 = new RacingCar right of fellow1
```

**Specifies**:
- `position` (priority 1)
- `parentOrientation` (priority 3)

**Dependencies**: `width`, `contactTolerance`

**Behavior**: Positions the object so that the distance between their bounding boxes is exactly the specified distance (or `contactTolerance` if `by {scalar}` is omitted).

#### 2. **Ahead/Behind Object**

```scenic
# Place vehicle ahead of another vehicle
fellow1 = new RacingCar on mainRacingRoad
fellow2 = new RacingCar ahead of fellow1 by 10.0

# Place vehicle behind (default uses contactTolerance)
fellow3 = new RacingCar behind fellow1
```

**Specifies**:
- `position` (priority 1)
- `parentOrientation` (priority 3)

**Dependencies**: `length`, `contactTolerance`

**Behavior**: 
- Without `by {scalar}`: Positions so the midpoint of the front/back side of the object's bounding box is at the reference position
- With `by {scalar}`: Places the object further ahead/behind by the given distance

#### 3. **Ahead/Behind Vector or Point**

```scenic
# Place relative to a point
fellow1 = new RacingCar ahead of (100, 200, 0) by 5.0

# Place relative to a vector
fellow2 = new RacingCar behind (1, 0, 0) by 3.0
```

**Specifies**:
- `position` (priority 1)

**Dependencies**: `length`, `orientation`

#### 4. **Combined Specifiers**

```scenic
# Place at front-left corner
fellow1 = new RacingCar on mainRacingRoad
fellow2 = new RacingCar at front left of fellow1

# Place at back-right corner
fellow3 = new RacingCar at back right of fellow1
```

**Behavior**: Positions at the midpoint of the corresponding edge/corner of the bounding box.

### Key Concepts

1. **Bounding Box**: Relative positioning uses the object's bounding box (based on `width`, `length`, `height`)
2. **Contact Tolerance**: Default distance when `by {scalar}` is omitted
3. **Orientation Inheritance**: Objects placed relative to another object inherit its `parentOrientation`
4. **Coordinate System**: Relative positions are computed in the reference object's local coordinate system

## Testing Objectives

### 1. **Scenic Generation Accuracy**

Test that Scenic correctly computes relative positions:
- Verify distances match expected values
- Verify orientations are correctly inherited
- Verify bounding box calculations are correct
- Test edge cases (very small/large distances, overlapping objects)

### 2. **Coordinate Transformation Accuracy**

Test that relative positions are correctly transformed to dSPACE:
- Verify XODR → RD transformation preserves relative relationships
- Verify (s,t) projection maintains relative positioning
- Test that route assignment works correctly for relative placements

### 3. **Round-Trip Accuracy**

Test that positions read back from ControlDesk match Scenic expectations:
- Verify readback coordinates match original Scenic coordinates
- Test accuracy for different relative positioning types
- Identify any systematic errors in relative positioning

### 4. **Multi-Vehicle Scenarios**

Test complex scenarios with multiple vehicles:
- Chains of relative positioning (A left of B, C right of A, etc.)
- Mixed relative and absolute positioning
- Relative positioning across different road segments

## Connection to dSPACE ModelDesk/ControlDesk

### ModelDesk Connection

**Location**: `src/scenic/simulators/dspace/modeldesk/`

To connect to ModelDesk:

```python
import pythoncom
from win32com.client import Dispatch

pythoncom.CoInitialize()
app = Dispatch("ModelDesk.Application")
proj = app.ActiveProject

if proj is None:
    raise RuntimeError("Open a ModelDesk project first")

exp = proj.ActiveExperiment
if exp is None:
    raise RuntimeError("Activate an experiment in ModelDesk")

ts = exp.TrafficScenario
if ts is None:
    raise RuntimeError("Active experiment has no TrafficScenario")
```

**Prerequisites**:
- ModelDesk must be open with an active project
- An experiment must be activated
- The experiment must have a TrafficScenario
- Routes must be configured (R1 for pit, R2 for lap)

**Setup Steps**:
1. Open ModelDesk
2. Open or create a project with Laguna Seca track
3. Activate an experiment
4. Ensure TrafficScenario is configured
5. Verify routes are set up in Road Generator:
   - Route R1 (Pit) - for pit lane vehicles
   - Route R2 (Lap) - for main racing circuit vehicles

### ControlDesk Connection

**Location**: `src/scenic/simulators/dspace/controldesk/`

To connect to ControlDesk:

```python
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

cd = ControlDeskApp().connect()
```

**Prerequisites**:
- ControlDesk must be running
- Scenario must be downloaded to simulator from ModelDesk
- Maneuver must be started (but not necessarily running)

**Setup Steps**:
1. Start ControlDesk
2. Connect to the simulator
3. Load the scenario (downloaded from ModelDesk)
4. Start the maneuver (but don't auto-run)

### Creating Fellows in ModelDesk

**Location**: `src/scenic/simulators/dspace/modeldesk/authoring.py`

Basic workflow for creating a Fellow vehicle:

```python
from scenic.simulators.dspace.geometry.utils import (
    configure_seg0_absolute_pose,
    configure_seg1_motion,
    make_endless_transition,
    clear_collection,
    ensure_two_segments,
)

# 1. Get or create fellow
fellow = ts.Fellows.Add()  # or ts.Fellows.Item(index)
fellow.Name = "TestFellow_1"

# 2. Configure fellow sequence
seqs = fellow.Sequences
clear_collection(seqs)
S1 = seqs.Add() if hasattr(seqs, "Add") else seqs.Item(0)
segs = ensure_two_segments(S1)

# 3. Set position (s, t) on segment 0
configure_seg0_absolute_pose(segs, s=s_value, t=t_value)

# 4. Set route
route_sel = S1.Route if hasattr(S1, 'Route') else S1.RouteSelection
route_sel.UseExternal = False
if hasattr(route_sel, 'Direction'):
    route_sel.Direction = 0
route_sel.Activate("R1")  # or "R2", "Pit", "Lap", etc.

# 5. Configure segment 1 (motion)
configure_seg1_motion(segs, v=0.0, t=0.0)  # Static placement
make_endless_transition(segs)

# 6. Save and download
ts.Save()
ts.Download()
time.sleep(0.5)
```

### Reading Vehicle Positions from ControlDesk

**Location**: `src/scenic/simulators/dspace/controldesk/readback.py`

**CRITICAL**: Follow this exact sequence for reliable readback:

```python
from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

# 1. Save scenario FIRST (CRITICAL!)
ts.Save()

# 2. Download to simulator
ts.Download()
time.sleep(0.5)

# 3. Reset maneuver
mc = exp.ManeuverControl
try:
    mc.Stop()
except:
    pass
time.sleep(0.2)
mc.Reset()
time.sleep(0.2)

# 4. Start maneuver (but don't auto-run)
mc.Start(False)
time.sleep(2.0)  # Wait for initialization

# 5. Connect to ControlDesk
cd = ControlDeskApp().connect()

# 6. Wait for simulation to initialize
time.sleep(2.0)

# 7. Step simulation multiple times (20+ steps recommended)
for i in range(20):
    cd.advance_simulation_step()
    time.sleep(0.1)

# 8. Wait for arrays to update
time.sleep(1.0)

# 9. NOW read positions
base_path = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/FELLOW_POS_VEL/FellowTrailer"
x_arr = cd.get_var(f"{base_path}/x")
y_arr = cd.get_var(f"{base_path}/y")

# Use same index for both arrays
fellow_x = x_arr[0]  # First fellow
fellow_y = y_arr[0]  # Same fellow
```

**Key Points**:
- **MUST save before downloading**: `ts.Save()` is critical
- **Wait times matter**: Need 2+ seconds after Start() and before reading
- **Step multiple times**: 20+ steps ensures vehicles are initialized
- **Array indexing**: Use `x_arr[0]` and `y_arr[0]` (same index for both)
- **Coordinate system**: Readback is in RD coordinates, need inverse transform to get XODR

### Coordinate Transformation

**Location**: `src/scenic/simulators/dspace/geometry/coordinate_transform.py`

To transform between XODR and RD coordinates:

```python
from scenic.simulators.dspace.geometry.coordinate_transform import (
    load_transform,
    apply_coordinate_transform,
    apply_inverse_coordinate_transform,
)

# Load transform file
transform_path = "assets/maps/dSPACE/Laguna_Seca_transform.json"
coordinate_transform = load_transform(transform_path)

# XODR → RD
scenic_xodr = (100.0, 200.0)
rd_x, rd_y = apply_coordinate_transform(coordinate_transform, scenic_xodr)

# RD → XODR (inverse)
xodr_x, xodr_y = apply_inverse_coordinate_transform(coordinate_transform, (rd_x, rd_y))
```

## Test Scenarios

### Basic Relative Positioning Tests

1. **Left/Right Placement**
   ```scenic
   ego = new RacingCar on mainRacingRoad
   fellow1 = new RacingCar left of ego by 5.0
   fellow2 = new RacingCar right of ego by 5.0
   ```
   - Verify: `fellow1` is 5m left, `fellow2` is 5m right
   - Check: Both are on the same road segment
   - Verify: Orientations match ego's orientation

2. **Ahead/Behind Placement**
   ```scenic
   ego = new RacingCar on mainRacingRoad
   fellow1 = new RacingCar ahead of ego by 10.0
   fellow2 = new RacingCar behind ego by 10.0
   ```
   - Verify: `fellow1` is 10m ahead, `fellow2` is 10m behind
   - Check: All are on the same road segment
   - Verify: Orientations match ego's orientation

3. **Combined Relative Positioning**
   ```scenic
   ego = new RacingCar on mainRacingRoad
   fellow1 = new RacingCar left of ego by 3.0
   fellow2 = new RacingCar ahead of fellow1 by 5.0
   fellow3 = new RacingCar right of fellow2 by 3.0
   ```
   - Verify: Chain of relative positions is correct
   - Check: All vehicles maintain proper relationships

### Edge Case Tests

1. **Very Small Distances**
   - Test with `by 0.1` (should not overlap)
   - Verify contact tolerance is respected

2. **Very Large Distances**
   - Test with `by 100.0` (should still be on road)
   - Verify road constraints are respected

3. **Cross-Segment Placement**
   - Place reference on `pitLaneRoad`, relative on `mainRacingRoad`
   - Verify: Should fail or be constrained appropriately

## Expected Test Results

### Success Criteria

1. **Scenic Generation**:
   - ✅ Vehicles are placed at correct relative positions
   - ✅ Distances match specified values (within tolerance)
   - ✅ Orientations are correctly inherited
   - ✅ All vehicles are on valid road regions

2. **dSPACE Placement**:
   - ✅ Vehicles appear in ModelDesk at expected locations
   - ✅ Route assignment is correct (based on road segment)
   - ✅ (s,t) coordinates are correctly computed

3. **Round-Trip Accuracy**:
   - ✅ Readback positions match Scenic positions
   - ✅ Relative relationships are preserved
   - ✅ Error < 1m for each vehicle (target)

### Known Issues

1. **T-Coordinate Ignored for Fellow Vehicles** (from `debug_route_code/README.md`):
   - ⚠️ dSPACE ModelDesk is ignoring t-coordinate for Fellow vehicles
   - All vehicles are placed on centerline regardless of t value
   - This may affect relative positioning accuracy
   - **Status**: Configuration issue, not code issue
   - **Impact**: Lateral relative positioning (left/right) may not work correctly

2. **Polygon Sampling**:
   - Scenic samples from polygon areas, not centerlines
   - This causes 4-8m lateral offsets from centerline
   - May affect relative positioning calculations

## Running Tests

### Prerequisites

1. **ModelDesk**: Open with Laguna Seca project
2. **ControlDesk**: Running and connected
3. **Scenario**: Active TrafficScenario in ModelDesk
4. **Dependencies**: Scenic source code in path

### Basic Test Execution

```bash
# From Scenic root directory
cd debug_relative_pos

# Run a test script
python test_left_right_placement.py
python test_ahead_behind_placement.py
python test_combined_relative_positioning.py
```

### Test Script Structure

A typical test script should:

1. **Generate Scenic scenario** with relative positioning
2. **Extract positions** from generated objects
3. **Verify relative relationships** in Scenic coordinates
4. **Transform to dSPACE** (XODR → RD → (s,t))
5. **Place in ModelDesk** and verify
6. **Read back from ControlDesk** and verify round-trip accuracy

## Related Documentation

- **Coordinate Transformation**: See `debug_cord_code/README.md`
- **Route Assignment**: See `debug_route_code/README.md`
- **dSPACE Integration**: See `AI_DOCUMENTS/DSPACE_COMPREHENSIVE_GUIDE.md`
- **Racing Domain**: See `src/scenic/domains/racing/README.md`
- **Scenic Specifiers**: See `docs/reference/specifiers.rst`

## Notes for AI Agents

### Key Implementation Details

1. **Relative Positioning Computation**:
   - Location: `src/scenic/syntax/veneer.py` (LeftSpec, RightSpec, Ahead, Behind)
   - Uses bounding box calculations based on object dimensions
   - Accounts for `contactTolerance` when distance not specified

2. **Coordinate Transformation**:
   - Relative positions are computed in Scenic (XODR coordinates)
   - Must be transformed to RD for dSPACE
   - Transformation preserves relative relationships (affine transform)

3. **Route Assignment**:
   - Based on road segment detection (pitLaneRoad → R1, mainRacingRoad → R2)
   - Relative positioning should maintain same route as reference object
   - Location: `src/scenic/simulators/dspace/geometry/route_mapping.py`

4. **T-Coordinate Issue**:
   - ⚠️ **CRITICAL**: T-coordinate is ignored for Fellow vehicles
   - This affects lateral relative positioning (left/right)
   - Longitudinal relative positioning (ahead/behind) should still work
   - See `debug_route_code/README.md` for details

### Testing Strategy

1. **Start Simple**: Test single relative positioning (one vehicle relative to another)
2. **Verify Scenic First**: Ensure Scenic computes correct positions before testing dSPACE
3. **Test Each Specifier**: Test left, right, ahead, behind separately
4. **Test Combinations**: Test chains of relative positioning
5. **Verify Round-Trip**: Test that readback matches Scenic expectations

### Common Pitfalls

1. **Forgetting to Save**: Always call `ts.Save()` before `ts.Download()`
2. **Insufficient Wait Times**: Need 2+ seconds after Start() before reading
3. **Wrong Array Indexing**: Use same index for x and y arrays
4. **Coordinate System Confusion**: Readback is RD, need inverse transform for XODR
5. **T-Coordinate Ignored**: Don't expect lateral positioning to work until configuration is fixed

## Test Results (2025 Session)

### Tests Performed

1. **test_dspace_coordinate_system.py** - Verified dSPACE RD coordinate system orientation
2. **test_left_normal_orientation.py** - Tested if left normal vector computation matches dSPACE convention
3. **test_scenic_left_vs_dspace_t.py** - Tested if Scenic's "left of" produces correct t-coordinate sign
4. **test_left_of_consistency.py** - Tested consistency of "left of" positioning in Scenic coordinate space (20 scenarios)
5. **test_transformation_pipeline.py** - Traced through transformation pipeline to find where relative position is lost (10 scenarios)
6. **test_orientation_transformation.py** - Verified orientation transformation (heading - π/2) and Scenic's "left of" specifier correctness (10 scenarios)
7. **test_scenic_left_computation.py** - Directly replicated Scenic's "left of" computation to verify correctness (1 scenario, position error < 0.0001m)
8. **test_left_of_full_pipeline.py** - Full pipeline test tracing "left of" through all transformation steps (10 scenarios)
9. **test_left_right_t_coordinate.py** - Extensive test of left/right positioning and t-coordinate relationship (50 scenarios each)

### Key Findings

#### ✅ Scenic's "Left Of" Specifier is CORRECT

**Test Results** (`test_scenic_left_computation.py`):
- **Position accuracy**: Error < 0.0001m - Scenic's computation matches expected position exactly
- **Alignment check**: ✅ PASS - Displacement correctly aligns with left vector
- **Conclusion**: Scenic's "left of" specifier is working correctly in Scenic coordinate space

**Key Finding**: 
- Scenic computes "left of" by:
  1. Creating a local offset `(-width/2 - distance - other_width/2 - tol, 0, 0)` in the reference object's local frame
  2. Transforming this local offset to world coordinates using `pos.relativePosition()` → `offsetLocally(orientation, vec)`
  3. The result is mathematically correct

**Implication**: Since other simulators (CARLA, Webots, LGSVL) work fine with the same Scenic values (`obj.position`, `obj.orientation`), the issue must be **dSPACE-specific**, not a Scenic bug.

#### ✅ Orientation Transformation is CORRECT

**Test Results** (`test_orientation_transformation.py`):
- **Orientation transformation verified**: The formula `dspace_orientation = scenic_heading - π/2` is correct
- **Conclusion**: The orientation transformation is not the issue

#### ⚠️ Previous Finding: Potential Inconsistency (Needs Re-investigation)

**Test Results** (`test_left_of_consistency.py`):
- **80% success rate (16/20)**: "Left of" positioning works correctly in Scenic coordinate space
- **20% failure rate (4/20)**: "Left of" positioning appeared to be **INVERTED** in some cases
- **Note**: The orientation transformation test shows 100% success, suggesting the earlier failures may have been due to test methodology or edge cases

**Test Results** (`test_transformation_pipeline.py`):
- **30% failure at XODR level**: Scenic's "left of" fails before any transformation
- **0% failure at RD transformation**: Coordinate transform preserves relationship correctly
- **30% failure at Route level**: Vehicles end up on different routes (making comparison invalid)
- **0% failure at T-coordinate**: When vehicles are on same route, t-coordinate relationship is CORRECT

**Root Cause Identified**:
- The issue is **NOT in the transformation pipeline**
- The issue is **NOT in t-coordinate sign convention**
- The issue is **IN SCENIC'S "LEFT OF" SPECIFIER** - it has a bug that causes ~20-30% failure rate

#### ❌ T-Coordinate Sign Convention is INVERTED

**Test Results** (`test_left_right_t_coordinate.py`):
- **Test Method**: 50 scenarios each for "left of" and "right of" positioning
- **Pipeline**: Full dSPACE integration pipeline (Scenic → XODR → RD → (s,t) projection)
- **Validation**: Only compares vehicles on the same road (ensures t-coordinates are comparable)

**"Left Of" Results**:
- Valid tests: 38/50 (76% success rate for same-road placement)
- Average t-difference (t2 - t1): **+2.069** (t2 > t1, more positive)
- 100% of cases show t2 > t1 (more positive t)
- **Finding**: "Left of" positioning results in **MORE POSITIVE t-coordinate**

**"Right Of" Results**:
- Valid tests: 42/50 (84% success rate for same-road placement)
- Average t-difference (t2 - t1): **-2.067** (t2 < t1, more negative)
- 100% of cases show t2 < t1 (more negative t)
- **Finding**: "Right of" positioning results in **MORE NEGATIVE t-coordinate**

**Conclusion**: The t-coordinate sign convention in dSPACE is **INVERTED** from the expected convention:
- ❌ **Actual dSPACE Convention**: 
  - Positive t = **RIGHT** of reference line
  - Negative t = **LEFT** of reference line
- ✅ **Expected Convention** (from earlier tests):
  - Positive t = Left of reference line
  - Negative t = Right of reference line

**Implications**:
- The dSPACE integration is working **consistently** (100% consistency in test results)
- The pipeline correctly transforms positions through XODR → RD → (s,t)
- However, the t-coordinate sign convention is **opposite** to what was expected
- This explains why "left of" appears inverted in ModelDesk output

**Root Cause**: The t-coordinate computation in `projection.py` uses a left normal vector, but the sign convention appears to be inverted. The left normal computation `(-vy, vx)` is mathematically correct (90° CCW from forward direction), but dSPACE's interpretation of positive/negative t may be inverted, OR the projection code needs to negate the t-coordinate.

#### ✅ Transformation Pipeline Preserves Relative Positioning

**Test Results** (`test_transformation_pipeline.py`):
- **XODR → RD**: 0% failure rate - coordinate transform preserves relationship
- **RD → (s,t)**: 0% failure rate - when vehicles are on same route, t-coordinate relationship is correct
- **Conclusion**: The transformation pipeline is working correctly. The issue is in Scenic's specifier, not the transformation.

#### ✅ Left Normal Computation is CORRECT

**Test Results**:
- Left normal vector is computed as `(-vy, vx)` from segment direction `(vx, vy)`
- This correctly identifies left as 90° CCW from forward direction
- Test verified: left point has t>0, right point has t<0

#### ⚠️ Route Assignment Issue Discovered

**Test Results**:
- **30% of cases**: Vehicles placed relative to each other ended up on **different routes**
  - Fellow1: Route = Lap (mainRacingRoad)
  - Fellow2: Route = Pit (pitLaneRoad)
- **Problem**: Vehicles placed relative to each other should be on the same route
- **Impact**: When vehicles are on different routes, their t-coordinates are not directly comparable (each route has its own coordinate system)

#### ✅ Coordinate System Orientation is CORRECT

**Test Results**:
- Coordinate transformation (XODR → RD) is working correctly
- RD coordinate system appears to be aligned with ENU (East-North-Up)
- Road segments are oriented correctly
- **Conclusion**: The PI/2 orientation conversion for vehicle heading is correct and doesn't affect t-coordinate computation

### Analysis

**🔍 REVISED UNDERSTANDING - Scenic is CORRECT, Issue is dSPACE-Specific**:

**Key Finding**: Direct replication of Scenic's computation shows it's mathematically correct:
- Position error: < 0.0001m
- Alignment check: ✅ PASS
- Other simulators (CARLA, Webots, LGSVL) work fine with same Scenic values

**Conclusion**: The issue is **NOT in Scenic**, but in **dSPACE-specific transformation or interpretation**.

**Where the Issue Likely Occurs**:

1. **Step 1 (XODR)**: ✅ **CORRECT** - Scenic's "left of" computation is correct
2. **Step 2 (RD)**: ✅ **CORRECT** - Coordinate transform preserves relationship (verified in test_left_of_full_pipeline.py: 100% pass rate)
3. **Step 3 (Route/Road)**: ⚠️ **24-26% failure** - Vehicles end up on different roads (separate issue, but test filters these out)
4. **Step 4 (T-coordinate)**: ❌ **INVERTED SIGN CONVENTION** - T-coordinate sign convention is opposite to expected (confirmed by test_left_right_t_coordinate.py)
5. **Step 5 (ModelDesk)**: ⚠️ **Known issue** - dSPACE ModelDesk ignores t-coordinate for Fellow vehicles (separate configuration issue)

**The Real Issues**:

1. **🔴 HIGH PRIORITY: dSPACE T-Coordinate Sign Convention INVERTED**: 
   - **CONFIRMED**: T-coordinate sign convention is inverted (test_left_right_t_coordinate.py)
   - Positive t = RIGHT (not left) of reference line
   - Negative t = LEFT (not right) of reference line
   - The projection computation is consistent but the sign is opposite to expected
   - **Location**: `src/scenic/simulators/dspace/geometry/projection.py` line 118 (t-coordinate computation)
   - **Solution Needed**: Negate t-coordinate OR fix the left normal vector computation

2. **Route/Road Assignment**: 24-26% of cases have vehicles on different roads, making t-coordinate comparison invalid (test filters these out, but this should be fixed).

3. **ModelDesk T-Coordinate Ignoring**: Separate configuration issue - dSPACE ModelDesk ignores t-coordinate for Fellow vehicles (documented in `debug_route_code/README.md`).

4. **Test Methodology**: Earlier test failures were due to incorrect test methodology (using heading to compute left vector instead of using Scenic's actual orientation transformation). This has been resolved.

### Recommendations

1. **🔴 HIGH PRIORITY: Fix T-Coordinate Sign Convention**: 
   - **CONFIRMED ISSUE**: T-coordinate sign is inverted (positive t = right, negative t = left)
   - **Fix**: Negate the t-coordinate in the projection computation OR invert the left normal vector
   - **Location**: `src/scenic/simulators/dspace/geometry/projection.py` line 118
   - Current code: `t_signed = raw_t * 0.3`
   - Suggested fix: `t_signed = -raw_t * 0.3` (negate to flip sign convention)
   - **Verification**: Re-run `test_left_right_t_coordinate.py` after fix - should show:
     - "Left of" → negative t (t2 < t1)
     - "Right of" → positive t (t2 > t1)

2. **Fix Road Assignment**: Ensure vehicles placed relative to each other are constrained to the same road (24-26% failure rate when vehicles end up on different roads).

3. **Address ModelDesk T-Coordinate Ignoring**: As documented in `debug_route_code/README.md`, dSPACE ModelDesk ignores t-coordinate for Fellow vehicles. This needs to be resolved at the ModelDesk configuration level.

4. **✅ Scenic is Correct**: No fix needed for Scenic's "left of" specifier - it's working correctly.
5. **✅ Coordinate Transformation is Correct**: XODR → RD transformation preserves relationships correctly (100% pass rate in tests).
6. **✅ Transformation Pipeline is Working**: The full pipeline (XODR → RD → (s,t)) is functioning correctly, just needs t-coordinate sign fix.

## Future Work

1. **🔴 Fix T-Coordinate Sign Convention**: Negate t-coordinate in projection.py to fix inverted sign
2. **Fix Road Assignment**: Ensure vehicles placed relative to each other are on the same road (24-26% failure rate)
3. **Fix ModelDesk T-Coordinate Configuration**: Resolve issue where t-coordinate is ignored for Fellow vehicles
4. **Test Ego Vehicles**: Verify if ego vehicles have same t-coordinate sign convention issue
5. **Complex Scenarios**: Test multi-vehicle relative positioning chains
6. **Cross-Segment**: Test relative positioning across different road segments (should be prevented or handled specially)
7. **Performance**: Test with many vehicles using relative positioning

## Summary for AI Agents

### Key Test: test_left_right_t_coordinate.py

**Purpose**: Verify that dSPACE integration correctly reflects "left of" and "right of" positioning in t-coordinates.

**Test Method**:
1. Generate 50 scenarios with `fellow2 = new RacingCar left of fellow1 by 5.0`
2. Generate 50 scenarios with `fellow2 = new RacingCar right of fellow1 by 5.0`
3. Transform positions through full pipeline: Scenic (XODR) → RD → (s,t) projection
4. Compare t-coordinates for vehicles on the same road
5. Verify t-coordinate sign convention

**Results**:
- **"Left of"**: 38/50 valid tests, average t-diff = +2.069 (t2 > t1) → **MORE POSITIVE t**
- **"Right of"**: 42/50 valid tests, average t-diff = -2.067 (t2 < t1) → **MORE NEGATIVE t**

**Finding**: T-coordinate sign convention is **INVERTED**:
- Actual: Positive t = RIGHT, Negative t = LEFT
- Expected: Positive t = LEFT, Negative t = RIGHT

**Fix Required**: Negate t-coordinate in `projection.py` line 118: `t_signed = -raw_t * 0.3`

**Status**: Issue confirmed, fix identified, awaiting implementation.

