# dSPACE Coordinate Transformation Pipeline

## Overview

This document traces the complete coordinate transformation pipeline from Scenic's world coordinates to dSPACE ModelDesk's (s,t) road coordinates. Understanding this pipeline is critical for debugging vehicle positioning issues and ensuring accurate placement in dSPACE simulations.

## Table of Contents

1. [Transformation Pipeline Overview](#transformation-pipeline-overview)
2. [Phase 1: Setup & Initialization](#phase-1-setup--initialization)
3. [Phase 2: XODR → RD Coordinate Transform](#phase-2-xodr--rd-coordinate-transform)
4. [Phase 3: Geometric Projection to (s,t)](#phase-3-geometric-projection-to-st)
5. [Phase 4: Orientation Conversion](#phase-4-orientation-conversion)
6. [Phase 5: Application to ModelDesk](#phase-5-application-to-modeldesk)
7. [Key Calibration Parameters](#key-calibration-parameters)
8. [Coordinate System Differences](#coordinate-system-differences)
9. [Troubleshooting](#troubleshooting)

---

## Transformation Pipeline Overview

The complete transformation flow:

```
Scenic Scene (world coordinates)
    ↓  obj.position = (scenic_x, scenic_y)
    ↓  obj.heading = heading_angle
  
[Optional: Coordinate Transform]
    ↓  XODR coordinates → RD coordinates
    ↓  (transformed_x, transformed_y)
  
[Geometric Projection]
    ↓  World (x,y) → Road (s,t)
    ↓  s_val = longitudinal along reference line
    ↓  t_val = lateral deviation (scaled × 0.3)
  
[Orientation Adjustment]
    ↓  Scenic heading → dSPACE yaw
    ↓  VehicleOrientation = heading - π/2
  
ModelDesk COM API
    ↓  seq.StartPosition = s_val
    ↓  seg0.LateralType.Constant = t_val
    ↓  seq.VehicleOrientation = dspace_orientation
```

**Key Code Locations:**
- Setup: `simulator.py` lines 111-168
- Transform: `simulator.py` lines 237-247, 352-362
- Projection: `geometry/projection.py` lines 64-148
- Orientation: `simulator.py` lines 297-301, 415-423
- Application: `simulator.py` lines 285, 404, 307-323

---

## Phase 1: Setup & Initialization

**Location:** `simulator.py` lines 111-168

### Purpose

Build road geometry indices and coordinate transformation from Scenic map parameters. This phase determines whether to use XODR-only or full XODR→RD transformation pipeline.

### Decision Tree

```
param map = 'LagunaSeca.xodr'
Expected: 'Laguna_Seca.rd' (check if exists)

IF RD file exists:
    → Build XODR → RD transformation
    → Use RD geometry for projection
    → Full accuracy pipeline
    
ELSE:
    → Use XODR-only geometry
    → Fallback mode (may have positioning errors)
```

### Coordinate Transformation Building

**Location:** `geometry/coordinate_transform.py` lines 17-135

**Purpose:** Automatically compute transformation between XODR and RD coordinate systems.

**Process:**

1. **Parse Geometries**
   - Parse RD file: `parse_rd_geometry(rd_path, step=0.5)`
   - Parse XODR file: `build_xodr_sec_points(xodr_path, step=2.0)`

2. **Sample Calibration Points**
   - Sample `num_samples` points (default: 100) at equal s-intervals
   - For each s: get `(xodr_x, xodr_y)` from XODR and `(rd_x, rd_y)` from RD
   - Store as calibration pairs

3. **Compute Transformation**
   - Calculate differences: `diffs = rd_coords - xodr_coords`
   - Mean offset: `mean_offset = mean(diffs)`
   - Standard deviation: `std_offset = std(diffs)`

4. **Choose Transform Type**
   ```python
   IF std_offset < 5m:
       Transform: 'translation'
       offset = mean_offset
   ELSE:
       Transform: 'affine'
       matrix A: least squares fit
       [rd_x, rd_y]^T = A × [xodr_x, xodr_y]^T + b
   ```

5. **Validate & Cache**
   - Validate on subset of calibration points
   - Mean error should be < 2m for good transform
   - Cache to `'_transform.json'` for reuse

**Cache File:** `Laguna_Seca_transform.json`

### Road Index Building

**RD Geometry:** `geometry/rd_parser.py` lines 105-166
- Parse RD file cubic polynomial segments
- Sample at 0.5m intervals for high precision
- Create `(x, y, s)` points in RD coordinate system
- **Independent s-coordinates:** each road starts at s=0

**XODR Geometry:** `geometry/xodr_parser.py` lines 60-125
- Parse XODR line/arc segments
- Sample at 2.0m intervals (balance accuracy/speed)
- Create `(x, y, s)` points in XODR coordinate system
- **Independent s-coordinates:** each road starts at s=0

**Output:**
```python
self._coordinate_transform: dict or None
self._road_index: dict with 'roads' key
```

---

## Phase 2: XODR → RD Coordinate Transform

**Location:** `simulator.py` lines 237-247, 352-362  
**Location:** `geometry/coordinate_transform.py` lines 196-223

### Purpose

Transform Scenic coordinates from XODR coordinate system to RD coordinate system before geometric projection. This step is critical when both XODR and RD files are available.

### Process

```python
# Extract Scenic coordinates
scenic_x, scenic_y = obj.position.x, obj.position.y

# Apply transformation if available
IF self._coordinate_transform is not None:
    transformed_x, transformed_y = apply_coordinate_transform(
        self._coordinate_transform, (scenic_x, scenic_y)
    )
ELSE:
    transformed_x, transformed_y = scenic_x, scenic_y
```

### Transformation Types

**Translation Transform** (simple offset):
```python
IF transform['type'] == 'translation':
    dx, dy = transform['offset']
    result = (x + dx, y + dy)
```

**Affine Transform** (matrix multiplication + offset):
```python
IF transform['type'] == 'affine':
    A = np.array(transform['matrix'])  # 2×2 matrix
    b = np.array(transform['offset'])  # 2D offset
    pos_vec = np.array([x, y])
    result = A @ pos_vec + b
```

This accounts for:
- Translation (origin offset)
- Rotation (different coordinate system orientation)
- Scale (different units or scaling factors)

**Example Output:**
```
Scenic coords (-101.920, -457.520) → RD coords (-98.123, -453.245)
```

---

## Phase 3: Geometric Projection to (s,t)

**Location:** `geometry/projection.py` lines 64-148

### Purpose

Project world coordinates `(x, y)` onto the nearest road reference segment, computing longitudinal `s` and lateral `t` coordinates.

### Algorithm

The projection algorithm finds the closest point on any road segment and computes `(s, t)` coordinates relative to that segment.

#### Step 1: Find Closest Point on Segment

For each segment: `point_0(x0,y0,s0) → point_1(x1,y1,s1)`

```python
# Segment direction vector
vx, vy = x1 - x0, y1 - y0
seg_len2 = vx*vx + vy*vy

# Vector from segment start to point
wx, wy = px - x0, py - y0

# Projection parameter (0 = start, 1 = end)
u = (wx*vx + wy*vy) / seg_len2
u = clamp(u, 0.0, 1.0)  # Ensure on segment

# Closest point on segment
qx = x0 + u*vx
qy = y0 + u*vy

# Distance to segment
dx = px - qx
dy = py - qy
dist2 = dx*dx + dy*dy
```

#### Step 2: Compute s-Coordinate (Longitudinal)

Interpolate along the segment based on the projection parameter:

```python
s_proj = s0 + u × (s1 - s0)
```

This gives the distance along the reference line where the projected point lies.

#### Step 3: Compute t-Coordinate (Lateral Deviation)

Compute signed lateral distance from reference line:

```python
# Segment length
seg_len = sqrt(seg_len2)

# Left normal vector (perpendicular to segment, pointing left)
nx_left = -vy/seg_len
ny_left =  vx/seg_len

# Raw lateral distance
raw_t = dx × nx_left + dy × ny_left

# Apply calibration scale
t_val = raw_t × 0.3
```

**Notes:**
- **Left normal:** Perpendicular to road direction, pointing left
- **Positive t:** Left of reference line
- **Negative t:** Right of reference line
- **Scale factor 0.3:** Calibrated to match expected lane width

#### Step 4: Select Closest Projection

```python
# Collect all projections from all segments
all_projections.append((dist2, s_proj, t_val, road_id, road_name))

# Sort by distance and take the closest
all_projections.sort(key=lambda x: x[0])
best = all_projections[0]

# Return (s, t)
return (s_proj, t_val)
```

**Example Output:**
```
World coordinates (-98.123, -453.245) → Road coordinates (s=1234.5, t=0.045)
```

---

## Phase 4: Orientation Conversion

**Location:** `simulator.py` lines 297-301, 415-423

### Purpose

Convert Scenic heading angles to dSPACE orientation angles, accounting for different coordinate system conventions.

### Coordinate System Differences

| System | Zero Angle Points | Rotation Convention |
|--------|------------------|---------------------|
| **Scenic** | North (+Y axis) | Counter-clockwise (CCW) |
| **dSPACE** | East (+X axis) | Counter-clockwise (CCW) |

**Scenic conventions:**
- `heading = 0` → points North (+Y axis)
- `heading = π/2` → points West (-X axis)
- Uses standard mathematical convention

**dSPACE conventions:**
- `yaw = 0` → points East (+X axis)
- `yaw = π/2` → points North (+Y axis)
- Uses engineering/OpenDRIVE convention

### Conversion

```python
IF hasattr(obj, 'heading'):
    dspace_orientation = obj.heading - π/2
    seq.VehicleOrientation = dspace_orientation
ELSE:
    VehicleOrientation = 0.0  # aligned with road
```

**Conversion Derivation:**
- Scenic: `0°` (North) = `90°` (East) in dSPACE
- Therefore: `dSPACE_angle = Scenic_angle - 90°` or `- π/2 radians`

**Example:**
```
Scenic heading: 45° → dSPACE orientation: -45°
Scenic heading: 90° → dSPACE orientation: 0°
Scenic heading: 180° → dSPACE orientation: 90°
```

---

## Phase 5: Application to ModelDesk

**Location:** `simulator.py` lines 285, 307-323, 404  
**Location:** `geometry/utils.py` lines 76-93

### Purpose

Apply the computed `(s, t)` coordinates and orientation to dSPACE ModelDesk via COM automation.

### EGO Vehicle Configuration

**API:** Maneuver Collection (`ts.Maneuver.Item(0)`)

```python
# Access ego maneuver
maneuver_collection = self.ts.Maneuver
ego_maneuver = maneuver_collection.Item(0)
seq = ego_maneuver.Sequences.Item(0)

# Set longitudinal position
seq.StartPosition = float(s_val)

# Set orientation
seq.VehicleOrientation = dspace_orientation

# Set lateral deviation (if significant)
IF |t_val| > 0.1:
    seg0 = seq.Segments.Item(0)
    
    # Activate Deviation mode
    seg0.LateralType.Activate("Deviation")
    
    # Set dependency to Absolute (not Relative)
    dep = seg0.LateralType.ActiveElement.DependencyType
    dep.Activate("Absolute")
    
    # Set deviation value
    seg0.LateralType.ActiveElement.SourceType.ActiveElement.Constant = t_val
```

**ModelDesk COM Properties:**
- `StartPosition`: Longitudinal position along reference line (meters)
- `VehicleOrientation`: Orientation relative to road direction (radians)
- `LateralType.Constant`: Lateral deviation from reference line (meters, absolute)

### FELLOW Vehicle Configuration

**API:** Fellows Collection (`ts.Fellows`)

```python
# Create fellow
F = self.ts.Fellows.Add()
F.Name = f"Fellow_{count}"

# Access sequence
seq = F.Sequences.Item(0)
segs = seq.Segments

# Configure seg0: absolute position
configure_seg0_absolute_pose(segs, s=s_val, t=t_val)

# Configure seg1: motion
configure_seg1_motion(segs, v=0.0, t=t_val)
make_endless_transition(segs)
```

**Inside `configure_seg0_absolute_pose` (`geometry/utils.py` lines 76-93):**

```python
# Longitudinal: Position (absolute along reference line)
lt0 = segs[0].Activity.LongitudinalType
lt0.Activate("Position")
lt0.ActiveElement.SourceType.ActiveElement.Constant = s

# Lateral: Deviation with Absolute dependency
lat0 = segs[0].Activity.LateralType
lat0.Activate("Deviation")
dep = lat0.ActiveElement.DependencyType
dep.Activate("Absolute")  # not Relative
lat0.ActiveElement.SourceType.ActiveElement.Constant = t
```

**Inside `configure_seg1_motion` (`geometry/utils.py` lines 95-104):**

```python
# Longitudinal: Velocity
lt1 = segs[1].Activity.LongitudinalType
lt1.Activate("Velocity")
lt1.ActiveElement.SourceType.ActiveElement.Constant = v

# Lateral: Continue (maintain current lateral position)
lat1 = segs[1].Activity.LateralType
lat1.Activate("Continue")
```

**Segment Structure:**
- **seg0:** Initial position and orientation (absolute placement)
- **seg1:** Motion parameters (velocity, steering, endless transition)

### Special Cases

**Pit Lane Wrapping** (`simulator.py` lines 368-373):

For fellow vehicles on pit lane where `s_val > 1000`:

```python
IF route_pref == "Pit" AND s_val > 1000:
    s_val = s_val % 883.4  # Wrap to pit lane length
```

This ensures vehicles on pit lane stay within the pit lane bounds.

---

## Key Calibration Parameters

### t-Coordinate Scale Factor: 0.3×

**Location:** `geometry/projection.py` line 118

```python
t_val = raw_t × 0.3
```

**Purpose:** Transform raw lateral distance to ModelDesk-compatible units.

**Calibration:**
- Typical lane width: 3-4 meters
- Without scaling: `t` values can be ±1.5 to ±2.0 meters
- With 0.3× scaling: `t` values in range ±0.45 to ±0.60 meters
- Matches expected ModelDesk calibration data

**Adjustment:** If vehicles appear laterally misaligned, modify this scale factor.

### Independent s-Coordinates

**Purpose:** Prevent coordinate clustering when multiple vehicles on same road.

**Implementation:**
- Each road segment has its own s-coordinate system
- Range: `0` to `road_length`
- Vehicles on same road use same s-coordinate range
- Prevents ambiguities from cumulative s-coordinates

**Example:**
```
Main road: s ∈ [0, 2484.6]
Pit lane:  s ∈ [0, 883.4]
```

### Sampling Intervals

**XODR:** 2.0 meters (`xodr_parser.py` line 60)
- Balance between accuracy and performance
- Sufficient for most scenarios
- Faster parsing

**RD:** 0.5 meters (`rd_parser.py` line 22)
- Higher precision for fine-grained positioning
- Slower parsing but more accurate
- Used when RD file is available

**Adjustment:** Smaller intervals = more accurate but slower. Larger intervals = faster but less accurate.

### Orientation Offset: π/2 radians

**Location:** `simulator.py` lines 299, 418

```python
dspace_orientation = obj.heading - math.pi / 2
```

**Purpose:** Account for coordinate system orientation difference.

**Fixed:** This is a constant that should not be modified unless coordinate systems change.

---

## Coordinate System Differences

### World Coordinate Systems

| Feature | Scenic/Domain | XODR | RD (dSPACE) |
|---------|--------------|------|-------------|
| **Based on** | OpenDRIVE | OpenDRIVE standard | dSPACE native |
| **Origin** | Scenario-dependent | `(x, y)` from file | `(x, y)` from file |
| **Units** | Meters | Meters | Meters |
| **Used by** | Scenic scenarios | Road network parsing | Aurelion/dSPACE |

**Key Insight:** XODR and RD represent the same physical track but in different coordinate systems. The automatic transformation aligns them.

### Orientation Systems

| Feature | Scenic | dSPACE | Conversion |
|---------|--------|--------|------------|
| **Zero direction** | North (+Y) | East (+X) | Subtract π/2 |
| **Rotation** | CCW | CCW | Same |
| **Range** | [0, 2π) | [0, 2π) | Additive offset |

**Mathematical Relationship:**
```
North in Scenic = 90° in dSPACE
West in Scenic  = 180° in dSPACE
South in Scenic = 270° in dSPACE
East in Scenic  = 0° in dSPACE
```

### Road Coordinate System (s,t)

**s-coordinate (Longitudinal):**
- Distance along reference line from start of road
- Range: `0` to `road_length`
- Independent per road segment
- Measured in meters

**t-coordinate (Lateral):**
- Signed distance perpendicular to reference line
- **Positive:** Left of reference line
- **Negative:** Right of reference line
- Scaled by 0.3× factor
- Measured in meters

**Reference Line:** Centerline or typical driving line of road segment.

---

## Troubleshooting

### Vehicles Appear in Wrong Location

**Checklist:**

1. **Verify coordinate transformation**
   ```
   Look for log: "Scenic coords (...) -> RD coords (...)"
   Check if transform exists: self._coordinate_transform
   ```

2. **Check s-coordinate wrapping**
   ```
   Look for: "World coordinates (...) -> Road coordinates (s=..., t=...)"
   Verify s is within expected range [0, road_length]
   ```

3. **Verify lateral scaling**
   ```
   If vehicle laterally offset: check t-coordinate scale factor (0.3)
   Raw t values should be ±0.5 to ±1.5 for typical lanes
   ```

### Positioning Errors up to 34 meters

**Cause:** Using XODR-only mode without RD coordinate transformation.

**Solution:**
1. Ensure `Laguna_Seca.rd` file exists next to `LagunaSeca.xodr`
2. Check logs for: `"⚠️  No RD file found - coordinate mismatches possible"`
3. Transform should build automatically on first run

### Vehicles Clustering at Same Position

**Cause:** Cumulative s-coordinates causing overlaps.

**Solution:**
- Ensure independent s-coordinates per road
- Check `build_xodr_sec_points` and `build_rd_road_index` create separate roads
- Verify each road starts at s=0

### Orientation Misalignment

**Cause:** Incorrect orientation conversion.

**Symptoms:** Vehicles facing wrong direction

**Solution:**
- Check: `"Set orientation: ... degrees (from Scenic heading ...)"`
- Verify formula: `dspace_orientation = heading - π/2`
- Ensure heading is in radians, not degrees

### High Validation Errors in Transform

**Cause:** XODR and RD geometries don't align well.

**Check Logs:**
```
Mean error: X.XXm
Max error: X.XXm
```

**Thresholds:**
- `< 2m`: ✅ Good transform
- `2-5m`: ⚠️  Moderate errors
- `> 5m`: ❌ High errors

**Solutions:**
1. Verify XODR and RD represent same track
2. Check for file corruption
3. Consider manual calibration points
4. Fall back to XODR-only mode

### Debug Logging

**Enable verbose logging:**

Look for these log messages during setup:

```python
"[Transform] Building automatic XODR→RD coordinate transformation..."
"[Geometry] Using RD geometry for accurate (s,t) projection"
"[Status] ✅ Full coordinate transformation pipeline active"
```

During object creation:

```python
"Scenic coords (...)-> RD coords (...)"
"World coordinates (...) -> Road coordinates (s=..., t=...)"
"Set orientation: ... degrees (from Scenic heading ...)"
```

**Key Files for Debugging:**
- `simulator.py`: Main transformation logic
- `geometry/projection.py`: (s,t) projection algorithm
- `geometry/coordinate_transform.py`: XODR→RD transformation
- `geometry/utils.py`: ModelDesk COM helpers

---

## Summary

The coordinate transformation pipeline ensures accurate vehicle placement in dSPACE by:

1. **Building coordinate transformation** from sample points when both XODR and RD files exist
2. **Transforming coordinates** from XODR to RD coordinate system
3. **Projecting to (s,t)** using geometric distance minimization
4. **Converting orientation** accounting for coordinate system differences
5. **Applying to ModelDesk** via COM automation with appropriate segment configuration

**Critical calibration factors:**
- t-coordinate scale: 0.3× (lateral deviation)
- Independent s-coordinates per road (prevent clustering)
- Orientation offset: -π/2 (Scenic → dSPACE)
- Sampling intervals: 2.0m (XODR), 0.5m (RD)

**Success indicators:**
- Mean transform error < 2m
- Vehicles placed accurately on track
- Correct orientation alignment
- No coordinate clustering

**Related Documents:**
- `DSPACE_SIMULATOR_STRUCTURE.md`: Overall simulator architecture
- `DSPACE_CONTROL_INTERFACES.md`: COM API details
- `SCENIC_DOMAIN_ARCHITECTURE_COMPLETE_GUIDE.md`: Domain architecture

