# Direct Scenic XODR → ControlDesk RD Calibration

## Objective

**Build a direct transformation between Scenic XODR coordinates and dSPACE ControlDesk RD coordinates.**

### The Problem

The current transformation pipeline goes through multiple steps:
```
Scenic XODR → RD (transform) → (s,t) projection → ModelDesk → ControlDesk RD
```

Each step introduces errors:
- XODR → RD transform (affine transformation from geometry comparison)
- RD → (s,t) projection (route-specific offsets, transition points)
- (s,t) → ModelDesk → ControlDesk RD (dSPACE internal conversion)

**Goal**: Find a direct mapping that bypasses intermediate steps:
```
Scenic XODR → ControlDesk RD (direct transformation)
```

### The Solution

Since ModelDesk requires (s,t) coordinates for vehicle placement, we cannot completely bypass the Frenet frame. However, we can:

1. **Use (s,t) as a placement tool** (necessary step)
2. **Measure actual ControlDesk RD outputs** (what we care about)
3. **Build a direct transform** that accounts for all intermediate errors

The resulting transform will be:
```python
def scenic_to_controldesk_direct(scenic_xodr):
    """
    Direct transformation: Scenic XODR → ControlDesk RD
    This transform accounts for:
    - XODR → RD coordinate transform
    - (s,t) projection errors
    - Route-specific offsets
    - dSPACE model behavior
    
    Returns: (rd_x, rd_y, rd_z) that ControlDesk will actually output
    """
    return controldesk_rd
```

## Calibration Process

### Step 1: Measure Actual ControlDesk Outputs

For each known Scenic XODR coordinate:
1. Transform XODR → RD (using existing transform)
2. Project RD → (s,t) using route-specific projection
3. Place vehicle in ModelDesk using (s,t)
4. Read actual ControlDesk RD output
5. Record: `(scenic_xodr_x, scenic_xodr_y, scenic_xodr_z) → (controldesk_rd_x, controldesk_rd_y, controldesk_rd_z)`

### Step 2: Analyze Patterns

Determine if the offset is:
- **Constant**: Simple additive correction
- **Position-dependent**: Needs mapping/interpolation
- **Route-dependent**: Separate transforms for R1/R2

### Step 3: Build Direct Transform

Create a transformation function based on the calibration data:
- If constant offset: `controldesk_rd = transform(scenic_xodr) + offset`
- If position-dependent: Use lookup table or fitted function
- If route-dependent: Separate transforms per route

## Files in This Folder

### `calibrate_scenic_to_controldesk.py`

**Purpose**: Measure and store Scenic XODR → ControlDesk RD mappings.

**What it does**:
1. Takes known Scenic XODR coordinates (from `fellow_fixed_placing.scenic`)
2. Uses (s,t) pipeline to place vehicles in ModelDesk
3. Reads actual ControlDesk RD outputs
4. Stores calibration data: `(scenic_xodr) → (controldesk_rd_actual)`
5. Analyzes offset patterns
6. Saves results to JSON file

**Output Files**:
- `calibration_data.json`: Raw measurement data
- `calibration_analysis.json`: Analysis results (offset patterns, statistics)
- `calibration_summary.txt`: Human-readable summary

### `build_direct_transform.py` (Future)

**Purpose**: Build direct transform function from calibration data.

**What it will do**:
1. Load calibration data
2. Analyze offset patterns
3. Fit transformation function (affine, polynomial, or lookup table)
4. Generate transform code
5. Test transform on validation set

## Usage

### Run Calibration

```bash
python debug_calibrate_cord/calibrate_scenic_to_controldesk.py
```

**Requirements**:
- ModelDesk open with project and experiment active
- ControlDesk running
- Coordinate transform file: `assets/maps/dSPACE/Laguna_Seca_transform.json`
- RD file: `assets/maps/dSPACE/Laguna_Seca.rd`

### Expected Output

1. **calibration_data.json**: Contains all measured mappings
2. **calibration_analysis.json**: Statistical analysis of offsets
3. **calibration_summary.txt**: Human-readable summary

## Key Questions to Answer

1. **Is the offset constant or position-dependent?**
   - If constant: Simple additive correction
   - If varying: Needs mapping/interpolation

2. **Is the offset route-dependent?**
   - Test same Scenic position on R1 vs R2
   - May need separate transforms per route

3. **How to handle Z coordinate?**
   - Current: Large Z errors suggest it's not transformed correctly
   - Options: Ignore Z, transform separately, or include in 3D transform

4. **How many calibration points needed?**
   - Start with 10 coordinates from `fellow_fixed_placing.scenic`
   - Add more if pattern is unclear

## Next Steps

1. ✅ **Calibration Phase** (this folder): Measure actual ControlDesk outputs
2. **Analysis Phase**: Determine offset patterns
3. **Transform Building**: Create direct transformation function
4. **Validation**: Test transform on new coordinates

## Related Documentation

- **Coordinate Transformation**: `debug_cord_code/README.md`
- **Route Assignment**: `debug_route_code/README.md`
- **T-Coordinate Issues**: `debug_ego_cord/README.md`

