# TTL Coordinate System Integration

## Overview

The TTL loading code has been updated to automatically detect and handle coordinate system transformations based on the folder containing the TTL files.

## Key Changes

### Automatic Coordinate System Detection

The system now automatically detects if TTL files are in the `transformed` folder:

- **`transformed` folder**: Files are already in XODR coordinates
  - Offset automatically set to `(0, 0)`
  - No transformation needed
  - Waypoints can be used directly with Scenic vehicle positions

- **Other folders** (e.g., `usable`, `raw`): Files are in ENU/RD coordinates
  - Default offset: `(-53.6, -15.7)`
  - Offset is applied during loading

### Updated Functions

#### `get_ttl_config(scene_params)`
- Automatically detects `transformed` folder in path
- Sets default offset to `(0, 0)` for transformed files
- Sets default offset to `(-53.6, -15.7)` for other files
- Warns if explicit offset is set for transformed files

#### `load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file_name=None)`
- Enhanced logging shows coordinate system and offset used
- Clear documentation of coordinate system handling

#### `attach_ttl(sim, obj, vehicle_type="vehicle")`
- Auto-detects offset based on folder path
- Respects explicit overrides via object properties or scene params
- Updated default folder to `transformed`

## Usage

### Default (Recommended)
```python
# Uses transformed folder with auto-detected offset (0, 0)
ego = new RacingCar at (72.567889, 107.574718, 0.0)
# TTL will be loaded from transformed folder with zero offset
```

### Explicit Override
```python
# Explicitly specify folder and offset
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV/transformed')
param ttlDX = 0.0
param ttlDY = 0.0
```

### Using Non-Transformed Files
```python
# Use files from other folders (will auto-detect offset)
param ttlFolder = localPath('../../assets/ttls/LS_ENU_TTL_CSV/usable')
# Offset will be auto-set to (-53.6, -15.7)
```

## Verification

The system logs coordinate system information:
```
[TTL] Loaded 3600 waypoints from ttl_17.csv (offset: (0.0, 0.0), coordinate system: XODR (already transformed))
```

## Benefits

1. **Automatic**: No manual configuration needed for transformed files
2. **Safe**: Warns if incorrect offset is used
3. **Flexible**: Supports both transformed and non-transformed files
4. **Consistent**: Waypoint coordinates match vehicle position coordinates (both XODR)

## Coordinate System Alignment

✅ **Waypoint Following Now Works Correctly**

- Vehicle positions: XODR coordinates (from Scenic)
- TTL waypoints: XODR coordinates (from transformed folder)
- Both use the same coordinate system → waypoint following works!

## Testing

Tested with:
- `ttl_17.csv` from `transformed` folder
- Offset correctly set to `(0, 0)`
- Waypoints successfully attached to vehicles
- Coordinate system correctly identified as XODR

