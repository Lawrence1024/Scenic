# Coordinate System Alignment for Waypoint Following

## Problem Statement

The coordinate transformation pipeline must be consistent across:
1. **Vehicle Placement**: Scenic XODR → RD (via affine transform) → ModelDesk (s,t)
2. **Waypoint Following**: Waypoints must be in the same coordinate system as vehicle positions

Currently, there's a **mismatch**:
- Vehicle positions use: `apply_coordinate_transform(transform, (x, y))` (affine transform)
- TTL waypoints use: `x + dx, y + dy` (simple offset)

This inconsistency will cause waypoint following to fail because waypoints and vehicle positions are in different coordinate systems.

## Current State

### Vehicle Placement Pipeline
```
Scenic XODR (x, y) 
  → apply_coordinate_transform(transform, (x, y))
  → RD (x', y')
  → project_world_to_st(road_index, (x', y'))
  → ModelDesk (s, t)
```

### Waypoint Loading Pipeline
```
TTL CSV (x, y) [assumed to be in RD/ENU]
  → x + dx, y + dy [simple offset]
  → Stored as waypoints in Scenic [assumed XODR?]
  → Used in behavior: self.position.x, self.position.y [XODR]
```

## The Issue

1. **TTL files** are likely in RD/ENU coordinates (from dSPACE)
2. **Simple offset (dx, dy)** is not the same as the affine transformation
3. **Waypoint following behavior** uses `self.position.x/y` which are in XODR coordinates
4. **Waypoints** need to be in XODR coordinates to match vehicle positions

## Solution Options

### Option 1: Transform Waypoints Using Affine Transform (Recommended)

Modify `load_ttl_region()` to use the same coordinate transformation:

```python
def load_ttl_region(ttl_folder, ttl_index, dx, dy, ttl_file_name=None, 
                   coordinate_transform=None):
    """Load TTL CSV and transform to XODR coordinates.
    
    If coordinate_transform is provided, use it to convert RD → XODR.
    Otherwise, use simple offset (backward compatibility).
    """
    # ... load points from CSV ...
    
    if coordinate_transform:
        from scenic.simulators.dspace.geometry.coordinate_transform import apply_inverse_coordinate_transform
        # TTL points are in RD, transform to XODR
        xodr_points = []
        for x, y in pts:
            xodr_x, xodr_y = apply_inverse_coordinate_transform(
                coordinate_transform, (x, y)
            )
            xodr_points.append((xodr_x, xodr_y))
        pts = xodr_points
    else:
        # Legacy: simple offset
        pts = [(x + dx, y + dy) for x, y in pts]
    
    return PolylineRegion(pts), pts
```

### Option 2: Ensure TTL Files Are Already in XODR

If TTL files are already in XODR coordinates (or can be pre-transformed), then:
- Remove the offset transformation
- Use waypoints directly
- Ensure consistency with vehicle placement

### Option 3: Transform Vehicle Positions to Match Waypoints

If waypoints must stay in RD coordinates:
- Transform vehicle positions to RD before waypoint following
- This is less ideal as it requires coordinate system switching

## Recommended Approach

**Use Option 1**: Transform waypoints using the same affine transform as vehicle placement.

This ensures:
- ✅ Waypoints and vehicle positions are in the same coordinate system (XODR)
- ✅ Consistent transformation pipeline
- ✅ Waypoint following will work correctly
- ✅ Backward compatible (if transform not provided, use offset)

## Implementation Notes

1. The coordinate transform is available in `DSpaceSimulation._coordinate_transform`
2. Pass it to `attach_ttl()` and `load_ttl_region()`
3. Use `apply_inverse_coordinate_transform()` to convert RD → XODR (same as vehicle placement uses forward transform)

## Testing

After implementing, verify:
1. Vehicle position in XODR matches waypoint coordinate system
2. Waypoint following behavior works correctly
3. No coordinate system mismatches in logs

