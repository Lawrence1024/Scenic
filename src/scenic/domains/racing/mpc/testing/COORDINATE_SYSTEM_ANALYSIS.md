# Coordinate System Analysis: OpenDRIVE (.xodr) vs RD (.rd) Files

## Key Finding from Documentation

According to `temp.md`, for **Laguna_Seca.rd** and **LagunaSeca.xodr**:
- **The map/world frame is the same** (no extra translation/rotation/scale)
- When sampling the **same road** from both files (e.g., "The Corkscrew1", "Pit Lane1_2", "Andretti Hairpin1_3"), the **reference-line points match up to floating-point noise (~1e-13 m)**

This means the coordinate systems are **identical at the reference line level**.

## How RD Files Work

### Segment Structure
Each road in the `.rd` file is broken into **Segments**. For a `Spline` segment:

1. **AbsoluteStartPosition** gives the segment's origin in world coordinates:
   - `(X0, Y0)` - world position
   - `Tangent` - heading in **degrees**

2. **Local Coordinate Frame** at segment start:
   - Local **+x** points forward along `Tangent`
   - Local **+y** is to the **left** of the heading (left-handed lateral offset convention)

3. **Cubic Polynomial Coefficients** (A, B, C, D are **2D vectors**, not scalars):
   - `t = s / Length`, where `s` is distance along segment, `t ∈ [0, 1]`
   - Local point: `p_local(t) = A + B*t + C*t² + D*t³` (each term is a 2D vector)

4. **World Transform**:
   - `p_world = [X0, Y0] + R(theta) * p_local`
   - where `theta = radians(Tangent)`
   - `R(theta)` is the standard 2D rotation matrix

### Implementation in Code
See `rd_parser.py` lines 66-68:
```python
def to_world(px, py):
    """Transform local coordinates to world coordinates."""
    return (x0 + px*cos0 - py*sin0, y0 + px*sin0 + py*cos0)
```

This matches the documented transformation exactly.

## How OpenDRIVE Files Work

### Reference Line vs Centerline

**Critical distinction:**
- The `planView` defines the **reference line** (not necessarily the centerline)
- Lanes are offsets from the reference line
- For Laguna Seca, the reference line sits on the **left boundary** of the road cross-section
- Total width to the right ≈ **14 m**
- The **midline of the pavement** would be an offset of about **−7 m** (to the right) from the reference line

### Implementation in Code

The XODR parser (`xodr_parser.py`) has an `apply_lane_offset` parameter:
- `apply_lane_offset=False`: Uses the **reference line** (matches RD reference line)
- `apply_lane_offset=True`: Shifts to **centerline** by applying lane offset (~-7m for Laguna Seca)

## Current Code Issue

### Problem: Mismatch in Coordinate Transform Building

In `coordinate_transform.py` line 54:
```python
xodr_index = build_xodr_sec_points(xodr_path)
```

This calls `build_xodr_sec_points` with **default** `apply_lane_offset=True`, which means:
- XODR points are shifted to **centerline** (~-7m offset)
- RD points are on the **reference line** (no offset)
- The transform builder compares **centerline** (XODR) vs **reference line** (RD)
- This creates an artificial ~7m offset that shouldn't exist

### Expected Behavior

Since the reference lines match (per documentation), the coordinate transform should:
1. Compare **reference lines** from both files (set `apply_lane_offset=False`)
2. Detect that no transformation is needed (or only a tiny identity transform)
3. The transform should be effectively **identity** (or near-identity with < 1e-10 offset)

### Solution

When building the coordinate transform, use:
```python
xodr_index = build_xodr_sec_points(xodr_path, apply_lane_offset=False)
```

This ensures we're comparing:
- XODR **reference line** ↔ RD **reference line**

Which should match to floating-point precision.

## Coordinate System Summary

| Aspect | XODR | RD | Relationship |
|--------|------|----|--------------|
| **World Frame** | OpenDRIVE standard | dSPACE native | **Same** (for Laguna Seca) |
| **Reference Line** | `planView` geometry | Spline segments | **Match** (~1e-13 m) |
| **Centerline** | Reference + lane offset | N/A (reference only) | Offset by ~-7m (right) |
| **Units** | Meters | Meters | Same |
| **Orientation** | Standard 2D | Standard 2D | Same |

## Usage in Codebase

### When to Use Reference Line vs Centerline

1. **Coordinate Transform Building** (`coordinate_transform.py`):
   - Should use **reference lines** (`apply_lane_offset=False`)
   - Compares XODR reference ↔ RD reference

2. **Vehicle Placement** (`modeldesk/placement.py`):
   - Uses Scenic positions (which may be on centerline or reference line depending on scenario)
   - Applies coordinate transform if needed
   - Projects to (s, t) coordinates

3. **TTL Waypoint Loading** (`ttl/loader.py`):
   - May need inverse transform to convert RD coordinates back to XODR
   - Should use same coordinate system as vehicle positions

## Recommendations

1. **Fix coordinate transform building** to use `apply_lane_offset=False` when comparing geometries
2. **Add identity transform detection**: If offset is < 1e-10, use identity transform
3. **Document clearly** which coordinate system (reference vs centerline) is used in each context
4. **Verify** that the transform is indeed near-identity for Laguna Seca files

## Testing

To verify the coordinate systems match:
```python
from scenic.simulators.dspace.geometry.rd_parser import parse_rd_geometry
from scenic.simulators.dspace.geometry.xodr_parser import build_xodr_sec_points

# Parse both with reference lines only
rd_roads = parse_rd_geometry("Laguna_Seca.rd", step=0.5)
xodr_index = build_xodr_sec_points("LagunaSeca.xodr", apply_lane_offset=False)

# Compare points at same s-coordinates
# Should match to ~1e-13 m per documentation
```

