# TTL Coordinate System and Offsets

## Overview

The TTL (Target Trajectory Line) CSV files are stored in **ENU (East-North-Up) coordinate system** with GPS origin at:
- Latitude: 36.5869133
- Longitude: -121.7559026
- Altitude: 231.9349051

## Coordinate Transformation

To align TTLs with the track map (XODR file), a coordinate offset must be applied:
- **Default X offset (dx)**: -53.6 meters
- **Default Y offset (dy)**: -15.7 meters

These offsets transform the ENU coordinates to match the track coordinate system used in the simulator.

## How TTLs Are Calculated

TTLs are **optimized racing lines** calculated to minimize lap time. They are NOT simple centerlines - they:

1. **Use the full track width** - Cutting corners, taking late apexes, using track-out points
2. **Optimize for speed** - Minimize time through corners by maximizing cornering speed
3. **Account for vehicle dynamics** - Consider acceleration, braking, and cornering limits

This is why TTLs may appear to go outside the track boundaries in some places - they're optimized racing lines that use the full width of the track, including:
- **Track-in points**: Entering corners wide
- **Apex points**: Hitting the inside of corners
- **Track-out points**: Exiting corners wide, using all available track width

## Visualization

When visualizing TTLs:
- **Always apply the coordinate offsets** (dx=-53.6, dy=-15.7) to see them aligned with the track
- The comparison tool (`compare_ttls.py`) now applies these offsets by default
- If TTLs still appear misaligned, the offsets may need adjustment for your specific track/map

## Adjusting Offsets

If TTLs don't align properly with the track:
1. Use the visualization tool to test different offsets
2. Adjust `--dx` and `--dy` parameters
3. The offsets are applied in the TTL loader: `x = raw_x + dx`, `y = raw_y + dy`

