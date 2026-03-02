# Resources for Creating Racing Lines for Laguna Seca (XODR)

## Overview
This document summarizes resources and methods for generating a raceable path (racing line) from your Laguna Seca XODR centerline data for MPC testing.

## Key Research Resources

### 1. Curvature-Integrated MPCC (CiMPCC)
**Paper:** "Reduce Lap Time for Autonomous Racing with Curvature-Integrated MPCC Local Trajectory Planning Method"
- **Link:** https://arxiv.org/html/2502.03695v1
- **GitHub:** https://github.com/zhouhengli/CiMPCC
- **Key Features:**
  - Maps track centerline curvature to reference velocity profiles
  - Integrates curvature directly into MPC cost function
  - Achieved 11.4%-12.5% lap time improvements on challenging tracks
  - **Directly applicable to your use case**

### 2. Hybrid MPC & Spline-Based Trajectory Planning
**Repository:** ITSC2023 Hybrid MPC & Spline-based Lane Change Controller
- **GitHub:** https://github.com/navil2000/ITSC2023-MPC-and-Splines
- **Key Features:**
  - Combines MPC with spline-based trajectory generation
  - Includes test models for lateral and longitudinal control
  - Spline generation utilities (`spline_generation.py`)
  - Complete framework for trajectory planning

### 3. Real-Time Autonomous Vehicle Navigation via Rule-Based Waypoint Selection
**Paper:** "Real-Time Autonomous Vehicle Navigation via Rule-Based Waypoint Selection and Spline-Guided MPC"
- **Key Features:**
  - Localized quintic splines for trajectory generation
  - Speed profile optimization integrated with MPC
  - Real-time obstacle avoidance and lane boundary constraints
  - 30% reduction in lateral jerk vs. Bézier methods
  - 25% faster computation

## Recommended Approach for Your Laguna Seca Track

### Method 1: Curvature-Based Racing Line Generation (Recommended)
Based on the CiMPCC approach, you can generate a racing line by:

1. **Extract centerline from XODR**
   - Use your existing `ttl_fellow_test_xodr_all.csv` (3591 points)
   - Already in XODR coordinates ✓

2. **Compute curvature at each point**
   - Use your existing `ReferenceBuilder` class which already computes curvature
   - Formula: `κ = |d²r/ds²|` where `r` is position and `s` is arc-length

3. **Generate racing line by offsetting from centerline**
   - For left turns: offset to the right (outside) before turn, cut inside at apex
   - For right turns: offset to the left (outside) before turn, cut inside at apex
   - Maximum offset should respect track width (5-10m as you identified)

4. **Apply curvature-based speed gating**
   - Use formula: `v_max(s) = sqrt(a_y_max / (|κ(s)| + ε))`
   - This is already implemented in your MPC! ✓

### Method 2: Spline-Based Path Optimization
Using spline interpolation:

1. **Fit splines to centerline**
   - Use your existing spline fitting in `ReferenceBuilder`
   - Arc-length parameterization (already implemented) ✓

2. **Optimize waypoints for racing line**
   - For each centerline point, compute optimal lateral offset
   - Constrain to track boundaries (left/right lane boundaries from XODR)
   - Smooth transitions using splines

3. **Generate velocity profile**
   - Based on curvature (already in your MPC)
   - Respect maximum lateral acceleration (8.0 m/s² in your config)

## Implementation Strategy

### Option A: Generate from Existing Centerline (Quick Start)
Since you already have:
- Centerline: `ttl_fellow_test_xodr_all.csv` (XODR coordinates)
- Spline fitting and curvature computation in `ReferenceBuilder`
- Curvature-based speed gating in MPC

**You can create a simple racing line generator:**

```python
# Pseudo-code for racing line generation
def generate_racing_line(centerline, curvature, track_width=10.0):
    racing_line = []
    for i, (point, kappa) in enumerate(zip(centerline, curvature)):
        # Determine optimal offset based on curvature
        if abs(kappa) > threshold:  # Corner
            # Offset to outside before turn, inside at apex
            offset = compute_optimal_offset(kappa, track_width)
        else:  # Straight
            offset = 0.0
        
        # Compute perpendicular offset
        normal = compute_normal_vector(centerline, i)
        racing_point = point + offset * normal
        racing_line.append(racing_point)
    
    return racing_line
```

### Option B: Use CiMPCC Framework (More Sophisticated)
1. Clone the CiMPCC repository: `https://github.com/zhouhengli/CiMPCC`
2. Adapt their racing line generation code to your XODR format
3. Integrate with your existing MPC controller

### Option C: Manual Racing Line Creation
1. Use track analysis tools to identify:
   - Corner entry/exit points
   - Apex locations
   - Optimal braking/acceleration zones
2. Manually create waypoints that follow racing line principles
3. Smooth using splines (your `ReferenceBuilder` can do this)

## Tools in Your Codebase

You already have several useful tools:

1. **`ReferenceBuilder`** (`src/scenic/domains/racing/mpc/reference_builder.py`)
   - Spline fitting ✓
   - Arc-length parameterization ✓
   - Curvature computation ✓
   - Can be extended to generate racing lines

2. **XODR Parser** (`src/scenic/formats/opendrive/xodr_parser.py`)
   - Can extract centerline and lane boundaries
   - Useful for track width constraints

3. **Racing Line Class** (`src/scenic/domains/racing/tracks.py`)
   - `RacingLine` class already defined
   - Supports speed profiles

## Addressing Your Current Issues

Based on your analysis showing:
- Mean deviation: 5.72m (good)
- Max deviation: 15.97m (problematic)
- 17.4% of points exceed 10m

**Recommendations:**
1. **Fix coordinate system issues first** - The large deviations suggest transformation problems
2. **Generate new racing line** using one of the methods above
3. **Validate deviations** - Ensure all points stay within 5-10m of centerline
4. **Use curvature-based constraints** - Your MPC already has this capability

## Next Steps

1. **Short-term:** Create a simple racing line generator using your existing `ReferenceBuilder` and curvature data
2. **Medium-term:** Explore the CiMPCC repository and adapt their methods
3. **Long-term:** Implement full optimization-based racing line generation

## Additional Resources

- **OpenDRIVE Specification:** https://www.asam.net/standards/detail/opendrive/
- **Racing Line Theory:** "The Racing Line" by Carroll Smith
- **MPC for Racing:** Various papers on Model Predictive Control for autonomous racing
