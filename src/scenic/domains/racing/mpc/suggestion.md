# MPC Improvement Roadmap - Living Document

**Last Updated:** 2024-12-19  
**Status:** In Progress  
**Current Focus:** Progress-based waypoint advancement

---

## Problem Statement

**Root Cause Identified (from log analysis):**
- MPC selects segments 3-4 waypoint indices behind the vehicle during sharp turns
- This causes wrong CTE perception: Behavior CTE = 11.5m LEFT, but MPC CTE = -0.9m RIGHT
- Delayed response: MPC switches to LEFT steering too late (6 steps after path starts turning)
- Radius-based waypoint advancement allows premature skipping when vehicle is far off-track

**Failure Chain:**
1. Curvature revealed too late (waypoint-by-waypoint instead of continuous)
2. Waypoint skipping masks lateral error (radius-based advancement)
3. Entering corners too fast (no curvature-based speed limiting)
4. Steering can't ramp fast enough (fixed slew limit in corners)

---

## Solution Overview

**Strategy:** Stop driving "waypoint-to-waypoint" at corners. Instead, give MPC a **continuous, curvature-aware reference** plus a simple **speed gate**.

**Why this approach:**
- ✅ Doesn't require a new MPC model
- ✅ Doesn't require retuning everything
- ✅ Fixes the exact failure chain identified
- ✅ Leverages existing spline infrastructure

---

## Implementation Status

### ✅ 1. Spline Fitting and Arc-Length Parameterization
**Status:** COMPLETE  
**Files:** `reference_builder.py`

**What's implemented:**
- `_fit_spline()`: Fits parametric cubic B-spline through waypoints (2D/3D support)
- `_compute_arc_length_parameterization()`: Computes uniform arc-length spacing
- `resample_waypoints()`: Resamples waypoints using splines with arc-length parameterization
- `build_reference()`: Uses spline-based reference building when `use_splines=True`

**Code References:**
- Lines 143-176: `_fit_spline()` method
- Lines 178-221: `_compute_arc_length_parameterization()` method
- Lines 281-334: `resample_waypoints()` method
- Lines 465-580: Spline-based reference building in `build_reference()`

**Notes:**
- Spline fitting already supports 3D waypoints
- Arc-length parameterization works for both 2D and 3D
- Caching mechanism exists (`_spline_cache`, `_spline_waypoints`)

---

### ⚠️ 2. Spline Projection and Progress Tracking
**Status:** PARTIALLY IMPLEMENTED  
**Priority:** HIGH (blocks progress-based advancement)

**What's needed:**
- `project_to_spline()`: Project vehicle position onto spline to get `s_0` (arc-length)
- Maintain `s_0` as primary progress metric instead of waypoint index
- Update `find_nearest_waypoint()` to use arc-length progress

**Current State:**
- `find_nearest_waypoint()` uses distance-based search with adaptive window
- No direct spline projection method exists
- Waypoint advancement still uses radius-based logic

**Implementation Plan:**
1. Add `project_to_spline(position, tck, u_param)` method to `ReferenceBuilder`
   - Use iterative projection (Newton-Raphson or bisection)
   - Return `(s_0, u_0, nearest_segment_idx)`
2. Modify `find_nearest_waypoint()` to optionally use spline projection
3. Add progress tracking state: `_current_s_0` in `ReferenceBuilder`

**Files to Modify:**
- `reference_builder.py`: Add projection method
- `behaviors.scenic`: Update waypoint advancement logic (lines 608-664)

**Estimated Effort:** 2-3 days

---

### ❌ 3. Progress-Based Waypoint Advancement
**Status:** NOT STARTED  
**Priority:** CRITICAL (fixes segment selection issue)

**Current Implementation:**
- Radius-based: Advances when `distance < HIT_THRESHOLD` (3-12m, dynamic)
- Uses "waypoint behind vehicle" detection via dot product
- Pass-through detection for large timesteps

**Target Implementation:**
- Progress-based: Advance based on arc-length progress `s_0`
- Current index = spline segment containing `s_0`
- No skipping ahead when laterally far off-track

**Implementation Plan:**
1. Replace radius-based logic in `behaviors.scenic` (lines 608-664)
2. Use `s_0` from spline projection as primary progress metric
3. Map `s_0` to waypoint index for backward compatibility
4. Remove "within radius" advancement logic

**Code Location:**
- `behaviors.scenic`: Lines 608-664 (waypoint advancement loop)

**Estimated Effort:** 2-3 days

**Dependencies:**
- Requires #2 (Spline Projection) to be complete

---

### ❌ 4. Curvature-Adaptive Sampling Density
**Status:** NOT STARTED  
**Priority:** MEDIUM (improves reference quality)

**Current Implementation:**
- Fixed `resample_dist = 0.2m` in `ReferenceBuilder.__init__`
- Uniform spacing regardless of curvature

**Target Implementation:**
- Adaptive spacing based on curvature:
  - `|κ| < 0.01` (R > 100m): sample every 1.0-2.0m
  - `|κ| ≥ 0.01`: sample every 0.25-0.5m

**Implementation Plan:**
1. Modify `resample_waypoints()` to compute curvature first
2. Use adaptive `resample_dist` based on local curvature
3. Ensure smooth transitions between regions

**Code Location:**
- `reference_builder.py`: `resample_waypoints()` method (lines 281-334)

**Estimated Effort:** 1 day

**Dependencies:**
- Requires curvature computation (already exists in `build_reference()`)

---

### ❌ 5. Curvature-Based Speed Gate
**Status:** NOT STARTED  
**Priority:** HIGH (safety critical)

**Formula:**
```
v_max(s) = sqrt(a_y_max / (|κ(s)| + ε))
v_ref = min(v_desired, min_{s∈[s_0, s_0+L]} v_max(s))
```

**Parameters:**
- `a_y_max`: Maximum lateral acceleration
  - Conservative (indoor sim): 6-8 m/s²
  - Race grip: 8-12 m/s²
- `ε`: Small epsilon to avoid division by zero (e.g., 0.001)

**Implementation Plan:**
1. Add curvature-based speed limiting in `behaviors.scenic`
2. Compute `v_max` profile over MPC horizon using curvature from reference builder
3. Apply speed gate before building `v_ref_profile` for longitudinal MPC
4. Add config parameter for `a_y_max`

**Code Location:**
- `behaviors.scenic`: Speed profile building section (around line 1000-1100)
- `config.py`: Add `max_lateral_acceleration` parameter
- `vehicle_mpc.yaml`: Add `max_lateral_acceleration` config value

**Estimated Effort:** 1 day

**Dependencies:**
- Requires curvature profile from reference builder (already available)

---

### ❌ 6. Curvature-Aware Steering Slew Limit
**Status:** NOT STARTED  
**Priority:** LOW (optional optimization)

**Current Implementation:**
- Fixed slew limit in `behaviors.scenic`
- Applied uniformly regardless of path curvature

**Target Implementation:**
- On straights: normal slew limit
- In corners (`|κ|` above threshold): allow 2× faster steering rate

**Implementation Plan:**
1. Modify steering slew limit calculation in `behaviors.scenic`
2. Scale slew rate based on current curvature
3. Add config parameter for curvature threshold

**Code Location:**
- `behaviors.scenic`: Steering slew limit section
- `config.py`: Add `curvature_slew_threshold` parameter

**Estimated Effort:** 0.5 days

**Dependencies:**
- Requires curvature information (already available from reference builder)

---

## Implementation Checklist

### Phase 1: Foundation (Critical Path)
- [ ] **2. Spline Projection** - Add `project_to_spline()` method
- [ ] **3. Progress-Based Advancement** - Replace radius-based logic
- [ ] **5. Curvature Speed Gate** - Add speed limiting

### Phase 2: Quality Improvements
- [ ] **4. Curvature-Adaptive Sampling** - Adaptive resampling density
- [ ] **6. Curvature-Aware Slew Limit** - Dynamic steering slew rate

---

## Testing & Validation

### Test Scenarios
- [ ] Sharp left turn (90°+) at moderate speed (10-12 m/s)
- [ ] Sharp right turn (90°+) at moderate speed
- [ ] S-curve transitions
- [ ] High-speed straight sections
- [ ] Off-track recovery scenarios

### Success Criteria
- [ ] MPC selects segments ahead of vehicle (not behind)
- [ ] MPC CTE matches Behavior CTE (within 0.5m tolerance)
- [ ] Vehicle enters turns at appropriate speed (curvature-limited)
- [ ] No premature waypoint advancement when far off-track
- [ ] Smooth steering transitions in corners

### Log Analysis Checklist
- [ ] Verify `seg_idx` is ahead of vehicle waypoint index
- [ ] Verify MPC CTE sign matches Behavior CTE sign
- [ ] Verify speed reduction before sharp turns
- [ ] Verify steering response time (steps to react to turn)

---

## Issues & Lessons Learned

### Issue #1: Segment Selection Behind Vehicle
**Date:** 2024-12-19  
**Status:** Identified, not fixed  
**Description:** MPC consistently selects segments 3-4 waypoints behind vehicle during sharp turns  
**Root Cause:** Radius-based waypoint advancement + distance-based segment selection  
**Solution:** Progress-based advancement (#3) + spline projection (#2)

### Issue #2: Wrong CTE Perception
**Date:** 2024-12-19  
**Status:** Identified, not fixed  
**Description:** Behavior CTE = 11.5m LEFT, but MPC CTE = -0.9m RIGHT  
**Root Cause:** MPC using wrong segment (behind vehicle) for CTE calculation  
**Solution:** Progress-based advancement will fix segment selection

### Issue #3: Delayed Turn Response
**Date:** 2024-12-19  
**Status:** Identified, not fixed  
**Description:** MPC switches to LEFT steering 6 steps after path starts turning left  
**Root Cause:** Curvature revealed too late (waypoint-by-waypoint)  
**Solution:** Continuous curvature from spline (#1, already implemented) + progress-based advancement

---

## Code References

### Key Files
- `reference_builder.py`: Spline fitting, arc-length parameterization, reference building
- `mpc_lateral.py`: Lateral MPC controller, segment selection logic (lines 560-820)
- `behaviors.scenic`: Waypoint advancement, speed control, steering logic
- `config.py`: MPC configuration parameters
- `vehicle_mpc.yaml`: MPC parameter values

### Critical Code Sections
- **Waypoint Advancement:** `behaviors.scenic` lines 608-664
- **Segment Selection:** `mpc_lateral.py` lines 560-627
- **Spline Fitting:** `reference_builder.py` lines 143-176
- **Reference Building:** `reference_builder.py` lines 465-580

---

## Next Steps

1. **Immediate (This Week):**
   - Implement spline projection method (#2)
   - Start progress-based waypoint advancement (#3)

2. **Short Term (Next Week):**
   - Complete progress-based advancement
   - Add curvature-based speed gate (#5)
   - Test on sharp turn scenarios

3. **Medium Term:**
   - Add curvature-adaptive sampling (#4)
   - Add curvature-aware slew limit (#6)
   - Comprehensive testing and tuning

---

## Notes & Observations

- Spline infrastructure is already in place - good foundation
- Main challenge is refactoring waypoint advancement logic
- Need to maintain backward compatibility during transition
- Consider A/B testing: keep old logic as fallback initially

---

## Related Issues

- Unicode encoding error in `controller.py` (line 87) - FIXED (replaced `→` with `->`)
- Segment selection penalty system exists but insufficient when far off-track
- Current `u_proj` penalty (lines 612-620 in `mpc_lateral.py`) helps but doesn't solve root cause

---

## Resources

- Scipy spline documentation: `splprep`, `splev` for spline fitting
- Arc-length parameterization: Standard technique for path following
- Curvature computation: From spline second derivatives (already implemented)
