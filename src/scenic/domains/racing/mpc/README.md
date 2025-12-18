# MPC Controller Module - Living Document

**Last Updated:** 2024-12-19  
**Status:** Initial Implementation

---

## Overview

This module implements a Model Predictive Control (MPC) controller for lateral (steering) control of racing vehicles in the Scenic racing domain. The MPC replaces PID controllers with predictive control for better performance on racing tracks, especially in high-speed cornering scenarios.

---

## Module Structure

```
mpc/
├── __init__.py              # Module exports
├── config.py                # Configuration management (YAML → Python)
├── reference_builder.py     # Waypoint → reference trajectory builder
├── mpc_lateral.py           # Main MPC controller implementation
├── io_adapter.py            # ControlDesk I/O integration
├── utils.py                 # Utility functions (filters, etc.)
├── calibration.py           # Steering scale calibration
├── README.md                # This file
└── testing/                 # Testing infrastructure
    ├── __init__.py
    ├── test_config.py       # Configuration tests
    ├── test_utils.py        # Utility function tests
    ├── test_reference_builder.py  # Reference builder tests
    ├── test_mpc_lateral.py  # MPC controller tests
    ├── run_tests.py         # Test runner script
    └── TEST_CASES.md        # Living document of test cases (guard)
```

---

## Current Status

### ✅ Completed
- [x] Module structure created
- [x] Configuration loader (`config.py`)
- [x] Reference trajectory builder (`reference_builder.py`)
- [x] MPC controller skeleton (`mpc_lateral.py`)
- [x] State computation (e_y, e_psi from waypoints) - **COMPLETED**
- [x] Safety checks (position/yaw error thresholds)
- [x] Utility functions (`utils.py` - low-pass filter)
- [x] I/O adapter skeleton (`io_adapter.py`)
- [x] Calibration utilities skeleton (`calibration.py`)
- [x] **Behavior integration** - `FollowRacingLineMPCBehavior` created ✅
- [x] **Testing infrastructure** - Comprehensive test suite (27 tests) ✅
- [x] **Integration tests** - Behavior, simulation, and scenario compilation tests ✅
- [x] **Simulation integration** - `getRacingControllers()` supports MPC ✅

### 🚧 In Progress
- [ ] ControlDesk integration testing (with real simulator)
- [x] Steering feedback reading (delta from ControlDesk) - **INFRASTRUCTURE COMPLETE** ✅
- [ ] Steering scale calibration implementation (skeleton exists)

### 📋 TODO
- [x] Unit tests for reference builder ✅
- [x] Unit tests for MPC formulation ✅
- [x] Unit tests for configuration ✅
- [x] Unit tests for utilities ✅
- [x] Test infrastructure and living document ✅
- [x] Integration with behaviors - `FollowRacingLineMPCBehavior` created ✅
- [x] Simulation integration - `getRacingControllers()` updated ✅
- [ ] Real-world testing with ControlDesk
- [ ] Performance tuning and weight optimization
- [ ] Integration tests with ControlDesk
- [ ] Documentation and examples

---

## Key Learnings & Decisions

### Configuration Management
- **Decision:** Use YAML config files (compatible with ROS-style parameter format)
- **Location:** `debug_mpc/vehicle_mpc.yaml`
- **Adaptation:** Config adapts to Scenic `timestep` automatically

### ControlDesk Integration
- **Read Paths:** Use existing `read_ego_state()` / `read_fellow_state()` functions
- **Write Path:** Use existing `VehicleController` infrastructure via `_control_state`
- **Steering Range:** ControlDesk expects -70 to +70 (degrees-like units)
- **Normalization:** MPC outputs [-1, 1], converted to ControlDesk range

### Coordinate Frames & Heading (Critical)
This project uses multiple coordinate/angle conventions at once. The most important debugging lesson was:

- **Waypoints in `assets/ttls/.../transformed/*.csv` are in XODR/Scenic coordinates** (already transformed).
- **ControlDesk provides yaw in degrees** via `Angle_Yaw_Vehicle_CoorSys_E[deg]` (see `debug_mpc/vehicle_mpc.yaml` path).
- **Do not blindly apply ±90° or ±180° offsets** to yaw in code. We tested both and they can silently break tracking.

#### Final decision (what worked)
- **Heading used by MPC must come from ControlDesk readback** (`self.dspaceActor.heading`) so the MPC state is consistent with the simulator.
  - In `src/scenic/domains/racing/behaviors.scenic`, the MPC behavior now prefers `dspaceActor.heading` (ControlDesk) over `self.heading`.
- **Yaw conversion in readback is: degrees -> radians -> normalize to [-pi, pi]**
  - In `src/scenic/simulators/dspace/controldesk/readback.py`, we keep yaw as:
    - `yaw_rad_raw = yaw_deg * pi/180`
    - `yaw_rad = atan2(sin(yaw_rad_raw), cos(yaw_rad_raw))`
  - This was validated by logs where `raw_yaw_deg≈236°` normalized to `heading_deg≈-123°`, matching the local track/waypoint direction.

#### Why earlier "flip by 180°" experiments happened
At one point the controller appeared to need a 180° heading flip. That turned out to be a **heading source mismatch**:
- the behavior used `self.heading` (not necessarily the ControlDesk yaw), while the debug prints were inspecting ControlDesk yaw.
- mixing these sources made the MPC fight a fake heading error and "swerve" off track.

### Waypoint Direction & 180° Reference Heading Flip
Even if yaw is correct, waypoints can be "geometrically reversed" relative to travel direction.

We implemented a robust rule in `src/scenic/domains/racing/mpc/mpc_lateral.py::_compute_errors`:
- Compute segment heading from waypoint geometry: `psi_ref_original = atan2(seg_dy, seg_dx)`
- If the segment heading differs from vehicle heading by > 90°, **flip the reference heading by 180°**:
  - `psi_ref = wrap(psi_ref_original + pi)`
- **If we flip reference heading, we must also flip CTE sign** (see next section).

This avoids "drive backwards along the segment" behavior without requiring CSV reversal.

### CTE Sign Conventions (Critical)
Within `_compute_errors` in `mpc_lateral.py`:
- Normal vector is defined as `n = (-dy, dx) / len` (LEFT of the segment direction).
- Raw CTE is `e_y_raw = (p - proj) · n`
- If we flip the reference heading by 180° (meaning "travel opposite direction"), then the notion of LEFT/RIGHT relative to travel direction also flips.

**Final rule:**
- Keep the normal vector based on the geometric segment (do NOT negate `n`).
- If `heading_flipped == True`, then apply:
  - `e_y = -e_y_raw`

This fixes the common bug where the vehicle consistently steers away from the line because CTE left/right is inverted.

### Steering Sign Conventions (Critical)
The MPC solves for a steering command `u0` (front wheel angle command in radians). In practice we observed a sign mismatch between:
- the sign of the QP solution (`u0_raw_rad`), and
- the direction the vehicle actually turns in the dSPACE visualization.

**Final decision (empirical):** Steering sign is **environment-dependent**; validate it with logs.

In the current (consistent-yaw) setup, the solver output is used **without negation**:
- Implemented in `src/scenic/domains/racing/mpc/mpc_lateral.py` (solve path):
  - `delta_cmd_rad = delta_cmd_rad_raw`

Why we document this explicitly:
- It is easy to accidentally "fix" this away when also changing yaw transforms.
- Always validate with logs + visualization: when CTE is RIGHT (negative), steering must turn LEFT (positive), and vice versa.

Practical validation:
- Use `[MPC Error Computation] CTE_raw=...` and `[MPC Actuation DBG] u0_raw_rad/u0_used_rad`.
- If `CTE_used > 0` (LEFT), `u0_used_rad` should steer RIGHT (negative) to reduce CTE (depending on your simulator steering convention).

### Waypoint Index Initialization & Progress
Two independent issues can cause "random swerves":
- tracking a waypoint that is behind the vehicle
- getting stuck on an old waypoint index

We added/used:
- **Initialize waypoint index using a forward dot-product check** (pick the first waypoint ahead of the vehicle).
- **Prefer ControlDesk heading** for that dot-product (see heading section above).
- **Increment waypoint index** using a hit-threshold (default 3m) to prevent oscillating index selection.

These changes live in `src/scenic/domains/racing/behaviors.scenic`.

### Debugging Playbook (What to Log)
When tracking is wrong, print the pipeline values so you can isolate sign/frame issues quickly.

Recommended log tags (added during debugging):
- `[Yaw Readback]`: shows yaw_deg -> yaw_rad -> normalized heading
- `[MPC Errors DBG]`: shows segment geometry, projection, normal vector
- `CTE_raw` vs `CTE_used`: shows whether heading flip changed CTE sign
- `[MPC Actuation DBG]`: shows `u0_raw_rad`, `u0_neg_rad`, normalized, filtered
- `[Steer Slew DBG]`: shows MPC output vs slew-limited command applied by behavior

### Waypoint Format
- **Format:** CSV files with `x,y` pairs (no speed profile initially)
- **Location:** `assets/ttls/LS_ENU_TTL_CSV/transformed/`
- **Coordinate System:** XODR coordinates (matches vehicle positions)

### Vehicle Parameters
From `dspace_iac_car.param.yaml`:
- Wheelbase: `2.9718 m`
- Max steering angle: `0.2816 rad` (≈16.1°)
- Steering time constant: `0.3 s` (from `aw_lat_mpc.param.yaml`)
- Steering rate limit: `1.0 rad/s` (conservative estimate)

---

## Implementation Details

### MPC Formulation

**State:** `x = [e_y, e_psi, delta]`
- `e_y`: Lateral error (meters)
- `e_psi`: Heading error (radians)
- `delta`: Front-wheel angle (radians)

**Control:** `u = delta_cmd` (desired front-wheel angle, radians)

**Dynamics:**
- Steering actuator: `delta_{k+1} = delta_k + (dt/tau) * (u_k - delta_k)`
- Lateral error: `e_y_{k+1} = e_y_k + v_k * e_psi_k * dt`
- Heading error: `e_psi_{k+1} = e_psi_k + (v_k/L) * delta_k * dt - v_k * kappa_ref_k * dt`

**Constraints:**
- Steering limits: `|delta_k| <= DELTA_MAX_RAD`
- Control limits: `|u_k| <= DELTA_MAX_RAD`
- Rate limits: `|delta_{k+1} - delta_k| <= DELTA_DOT_MAX * dt`

**Cost Function:**
- Tracking: `w_ey * e_y^2 + w_epsi * e_psi^2`
- Smoothness: `w_u * u^2 + w_du * (u_k - u_{k-1})^2`
- Terminal: `wT_ey * e_y_N^2 + wT_epsi * e_psi_N^2`

### Reference Builder

**Nearest Waypoint Search:**
- Forward-only search starting from last known index
- Prevents backtracking along path
- Search window: ±50 waypoints from last index

**Curvature Computation:**
- 3-point method: uses waypoints at `[i-1, i, i+1]`
- Formula: `kappa = 2 * cross(v1, v2) / (|v1| * |v2| * avg_length)`

**Reference Generation:**
- Interpolates along waypoint segments for horizon steps
- Computes heading from segment tangent
- Computes curvature using 3-point method

---

## Configuration Parameters

### Timing
- `ctrl_period`: Control update period (seconds) - adapts to Scenic timestep
- `mpc_prediction_horizon`: Number of prediction steps (default: 30)
- `mpc_prediction_dt`: Time step for prediction (seconds)

### Vehicle
- `wheel_base`: Wheelbase (meters)
- `max_steer_angle`: Maximum front-wheel angle (radians)
- `steer_tau`: Steering actuator time constant (seconds)
- `steer_rate_lim`: Steering rate limit (rad/s)
- `steer_cmd_max`: Maximum steering command in ControlDesk units

### MPC Weights
- `w_ey`: Lateral error weight
- `w_epsi`: Heading error weight
- `w_u`: Control input weight
- `w_du`: Control rate weight
- `wT_ey`: Terminal lateral error weight
- `wT_epsi`: Terminal heading error weight

### Safety
- `admissible_position_error`: Maximum position error before disabling MPC (meters)
- `admissible_yaw_error_rad`: Maximum yaw error before disabling MPC (radians)
- `max_invalid_count`: Maximum consecutive solver failures before zeroing steering

### Filtering
- `steering_lpf_cutoff_hz`: Low-pass filter cutoff frequency for steering output (Hz)

### Waypoint
- `traj_resample_dist`: Distance between resampled waypoints (meters)

---

## Integration Points

### With Scenic Behaviors
- **New Behavior:** `FollowRacingLineMPCBehavior` ✅ **CREATED**
  - Located in `src/scenic/domains/racing/behaviors.scenic`
  - Uses MPC for lateral control, PID for longitudinal control
  - Same interface as `FollowRacingLineBehavior` with additional `mpc_config_path` parameter
  - Example usage: `ego.behavior = FollowRacingLineMPCBehavior(target_speed=30)`

### With dSPACE Simulator
- **Read State:** Via `read_state_from_controldesk()` → uses existing `read_ego_state()`
- **Write Commands:** Via `write_steering_to_controldesk()` → uses `VehicleController`

### With Racing Domain
- **Controller Interface:** `MPCLateralController.run_step()` returns normalized steering [-1, 1]
- **Compatibility:** Drop-in replacement for `PIDLateralController` interface

---

## Testing Infrastructure

### Test Suite Location
All tests are located in `mpc/testing/` directory.

### Running Tests

#### Quick Test Run
```bash
cd Scenic/src/scenic/domains/racing/mpc/testing
python run_tests.py
```

#### Using pytest (Recommended)
```bash
cd Scenic/src/scenic/domains/racing/mpc/testing
python -m pytest test_*.py -v
```

#### Run Specific Test File
```bash
python test_reference_builder.py
```

#### Run Specific Test Case
```bash
python -m pytest test_reference_builder.py::TestReferenceBuilder::test_find_nearest_waypoint_simple -v
```

### Test Categories

#### 1. Unit Tests (Implemented ✅)
- **`test_config.py`**: Configuration loading and parameter validation
- **`test_utils.py`**: Low-pass filter and utility functions
- **`test_reference_builder.py`**: Waypoint search, curvature computation, reference generation
- **`test_mpc_lateral.py`**: State computation, error handling, fallback behavior

#### 2. Test Cases Document
- **`TEST_CASES.md`**: Living document serving as a guard/regression test suite
  - Documents all test cases with expected behavior
  - Explains why each test is important
  - Tracks test status (PASS/FAIL/SKIP)
  - Must be updated when adding/modifying tests

### Test Status
- **Total Tests:** 18
- **Passing:** 18 ✅
- **Skipped:** 0
- **Failing:** 0

**All tests passing!** The test suite validates:
- Configuration loading and parameter validation
- Reference trajectory building (waypoint search, curvature computation)
- State computation (lateral and heading errors)
- MPC controller functionality (with OSQP solver)
- Utility functions (low-pass filter)

### Test Maintenance Guidelines

#### Before Committing Changes
1. ✅ Run all tests: `python run_tests.py`
2. ✅ Ensure all tests pass (or are appropriately skipped)
3. ✅ Update `TEST_CASES.md` if adding new tests
4. ✅ Update this README if test infrastructure changes

#### When Tests Fail
1. **Identify root cause:** Which test failed and why?
2. **Fix the issue:** Don't just disable the test
3. **Verify fix:** Re-run all tests
4. **Update status:** Mark test as PASS in `TEST_CASES.md`

#### Adding New Tests
1. Add test case to appropriate test file
2. Document test case in `TEST_CASES.md` with:
   - Purpose
   - Expected behavior
   - Why it's important
   - Status (PASS/FAIL/SKIP)
3. Update test status summary in `TEST_CASES.md`
4. Run all tests to ensure nothing breaks

### Future Test Additions

#### Integration Tests (To Be Added)
- [ ] Test with real ControlDesk connection
- [ ] Test with real waypoint files
- [ ] Test end-to-end behavior integration
- [ ] Test calibration procedure

#### Performance Tests (To Be Added)
- [ ] QP solve time benchmark (< 10ms target)
- [ ] Memory usage profiling
- [ ] Warm-start effectiveness

#### Regression Tests (To Be Added)
- [ ] Test known good scenarios
- [ ] Test edge cases (very slow, very fast, sharp turns)
- [ ] Test error handling (invalid waypoints, solver failures)

### Test Dependencies
- **Required:** `unittest` (standard library)
- **Optional:** `pytest` (for better output formatting)
- **Optional:** `osqp` (for full MPC controller tests)

### Coding Guidelines for Logging
**IMPORTANT:** When printing logs or test output, use **text-only** (ASCII) characters. Do NOT use Unicode symbols (✓, ✗, →, etc.) as they cause encoding errors on Windows consoles.

**Good:**
```python
print("[OK] Test passed")
print("[FAIL] Test failed")
print("[SUCCESS] All tests passing!")
```

**Bad:**
```python
print("✓ Test passed")  # Unicode - will fail on Windows
print("✗ Test failed")  # Unicode - will fail on Windows
```

This guideline applies to all print statements, test output, and logging throughout the MPC module.

### Testing with Scenic Commands
**IMPORTANT:** Do NOT run `scenic` commands directly. Instead, provide the user with the exact command to run. Running scenic commands requires specific setup (ModelDesk/ControlDesk connections, simulation state, etc.) that the user must handle manually.

**Good:**
```markdown
To test the MPC scenario, run:
```powershell
scenic examples/racing/ego_mpc_behavior.scenic --simulate --time 10 --count 1 --model scenic.simulators.dspace.racing_model
```
Note: `--count 1` ensures only one scene is generated and simulated (prevents infinite loop).
```

**Bad:**
- Running `scenic` commands via `run_terminal_cmd` tool
- Assuming simulation environment is ready for automated testing
- Forgetting to add `--count 1` which causes infinite scene generation loop

**Important Notes:**
- Without `--count`, Scenic will generate scenes indefinitely in a loop
- The `--time` parameter specifies simulation duration in seconds (not number of steps)
- For step-by-step mode, the simulation will run until `--time` duration is reached or manually stopped

This guideline applies to all testing and validation that requires running Scenic scenarios with dSPACE simulator.

---

## Known Issues & Limitations

### Current Limitations
1. **Steering Feedback:** `delta` (actual steering angle) reading implemented using `Angle_SteeringGear[deg]` - **NOTE: This may need verification/adjustment in the future** if `Angle_SteeringGear` is not exactly the front wheel angle. Falls back to previous control estimate if path not available.
2. **Calibration:** Steering scale calibration procedure skeleton exists but not yet fully implemented
3. **Speed Profile:** Reference speed is constant - no speed profile from waypoints yet
4. **QP Solver Testing:** QP formulation needs testing with real data to verify correctness

### Future Enhancements
1. **Longitudinal MPC:** Add throttle/brake control (Phase 2)
2. **Adaptive Weights:** Adjust weights based on curvature/speed
3. **Multi-rate Control:** Different control rates for lateral vs longitudinal
4. **Advanced Safety:** Collision avoidance constraints

---

## Dependencies

### Required Packages
- `numpy`: Numerical computations
- `scipy`: Sparse matrices for QP
- `osqp`: QP solver (OSQP)
- `pyyaml`: Configuration file parsing

### Optional Packages
- `matplotlib`: For visualization/debugging (future)

---

## Usage Example

### Using MPC Behavior in Scenic Scenario

```scenic
# Example: ego_mpc_behavior.scenic
param map = "maps/LagunaSeca.xodr"
param ttl_folder = localPath("../../assets/ttls/LS_ENU_TTL_CSV/transformed")
param ttl_index = 17

ego = new RacingCar on mainRacingRoad, \
    with raceNumber 1, \
    with waypoints (loadWaypoints(ttl_folder, ttl_index))

# Use MPC behavior for improved racing performance
ego.behavior = FollowRacingLineMPCBehavior(
    target_speed=30,      # 30 m/s (~108 km/h)
    manage_gears=True,    # Auto gear shifting
    use_waypoints=True,   # Use waypoint-based control
    lookahead=20.0,       # 20m lookahead distance
    mpc_config_path=None  # Use default MPC config
)
```

### Using MPC Controller Directly (Python)

```python
from scenic.domains.racing.mpc import MPCLateralController, load_mpc_config

# Load configuration
config = load_mpc_config('debug_mpc/vehicle_mpc.yaml')

# Create controller
mpc = MPCLateralController(config, timestep=0.05)

# In behavior loop:
vehicle_state = {
    'x': obj.position.x,
    'y': obj.position.y,
    'yaw': obj.heading,
    'speed': obj.speed,
}
waypoints = obj.waypoints  # List of (x, y) tuples

steering = mpc.run_step(vehicle_state, waypoints)
# steering is in range [-1.0, 1.0]
```

### Using MPC via Simulation Method

```python
# In simulator implementation:
lon_controller, lat_controller = sim.getRacingControllers(
    agent, 
    use_mpc=True,  # Enable MPC
    mpc_config_path='debug_mpc/vehicle_mpc.yaml'  # Optional custom config
)
```

---

## References

- **Spec Document:** `debug_mpc/starting_guide.md`
- **Vehicle Params:** `debug_mpc/dspace_iac_car.param.yaml`
- **MPC Params:** `debug_mpc/aw_lat_mpc.param.yaml`
- **ControlDesk Paths:** `src/scenic/simulators/dspace/controldesk/readback.py`

---

## Changelog

### 2024-12-21 - Waypoint Search Fix (Critical Bug - Waypoint Index Stuck)
- ✅ **Fixed waypoint index getting stuck at 0 when vehicle is off-track**
  - **Problem:** Waypoint index never updated from 0, distance to waypoint 0 kept increasing (6.76m → 66m), vehicle drove away from path
  - **Root Cause:** Ahead-only search change removed lookback completely. When vehicle started off-track (6.76m from waypoint 0), it couldn't find waypoints behind it, so always returned index 0 as "nearest"
  - **Impact:** Vehicle always tried to reach waypoint 0, which kept getting further away. MPC was running but with wrong reference, so steering was ineffective
  - **Solution:** 
    - When CTE > 5.0m (far off-track), use aggressive lookback (50% of forward window) to find actual nearest waypoint
    - When CTE < 5.0m (on-track), use small lookback (base_lookback) for safety
    - This allows waypoint index to update correctly when vehicle is off-track
  - **Implementation:**
    - `lookback_window = int(base_forward * scale * 0.5)` when `cte_magnitude >= 5.0`
    - `lookback_window = int(base_lookback * scale)` when `cte_magnitude < 5.0`
  - **Expected behavior:**
    - Waypoint index should update correctly even when vehicle starts off-track
    - Vehicle should be able to "catch up" to waypoints when far from path
    - MPC should get correct reference waypoints for steering computation

### 2024-12-21 - MPC Threshold Adjustments (Allow MPC to Run More Often)
- ✅ **Increased safety thresholds to allow MPC to run more often**
  - **Problem:** MPC was constantly disabled, vehicle using weak fallback steering, poor tracking performance
  - **Root Cause:** 
    - Position error threshold (5.0m) was too strict - MPC disabled when CTE was just slightly over (5.1m, 5.3m)
    - Yaw error threshold (1.57rad = 90 deg) was too strict - MPC disabled when vehicle was off-track (common yaw errors of 2.5-3.0 rad)
    - Config file had old values, code changes didn't take effect
  - **Solution:** 
    - **Increased position error threshold:** 5.0m → 8.0m (allows MPC to run when CTE is moderate)
    - **Increased yaw error threshold:** 1.57rad (90 deg) → 2.36rad (135 deg) (allows MPC when off-track)
    - **Updated config file:** Changed values in `vehicle_mpc.yaml` (code defaults were overridden by YAML)
  - **Implementation:**
    - Config file: `admissible_position_error: 8.0`, `admissible_yaw_error_rad: 2.36`
    - Code defaults: Updated in `config.py` to match (for cases where YAML doesn't specify)
  - **Expected behavior:**
    - MPC should run more often, providing better control than fallback steering
    - Vehicle should track better when CTE is moderate (5-8m)
    - MPC can handle large yaw errors better than fallback steering

### 2024-12-21 - Fallback Steering Improvements (Prevent Overshooting When Close to Track)
- ✅ **Improved fallback steering for small CTE errors to prevent overshooting**
  - **Problem:** When CTE was small (1-2m), fallback steering was too weak, vehicle overshot from left to right
  - **Root Cause:** 
    - Small error branch had low proportional gain (0.3) and large max_error_for_full_steer (5.0m)
    - Heading error correction was counteracting lateral error correction
    - Steering commands were too small (e.g., -0.002) when close to track
  - **Solution:** 
    - **Increased steering authority for small errors:**
      - Proportional gain: 0.3 → 0.4 (stronger correction)
      - Max error for full steer: 5.0m → 2.0m (more responsive)
    - **Improved heading error correction:**
      - Reduce heading gain when lateral error is large (0.05 for >5m, 0.08 for 2-5m, 0.1 for <2m)
      - Prevent heading correction from counteracting lateral correction (reduce by 50% if opposing)
  - **Implementation:**
    - Small error branch: `proportional_gain = 0.4`, `max_error_for_full_steer = 2.0`
    - Heading error: Adaptive gain based on lateral error magnitude, conflict detection
  - **Expected behavior:**
    - Stronger steering correction when CTE is small (1-2m)
    - Less overshooting when approaching track
    - Heading error correction doesn't counteract lateral correction

### 2024-12-21 - Overshooting Prevention Fixes
- ✅ **Fixed vehicle overshooting track when CTE is moderate (2-4m)**
  - **Problem:** Vehicle overshooting from left to right side of track, MPC disabled due to large yaw error, weak fallback steering, speed too high when approaching track
  - **Root Cause:** 
    - Speed limits too high for moderate CTE (10 m/s at 2-3m CTE, 5 m/s at 3-5m CTE)
    - Yaw error threshold too strict (1.57 rad = 90 deg), disabling MPC when vehicle is off-track
    - Fallback steering too weak for moderate CTE (0.3 gain for <5m errors)
    - Vehicle accelerating too much when CTE reduces to 2-4m
  - **Solution:** 
    - **Reduced speed limits for moderate CTE:**
      - 2-3m CTE: 5 m/s (reduced from 10 m/s)
      - 3-5m CTE: 4 m/s (reduced from 5 m/s)
      - Prevents vehicle from overshooting when approaching track
    - **Increased fallback steering authority for moderate CTE:**
      - 2-5m CTE: 0.5 gain (increased from 0.3), full steering at 5m (reduced from 10m)
      - More responsive correction when CTE is moderate
    - **Relaxed yaw error threshold:**
      - Changed from 1.57 rad (90 deg) to 2.36 rad (135 deg)
      - Allows MPC to run more often when vehicle is off-track
      - MPC handles large yaw errors better than fallback steering
    - **Additional throttle reduction for moderate CTE at high speed:**
      - When CTE is 2-5m and speed > 4 m/s, apply additional throttle reduction
      - At 4 m/s: 0% reduction, at 6 m/s: 50% reduction, at 8+ m/s: 80% reduction
      - Prevents acceleration when approaching track at high speed
  - **Implementation:**
    - Speed limits: `effective_target_speed = 5.0` for 2-3m CTE, `4.0` for 3-5m CTE
    - Fallback steering: `proportional_gain = 0.5` for 2-5m CTE, `max_error_for_full_steer = 5.0`
    - Yaw error threshold: `admissible_yaw_error_rad = 2.36` (135 degrees)
    - Moderate CTE throttle reduction: Additional 0-80% reduction based on speed
  - **Expected behavior:**
    - Vehicle should not overshoot when CTE is moderate (2-4m)
    - MPC should run more often, providing better control than fallback
    - Stronger steering correction when CTE is moderate
    - Lower speeds when approaching track prevent overshooting

### 2024-12-21 - Smooth Driving Improvements (Prefer Throttle Reduction Over Braking)
- ✅ **Improved driving smoothness by preferring throttle reduction over braking**
  - **Problem:** Drive-brake-drive-brake cycles causing jerky motion, simultaneous throttle and brake application
  - **Root Cause:** 
    - When CTE 5-7m and speed 2-3 m/s, both throttle (0.05) and brake (0.025-0.05) were applied simultaneously
    - Binary throttle on/off (0.0 or 0.05) created abrupt transitions
    - Brake was applied even when throttle reduction would be sufficient
  - **Solution:** 
    - **Speed-based brake application:**
      - Only apply brake when speed > 5 m/s (high speed requires active braking)
      - For speed 3-5 m/s: reduce throttle to 0.0, no brake (smooth deceleration)
      - For speed 2-3 m/s: gradual throttle reduction, no brake
      - Prefer throttle reduction as primary speed control mechanism
    - **Avoid simultaneous throttle and brake:**
      - When both throttle and brake are present at moderate CTE (5-7m):
        - Low speed (<4 m/s): remove brake, reduce throttle instead
        - High speed (≥4 m/s): remove throttle, keep brake (brake necessary)
    - **Gradual throttle reduction:**
      - Use PID output with reduction factor instead of binary on/off
      - Smooth transitions between throttle levels
  - **Implementation:**
    - Speed thresholds: Brake only when speed > 5 m/s for CTE 5-7m, > 4 m/s for CTE 7-10m
    - Throttle reduction: Zero throttle for 3-5 m/s, gradual reduction for 2-3 m/s
    - Conflict resolution: Remove brake or throttle when both present at moderate CTE
    - Better logging to track smooth driving decisions
  - **Expected behavior:**
    - Smoother deceleration when CTE is moderate (5-7m)
    - No simultaneous throttle and brake application
    - Gradual speed reduction through throttle reduction instead of abrupt braking
    - More natural driving feel, less jerky motion

### 2024-12-21 - Aggressive Waypoint Search (Fix Stuck Waypoint Index) - UPDATED
- ✅ **Fixed waypoint index getting stuck when vehicle moves far from waypoint**
  - **Problem:** Waypoint index stuck at same value (e.g., 3396), distance to waypoint increasing (5m → 42m), vehicle moving but not making progress
  - **Root Cause:** 
    - Waypoint search was using `last_known_index` as starting point, biasing search toward old index
    - Even with aggressive search, `find_best_racing_waypoint` kept finding the same waypoint (3396) because it started from that index
    - Vehicle had passed waypoint 3396, but search wasn't finding the next waypoint
  - **Solution:** 
    - **Find nearest waypoint first when distance > 10m:**
      - Before doing aggressive search, scan waypoints to find the actual nearest one
      - Scan ±500 waypoints around current index first (fast)
      - If nearest is still >10m, scan entire waypoint list (brute force)
      - Use nearest waypoint as starting point for aggressive search, not old index
    - **Fallback if aggressive search returns old index:**
      - If aggressive search still returns old index with large distance, use nearest waypoint instead
      - Prevents waypoint index from getting stuck on old waypoint
    - **Enhanced manual scan fallback:**
      - When distance > 20m, scan both forward and backward waypoints
      - Find nearest waypoint regardless of direction
  - **Implementation:**
    - When `current_wp_dist > 10.0m`: First find nearest waypoint by brute force scan
    - Use nearest waypoint as `last_known_index` for aggressive search (not old index)
    - If aggressive search returns old index with distance > 10m, override with nearest waypoint
    - Search parameters: `max_search_distance=500.0`, `forward_bias=0.5`, `forward_only=False`
    - Manual scan: scans backward up to 200 waypoints when distance > 20m
    - Better debug logging to track waypoint search attempts and nearest waypoint found
  - **Expected behavior:**
    - Waypoint index should update to nearest waypoint when vehicle moves far from current waypoint
    - Vehicle should be able to "catch up" to waypoints even after overshooting
    - Distance to waypoint should decrease over time, not increase
    - CTE should reduce as vehicle gets closer to reference path

### 2024-12-21 - Progressive Brake & Throttle Control (Prevent Brake-Accelerate Cycles)
- ✅ **Fixed brake-accelerate cycle preventing vehicle progress**
  - **Problem:** Vehicle stuck in cycle: accelerate → brake (0.2) → stop → accelerate → brake → stop
  - **Root Cause:** 
    - Brake (0.2) was too strong, stopping vehicle completely
    - Min throttle (0.05) was too weak to make meaningful progress
    - Speed threshold (1.0 m/s) was too low, brake reapplied too soon
    - No throttle allowed when moving, preventing slow progress toward reducing CTE
  - **Solution:** 
    - **Progressive brake strength based on CTE:**
      - 5-7m CTE: 0.05 brake (light)
      - 7-10m CTE: 0.1 brake (moderate)
      - 10m+ CTE: 0.2-0.3 brake (strong)
    - **Increased min throttle when stopped:** 0.1 instead of 0.05
    - **Allow small throttle when moving slowly:** 0.05 throttle when speed < 3 m/s and CTE 5-7m
    - **Increased speed threshold for brake:** 2.0 m/s instead of 1.0 m/s
    - **Reduce brake when speed is low:** 50% reduction when speed < 2.5 m/s (prevents complete stop)
  - **Implementation:**
    - Speed threshold: `SPEED_THRESHOLD_FOR_BRAKE = 2.0 m/s` - allows more movement before braking
    - Minimum throttle when stopped: `MIN_THROTTLE_WHEN_STOPPED = 0.1` - stronger throttle to start moving
    - Minimum throttle when moving slowly: `MIN_THROTTLE_WHEN_MOVING_SLOW = 0.05` - enables slow progress
    - Progressive brake: Light (0.05) for 5-7m, Moderate (0.1) for 7-10m, Strong (0.2-0.3) for 10m+
    - Speed-based brake reduction: 50% reduction when speed < 2.5 m/s
  - **Expected behavior:**
    - Vehicle can make slow progress even with moderate CTE (5-7m)
    - Brake strength scales with CTE magnitude (not binary on/off)
    - Small throttle allowed when moving slowly enables progress toward reducing CTE
    - Brake reduced when speed is low prevents complete stop
    - Vehicle should gradually reduce CTE instead of getting stuck in brake-accelerate cycles

### 2024-12-19 - Fixed Simulation Loop Bug
- ✅ Removed debug `exit()` call that was terminating simulation after 10 steps
- ✅ Added note about `--count 1` parameter to prevent infinite scene generation loop
- ✅ Updated testing guidelines to include proper command-line parameters
- ⚠️ **Issue Found:** Scenic's main loop generates scenes indefinitely without `--count` parameter

### 2024-12-19 - Steering Sign Fix & TTL Coordinate System Verification
- ✅ **Fixed steering sign inversion in controller.py**
  - **Problem:** Negative sign in `controller.py` line 80 was flipping steering direction
  - **Impact:** Vehicle was steering RIGHT when MPC commanded LEFT (and vice versa)
  - **Root Cause:** `steer_val = -float(...) * 70.0` inverted the sign
  - **Fix:** Removed negative sign - now positive steering = LEFT in ControlDesk (matches joystick convention)
  - **Verification:** ControlDesk joystick docs confirm: positive = LEFT, negative = RIGHT
- ✅ **TTL Waypoint Coordinate System Verified**
  - Ran `evaluate_ttl_coordinates.py` to verify waypoint coordinate system
  - **Result:** TTL files in `transformed` folder are correctly in XODR coordinate space
  - Waypoints project correctly to road geometry when treated as XODR coordinates
  - **Conclusion:** Waypoint coordinate system is correct - not the source of tracking errors
  - Large initial CTE values (8-10m) are likely due to:
    - Initial vehicle placement offset from waypoint path
    - Waypoint path may not perfectly align with actual track centerline
    - Possible t-coordinate sign convention mismatch (see note below)
- ⚠️ **Note on dSPACE t-coordinate Sign Convention:**
  - Documentation indicates dSPACE t-coordinate convention is INVERTED:
    - Positive t = RIGHT of reference line (not LEFT as expected)
    - Negative t = LEFT of reference line (not RIGHT as expected)
  - This may affect lateral error computation if waypoint path uses same convention
  - MPC uses standard convention: Positive e_y = LEFT, Negative e_y = RIGHT
  - **Recommendation:** Verify if waypoint path computation needs sign adjustment

### 2024-12-19 - CTE-Aware PID Controller & Progressive Throttle Reduction (Updated)
- ✅ **Made PID controller CTE-aware (CRITICAL FIX)**
  - **Problem:** PID controller didn't account for vehicle's natural acceleration (momentum, gravity, etc.)
  - **Impact:** PID saw `speed_error = 30 - 6.29 = 23.71 m/s` and commanded full throttle even when CTE was large
  - **Root Cause:** Vehicle accelerates naturally even without throttle, but PID only sees speed difference
  - **Solution:** Modify effective target speed based on CTE magnitude before computing speed error
    - **CTE < 2m:** Full target speed (30 m/s)
    - **2-10m CTE:** Linear reduction from 100% to 50% of target speed
    - **10-15m CTE:** Linear reduction from 50% to 30% of target speed
    - **15-50m CTE:** 30% of target speed (encourages braking)
    - **>50m CTE:** 10% of target speed (heavy braking)
  - **Result:** PID now commands braking or zero throttle when CTE is large, instead of trying to accelerate
- ✅ **Implemented progressive throttle reduction based on CTE magnitude**
  - **Problem:** Fixed throttle (0.1) caused vehicle to accelerate too fast when starting with large CTE errors
  - **Impact:** Vehicle reached 11+ m/s before steering could correct, leading to overshoot and 24m+ CTE errors
  - **Solution:** Multi-zone progressive throttle reduction:
    - **2-10m CTE:** Linear reduction from base throttle (0.1) to minimum (0.03) - **LOWERED from 5m to 2m**
    - **10-15m CTE:** Further reduction to 0.3 throttle limit
    - **15-50m CTE:** Progressive braking (existing logic)
    - **>50m CTE:** Full brake (existing logic)
- ✅ **Added speed-based throttle reduction (Enhanced)**
  - When CTE > 2m and speed > 3 m/s, additional throttle reduction based on speed
  - **More aggressive speed penalty:**
    - At 3 m/s: 0% reduction
    - At 8 m/s: 50% reduction
    - At 13+ m/s: 80% reduction (increased from 50%)
  - Prevents vehicle from overshooting when correcting large errors at high speed
- ✅ **PID output clamping**
  - Clamp PID controller output to [-1.0, 1.0] range before applying CTE/speed reductions
  - Prevents excessive throttle commands from PID controller
- ✅ **Throttle limit enforcement**
  - Ensure `local_throttle_limit` never exceeds base `throttle_limit`
  - Final throttle is clamped to `local_throttle_limit` (includes all reductions)
- ✅ **Throttle reduction thresholds:**
  - `cte_throttle_reduction_start = 2.0m`: Start progressive reduction (lowered from 5.0m)
  - `cte_throttle_reduction_max = 10.0m`: Maximum reduction zone
  - `min_throttle_at_large_cte = 0.03`: Minimum throttle when CTE > 10m
- **Expected behavior:** 
  - PID controller now commands braking/zero throttle when CTE is large (addresses natural acceleration issue)
  - Vehicle should now accelerate more slowly when off-track (even with small CTE errors)
  - Throttle reduction continues even when CTE drops below 5m (prevents sudden throttle jumps)
  - More aggressive speed-based reduction prevents overshooting at high speeds

### 2024-12-19 - Proportional Fallback Steering & Safety Improvements
- ✅ **Implemented proportional fallback steering** to prevent catch-22 situations
  - **Problem:** Large errors (>5m) disabled MPC → zero steering → larger errors → MPC stays disabled
  - **Solution:** Fallback now uses proportional control based on lateral error (e_y)
  - Proportional gain: 0.3 (full steering at ~10m error)
  - Blends with last valid steering for smooth transitions
  - Prevents vehicle from drifting further when MPC is disabled
- ✅ Updated `_fallback_steering()` to accept error information (e_y, e_psi)
- ✅ Modified safety check calls to pass error information to fallback
- ✅ Added heading error correction in fallback (smaller contribution)

### 2024-12-19 - Steering Feedback Reading Infrastructure
- ✅ Added `steer_actual` path configuration in `vehicle_mpc.yaml`
- ✅ Implemented steering feedback reading in `io_adapter.py`
- ✅ Updated MPC controller to use actual steering feedback from ControlDesk
- ✅ Added fallback to previous state estimate if feedback not available
- ✅ Integrated steering feedback reading into `FollowRacingLineMPCBehavior`
- ✅ Stored MPC config in simulation for io_adapter access
- ✅ **Integrated `Angle_SteeringGear[deg]` as steering feedback path**
- ⚠️ **Note:** Using `Angle_SteeringGear` - this may need verification/adjustment in the future if it's not exactly the front wheel angle. Alternative paths available: `Angle_SteeringWheel` (has steering ratio) and `Displ_Steering` (displacement, not angle).

### 2024-12-19 - Testing Complete & Path Fix
- ✅ Created comprehensive integration test suite (27 tests total)
- ✅ Added behavior integration tests (`test_behavior_integration.py`)
- ✅ Added simulation integration tests (`test_simulation_integration.py`)
- ✅ Added scenario compilation tests (`test_scenario_compilation.py`)
- ✅ **Fixed config path resolution** - now correctly finds `debug_mpc/vehicle_mpc.yaml`
- ✅ **All 27 tests passing!** (1 skipped - requires dSPACE simulator)
- ✅ Updated test runner to include all new tests
- ✅ Updated TEST_CASES.md with integration test documentation
- ✅ Fixed example scenario (`ego_mpc_behavior.scenic`) to use correct TTL loading

### 2024-12-19 - Behavior Integration Complete
- ✅ Created `FollowRacingLineMPCBehavior` in `behaviors.scenic`
- ✅ Updated `DSpaceSimulation.getRacingControllers()` to support `use_mpc=True` parameter
- ✅ Updated abstract base class `RacingSimulation.getRacingControllers()` signature
- ✅ Created example scenario: `examples/racing/ego_mpc_behavior.scenic`
- ✅ MPC controller integrated with waypoint following logic
- ✅ Maintains compatibility with existing PID-based behaviors

### 2024-12-19 - Testing Infrastructure & Fixes
- ✅ Created comprehensive test suite in `testing/` directory
- ✅ Added unit tests for all major components (18 test cases)
- ✅ Created `TEST_CASES.md` living document as guard/regression test suite
- ✅ Added test runner script (`run_tests.py`)
- ✅ Fixed OSQP solver update issue (now uses setup() each step)
- ✅ Fixed test expectation for lateral error sign convention
- ✅ Updated README with testing information for future AI agents
- ✅ **All 18 tests passing!** ✅

### 2024-12-19 - State Computation Implementation
- ✅ Implemented state computation (`_compute_errors()`)
  - Lateral error (e_y) computed from waypoint projection
  - Heading error (e_psi) computed from segment direction
  - Safety checks for large errors (disable MPC if exceeded)
- ✅ Added error computation based on existing `FollowRacingLineBehavior` CTE logic
- ✅ Integrated state computation into `run_step()`

### 2024-12-19 - Initial Implementation
- Created module structure
- Implemented configuration loader
- Implemented reference trajectory builder
- Created MPC controller skeleton
- Added utility functions (low-pass filter)
- Created I/O adapter skeleton
- Created calibration utilities skeleton

---

## Next Steps

### Priority 1: Integration with Behaviors
1. **Create MPC Behavior:**
   - Create `FollowRacingLineMPCBehavior` in `behaviors.scenic`
   - Or add `use_mpc=True` parameter to existing `FollowRacingLineBehavior`
   - Integrate MPC controller with waypoint following logic

2. **Simulation Integration:**
   - Update `DSpaceSimulation.getRacingControllers()` to support MPC option
   - Test with ControlDesk connection
   - Validate coordinate systems match

### Priority 2: Steering Feedback & Calibration
3. **Steering Feedback:**
   - Read actual steering angle (delta) from ControlDesk
   - Update state computation to use real steering feedback

4. **Calibration:**
   - Implement steering scale calibration procedure
   - Test calibration with real vehicle
   - Update config with calibrated values

### Priority 3: Testing & Tuning
5. **Integration Testing:**
   - Test with real waypoint files
   - Test end-to-end behavior
   - Validate performance

6. **Tuning:**
   - Initial weight tuning based on test results
   - Safety threshold tuning
   - Performance optimization (if needed)

---

*This is a living document - update as implementation progresses and learnings are gathered.*

