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

### 🚧 In Progress
- [ ] Complete MPC QP formulation testing (verify dynamics matrices)
- [ ] Steering feedback reading (delta from ControlDesk)
- [ ] ControlDesk integration testing
- [ ] Steering scale calibration implementation

### 📋 TODO
- [x] Unit tests for reference builder ✅
- [x] Unit tests for MPC formulation ✅
- [x] Unit tests for configuration ✅
- [x] Unit tests for utilities ✅
- [x] Test infrastructure and living document ✅
- [ ] Integration with `FollowRacingLineBehavior`
- [ ] Performance tuning and weight optimization
- [ ] Integration tests with ControlDesk
- [ ] Documentation and examples

---

## Key Learnings & Decisions

### Configuration Management
- **Decision:** Use YAML config files (compatible with ROS-style parameter format)
- **Location:** `debug_mpc/vehicle_mpc.yaml` (to be created)
- **Adaptation:** Config adapts to Scenic `timestep` automatically

### ControlDesk Integration
- **Read Paths:** Use existing `read_ego_state()` / `read_fellow_state()` functions
- **Write Path:** Use existing `VehicleController` infrastructure via `_control_state`
- **Steering Range:** ControlDesk expects -70 to +70 (degrees-like units)
- **Normalization:** MPC outputs [-1, 1], converted to ControlDesk range

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
- **New Behavior:** `FollowRacingLineMPCBehavior` (to be created)
- **Alternative:** Add `use_mpc=True` parameter to existing `FollowRacingLineBehavior`

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

---

## Known Issues & Limitations

### Current Limitations
1. **Steering Feedback:** `delta` (actual steering angle) not yet read from ControlDesk - currently uses previous control estimate
2. **Calibration:** Steering scale calibration not yet implemented
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

## Usage Example (Planned)

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

---

## References

- **Spec Document:** `debug_mpc/starting_guide.md`
- **Vehicle Params:** `debug_mpc/dspace_iac_car.param.yaml`
- **MPC Params:** `debug_mpc/aw_lat_mpc.param.yaml`
- **ControlDesk Paths:** `src/scenic/simulators/dspace/controldesk/readback.py`

---

## Changelog

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

