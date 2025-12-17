# dSPACE Simulator Integration - Logging and Debugging

## Overview

This folder contains the dSPACE simulator integration for Scenic, including coordinate transformation, vehicle placement, and simulation control. Recent work has focused on improving logging to help diagnose coordinate transformation and placement issues.

## What Has Been Done

### 1. Logging Improvements

#### Removed Verbose/Unnecessary Logs
- Removed "Creating EGO/FELLOW vehicle" messages
- Removed "Checking objects for dynamic control" messages
- Removed "No racing behaviors found" messages
- Removed verbose RouteProjection debug messages
- Removed DEBUG ego state reading messages
- Removed verbose ControlDesk connection step-by-step messages
- Removed verbose VesiInterface initialization messages
- Removed verbose warm-up attempt messages

#### Added Clear Transformation Chain Logs
During vehicle placement, each vehicle now logs its complete transformation chain:
```
[VehicleName] XODR: (x, y) → RD: (x, y) → Route RouteName (s=s_val, t=t_val)
```

This shows:
- Original XODR coordinates from Scenic
- Transformed RD coordinates (after coordinate transform)
- Detected route (Pit/Lap)
- Calculated route-relative (s,t) coordinates

**Location**: `modeldesk/placement.py` - `place_ego()` and `place_fellow()` functions

#### Added Readback Comparison Logs
On first readback from ControlDesk, each vehicle logs:
```
[VehicleName Readback] RD: (actual_rd) [expected: (expected_rd), error: X.XXXm]
[VehicleName Readback] XODR: (actual_xodr) [expected: (expected_xodr), error: X.XXXm]
```

This shows:
- Actual position read from ControlDesk (in RD coordinates)
- Expected position (what we placed)
- Error distance in meters
- Same comparison in XODR coordinates (after inverse transform)

**Location**: `controldesk/readback.py` - `read_ego_state()` and `read_fellow_state()` functions

### 2. Coordinate Transformation Pipeline

The transformation pipeline is:
1. **XODR → RD**: Apply coordinate transform (rotation + translation)
2. **Route Detection**: Determine if vehicle is on pitLane or mainRacing road
3. **RD → (s,t)**: Project RD coordinates to route-relative (s,t) using route-specific road sequences
4. **Placement**: Set (s,t) in ModelDesk with appropriate route
5. **Readback**: Read actual position from ControlDesk and compare with expected

**Key Files**:
- `geometry/coordinate_transform.py`: XODR ↔ RD transformation
- `geometry/route_projection.py`: RD → route-specific (s,t) projection
- `geometry/route_mapping.py`: Route detection (pitLane vs mainRacing)
- `modeldesk/placement.py`: Vehicle placement in ModelDesk
- `controldesk/readback.py`: Position readback from ControlDesk

### 3. Known Limitations

#### T-coordinate (Lateral Deviation) Limitation
dSPACE ModelDesk may ignore lateral deviation (`t-coordinate`) settings for both ego and fellow vehicles. Testing shows:
- Ego: "Could not activate Deviation mode" warning
- Fellow: Vehicles placed on centerline regardless of `t` value

This is a **dSPACE ModelDesk configuration issue**, not a bug in our code. See:
- `debug_ego_cord/README.md`
- `debug_route_code/README.md`

**Location**: `modeldesk/placement.py` - documented in code comments

### 4. Simulation Control

Currently configured for **continuous running** (pause/step functionality temporarily disabled):
- `simulator.py`: `setup()` - pause call commented out
- `simulator.py`: `step()` - manual stepping replaced with `time.sleep()`

## What Needs to Be Done

### Primary Task: Run Simulation and Analyze Logs

1. **Activate virtual environment**:
   ```powershell
   c:/Users/bklfh/Documents/Scenic/venv/Scripts/Activate.ps1
   ```

2. **Run scenic command**:
   ```powershell
   scenic examples/racing/fellow_fixed_placing.scenic --2d --model scenic.simulators.dspace.racing_model --simulate --time 10
   ```

3. **Analyze the logs** to understand:
   - Are transformation chains correct?
   - Are routes detected correctly?
   - Are (s,t) values calculated correctly?
   - What are the readback errors?
   - Are there systematic issues or just individual vehicle issues?

### Expected Log Output

You should see logs like:

```
[Ego] XODR: (163.545000, 48.302000) → RD: (xxx.xxxxxx, yyy.yyyyyy) → Route Pit (s=xxx.xx, t=xxx.xxxxxx)
[Fellow_1] XODR: (-101.919263, -457.524908) → RD: (xxx.xxxxxx, yyy.yyyyyy) → Route Lap (s=xxx.xx, t=xxx.xxxxxx)
...
[Ego Readback] RD: (xxx.xxxxxx, yyy.yyyyyy) [expected: (xxx.xxxxxx, yyy.yyyyyy), error: X.XXXm]
[Ego Readback] XODR: (xxx.xxxxxx, yyy.yyyyyy) [expected: (xxx.xxxxxx, yyy.yyyyyy), error: X.XXXm]
[Fellow_1 Readback] RD: (xxx.xxxxxx, yyy.yyyyyy) [expected: (xxx.xxxxxx, yyy.yyyyyy), error: X.XXXm]
[Fellow_1 Readback] XODR: (xxx.xxxxxx, yyy.yyyyyy) [expected: (xxx.xxxxxx, yyy.yyyyyy), error: X.XXXm]
...
```

### Analysis Questions

After running, check:

1. **Transformation Chain**:
   - Do XODR → RD transformations look correct?
   - Are routes detected correctly (Pit for ego, Lap for fellows)?
   - Are (s,t) values reasonable?

2. **Readback Errors**:
   - What are the RD coordinate errors? (Should be < 1m for most positions, ~10m at transition points)
   - What are the XODR coordinate errors? (Should be < 1m after round-trip)
   - Are errors systematic (all vehicles) or individual?

3. **Comparison with ControlDesk**:
   - Compare the logged RD coordinates with ControlDesk values
   - Are they matching? If not, where is the discrepancy?

### Potential Issues to Investigate

1. **Route Detection**: Are vehicles assigned to correct routes?
2. **Route-Specific Projection**: Are (s,t) values calculated correctly for each route?
3. **Placement Accuracy**: Are vehicles placed where expected in ModelDesk?
4. **Readback Accuracy**: Do readback positions match expected positions?
5. **T-coordinate**: Are lateral deviations being applied? (Known limitation - may be ignored)

### Next Steps Based on Logs

After analyzing logs:

1. **If transformation chain is wrong**: Check coordinate transform parameters
2. **If route detection is wrong**: Check `route_mapping.py` and road ID detection
3. **If (s,t) values are wrong**: Check `route_projection.py` and route-specific calculations
4. **If readback errors are high**: Check placement logic and ModelDesk configuration
5. **If systematic errors**: Check coordinate transform or route projection logic
6. **If individual errors**: Check specific vehicle coordinates or road assignments

## File Structure

```
src/scenic/simulators/dspace/
├── README.md                          # This file
├── simulator.py                       # Main simulator class
├── geometry/
│   ├── coordinate_transform.py        # XODR ↔ RD transformation
│   ├── route_projection.py            # RD → route-specific (s,t)
│   ├── route_mapping.py               # Route detection
│   └── ...
├── modeldesk/
│   ├── placement.py                   # Vehicle placement (with new logs)
│   └── ...
└── controldesk/
    ├── readback.py                    # Position readback (with new logs)
    └── ...
```

## Related Documentation

- `debug_cord_code/README.md`: Route-specific projection details
- `debug_ego_cord/README.md`: Ego vehicle coordinate issues
- `debug_route_code/README.md`: Route detection and fellow vehicle issues

## Notes

- Logs are designed to be clear and concise
- Transformation chain logs show the complete pipeline
- Readback logs show expected vs actual for debugging
- Error distances help identify accuracy issues
- All logs use consistent formatting: `[VehicleName] Message`
