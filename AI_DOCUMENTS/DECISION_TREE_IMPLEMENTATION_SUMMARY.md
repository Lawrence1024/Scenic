# Decision Tree Implementation Summary

## Overview

This document summarizes the implementation of Actions and Behaviors for executing the race decision tree logic in Scenic. The implementation follows the plan outlined in the architecture guide, providing a bridge between the ROS2-based race decision engine and Scenic's behavior system.

## Implementation Status

### ✅ Completed Components

#### 1. Extended DSpaceVehicleActor (`simulator.py`)
- Added decision tree state variables to store:
  - `speed_limit`: Current speed limit (m/s)
  - `speed_type`: Speed type string ("stop", "pit_crawl", "pit_lane", "yellow", "green", etc.)
  - `ttl_selection`: TTL selection ("left", "right", "race", "optimal", "pit")
  - `target_gap`: Target following gap (meters)
  - `gap_type`: Gap type string ("no_gap", "attacker_preparing", etc.)
  - `strategy_type`: Strategy mode ("cruise_control" or "follow_mode")
  - `scale_factor`: Speed scale factor (0.0-1.0)
  - `powertrain_mode`: Powertrain mode ("pit_lane", "quiet", "nominal", "race", "overboost")
  - `push2pass_active`: Push2Pass activation state

#### 2. Created RacingSteers Protocol (`domains/racing/actions.py`)
- Extended protocol for racing-specific vehicle control
- Methods:
  - `setSpeedLimit(speed_limit)`: Set maximum speed limit
  - `setTTLSelection(selection)`: Select TTL (left/right/race/optimal/pit)
  - `setTargetGap(gap)`: Set target following gap
  - `setStrategy(strategy_type)`: Set strategy mode
  - `setPowertrainMode(mode)`: Set powertrain mode
  - `setScaleFactor(scale_factor)`: Apply speed scale factor
  - `setPush2Pass(active)`: Activate/deactivate push2pass

#### 3. Added New Actions (`domains/racing/actions.py`)
- **SetSpeedLimitAction**: Set speed limit based on speed type
- **SetTTLSelectionAction**: Select TTL based on decision tree logic
- **SetTargetGapAction**: Set target following gap
- **SetStrategyAction**: Set driving strategy (cruise_control/follow_mode)
- **SetPowertrainModeAction**: Set powertrain mode
- **SetScaleFactorAction**: Apply speed scale factor
- **SetPush2PassAction**: Activate/deactivate push2pass
- **StopCarAction**: Emergency/immediate/safe stop

#### 4. Implemented Protocol Methods (`simulators/dspace/model.scenic`)
- All `RacingSteers` protocol methods implemented in `DSPACERacingCar`
- Methods store state in `dspaceActor` and update control parameters
- State is accessible for behaviors and simulator control loop

#### 5. Created Basic Behaviors (`domains/racing/behaviors.scenic`)
- **FlagBasedSpeedBehavior**: Set speed based on flag type (yellow, green, etc.)
- **LaneSelectionBehavior**: Select TTL based on attacker/defender flags
- **StopBehavior**: Stop car with specified stop type
- **FollowModeBehavior**: Follow another car maintaining target gap
- **PitLaneBehavior**: Handle pit lane speeds

#### 6. Waypoints + Lookahead Steering (New)
- dSPACE TTL loader assigns transformed TTL points to `ego.waypoints`
- `FollowRacingLineBehavior` and `FollowModeBehavior` accept:
  - `use_waypoints=True` and `lookahead=20.0` (meters)
  - If waypoints exist, behaviors target a lookahead point along the TTL polyline and compute signed lateral error to the local segment normal; otherwise they fall back to region-based `signedDistanceTo`.
- Effect: reduced steering saturation when frames are slightly offset; smoother progress along TTL.

#### 7. Gear Management Parameter (New)
- Behaviors accept `manage_gears=True`:
  - Shift to gear 1 if neutral; up/down with simple speed thresholds
  - Applies only when the actor supports `setGear`; pass `manage_gears=False` for fellows or non-manual vehicles.

## Usage Example

```scenic
model scenic.simulators.dspace.model

# Create racing car
ego = new RacingCar on mainRacingRoad, with raceNumber 1

# Use decision tree behaviors
ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0)

# Or use lane selection
ego.behavior = LaneSelectionBehavior(ttl_selection="race")

# Or use follow mode
opponent = new RacingCar ahead of ego by 50, with raceNumber 2
ego.behavior = FollowModeBehavior(target_car=opponent, target_gap=31.0)

# Or use pit lane behavior
ego.behavior = PitLaneBehavior()

# Or use stop behavior
ego.behavior = StopBehavior(stop_type="safe")
```

## Architecture

### Action Flow
```
Behavior
    ↓
Action (e.g., SetSpeedLimitAction)
    ↓
Protocol Method (e.g., setSpeedLimit)
    ↓
DSPACERacingCar Implementation
    ↓
dspaceActor State Storage
    ↓
Simulator Control Loop
```

### State Storage
- **dspaceActor**: Stores decision tree state variables
- **DSPACERacingCar**: Implements protocol methods
- **Behaviors**: Compose actions based on race conditions

## Next Steps

### Phase 2: Race State Utilities (TODO)
1. Create `RaceControlFlags` utility class
2. Create `RivalCarInfo` utility class
3. Add location detection functions
4. Add timeout monitoring

### Phase 3: Advanced Behaviors (TODO)
1. **EmergencyStopBehavior**: Handle emergency stops
2. **ImmediateStopBehavior**: Handle immediate stops
3. **SafeStopBehavior**: Handle safe stops
4. **OvertakingBehavior**: Handle overtaking logic
5. **DefensiveFollowingBehavior**: Defensive following when rival detected
6. **PitEntryExitBehavior**: Handle pit entry/exit transitions
7. **ApplyScaleFactorBehavior**: Apply region-based speed modifiers

### Phase 4: Main Decision Tree Behavior (TODO)
1. **RaceDecisionTreeBehavior**: Main reactive behavior implementing full decision tree logic
2. Priority-based decision making (emergency > immediate > safe > normal)
3. Integration with race state utilities
4. Full flag-based logic implementation

## Notes

- All actions follow the protocol pattern (actions call protocol methods, simulators implement them)
- State is stored in `dspaceActor` for access by simulator control loop
- Behaviors compose actions and can call other behaviors
- TTL loader (dSPACE) defaults to `ttl_17.csv` and applies a global offset; it assigns both `ego.ttl` (PolylineRegion) and `ego.waypoints` (list of transformed points).
- dSPACE authoring avoids creating duplicate Fellows (skips ego and already-placed fellows), and the simulator only drives fellows that actually have behaviors assigned.
- Fellows’ Segment 1 is configured to Velocity=0 (constant) and Lateral=Continue (Endless), making them stationary unless explicitly controlled.
- VesiInterface gear/clutch one-shot actions are applied even if no throttle/brake/steer were set in the same tick.
- Implementation is extensible - new actions/behaviors can be added following the same pattern

## Files Modified

1. `src/scenic/simulators/dspace/simulator.py`: Extended `DSpaceVehicleActor`
2. `src/scenic/domains/racing/actions.py`: Added `RacingSteers` protocol and new actions
3. `src/scenic/simulators/dspace/model.scenic`: Implemented `RacingSteers` protocol methods
4. `src/scenic/domains/racing/behaviors.scenic`: Added decision tree behaviors; waypoint lookahead and gear management parameter
5. `src/scenic/simulators/dspace/vehicle/controller.py`: Apply gear/clutch one-shots even without throttle/brake/steer that tick
6. `src/scenic/simulators/dspace/simulator.py`: Avoid duplicate Fellows; drive only fellows with behaviors; fellows seg1 Velocity=0; TTL loader default and assignments

## Testing

To test the implementation:

1. Create a simple scenario using the new behaviors
2. Verify actions are called correctly
3. Verify state is stored in `dspaceActor`
4. Verify behaviors compose correctly

Example test scenario:
```scenic
model scenic.simulators.dspace.model

ego = new RacingCar on mainRacingRoad, with raceNumber 1
ego.behavior = FlagBasedSpeedBehavior(speed_type="green", speed_limit=120.0, manage_gears=True)
```

