# Racing examples

Scenarios for the racing domain using the dSPACE racing simulator. All examples use `scenic.simulators.dspace.racing_model` and the Laguna Seca map unless noted.

| Example | Description |
|--------|-------------|
| **ego_mpc_behavior.scenic** | Ego follows the racing line using MPC (recommended for best performance). Uses TTL waypoints from `LS_ENU_TTL_CSV` (e.g. `ttl_main_road.csv` or `ttl_pitlane.csv`) and 100 Hz sim / 20 Hz control. Ego route (Lap vs Pit) is chosen by TTL distance; when similar, main road is preferred. |
| **ego_fixed_behavior.scenic** | Ego at fixed position with PID-based `FollowRacingLineBehavior`. |
| **ego_fixed_placing.scenic** | Single ego at fixed coordinates (no behavior). |
| **fellow_fixed_placing.scenic** | Ego + fellows at fixed waypoint positions along the track. |
| **three_segments.scenic** | Vehicles on `mainRacingRoad` vs `pitLaneRoad`; route assignment from OpenDRIVE. |
| **test_relative.scenic** | Fellows placed relatively (ahead/behind). |
| **ego_calibration_accel_decel.scenic** | Throttle/brake calibration for MPC tuning; prints acceleration/deceleration for `vehicle_mpc.yaml`. |
| **decision_tree_example.scenic** | Decision-tree behaviors: `FlagBasedSpeedBehavior`, `LaneSelectionBehavior`, `StopBehavior`, `FollowModeBehavior`. |

Run with the racing model, e.g.:

```bash
scenic examples/racing/ego_mpc_behavior.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1
```
