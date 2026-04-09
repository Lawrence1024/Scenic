# Racing examples

Scenarios for the racing domain using the dSPACE racing simulator. All examples use `scenic.simulators.dspace.racing_model` and the Laguna Seca map unless noted.

| Example | Description |
|--------|-------------|
| **ego_mpc_behavior.scenic** | Ego follows the TTL with `FollowRacingLineMPCBehavior` (MPC; recommended). Uses TTL waypoints from `LS_ENU_TTL_CSV` (e.g. `ttl_main_road.csv` or `ttl_pitlane.csv`) and 100 Hz sim / 20 Hz control. Ego route (Lap vs Pit) is chosen by TTL distance; when similar, main road is preferred. |
| **ego_fixed_placing.scenic** | Single ego at fixed coordinates (no behavior). |
| **fellow_fixed_placing.scenic** | Ego + fellows at fixed waypoint positions along the track. |
| **three_segments.scenic** | Vehicles on `mainRacingRoad` vs `pitLaneRoad`; route assignment from OpenDRIVE. |
| **test_relative.scenic** | Fellows placed relatively (ahead/behind). |
| **ego_calibration_accel_decel.scenic** | Throttle/brake calibration for MPC tuning; prints acceleration/deceleration for `vehicle_mpc.yaml`. |
| **decision_tree_example.scenic** | Decision-tree behaviors: `FlagBasedSpeedBehavior`, `LaneSelectionBehavior`, `StopBehavior`, `FollowModeBehavior`. |
| **phase0_benchmark/** | Phase 0 baseline scenario bank + runner-oriented set (no opponent, slower opponent variants, weaving, corner approach, side-by-side). |
| **phase1_planner/** | Phase 1 planner-to-MPC scripted handoff tests (optimal->left, left->right, right->optimal). |

Run with the racing model, e.g.:

```bash
scenic examples/racing/ego_mpc_behavior.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1
```

Run the full Phase 0 benchmark bank:

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 3000
```

(Default `--inter-run-delay-s` is 15; use `--scenario` / `--scenario-glob` to run a subset.)

Run all Phase 1 scripted TTL-switch validation scenarios:

```bash
python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000
```

(Same delay and filtering flags as Phase 0; see `examples/racing/phase1_planner/README.md`.)
