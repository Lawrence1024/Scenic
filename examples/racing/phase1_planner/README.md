# Phase 1 Planner-MPC Integration Scenarios

These scenarios validate the Phase 1 scripted planner integration point:

- `01_optimal_to_left.scenic` (switch at `t=10s`)
- `02_left_to_right.scenic` (switch at `t=10s`)
- `03_right_to_optimal.scenic` (switch at `t=10s`)

All scenarios use:

- `FollowRacingLineMPCBehavior(..., planner_enabled=True, ttl_schedule=..., target_speed_cap=...)`
- dSPACE racing model
- step size `0.01s`, control period `0.05s`

Run example:

```bash
python -m scenic examples/racing/phase1_planner/01_optimal_to_left.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1 --time 3000
```

Run all Phase 1 scripted-switch scenarios with automatic logs + summary:

```bash
python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000
```

Run one specific Phase 1 scenario with the runner:

```bash
python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000 --scenario 02_left_to_right.scenic
```

Expected runtime log marker:

- `[Phase1Planner] t=... ttl_switch <from>-><to>`

**Phase 1 exit:** all three scenarios should show exactly one scripted switch in the runner summary (`phase1_switch_observed`, `ttl_switches` ≥ 1) with no collision/off-track flags. See `src/scenic/domains/racing/plans/phase-1-planner-mpc-integration.md`.
