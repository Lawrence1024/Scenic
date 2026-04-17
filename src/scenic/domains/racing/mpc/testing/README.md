# MPC module testing

Unit and integration tests for the MPC submodule and the intelligence pipeline. All test cases should pass before considering the module ready for integration.

## Running tests

```bash
cd src/scenic/domains/racing
python -m pytest mpc/testing/ -x -q
```

Run a specific file: `python -m pytest mpc/testing/test_tactical_planner.py -v`. Do not run `scenic` from automated scripts; use `--count 1` when running scenarios manually.

## Test categories

### MPC controllers

| Category | File | Coverage |
|----------|------|----------|
| Reference builder | `test_reference_builder.py` | find_nearest_waypoint, curvature, build_reference (shapes, 6-tuple return), resample |
| MPC lateral | `test_mpc_lateral.py` | _compute_errors (e_y, e_psi), fallback steering, run_step basic |
| Config | `test_config.py` | Defaults, custom values, timestep adaptation, YAML load |
| Utilities | `test_utils.py` | Low-pass filter (step, smoothing, reset) |

### Intelligence pipeline (4 layers)

| Category | File | Coverage |
|----------|------|----------|
| Tactical planner | `test_tactical_planner.py` | FREE_RUN / FOLLOW / SETUP / COMMIT / ABORT state machine; `asymmetric_opening` suppression of safety pressure; `CommitPlannerState` lifecycle; protected-follow latch and release; segment-aware gating; 37 tests |
| Race situation assessment | `test_phase8_assessment.py` | `assess_race_situation()` gap-ok, corridor-open, closing, emergency-risk classification |
| Stability guard | `test_stability_guard.py` | Steer slew limiting, TTL switch rate limiting, emergency-stable mode, reapproach throttle hold |
| Fellow predictor | `test_fellow_predictor.py` | One-step-ahead CV prediction, recency-weighted history, baseline error metrics |
| Behavior integration | `test_behavior_integration.py` | MPC controller interface, PID compatibility |
| Log metrics | `test_log_metrics.py` (various) | Log line parsing for `[Planner]`, `[Assessment]`, `[Guard]`, `[Commit]` tags |

## Log tags emitted by the pipeline

| Tag | Source | Enabled by |
|-----|--------|-----------|
| `[Prediction]` | `prediction/fellow_predictor.py` | `prediction_enabled=True` |
| `[Assessment]` | `assessment/race_situation.py` | `assessment_enabled=True` |
| `[Planner]` | `behaviors.scenic` | `tactical_planner_enabled=True` |
| `[Commit]` | `behaviors.scenic` | `commit_abort_enabled=True` |
| `[Hazard]` | `behaviors.scenic` | `tactical_planner_enabled=True` |
| `[Guard]` | `safety/stability_guard.py` | `stability_guard_enabled=True` |

## Limitations

- **OSQP:** Some tests require `osqp`; they skip if not installed.
- **Integration:** Unit tests only; ControlDesk / real waypoint integration tests are separate.
- **Performance:** No solve-time or memory benchmarks yet.

When adding tests, add to the appropriate test file and run the full suite before committing.
