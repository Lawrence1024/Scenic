# MPC module testing

Unit and integration tests for the MPC submodule. All test cases should pass before considering the module ready for integration.

## Running tests

```bash
cd src/scenic/domains/racing/mpc/testing
python -m pytest test_*.py -v
# or
python run_tests.py
```

Run a specific file: `python -m pytest test_reference_builder.py -v`. Do not run `scenic` from automated scripts; use `--count 1` when running scenarios manually.

## Test categories

| Category | File | Coverage |
|----------|------|----------|
| Reference builder | `test_reference_builder.py` | find_nearest_waypoint, curvature, build_reference (shapes, 6-tuple return), resample |
| MPC lateral | `test_mpc_lateral.py` | _compute_errors (e_y, e_psi), fallback steering, run_step basic |
| Config | `test_config.py` | Defaults, custom values, timestep adaptation, YAML load |
| Utilities | `test_utils.py` | Low-pass filter (step, smoothing, reset) |

## Limitations

- **OSQP:** Some tests require `osqp`; they skip if not installed.
- **Integration:** Unit tests only; ControlDesk / real waypoint integration tests are separate.
- **Performance:** No solve-time or memory benchmarks yet.

When adding tests, add to the appropriate test file and run the full suite before committing.
