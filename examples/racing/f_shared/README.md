# Shared F Scenario Bank (`f_shared`)

This folder is the reusable scenario bank for post-Phase-5 planning and testing.
It exists to avoid cloning almost-identical scenario directories for each new phase.

## Scenario IDs

- `F0` ego alone baseline
- `F1` fellow behind, same TTL, cruise
- `F2` fellow ahead, same TTL, slower cruise
- `F3L` fellow ahead on left TTL
- `F3R` fellow ahead on right TTL
- `F4` fellow ahead then sudden stop
- `F5` fellow ahead then right-left swerve + stop
- `F6` deterministic left-occupied corridor
- `F7` deterministic right-occupied corridor
- `F8` corner-entry fellow-ahead stress

## How to run Phase 6 default subset (`F0`,`F1`,`F2`)

```bash
python -m scenic.domains.racing.benchmarks.phase6_runner
```

## Run any subset from this shared bank

```bash
python -m scenic.domains.racing.benchmarks.phase6_runner --scenario F4_fellow_ahead_sudden_stop.scenic
python -m scenic.domains.racing.benchmarks.phase6_runner --scenario-glob "F[0-8]_*.scenic"
```

`phase6_runner` uses `PhaseRunnerSpec.default_scenario_names` to keep a small default
set, while still allowing ad-hoc scenario selection from the shared folder.
