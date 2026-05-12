# Racing examples

Scenarios for the racing domain using the dSPACE racing simulator. All examples use `scenic.simulators.dspace.racing_model` and the Laguna Seca map (`assets/maps/dSPACE/LGS_v1.xodr`).

## Layout

| Folder | Purpose |
|---|---|
| **f_shared/** | F-bank regression suite (`F0`..`F14`, plus `F3L`/`F3R` and `F13c` variants). Primary smart-ego regression set. |
| **falsifiable/** | Falsification templates with `VerifaiRange(...)` parameters (`S1_falsify.scenic`, `S2_falsify.scenic`, `S3_blocker_falsify.scenic`, `F_curve_falsify.scenic`). |
| **sampled/** | Sampled-parameter scenarios (random / Halton smoke tests). |
| **calibration/** | One-off calibration scenarios (`measure_lgs_v1_centerline.scenic`, `F2_tactical.scenic`). |
| **dSPACE/** | dSPACE-specific demo scenarios (`art_control`, `constant_speed_fellow`, `relative_pos`, `three_tracks`, `ttl_fellow`). |

## Running

Pick a scenario and run it under the dSPACE racing model. Memory: pass `--count 1` to avoid wasteful multi-sample generation.

```powershell
scenic examples/racing/f_shared/F1_fellow_behind_optimal_cruise.scenic --2d --simulate --count 1 -b
```

## Benchmarks (regression sweeps)

The runner registry lives in [`src/scenic/domains/racing/README.md`](../../src/scenic/domains/racing/README.md). The two you usually want:

**F-bank regression — all F-scenarios with the full smart-ego stack:**

```powershell
python -m scenic.domains.racing.benchmarks.full_stack_runner
# subset:
python -m scenic.domains.racing.benchmarks.full_stack_runner --scenario F1_fellow_behind_optimal_cruise.scenic --repeats 1
```

**Falsification sweep — Scenic-ego or ART-ego on a `VerifaiRange` template:**

```powershell
# Scenic-controlled smart ego (the SD-44 baseline used 15-sample Halton)
python -m scenic.domains.racing.benchmarks.verifai_runner `
    examples/racing/falsifiable/S2_falsify.scenic `
    --scenic-control --sampler halton --monitor min `
    --count 15 --seed 42 --time 3000 --label report_s2

# ART-controlled ego (paper comparison)
python -m scenic.domains.racing.benchmarks.verifai_runner `
    examples/racing/falsifiable/S2_falsify.scenic `
    --no-scenic-control --sampler halton --monitor min `
    --count 15 --seed 42 --time 3000 --label report_s2_art
```

Outputs land in `src/scenic/domains/racing/benchmarks/results/<label>_<timestamp>/` (gitignored).

## See also

- [`src/scenic/domains/racing/README.md`](../../src/scenic/domains/racing/README.md) — full runner registry + control contract
- [`docs/falsification_pipeline.md`](../../docs/falsification_pipeline.md) — verifai-runner pipeline reference
- [`docs/racing_smart_driving.md`](../../docs/racing_smart_driving.md) — smart-ego architecture (SD-* cycle)
- [`docs/frames.md`](../../docs/frames.md) — coordinate frames + track elevation
