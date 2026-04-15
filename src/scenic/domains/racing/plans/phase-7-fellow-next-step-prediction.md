# Phase 7: Fellow Next-Step Prediction

## Prerequisites (handoff from Phase 6)

Phase 6 established the layered runtime path and per-cycle observability.
Phase 7 introduces the first new decision-quality capability: predicting fellow
next-step motion from pose history.

This phase **adds** prediction quality and prediction metrics; it does **not** yet
change pass lifecycle semantics.

## Current Status

**Implemented (clean sign-off)** — recency-weighted one-step CV predictor,
baselines, ego logging, and benchmark gating.

- Predictor: `src/scenic/domains/racing/prediction/fellow_predictor.py`
- Ego wiring: `FollowRacingLineMPCBehavior(..., phase7_prediction_enabled=True)` (or global `param phase7_prediction_enabled` in `examples/racing/f_shared/*.scenic`)
- Unit tests: `src/scenic/domains/racing/mpc/testing/test_fellow_predictor.py`
- Runner: `src/scenic/domains/racing/benchmarks/phase7_runner.py` (passes `-p phase7_prediction_enabled True` via `PhaseRunnerSpec.scenic_extra_args`)
- Scenario subset: `PHASE7_F_SCENARIO_NAMES` in `f_scenario_bank.py` (`F2`, `F4`, `F5`, `F6`, `F7`)
- Log parsing / CSV: `collect_metrics_from_log` in `phase_run_common.py`
  (`phase7_prediction_*` aggregates; default startup filter
  `--analysis-ignore-before-s=1.0`)

## Goal

Replace purely reactive opponent handling with bounded-error next-step estimates that
can be consumed by assessment and planning.

## What to Build

- Implement fellow motion history buffering.
- Implement next-step predictor outputs (minimum):
  - predicted position (`x`, `y`)
  - predicted progress (`s`) or heading
- Emit per-cycle prediction error metric against realized next-step pose.
- Compare predictor against simple baselines:
  - zero-motion
  - hold-last velocity (CV to current time — same family as the main model for two-point history)

## Why It Matters

Phases 8-12 depend on tactical decisions made from short-horizon expectations, not
single-frame geometry only.

## Success Criteria

In cruise scenarios (`F2`, `F6`, `F7`):

- prediction error remains small and stable (track mean / max from logs).

In disturbance scenarios (`F4`, `F5`):

- predictor responds to stop/swerve transitions; compare mean errors vs baselines on the same logs.
- required baseline outcome for sign-off:
  - strong gain vs zero-motion (`phase7_prediction_gain_vs_zero_mean > 0`)
  - bounded regret vs hold-last (`phase7_prediction_ratio_vs_hold_mean <= 1.30`)

Global:

- prediction runs on ego full-control steps when enabled,
- errors are bounded and reported per scenario via benchmark CSV / digest.

## Required Telemetry (Phase 7)

Log prefix: **`[Phase7Prediction]`** (one line per full-control step when a nearest fellow exists; predictor reset when no opponent).

Fields:

- `fellow_pred_x`, `fellow_pred_y`, `fellow_pred_s` (optional / `na` if unavailable)
- `prediction_error_next_step` (vs previous-tick forecast)
- `prediction_error_zero_motion`
- `prediction_error_hold_last`

Example:

```text
[Phase7Prediction] t=12.50s fellow_pred_x=101.2345 fellow_pred_y=-220.1000 fellow_pred_s=na prediction_error_next_step=0.0421 prediction_error_zero_motion=0.1100 prediction_error_hold_last=0.0380
```

## Benchmark / Scenario Guidance

Scenarios:

- `F2` ahead slower cruise on optimal
- `F4` sudden stop onset
- `F5` out-of-control swerve + stop
- `F6` deterministic left occupancy cruise
- `F7` deterministic right occupancy cruise

Runner:

```bash
python -m scenic.domains.racing.benchmarks.phase7_runner --time 1000
```

Runtime policy:

- Use **10 s default** (`--time 1000`) for iteration.
- Use **15 s max** (`--time 1500`) for confirmation.

This enables prediction via CLI override: `-p phase7_prediction_enabled True` (wired through `scenic_extra_args`).

Scenario summary (`summary.csv`) includes:

- `phase7_prediction_line_count`
- `phase7_prediction_error_next_step_mean` / `_max`
- `phase7_prediction_error_zero_motion_mean`
- `phase7_prediction_error_hold_last_mean`
- `phase7_prediction_gain_vs_zero_mean`
- `phase7_prediction_regret_vs_hold_mean`
- `phase7_prediction_ratio_vs_hold_mean`

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/prediction/fellow_predictor.py`
- `src/scenic/domains/racing/behaviors.scenic` — Phase 7 block (ego + nearest fellow, after shared nearest-object scan with Phase 6)
- `src/scenic/domains/racing/benchmarks/phase_run_common.py` — regex + aggregates
- `examples/racing/f_shared/*.scenic` — `param phase7_prediction_enabled = False` and behavior argument

## Exit Checklist

- [x] Predictor runs on ego full-control steps when enabled and a fellow is present.
- [x] Prediction output fields are present and parseable in logs (`[Phase7Prediction]`).
- [x] Error summaries are generated per scenario (benchmark CSV + digest keys).
- [x] Predictor beats zero-motion and stays within bounded regret vs hold-last
  on target scenarios under startup-filtered analysis (`t >= 1.0s`) —
  **quantitative gate** using `phase7_prediction_gain_vs_zero_mean` and
  `phase7_prediction_ratio_vs_hold_mean`.
- [x] Run artifacts and known limits documented for pinned `phase7_*` run ids.

## Handoff to Phase 8

Proceed to [Phase 8: Situation assessment and dynamic gap](./phase-8-situation-assessment-and-dynamic-gap.md):
consume current + predicted state to classify relation/closure and corridor openness,
then compute speed-sensitive follow safety margins.
