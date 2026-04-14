# Phase 7: Fellow Next-Step Prediction

## Prerequisites (handoff from Phase 6)

Phase 6 established the layered runtime path and per-cycle observability.
Phase 7 now introduces the first new decision-quality capability: predicting fellow
next-step motion from pose history.

This phase **adds** prediction quality and prediction metrics; it does **not** yet
change pass lifecycle semantics.

## Current Status

**Planned** (prediction module and benchmarked forecast accuracy).

- Source architecture intent: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed criteria: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

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
  - hold-last-pose

## Why It Matters

Phases 8-12 depend on tactical decisions made from short-horizon expectations, not
single-frame geometry only.

## Success Criteria

In cruise scenarios (`F2`, `F6`, `F7`):

- prediction error remains small and stable.

In disturbance scenarios (`F4`, `F5`):

- predictor responds to stop/swerve transitions better than naive baseline.

Global:

- prediction runs every cycle,
- errors are bounded and reported per scenario.

## Required Telemetry (Phase 7)

- `fellow_pred_x`
- `fellow_pred_y`
- `fellow_pred_s`
- `prediction_error_next_step`
- optional comparator fields:
  - `prediction_error_zero_motion`
  - `prediction_error_hold_last`

## Benchmark / Scenario Guidance

Use scenarios that cover both steady-state and disturbance:

- `F2` ahead slower cruise on optimal
- `F4` sudden stop onset
- `F5` out-of-control swerve + stop
- `F6` deterministic left occupancy cruise
- `F7` deterministic right occupancy cruise

Runner guidance (placeholder naming convention):

```bash
python -m scenic.domains.racing.benchmarks.phase7_runner --time 2000
```

Scenario summary should include:

- average next-step position error
- max next-step position error
- baseline-comparison deltas

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/prediction/fellow_predictor.py`
- `src/scenic/domains/racing/prediction/` history and model helpers
- planner/assessment integration points that consume prediction outputs
- benchmark metric parsing updates for prediction-error fields

## Exit Checklist

- [ ] Predictor runs every cycle on all phase scenarios.
- [ ] Prediction output fields are present and parseable in logs.
- [ ] Error summaries are generated per scenario.
- [ ] Predictor outperforms zero-motion or hold-last baseline on dynamic case (`F5`).
- [ ] Run artifacts and known limits are documented.

## Handoff to Phase 8

Proceed to [Phase 8: Situation assessment and dynamic gap](./phase-8-situation-assessment-and-dynamic-gap.md):
consume current + predicted state to classify relation/closure and corridor openness,
then compute speed-sensitive follow safety margins.
