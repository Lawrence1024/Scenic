# Phase 8: Situation Assessment and Dynamic Gap

## Prerequisites (handoff from Phase 7)

Phase 7 provides per-cycle fellow predictions with explicit error metrics.
Phase 8 converts current + predicted state into tactical facts and follow-safety
constraints.

This phase **adds** assessment semantics and dynamic safety gap logic; it does **not**
yet introduce commit/abort pass states.

## Current Status

**Planned** (assessment expansion for planner-ready tactical facts).

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed scenario truth map: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

## Goal

Generate stable and explainable opponent relation/corridor facts plus speed-sensitive
safe following distance for downstream tactical decisions.

## What to Build

- Relation classification:
  - ahead / behind
  - closing / not closing
  - overlap / no-overlap
- Dynamic safe-gap computation:
  - speed + time-headway baseline in this phase
  - braking-aware extension may follow later
- Corridor openness outputs:
  - `optimal_open`
  - `left_open`
  - `right_open`
- Gap status signal:
  - `gap_ok` from `actual_gap` vs `safe_gap`

## Why It Matters

Planner decisions without stable relation and corridor signals will chatter, make poor
setup choices, and produce fragile safety behavior.

## Success Criteria

Scenario truth alignment:

- `F1`: relation should remain `behind` most of the run.
- `F2`: relation `ahead`; `safe_gap` scales with speed; `gap_ok` flips when needed.
- `F6`: left corridor should be blocked/penalized.
- `F7`: right corridor should be blocked/penalized.
- `F4`: stop onset should raise emergency-risk indicators.

Stability:

- labels remain consistent without pathological frame-to-frame flicker.

## Required Telemetry (Phase 8)

- `fellow_relation`
- `closing_flag`
- `actual_gap`
- `safe_gap`
- `gap_ok`
- `optimal_open`
- `left_open`
- `right_open`
- optional confidence/hysteresis diagnostics for flicker tracking

## Benchmark / Scenario Guidance

Recommended validation set:

- `F1`, `F2`, `F4`, `F6`, `F7`

Runner guidance (placeholder naming convention):

```bash
python -m scenic.domains.racing.benchmarks.phase8_runner --time 2000
```

Expected-label table should be part of run review:

- `F1 -> behind`
- `F2 -> ahead/follow candidate`
- `F6 -> left occupied`
- `F7 -> right occupied`
- `F4 -> emergency risk rises after stop onset`

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/assessment/race_situation.py`
- dynamic-gap helper(s) in `src/scenic/domains/racing/assessment/`
- integration points where planner consumes assessment outputs
- benchmark parsers for Phase 8 telemetry columns

## Exit Checklist

- [ ] Assessment outputs are emitted every cycle on all Phase 8 scenarios.
- [ ] Dynamic `safe_gap` grows with ego speed in logs.
- [ ] Corridor occupancy labels match scenario truth (`F6`/`F7` symmetry).
- [ ] Flicker/chatter behavior is bounded and documented.
- [ ] Scenario results and caveats are recorded with run artifacts.

## Handoff to Phase 9

Proceed to [Phase 9: Tactical planner v1](./phase-9-tactical-planner-v1.md):
use assessment outputs to choose `FREE_RUN`, `FOLLOW`, `SETUP_PASS_LEFT`,
`SETUP_PASS_RIGHT` with clear reason strings and stable transitions.
