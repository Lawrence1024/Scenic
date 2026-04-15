# Phase 8: Situation Assessment and Dynamic Gap

## Prerequisites (handoff from Phase 7)

Phase 7 provides per-cycle fellow predictions with explicit error metrics.
Phase 8 converts current + predicted state into tactical facts and follow-safety
constraints.

This phase **adds** assessment semantics and dynamic safety gap logic; it does **not**
yet introduce commit/abort pass states.

## Current Status

**Implemented (ready for Phase 9 handoff)** — phase-8 assessment module,
ego log wiring, benchmark parser/runner, and stateful relation/risk semantics.

- Assessment module: `src/scenic/domains/racing/assessment/race_situation.py`
- Ego wiring/logging: `src/scenic/domains/racing/behaviors.scenic` (`[Phase8Assessment]`)
- Benchmark parser/runner: `src/scenic/domains/racing/benchmarks/phase_run_common.py`,
  `src/scenic/domains/racing/benchmarks/phase8_runner.py`
- Latest validation record: `phase8_20260415_071936` (`summary.json` / digest in run artifacts)
- Tightening pass (design-level, not parameter tuning):
  - stateful relation hysteresis using `delta_s` semantics
  - emergency risk decomposition using gap pressure + TTC pressure + overlap dominance
  - short risk latch to reduce frame-level flicker

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

Phase 8 parsing uses the startup filter from `phase_run_common.py`
(`--analysis-ignore-before-s`, default `1.0`).

Determinism note:

- Benchmark scenarios are treated as deterministic for acceptance review.
- Re-running the same scenario set without code/config changes is not expected to change
  outcomes; only rerun after implementation changes or explicit environment changes.

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

- [x] Assessment outputs are emitted every cycle on all Phase 8 scenarios.
- [x] Dynamic `safe_gap` grows with ego speed in logs.
- [x] Corridor occupancy labels match scenario truth (`F6`/`F7` symmetry).
- [x] Flicker/chatter behavior is bounded and documented.
- [x] Scenario results and caveats are recorded with run artifacts.

## Observed Results (Phase8Runner `phase8_20260415_071936`)

From the latest deterministic run on `F1`, `F2`, `F4`, `F6`, `F7` with
`--analysis-ignore-before-s 1.0`:

- **Pipeline health**
  - all scenarios returned `rc=0`
  - `phase8_assessment_line_count=280` for each scenario
- **F1 (behind cruise)**
  - relation behavior aligns: `behind_count=280`, `ahead_count=0`
  - `gap_ok_rate=1.0`, corridor openness all `1.0`
- **F2 / F4 (ahead + disturbance)**
  - dynamic gap active (`safe_gap_mean` around `36–37m`)
  - `gap_ok_rate` drops below `1.0` as expected under closing pressure
  - relation counts are mixed over the full run because ego eventually overtakes
    after contact; this is expected in these deterministic traces and should be
    interpreted with event timing, not only full-run ahead/behind totals
- **F6 / F7 (left/right occupancy symmetry)**
  - F6: `left_open_rate=0.804`, `right_open_rate=1.0`
  - F7: `right_open_rate=0.796`, `left_open_rate=1.0`
  - symmetry intent appears directionally correct

## Residual Caveats (carried into Phase 9 work)

- **Emergency-risk separation risk**
  - `phase8_emergency_risk_mean` for `F4` is close to `F2`, so stop-onset scenarios may not
    be distinguished strongly enough from cruise-follow scenarios.
- **Safety-outcome non-closure risk**
  - collisions remain in `F2`, `F4`, and `F7` (`collision_eval_hull_overlap=true`), even though
    Phase 8’s primary objective is assessment quality rather than collision elimination.
- **Interpretation caveat**
  - For `F2`/`F4`, full-run ahead/behind totals are not sufficient acceptance evidence;
    use timestamped relation + eval-event timing windows to account for overtake after contact.

## Handoff to Phase 9

Proceed to [Phase 9: Tactical planner v1](./phase-9-tactical-planner-v1.md):
use assessment outputs to choose `FREE_RUN`, `FOLLOW`, `SETUP_PASS_LEFT`,
`SETUP_PASS_RIGHT` with clear reason strings and stable transitions.
