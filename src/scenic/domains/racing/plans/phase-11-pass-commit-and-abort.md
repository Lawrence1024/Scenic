# Phase 11: Pass Commit and Abort

## Prerequisites (handoff from Phase 10)

Phase 10 provides guard-level stability enforcement and emergency containment.
Phase 11 adds explicit pass lifecycle states and triggers while retaining those
safety constraints.

This phase **adds** commit/abort tactical execution; it should not weaken Phase 10
stability protections.

## Current Status

**Planned** (explicit overtake lifecycle with deterministic success/abort evidence).

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed scenario intent: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

## Goal

Enable robust overtake execution with clear commit and abort decision paths that remain
safe under sudden stop and swerve disruptions.

## What to Build

Extend tactical state machine with:

- `COMMIT_PASS_LEFT`
- `COMMIT_PASS_RIGHT`
- `ABORT_PASS`

Add explicit lifecycle triggers:

- commit trigger conditions
- abort trigger conditions
- post-event recovery state selection

Keep active guard integration:

- unsafe commits are blocked or aborted
- disruption-driven invalidation enters `ABORT_PASS` or `EMERGENCY_STABLE`

## Why It Matters

Without explicit lifecycle control, setup behavior cannot become reliable race action,
and disrupted routes tend to produce late unsafe persistence.

## Success Criteria

Scenario expectations:

- `F2`: when a corridor stays open, ego can eventually commit and complete safely.
- `F6`: occupied-left case should favor right-side commit chain.
- `F7`: symmetric occupied-right case should favor left-side commit chain.
- `F4`: sudden stop should prevent forced bad commit; route invalidation triggers safe fallback.
- `F5`: swerve invalidation triggers clean abort or emergency stabilization.

Outcome expectations:

- deterministic cruise cases (`F6`/`F7`) show repeated successful bypasses.
- disruption cases (`F4`/`F5`) show repeated safe abort outcomes.

## Required Telemetry (Phase 11)

- `commit_trigger`
- `abort_trigger`
- `pass_success`
- `abort_success`
- `post_event_state`
- optional chain diagnostics:
  - commit latency
  - abort latency
  - aborted-commit ratio

## Benchmark / Scenario Guidance

Recommended set:

- `F2`, `F4`, `F5`, `F6`, `F7`

Runner guidance:

```bash
python -m scenic.domains.racing.benchmarks.phase11_runner --time 1000
```

Expected chain examples:

- `F6`: `FOLLOW -> SETUP_PASS_RIGHT -> COMMIT_PASS_RIGHT -> FREE_RUN`
- `F7`: `FOLLOW -> SETUP_PASS_LEFT -> COMMIT_PASS_LEFT -> FREE_RUN`

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/tactical_planner.py` (state extensions)
- pass lifecycle helper logic under `src/scenic/domains/racing/`
- `src/scenic/domains/racing/behaviors.scenic` (planner/guard execution integration)
- guard/planner integration for forced abort and emergency fallback
- benchmark parsing for commit/abort KPIs

## Exit Checklist

- [ ] Commit and abort states are explicit, reachable, and unit-tested.
- [ ] Trigger reasons are logged and consistent with scenario events.
- [ ] Deterministic cruise scenarios show reproducible bypass success.
- [ ] Disruption scenarios show safe abort behavior instead of bad-commit persistence.
- [ ] No collisions are attributable to delayed abort of invalid routes.

## Handoff to Phase 12

Proceed to [Phase 12: Segment-aware tactical intelligence](./phase-12-segment-aware-tactical-intelligence.md):
add segment-conditioned timing/shaping so commit decisions improve on straights and
become more conservative at corner entry while preserving Phase 11 safety.
