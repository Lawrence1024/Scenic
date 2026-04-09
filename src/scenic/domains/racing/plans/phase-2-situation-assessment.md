# Phase 2: Situation Assessment

## Current Status

**Implemented** (interpreter module, unit snapshot tests, `[Phase2]` runtime log from `FollowRacingLineMPCBehavior`).

- Code: `src/scenic/domains/racing/situation_assessment.py`
- Tests: `src/scenic/domains/racing/mpc/testing/test_situation_assessment.py`
- Usage notes: `examples/racing/phase2_assessment/README.md`

Validation: run pytest (snapshots) and any Phase 0 opponent scenario to inspect `[Phase2]` lines in the log.

## Goal

Build a stable opponent-state interpreter that converts raw geometry into race-state features for planning.

## What to Build

For one opponent, compute:

- ahead/behind relation
- relative progress `Δs`
- relative lateral relation
- closing speed
- overlap state:
  - clear behind
  - closing behind
  - partial overlap
  - side-by-side
  - clear ahead
- short-horizon collision risk
- segment context:
  - straight
  - corner entry
  - corner body
  - corner exit

This phase provides inputs only; it does not yet decide or execute overtakes.

## Why It Matters

Planner decisions should be based on race-state semantics, not only raw positions.

## Success Criteria

Labeled scenario snapshots classify correctly and stably, including:

- opponent 10 m ahead and slower
- ego overlapping on right
- corner entry with opponent ahead
- side-by-side on straight

No frame-to-frame flicker in simple/static cases.

## Exit Checklist

- [x] Interpreter outputs all required features each cycle (computed on the same cadence as the Phase 0 telemetry block; see `[Phase2]` log fields).
- [x] Snapshot test set exists with expected labels (`test_situation_assessment.py`).
- [x] Classification is stable under small measurement noise (lateral param sweep + overlap hysteresis).
- [x] Outputs are ready for planner consumption in next phase (`OpponentSituation` + `assess_nearest_opponent` API).

## Handoff to Phase 3

Phase 3 can call `assess_nearest_opponent` (or reuse its rules) to choose conservative TTL / speed caps before wiring into the Phase 1 planner inputs on `FollowRacingLineMPCBehavior`.
