# Phase 0: Baseline and Visibility

## Current Status

Phase 0 implementation is complete for instrumentation and benchmark execution.

- Scenario bank `00..06` is in place and executable.
- Runner emits automatic per-run artifacts (`summary.json`, `summary.csv`, per-scenario logs).
- Scenario filtering and inter-run delay controls are available in the runner.
- Baseline behavior tuning (off-track heuristic sensitivity) is intentionally deferred.

## Goal

Freeze baseline behavior and make performance/safety outcomes measurable and repeatable.

## What to Build

- Add logging for:
  - current TTL
  - planner mode
  - ego `s` and speed
  - opponent relative `Δs` and relative speed
  - line-switch events
  - collision/off-track/near-miss events
  - lap time
- Create a benchmark scenario bank:
  - no opponent
  - slower opponent on optimal
  - slower opponent on left
  - slower opponent on right
  - lightly weaving opponent
  - opponent just ahead into a corner
  - side-by-side start

## Why It Matters

Without a fixed benchmark and logs, later planner changes cannot be evaluated objectively.

## Success Criteria

Every benchmark scenario runs to completion and automatically produces:

- lap completion status
- lap time
- number of TTL switches
- minimum opponent distance
- collision yes/no
- off-track yes/no

## Exit Checklist

- [x] All required metrics are logged per run.
- [x] Scenario bank exists and runs non-interactively.
- [x] Auto-generated metrics report is produced for every scenario.
- [ ] Baseline output is stable across repeated runs. (deferred follow-up)

## Handoff to Phase 1

Phase 0 infrastructure is sufficient to proceed to Phase 1 (planner-to-MPC integration).  
Keep a follow-up item to revisit baseline/off-track event calibration after Phase 1 plumbing is in place.
