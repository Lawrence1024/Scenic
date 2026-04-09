# Phase 0: Baseline and Visibility

## Current Status

**Phase 0 is closed** (instrumentation, scenario bank, and exit checklist).

- Scenario bank `00..06` is in place and executable.
- Runner emits automatic per-run artifacts (`summary.json`, `summary.csv`, per-scenario logs).
- Scenario filtering and inter-run delay controls are available in the runner (`--inter-run-delay-s`, default 15 s).
- Full-bank runs complete cleanly with consistent lap metrics (e.g. `phase0_20260409_155011`).
- Fine-grained **off-track / near-miss threshold tuning** remains an optional follow-up if heuristics need tightening; it does not block later phases.

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
- [x] Baseline output is stable across repeated runs. (validated: full bank completes with repeatable lap times and no flaky failures; heuristic calibration still optional)

## Handoff

Phase 0 infrastructure supported Phase 1 (planner-to-MPC integration), which is now complete—see [Phase 1](./phase-1-planner-mpc-integration.md).  
Next: [Phase 2: Situation assessment](./phase-2-situation-assessment.md).

Optional follow-up: revisit off-track / near-miss thresholds if logs show false negatives or noisy events in new scenarios.
