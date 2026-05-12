# Racing planner — historical plan archive

This directory previously held 18 phase-numbered plan documents covering the
Phase 0–12 development cycle that built up the opponent-aware ego planner.
Those documents were deleted in **CC-4** (cleanup cycle, 2026-04-26) along
with the phase numbering they referenced.

## Where the work lives now

- **Architecture / control-side cleanup**: see
  [`docs/cleanup_inventory.md`](../../../../docs/cleanup_inventory.md) for
  the CC-* cleanup (dead-code deletion, phase-number-to-descriptive-name
  renames). RC-* refactor notes that previously lived in
  `docs/racing_controller_cleanup.md` were folded into commit history.
- **Smart-driving (one opponent)**: see
  [`docs/racing_smart_driving.md`](../../../../docs/racing_smart_driving.md)
  for the SD-* cycle (assessment-side fixes that allow the tactical planner
  to attempt overtakes against a centered slow opponent).
- **Frame calibration history**: see
  [`docs/frames.md`](../../../../docs/frames.md) for the B5/B6 frame-migration
  (different "phase" concept — frames cycle, not control cycle).
- **Cleanup inventory**: [`docs/cleanup_inventory.md`](../../../../docs/cleanup_inventory.md)
  is the change log of what was deleted/renamed during CC-* and why.

## What "phase" used to mean (historical reference only)

| Old phase number | New name | What the phase built |
|---|---|---|
| Phase 0 | baseline | Benchmark scaffolding + ego-alone visibility metrics |
| Phase 1 | scripted | Scripted TTL schedule + MPC handoff |
| Phase 2 | opponent | Situation-assessment module (overlap state) |
| Phase 3 | tactical | Tactical planner v1 (FREE_RUN/FOLLOW/SETUP) |
| Phase 4 | shield | Pass commit/abort/shield (early version) |
| Phase 5 | segment | Segment-aware tactics |
| Phase 6 | orchestration | Architecture skeleton + observability |
| Phase 7 | prediction | Fellow next-step prediction |
| Phase 8 | assessment | Race-situation assessment + dynamic gap |
| Phase 9 | hazard / tactical-v1 | Tactical planner authority mode (uses hazard brake floor) |
| Phase 10 | guard | Stability guard + emergency policy |
| Phase 11 | commit | Pass commit + abort lifecycle (final version) |
| Phase 12 | segment-aware | Segment-conditioned commit gating |

The implementations live in:
- `src/scenic/domains/racing/tactical_planner.py`
- `src/scenic/domains/racing/assessment/race_situation.py`
- `src/scenic/domains/racing/prediction/fellow_predictor.py`
- `src/scenic/domains/racing/safety/stability_guard.py`
- `src/scenic/domains/racing/segments/`
- `src/scenic/domains/racing/situation_assessment.py`
- `src/scenic/domains/racing/behaviors.scenic` (orchestration)

The benchmarks live in `src/scenic/domains/racing/benchmarks/`. The current
runners are `full_stack_runner.py` (F-bank regression) and `verifai_runner.py`
(falsification sweeps). The per-phase runners that grew during the Phase 0–12
build cycle (baseline / scripted / opponent / tactical / prediction /
assessment / hazard / guard / commit_pass / segment_aware) plus the
validation orchestrators (`validation_full_stack`, `validation_orchestration`)
and fellow harnesses (`fellow_runner`, `fellow_placement_debug_runner`) were
removed once `full_stack_runner` superseded them; see commit history for
details.

The canonical scenario set is `examples/racing/f_shared/` (F0–F14 plus
F3L, F3R, F13c variants).
