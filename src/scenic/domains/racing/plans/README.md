# Opponent-Aware Ego Planner Plan Set

This folder breaks the master roadmap in `overall_plan.md` into actionable per-phase documents for building an opponent-aware ego planner that selects `optimal / left / right` TTLs and feeds the existing ego MPC, while keeping current pit handling unchanged.

## Phase Plans

- [Phase 0: Baseline and visibility](./phase-0-baseline-and-visibility.md)
- [Phase 1: Planner-MPC integration](./phase-1-planner-mpc-integration.md)
- [Phase 2: Situation assessment](./phase-2-situation-assessment.md)
- [Phase 3: Smart follow and stable TTL choice](./phase-3-smart-follow-and-stable-ttl.md)
- [Phase 4: Pass commit/abort and safety shield](./phase-4-pass-commit-abort-and-shield.md)
- [Phase 5: Segment-aware tactics](./phase-5-segment-aware-tactics.md)
- [Phase 6: Multi-opponent robustness and stability](./phase-6-multi-opponent-and-stability.md)

## Supporting Documents

- [Deferred scope](./deferred-scope.md)
- [Success definition](./success-definition.md)

## Execution Order Checklist

- [x] Complete Phase 0 metrics and benchmark scenarios.
- [ ] Complete Phase 1 planner-to-MPC integration plumbing.
- [ ] Complete Phase 2 opponent-state interpreter.
- [ ] Complete Phase 3 conservative tactical behavior.
- [ ] Complete Phase 4 commit/abort overtaking with safety shield.
- [ ] Complete Phase 5 segment-aware tactical improvements.
- [ ] Complete Phase 6 multi-opponent and long-run robustness.

## Notes

- This split is organizational only; the source intent remains in `overall_plan.md`.
- Pit logic redesign remains out of scope for these phases.
- Baseline/off-track threshold calibration from Phase 0 is tracked as a deferred follow-up and does not block Phase 1 start.
