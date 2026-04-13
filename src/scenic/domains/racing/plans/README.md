# Opponent-Aware Ego Planner Plan Set

This folder breaks the master roadmap in `overall_plan.md` into actionable per-phase documents for building an opponent-aware ego planner that selects `optimal / left / right` TTLs and feeds the existing ego MPC, while keeping current pit handling unchanged.

**Scope:** The implemented planner and benchmark banks target **one dynamic opponent** (plus ego). General multi-opponent racing is **not** part of the current roadmap.

## Roadmap status

- **Phase 0** — complete (baseline metrics, scenario bank, `phase0_runner`).
- **Phase 1** — complete (scripted TTL schedule + MPC handoff, `phase1_runner`, three validated switches).
- **Phase 2** — complete (situation assessment module, snapshot tests, `[Phase2]` logs).
- **Phase 3** — complete (tactical planner + `tactical_planner_enabled`; Phase 0–aligned bank via `phase3_runner` on `examples/racing/phase3_tactical/`, `BENCHMARK_AI_DIGEST` / `summary.json`). See [Phase 3 plan](./phase-3-smart-follow-and-stable-ttl.md#validated-benchmarks-dspace).
- **Phase 4** — complete (pass commit/abort/shield in `pass_commit_shield.py` + `pass_commit_shield_enabled`; seven-scenario bank in `examples/racing/phase4_pass_shield/` validated via `phase4_runner`); [plan](./phase-4-pass-commit-abort-and-shield.md).
- **Phase 5** — complete (segment-aware shaping in `phase5_segment_tactics.py` + `phase5_segment_tactics_enabled`; benchmark bank `examples/racing/phase5_segments/` including `07`–`08` corner cases and `09`–`10` straight-opening symmetry; `phase5_runner` + digest KPIs). Validated run record and follow-ups: [Phase 5 plan](./phase-5-segment-aware-tactics.md#validated-benchmarks-record).

## Phase Plans

- [Phase 0: Baseline and visibility](./phase-0-baseline-and-visibility.md)
- [Phase 1: Planner-MPC integration](./phase-1-planner-mpc-integration.md)
- [Phase 2: Situation assessment](./phase-2-situation-assessment.md)
- [Phase 3: Smart follow and stable TTL choice](./phase-3-smart-follow-and-stable-ttl.md)
- [Phase 4: Pass commit/abort and safety shield](./phase-4-pass-commit-abort-and-shield.md)
- [Phase 5: Segment-aware tactics](./phase-5-segment-aware-tactics.md)

## Supporting Documents

- [Deferred scope](./deferred-scope.md)
- [Success definition](./success-definition.md)
- [Comprehensive planner validation & stress-test campaign](./comprehensive-planner-validation-runner.md) — post–Phase 5 full-stack testing; **runner:** `python -m scenic.domains.racing.benchmarks.validation_full_stack_runner`
- Fellow / traffic harness (placement + `[FellowHarness]` readback, `fellow_runner`): see [fellow_smoke README](../../../../../examples/racing/fellow_smoke/README.md).

**Benchmark runners:** New `*.scenic` files under each phase’s example folder are picked up automatically by that phase’s runner (no filename list in code). Runners print a **`BENCHMARK_AI_DIGEST_*`** JSON block plus `summary.json` / `summary.csv` under `benchmarks/results/<run_id>/`, and after each scenario a **`Log file:`** line with the absolute path to that run’s captured stdout/stderr (under `results/<run_id>/logs/<stem>.log`). Default simulation length is **2000** steps (~20 s at 0.01 s/step) unless overridden (pass ``--time 3000`` for ~30 s). When extending phases 4–5, revisit the runner and `phase_run_common.collect_metrics_from_log` for new KPI columns and log parsers—see [Racing examples README](../../../../../examples/racing/README.md) (sections **Sharing benchmark output** and **Phases 4–5**).

## Execution Order Checklist

- [x] Complete Phase 0 metrics and benchmark scenarios.
- [x] Complete Phase 1 planner-to-MPC integration plumbing.
- [x] Complete Phase 2 opponent-state interpreter.
- [x] Complete Phase 3 conservative tactical behavior (code, unit tests, and Phase 0 bank cross-check on dSPACE).
- [x] Complete Phase 4 commit/abort overtaking with safety shield (logic, scenario bank, and dSPACE sign-off run completed).
- [x] Complete Phase 5 segment-aware tactical improvements (implementation + Phase 5 bank; see [Phase 5 validated record](./phase-5-segment-aware-tactics.md#validated-benchmarks-record)).

## Notes

- This split is organizational only; the source intent remains in `overall_plan.md`.
- Pit logic redesign remains out of scope for these phases.
- Baseline/off-track threshold calibration from Phase 0 remains an optional follow-up; Phase 0 and Phase 1 are complete without it.
