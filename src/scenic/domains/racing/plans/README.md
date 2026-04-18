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
- **Phase 6** — complete as orchestration/observability baseline (phase6 orchestration shells + per-cycle `[Phase6*]` logs, shared F-bank `examples/racing/f_shared/`). Note: `phase6_runner` was not carried forward; use `phase7_runner` or `full_stack_runner` for F-bank runs.
- **Phase 7** — complete (recency-weighted one-step fellow prediction + prediction-error benchmarking, startup-filtered analysis default `t>=1.0s`).
- **Phase 8** — complete (assessment + dynamic-gap telemetry ready for tactical-planner handoff; residual caveats documented in Phase 8 plan).
- **Phase 9** — complete as tactical baseline (Phase-8-informed tactical planner path + `[Phase9Planner]` telemetry + `phase9_runner`; legacy tactical overlays retired in Phase 9 authority mode; setup-pass intent consistently observed in occupancy scenarios. Full pass commit/abort lifecycle intentionally deferred to Phase 11).
- **Phase 10** — complete as stability baseline (`[Phase10Guard]` guard path, emergency-stable containment, and post-emergency re-approach suppression wired in `phase10_runner`; validated clean on `F2/F4/F5/F6/F7` at 15 s windows in deterministic runs).
- **Phase 11** — complete (explicit commit/abort lifecycle in tactical planner, `[Phase11Planner]` telemetry, and `phase11_runner` benchmark path; F4/F5 safe abort and F6/F7 pass success validated; closing+emergency-risk structural guards added).
- **Phase 12** — skeleton in place (`segment_aware_enabled` flag + `segment_modifier` state field in `tactical_planner.py`; `phase12_runner` exists); full segment-conditioned commit gating not yet implemented.

## Phase Plans

- [Phase 0: Baseline and visibility](./phase-0-baseline-and-visibility.md)
- [Phase 1: Planner-MPC integration](./phase-1-planner-mpc-integration.md)
- [Phase 2: Situation assessment](./phase-2-situation-assessment.md)
- [Phase 3: Smart follow and stable TTL choice](./phase-3-smart-follow-and-stable-ttl.md)
- [Phase 4: Pass commit/abort and safety shield](./phase-4-pass-commit-abort-and-shield.md)
- [Phase 5: Segment-aware tactics](./phase-5-segment-aware-tactics.md)
- [Phase 6: Architecture skeleton and observability](./phase-6-architecture-skeleton-and-observability.md)
- [Phase 7: Fellow next-step prediction](./phase-7-fellow-next-step-prediction.md)
- [Phase 8: Situation assessment and dynamic gap](./phase-8-situation-assessment-and-dynamic-gap.md)
- [Phase 9: Tactical planner v1](./phase-9-tactical-planner-v1.md)
- [Phase 10: Stability guard and emergency policy](./phase-10-stability-guard-and-emergency-policy.md)
- [Phase 11: Pass commit and abort](./phase-11-pass-commit-and-abort.md)
- [Phase 12: Segment-aware tactical intelligence](./phase-12-segment-aware-tactical-intelligence.md)

## Supporting Documents

- [Deferred scope](./deferred-scope.md)
- [Success definition](./success-definition.md)
- [Comprehensive planner validation & stress-test campaign](./comprehensive-planner-validation-runner.md) — post–Phase 5 full-stack testing; **runner:** `python -m scenic.domains.racing.benchmarks.validation_full_stack_runner`
- [Phase 6-12 master rollout](./phase-6-12-master-rollout.md) — orchestration document for layering, sequencing, and acceptance gates after Phase 5.
- Fellow / traffic harness (placement + `[FellowHarness]` readback, `fellow_runner`): see [fellow_smoke README](../../../../../examples/racing/fellow_smoke/README.md).

**Benchmark runners:** New `*.scenic` files under each phase’s example folder are picked up automatically by that phase’s runner (no filename list in code). Runners print a **`BENCHMARK_AI_DIGEST_*`** JSON block plus `summary.json` / `summary.csv` under `benchmarks/results/<run_id>/`, and after each scenario a **`Log file:`** line with the absolute path to that run’s captured stdout/stderr (under `results/<run_id>/logs/<stem>.log`).

Available runners (under `benchmarks/`): `phase0_runner`, `phase1_runner`, `phase2_runner`, `phase3_runner`, `phase3_on_phase0_runner`, `phase7_runner` – `phase12_runner`, `full_stack_runner` (all F-scenarios, complete intelligence stack), `validation_full_stack_runner` (stress/validation campaign), `validation_phase6_12_runner` (phase 6-12 regression), `fellow_runner` (fellow harness smoke), `fellow_placement_debug_runner`. Note: `phase4_runner`, `phase5_runner`, and `phase6_runner` were not carried forward into the new architecture.
**Runtime policy (required):** keep benchmark windows short. Use **10 s default** (`--time 1000`) and **15 s max** (`--time 1500`) for phase bring-up/regression checks. Do not run 20 s+ by default; only exceed 15 s with an explicit, documented reason in the phase note or PR.
**Wall-clock expectation (required):** simulation runs at about **11x real-time cost** on this setup (about **11 seconds wall-clock per 1 simulated second**). A **20 s** simulation therefore takes about **220 seconds** wall-clock. This is expected behavior; even small simulated-time changes can create large real-time runtime differences.
**Execution workflow (required):** the assistant should **not** run simulations directly. When simulation evidence is needed, the assistant provides the exact command and the user runs it; analysis proceeds from the returned digest/logs.
When extending phases 4–5, revisit the runner and `phase_run_common.collect_metrics_from_log` for new KPI columns and log parsers—see [Racing examples README](../../../../../examples/racing/README.md) (sections **Sharing benchmark output** and **Phases 4–5**).

## Execution Order Checklist

- [x] Complete Phase 0 metrics and benchmark scenarios.
- [x] Complete Phase 1 planner-to-MPC integration plumbing.
- [x] Complete Phase 2 opponent-state interpreter.
- [x] Complete Phase 3 conservative tactical behavior (code, unit tests, and Phase 0 bank cross-check on dSPACE).
- [x] Complete Phase 4 commit/abort overtaking with safety shield (logic, scenario bank, and dSPACE sign-off run completed).
- [x] Complete Phase 5 segment-aware tactical improvements (implementation + Phase 5 bank; see [Phase 5 validated record](./phase-5-segment-aware-tactics.md#validated-benchmarks-record)).
- [x] Phase 6: layer extraction and per-cycle observability path active (`phase6_orchestration_enabled`, `[Phase6State]/[Phase6Planner]/[Phase6Guard]/[Phase6Executor]`, shared `f_shared` bank + `phase6_runner`).
- [x] Phase 7: next-step fellow prediction with bounded and benchmarked error.
- [x] Phase 8: stable tactical assessment outputs and dynamic safe gap.
- [x] Phase 9: tactical planner v1 setup-pass behavior with bounded switching (frozen tactical baseline; commit/abort intentionally deferred).
- [x] Phase 10: guard-driven stability controls and emergency handling (validated baseline on `F2/F4/F5/F6/F7`).
- [x] Phase 11: commit/abort lifecycle with deterministic success/abort evidence (validated: F4/F5 safe abort, F6/F7 pass success, no collisions).
- [ ] Phase 12: segment-aware decision timing improvements with safety non-regression (skeleton in place; full implementation pending).

## Notes

- This split is organizational only; the source intent remains in `overall_plan.md`.
- Pit logic redesign remains out of scope for these phases.
- Baseline/off-track threshold calibration from Phase 0 remains an optional follow-up; Phase 0 and Phase 1 are complete without it.
- Setup state naming: code defines `SETUP_LEFT` / `SETUP_RIGHT` as the internal names and `SETUP_PASS_LEFT` / `SETUP_PASS_RIGHT` as aliases (both present in `tactical_planner.py`). Logs canonicalize to `SETUP_PASS_LEFT` / `SETUP_PASS_RIGHT` via `_canonical_mode()`. Plan documents use the canonical alias form.
