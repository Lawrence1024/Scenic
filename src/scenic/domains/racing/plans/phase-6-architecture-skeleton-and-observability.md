# Phase 6: Architecture Skeleton and Observability

## Prerequisites (handoff from Phase 5)

Phase 5 is **closed** for segment-aware tactics on top of tactical + pass-shield behavior.
Phase 6 starts the next roadmap by restructuring internals while preserving the same
behavior envelope.

This phase **adds** layered orchestration and telemetry contracts; it does **not** add
new pass intelligence yet.

## Current Status

**Implemented** — architecture shells, ego integration, shared F-bank benchmarks, and per-cycle telemetry.

- Source architecture intent: `src/scenic/domains/racing/restrcture_plan.md`
- Execution guidance: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`
- Runtime shell module: `src/scenic/domains/racing/phase6_runtime.py`
- Ego wiring: `FollowRacingLineMPCBehavior(..., phase6_orchestration_enabled=True)` in `src/scenic/domains/racing/behaviors.scenic`
- Unit tests: `src/scenic/domains/racing/mpc/testing/test_phase6_runtime.py`
- Runner + shared bank: `src/scenic/domains/racing/benchmarks/phase6_runner.py`,
  `examples/racing/f_shared/` (canonical scenarios `F0`–`F8`; Phase 6 default subset `F0`–`F2` via `PHASE6_F_SCENARIO_NAMES` in `f_scenario_bank.py`)

## Goal

Stand up the new layered runtime path so ego no longer depends solely on monolithic
control logic, while preserving baseline driving outcomes.

## What to Build

- Add runtime-callable shells for:
  - state extraction
  - prediction shell
  - assessment shell
  - tactical planner shell
  - safety guard shell
  - top-level ego orchestrator
- Route ego control through this orchestration path every cycle.
- Preserve compatibility with the existing MPC executor path.
- Emit per-cycle decision observability:
  - `planner_state`
  - `active_ttl`
  - `decision_reason`

## Why It Matters

Without explicit layer boundaries and runtime evidence, later phases cannot be
implemented safely or debugged reliably.

## Success Criteria

- New modules are invoked every control cycle (not dead code).
- Monolithic behavior path is no longer the only active route.
- Baseline scenarios complete with no architecture-caused instability.
- Per-cycle logs include planner state, TTL, and decision reason.

## Required Telemetry (Phase 6)

Implemented log tags (full-control steps; fast path emits `[Phase6Executor]` reuse lines only):

- `[Phase6State]` — `has_opponent`, `pit_mode`, `ttl`, speeds, distance, overlap, segment, ahead
- `[Phase6Planner]` — `planner_state`, `active_ttl`, `target_speed_cap`, `decision_reason`
- `[Phase6Guard]` — guard flags and `emergency_stable_mode`
- `[Phase6Executor]` — whether the MPC executor ran and final steer/throttle/brake

Safety / eval (from existing eval-contact pipeline, parsed by benchmarks):

- `[EvalEvent]` with `type=eval_contact` — overlap / near counts drive `collision` / `near_miss` in digest

## Benchmark / Scenario Guidance

Use foundational subset from shared F-bank:

- `F0` ego alone
- `F1` fellow behind, same TTL, cruise
- `F2` fellow ahead, same TTL, slower, cruise

Shared bank path: `examples/racing/f_shared/` (reused by post-Phase-5 phases to avoid
duplicated scenario folders).

Runner:

```bash
python -m scenic.domains.racing.benchmarks.phase6_runner --time 1000
```

Runtime policy:

- Use **10 s default** (`--time 1000`) for iteration.
- Use **15 s max** (`--time 1500`) for confirmation.

Artifacts: `src/scenic/domains/racing/benchmarks/results/phase6_YYYYMMDD_HHMMSS/` (`summary.json`, `summary.csv`, per-scenario logs).

## Implementation (code)

Delivered layout:

- `src/scenic/domains/racing/phase6_runtime.py` — state snapshot, planner, guard, log formatters.
  Follow engagement uses a **time headway × ego speed** threshold with a low-speed
  **floor** (same idea as Phase 8 `safe_gap`), keeping Phase 6 logic simple.
- `src/scenic/domains/racing/prediction/` — reserved for Phase 7+ (see Phase 7 plan)
- Ego integration in `FollowRacingLineMPCBehavior` (Phase 6 block before Phase 3 tactical); uses `_phase3_speed_cap` from guard when set, consistent with Phase 3 follow cap

Compatibility intent:

- Existing MPC executor remains the final control sink.
- Phase 6 is additive at integration points.

## Validated outcomes and caveats

- **Digest KPIs:** Phase 6 runner aggregates line counts for State / Planner / Guard / Executor, `phase6_guard_active_count`, fellow harness fields, and eval-contact collision flags.
- **F2 hull contact:** Benchmarks treat **`[EvalEvent]` eval hull overlap** as the canonical “collision” signal. A run can report **`return_code=0`** while **`collision=True`** / `eval_contact_overlap_count > 0` if the eval-contact classifier logged overlap. Treat that as a **safety / geometry caveat**, not a clean “no contact” sign-off for F2 until overlap is resolved or explained.
- **Scope:** Phase 6 validates **wiring and observability**, not full tactical optimality.

## Exit Checklist

- [x] Layer shells exist in codebase with clear responsibilities (`phase6_runtime.py`).
- [x] Ego path invokes Phase 6 layers on full-control steps when `phase6_orchestration_enabled=True`.
- [x] Scenario subset (`F0`, `F1`, `F2`) runnable via `phase6_runner`; results and caveats documented above.
- [x] Logs contain `planner_state`, active TTL, and `decision_reason` when Phase 6 is enabled (plus guard/executor lines as specified).
- [x] Results and caveats documented with runner id pattern and artifact paths.

## Handoff to Phase 7

Proceed to [Phase 7: Fellow next-step prediction](./phase-7-fellow-next-step-prediction.md):
use Phase 6 orchestration and telemetry as the substrate, then add measurable forecast
quality (`prediction_error_next_step`) on dynamic fellow scripts.
