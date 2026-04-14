# Phase 6: Architecture Skeleton and Observability

## Prerequisites (handoff from Phase 5)

Phase 5 is **closed** for segment-aware tactics on top of tactical + pass-shield behavior.
Phase 6 starts the next roadmap by restructuring internals while preserving the same
behavior envelope.

This phase **adds** layered orchestration and telemetry contracts; it does **not** add
new pass intelligence yet.

## Current Status

**Implemented (initial)** (architecture and observability foundation for Phases 7-12).

- Source architecture intent: `src/scenic/domains/racing/restrcture_plan.md`
- Execution guidance: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`
- Runtime shell module: `src/scenic/domains/racing/phase6_runtime.py`
- Runner + shared bank: `src/scenic/domains/racing/benchmarks/phase6_runner.py`,
  `examples/racing/f_shared/`

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

- Layer-invocation markers:
  - state extraction call
  - planner call
  - guard call
  - executor call
- Decision telemetry:
  - `planner_state`
  - `active_ttl`
  - `decision_reason`
- Safety outcomes:
  - collision
  - off-track
  - forced-stop flags

## Benchmark / Scenario Guidance

Use foundational subset from shared F-bank:

- `F0` ego alone
- `F1` fellow behind, same TTL, cruise
- `F2` fellow ahead, same TTL, slower, cruise

Shared bank path: `examples/racing/f_shared/` (reused by post-Phase-5 phases to avoid
duplicated scenario folders).

These are sufficient because the phase validates architecture wiring and observability,
not tactical sophistication.

Runner guidance:

```bash
python -m scenic.domains.racing.benchmarks.phase6_runner --time 2000
```

## Implementation (code)

Target module boundaries:

- `src/scenic/domains/racing/state/`
- `src/scenic/domains/racing/prediction/` (shell only for this phase)
- `src/scenic/domains/racing/assessment/` (shell only for this phase)
- `src/scenic/domains/racing/planner/` (shell only for this phase)
- `src/scenic/domains/racing/safety/` (shell only for this phase)
- `src/scenic/domains/racing/behaviors/ego_main.py` (or equivalent top-level orchestrator)

Compatibility intent:

- Existing MPC executor remains the final control sink.
- Phase 6 should be additive and reversible at integration points.

## Exit Checklist

- [ ] Layer shells exist in codebase with clear responsibilities.
- [ ] Top-level ego path invokes layers every cycle at runtime.
- [ ] Scenario subset (`F0`, `F1`, `F2`) completes without architecture-induced regression.
- [ ] Logs contain `planner_state`, `active_ttl`, `decision_reason` every cycle.
- [ ] Results and caveats are documented with run id and artifact paths.

## Handoff to Phase 7

Proceed to [Phase 7: Fellow next-step prediction](./phase-7-fellow-next-step-prediction.md):
use Phase 6 orchestration and telemetry as the substrate, then add measurable forecast
quality (`prediction_error_next_step`) on dynamic fellow scripts.
