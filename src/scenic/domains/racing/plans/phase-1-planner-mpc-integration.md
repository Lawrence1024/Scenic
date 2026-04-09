# Phase 1: Planner-MPC Integration

## Current Status

**Phase 1 is closed** (scripted planner handoff validated end-to-end).

- `FollowRacingLineMPCBehavior` accepts planner-style inputs via:
  - `planner_enabled=True`
  - `ttl_schedule='10:left,20:right,...'` (simulation-time switches)
  - `target_speed_cap=<m/s>`
- Runtime TTL switching supports `optimal`, `left`, `right` with waypoint/segment-map reset and TTL preload from `ttlFolder`.
- Initial active TTL aligns with the scenario’s starting `ttlFileName` when `ttl_selection` is unset (so scripted schedules like `left→right` and `right→optimal` log the correct `from`/`to`).
- Manual switch scenarios live in `examples/racing/phase1_planner/`; automation via `python -m scenic.domains.racing.benchmarks.phase1_runner`.
- Validation run example: `phase1_20260409_152551` — all three scenarios reported one Phase 1 switch, lap completed, no collision/off-track flags.

## Goal

Create the integration point where a planner can choose active TTL every control cycle and feed ego MPC.

## What to Build

- Refactor ego control flow so planner output selects active reference each cycle.
- Extract reusable "one MPC step with selected reference" logic from current ego line-follow behavior.
- Add a top-level planner behavior/node that outputs:
  - `active_ttl ∈ {optimal, left, right}`
  - target speed cap
- Wire planner output into existing ego MPC.
- Keep selection scripted/manual in this phase (no tactical intelligence yet).

## Why It Matters

This is the core architectural unlock that enables all later smart-driving logic.

## Success Criteria

Manual switch tests pass at race pace on main loop:

- optimal -> left
- left -> right
- right -> optimal

For each switch:

- no crash during switch
- no oscillation/chatter
- no immediate off-track event
- MPC remains tracking after switch

## Exit Checklist

- [x] Planner output contract is defined and used by ego control path (`planner_enabled`, `ttl_schedule`, `target_speed_cap` on `FollowRacingLineMPCBehavior`).
- [x] Active TTL can be switched during run without controller reset failures (waypoints + segment map rebuilt on switch).
- [x] Three manual-switch tests pass with stable tracking (`01_optimal_to_left`, `02_left_to_right`, `03_right_to_optimal` via `phase1_runner`).
- [x] Integration works on main-track lap behavior (dSPACE racing model, Laguna Seca TTL set).

## Handoff to Phase 2

Proceed to [Phase 2: Situation assessment](./phase-2-situation-assessment.md): interpret opponent/ego state for tactical TTL choice (still feeding the same MPC integration point).
