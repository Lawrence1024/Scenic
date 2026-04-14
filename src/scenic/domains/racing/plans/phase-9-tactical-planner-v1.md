# Phase 9: Tactical Planner v1

## Prerequisites (handoff from Phase 8)

Phase 8 provides stable tactical facts (`fellow_relation`, corridor openness, `gap_ok`)
and dynamic safe-gap inputs.

Phase 9 introduces the first explicit tactical state machine on top of those facts.

This phase **adds** setup-pass tactical choices and speed-cap shaping; it does **not**
yet add hard commit/abort pass states.

## Current Status

**Planned** (first tactical state machine with explainable decisions).

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed transitions intent: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

## Goal

Deliver stable tactical behavior that distinguishes free-run, disciplined follow, and
setup-pass positioning while avoiding corridor-ignorant choices.

## What to Build

Implement planner states:

- `FREE_RUN`
- `FOLLOW`
- `SETUP_PASS_LEFT`
- `SETUP_PASS_RIGHT`

Decision outputs:

- chosen TTL
- target speed cap
- decision reason string

Stability controls:

- setup-side hysteresis / cooldown
- bounded transition frequency to avoid chatter

## Why It Matters

This is the first phase where tactical intent becomes explicit and testable in logs,
rather than inferred from low-level motion alone.

## Success Criteria

Expected behavior by scenario:

- `F0`: remain `FREE_RUN`, mostly on `optimal`.
- `F1`: remain `FREE_RUN`; no unnecessary caution from behind fellow.
- `F2`: transition to `FOLLOW`; reduce speed cap as needed; avoid rear-end.
- `F6`: avoid choosing occupied left side; favor `FOLLOW` or `SETUP_PASS_RIGHT`.
- `F7`: symmetric to `F6`; favor `FOLLOW` or `SETUP_PASS_LEFT`.

Transition quality:

- planner should avoid pathological state oscillation.

## Required Telemetry (Phase 9)

- `planner_state`
- `chosen_ttl`
- `target_speed_cap`
- `decision_reason`
- optional transition diagnostics:
  - `state_change_count`
  - `state_dwell_time`

## Benchmark / Scenario Guidance

Recommended set:

- `F0`, `F1`, `F2`, `F6`, `F7`

Runner guidance (placeholder naming convention):

```bash
python -m scenic.domains.racing.benchmarks.phase9_runner --time 2000
```

Example acceptable sequences:

- `F2`: `FREE_RUN -> FOLLOW`
- `F6`: `FOLLOW -> SETUP_PASS_RIGHT`
- `F7`: `FOLLOW -> SETUP_PASS_LEFT`

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/planner/tactical_planner.py`
- planner state machine and transition guards in `src/scenic/domains/racing/planner/`
- behavior wiring where planner outputs TTL and speed-cap decisions
- parser/summary updates for Phase 9 state and reason metrics

## Exit Checklist

- [ ] State machine emits only supported Phase 9 states.
- [ ] Decision telemetry is present and parseable for all scenarios.
- [ ] `F6`/`F7` occupancy symmetry is reflected in setup-side decisions.
- [ ] Follow behavior in `F2` remains safe and stable.
- [ ] No new chatter pattern is introduced relative to prior phase baseline.

## Handoff to Phase 10

Proceed to [Phase 10: Stability guard and emergency policy](./phase-10-stability-guard-and-emergency-policy.md):
enforce anti-chaotic limits on brake/steer/switch behavior and add explicit
`EMERGENCY_STABLE` enforcement pathways.
