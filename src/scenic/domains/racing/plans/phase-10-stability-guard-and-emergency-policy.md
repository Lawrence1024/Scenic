# Phase 10: Stability Guard and Emergency Policy

## Prerequisites (handoff from Phase 9)

Phase 9 established planner states for free-run, follow, and setup-pass choices.
Phase 10 adds guard-level enforcement so tactical intent cannot produce physically
chaotic outputs.

This phase **adds** anti-swerve and anti-chatter safety constraints, including explicit
`EMERGENCY_STABLE` handling.

## Current Status

**Planned** (safety guard integration and measurable intervention metrics).

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Scenario and metric guidance: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

## Goal

Ensure the ego stack remains controllable and stable under sudden-stop, swerve, and
tight-gap disturbances by constraining dangerous command combinations.

## What to Build

- Guard policies for:
  - brake-steer coupling limits
  - steering slew / snap limits
  - TTL-switch rate limiting and temporary blocks
- Emergency pathway:
  - enter and maintain `EMERGENCY_STABLE` when risk is high
  - transition out with hysteresis and explicit reasoning
- Guard-overrides planner output when planner output is unsafe.

## Why It Matters

Commit/abort logic in Phase 11 is unsafe without a robust stabilizing layer during
rapidly changing disturbance scenarios.

## Success Criteria

Scenario expectations:

- `F4`: stop onset should produce controlled braking behavior without panic lateral jump.
- `F5`: ego should not mirror right-left disturbances with unstable oscillation.
- tight-gap `F2`: follow stabilization should dominate over conflicting commands.
- aggressive-closing `F6`/`F7`: guard should block unsafe switching bursts.

Metric expectations:

- steering rate, brake spikes, and TTL switch rate remain within defined limits.
- guard activations occur in expected scenarios and remain explainable.

## Required Telemetry (Phase 10)

- `guard_active`
- `guard_reason`
- `steer_limited`
- `brake_limited`
- `ttl_switch_blocked`
- `emergency_stable_mode`
- optional boundedness metrics:
  - max steering rate
  - max brake command
  - TTL changes per second

## Benchmark / Scenario Guidance

Recommended set:

- `F4`
- `F5`
- tight-gap variant of `F2`
- aggressive-closing variants of `F6` and `F7`

Runner guidance (placeholder naming convention):

```bash
python -m scenic.domains.racing.benchmarks.phase10_runner --time 2000
```

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/safety/stability_guard.py`
- safety policy config and thresholds in `src/scenic/domains/racing/safety/`
- planner/behavior integration to accept guard overrides
- benchmark parser columns for guard-specific KPIs

## Exit Checklist

- [ ] Guard constraints are enforced at runtime on all targeted disturbances.
- [ ] `EMERGENCY_STABLE` entry/exit behavior is logged and understandable.
- [ ] Bound metrics remain within configured limits across stress scenarios.
- [ ] No crash/off-track caused by chaotic ego reaction patterns.
- [ ] Guard behavior is documented with run artifacts and caveats.

## Handoff to Phase 11

Proceed to [Phase 11: Pass commit and abort](./phase-11-pass-commit-and-abort.md):
build explicit overtake lifecycle transitions while keeping Phase 10 guardrails active
on every commit/abort decision chain.
