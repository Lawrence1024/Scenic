# Phase 10: Stability Guard and Emergency Policy

## Prerequisites (handoff from Phase 9)

Phase 9 established planner states for free-run, follow, and setup-pass choices.
Phase 10 adds guard-level enforcement so tactical intent cannot produce physically
chaotic outputs.

This phase **adds** anti-swerve and anti-chatter safety constraints, including explicit
`EMERGENCY_STABLE` handling.

## Current Status

**Complete as stability baseline** — command-level stability guard is wired with
`[Phase10Guard]` telemetry and benchmark runner support, including emergency
containment and post-emergency recovery hardening. This baseline is ready for
Phase 11 integration work.

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

Runner guidance:

```bash
python -m scenic.domains.racing.benchmarks.phase10_runner --time 1000
```

Runtime policy for this phase:

- Use **10 s default** (`--time 1000`) for iteration.
- Use **15 s max** (`--time 1500`) for confirmation.
- Do not run 20 s+ unless explicitly justified and documented.

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/safety/stability_guard.py`
- safety policy config and thresholds in `src/scenic/domains/racing/safety/`
- planner/behavior integration to accept guard overrides
- benchmark parser columns for guard-specific KPIs

Initial implementation delivered:

- Added Phase 10 guard module (`stability_guard.py`) with:
  - steer slew limiting,
  - brake-steer coupling limiter,
  - TTL switch rate limiting helper,
  - emergency-stable latch with hysteresis.
- Integrated guard into `FollowRacingLineMPCBehavior` command path with
  `[Phase10Guard]` per-cycle logs and executor-facing command overrides.
- Added benchmark support:
  - runner: `src/scenic/domains/racing/benchmarks/phase10_runner.py`,
  - parser: `phase_run_common.py` (`phase10_guard_*` metrics),
  - scenario bank defaults in `f_scenario_bank.py`,
  - sequence wiring in `run_all_benchmarks_so_far.py`.
- Added unit tests for guard behavior in
  `src/scenic/domains/racing/mpc/testing/test_stability_guard.py`.

Structural hardening update (targeting repeated-contact behavior in `F4`):

- Emergency longitudinal dominance now uses hazard-aware brake floors:
  - overlap-triggered containment: stronger brake floor,
  - unsafe closing with blocked gap: stronger brake floor,
  - baseline emergency floor retained for general emergency latching.
- Emergency trigger sensitivity for blocked-gap closing was moved earlier so the
  guard can engage before deep-contact loops.
- Added post-emergency **re-approach suppression**:
  - recovery-hold latch persists after emergency,
  - throttle is capped and light braking is enforced during unsafe re-approach,
  - recovery exits only under clear-gap / non-closing / low-risk conditions.
- This update is intentionally structural (state/latch and policy layering), not
  a one-off per-scenario parameter tweak.

Latest validation snapshot (deterministic user-run digests):

- `phase10_20260415_202041`: `F2/F4/F7` all collision-free and off-track-free.
- `phase10_20260415_204406`: `F5/F6` collision-free and off-track-free.
- Combined targeted disturbance set (`F2/F4/F5/F6/F7`) is clean at 15 s windows,
  with guard intervention visible and scenario-appropriate (`guard_active`,
  `emergency_stable_mode`, `ttl_switch_blocked`, `steer_limited` where expected).

## Exit Checklist

- [x] Guard constraints are enforced at runtime on all targeted disturbances.
- [x] `EMERGENCY_STABLE` entry/exit behavior is logged and understandable.
- [x] Bound metrics remain within configured limits across stress scenarios.
- [x] No crash/off-track caused by chaotic ego reaction patterns.
- [x] Guard behavior is documented with run artifacts and caveats.

## Handoff to Phase 11

Proceed to [Phase 11: Pass commit and abort](./phase-11-pass-commit-and-abort.md):
build explicit overtake lifecycle transitions while keeping Phase 10 guardrails active
on every commit/abort decision chain.
