# Phase 3: Smart Follow and Stable TTL Choice

## Current Status

**Implemented** in `FollowRacingLineMPCBehavior(..., tactical_planner_enabled=True)`:

- Module: `src/scenic/domains/racing/tactical_planner.py` (modes + hysteresis + `apply_ttl_key_to_agent`).
- Ego-only: runs after segment/pit resolution, before curvature speed gate; refreshes `wp_list` on TTL switch.
- Follow speed cap combined into `effective_target_speed` (with Phase 1 cap and curvature/CTE limits).
- Example: `examples/racing/phase3_tactical/01_slower_opponent_tactical.scenic`
- Tests: `src/scenic/domains/racing/mpc/testing/test_tactical_planner.py`

Tune thresholds via `TacticalPlannerConfig` in `behaviors.scenic` (currently default instance).

## Goal

Introduce conservative tactical behavior that follows safely and repositions without unstable line switching.

## What to Build

Add planner modes:

- `FREE_RUN`
- `FOLLOW`
- `SETUP_LEFT`
- `SETUP_RIGHT`

Behavior rules:

- with no relevant opponent: stay on `optimal`
- when blocked and pass is not safe: `FOLLOW`
- when one side looks promising: `SETUP_LEFT` or `SETUP_RIGHT`
- add hysteresis to prevent rapid left/right bouncing

In this phase, setup is positioning only, not hard pass commitment.

## Why It Matters

This is the first smart-driving layer: ego should avoid blindly driving into slower traffic.

## Success Criteria

In blocked-opponent scenarios:

- ego does not rear-end opponent
- ego follows safely or repositions to a better TTL
- unnecessary TTL switching remains low
- planner stays stable over multiple laps

Suggested targets:

- 0 collisions in follow benchmarks
- minimum gap always above safety threshold
- bounded/non-oscillatory TTL switches per lap
- in free run, most time spent on `optimal`

## Exit Checklist

- [x] Mode transitions are deterministic and hysteresis-backed (see `tactical_planner_step`, setup flip cooldown).
- [x] Follow behavior maintains safety gap across benchmarks — validated on dSPACE using `phase3_on_phase0_runner` over `examples/racing/phase3_on_phase0_bank/` (same seven layouts as Phase 0, ego with `tactical_planner_enabled=True`). Aggregate run: **0 collisions**, **0 off-track**, **0 near-miss events**, all scenarios **`return_code` 0**, laps **completed** at default **3000** steps (~30 s sim). See [Validated benchmarks (dSPACE)](#validated-benchmarks-dspace).
- [x] Repositioning without pathological chatter — `[Phase3Tactical]` logs show bounded TTL switches on stable opponent cases (typically two switches: setup lane then return toward optimal); **04_opponent_weaving** shows more switches (reactive opponent), which is expected.
- [ ] Multi-lap stability for blocked scenarios — **optional follow-up** (longer `--time`, dedicated multi-lap harness). Single-lap / ~30 s horizon validated.

## Validated benchmarks (dSPACE)

Cross-check command (repo root):

```bash
python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0
```

This uses default **`--time` 3000** (override if you need more simulated time per scenario). Outputs: `src/scenic/domains/racing/benchmarks/results/<run_id>/summary.json`, per-scenario `logs/*.log`, and terminal **`BENCHMARK_AI_DIGEST_BEGIN` … `END`**.

Smaller smoke set: `phase3_runner` on `examples/racing/phase3_tactical/` (single tactical scenario folder).

## Handoff to Phase 4

Phase 3 provides **FREE_RUN / FOLLOW / SETUP_LEFT / SETUP_RIGHT** and hysteresis only — **no** committed pass or shield. Phase 4 adds `COMMIT_PASS_*`, `ABORT_PASS`, `EMERGENCY_AVOID`, and a **safety shield** that may override tactical intent; build on the same TTL preload, MPC stack, and `[Phase2]` / `[Phase3Tactical]` logging patterns. See [Phase 4 plan](./phase-4-pass-commit-abort-and-shield.md).
