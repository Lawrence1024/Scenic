# Phase 4: Pass Commit, Abort, and Safety Shield

## Prerequisites (handoff from Phase 3)

Phase 3 is **closed** for conservative tactical behavior: `tactical_planner_enabled`, modes **FREE_RUN / FOLLOW / SETUP_***, hysteresis, Phase 2 assessment in the loop, and a **dSPACE-validated** run of the Phase 0 scenario bank with tactical on ego (`phase3_on_phase0_runner` → `examples/racing/phase3_on_phase0_bank/`). Aggregated sign-off run observed **no collisions, no off-track, no near-miss events** at default **3000** simulation steps per scenario.

Phase 4 **adds** explicit commit/abort/shield; it does **not** re-validate Phase 3. New benchmarks should target **pass completion**, **abort when the corridor closes**, and **shield preemption** (see Exit Checklist below).

## Goal

Enable real overtaking behavior with explicit commit/abort logic and a protective safety layer.

## What to Build

Add planner modes:

- `COMMIT_PASS_LEFT`
- `COMMIT_PASS_RIGHT`
- `ABORT_PASS`
- `EMERGENCY_AVOID`

Add a safety shield that can override tactical intent:

- abort when corridor collapses
- freeze/abort when overlap becomes unsafe
- brake or tuck in when opponent closes too quickly
- abort when boundary margin becomes too small

## Why It Matters

This phase upgrades behavior from passive positioning to true race tactics with safety-first fallback.

## Success Criteria

In overtake-focused scenarios:

- safe pass completion when corridor is truly available
- safe abort when corridor becomes unsafe
- no persistence in doomed pass attempts

Suggested targets:

- high pass success in clear-opportunity cases
- high abort success in closing-corridor cases
- 0 collisions in abort tests
- 0 boundary violations caused by pass attempts

## Exit Checklist

- [ ] Commit and abort transitions are explicit and test-covered.
- [ ] Safety shield preempts unsafe tactical actions.
- [ ] Dedicated overtake/abort benchmarks pass.
- [ ] Collision and boundary safety targets are met.
