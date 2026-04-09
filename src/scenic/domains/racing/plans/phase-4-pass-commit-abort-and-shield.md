# Phase 4: Pass Commit, Abort, and Safety Shield

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
