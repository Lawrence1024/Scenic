# Phase 4: Pass Commit, Abort, and Safety Shield

## Prerequisites (handoff from Phase 3)

Phase 3 is **closed** for conservative tactical behavior: `tactical_planner_enabled`, modes **FREE_RUN / FOLLOW / SETUP_***, hysteresis, Phase 2 assessment in the loop, and a **dSPACE-validated** run of the Phase 0 scenario bank with tactical on ego (`phase3_runner` → `examples/racing/phase3_tactical/`). Aggregated sign-off run observed **no collisions, no off-track, no near-miss events** at default **3000** simulation steps per scenario.

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

## Implementation (code)

- **Module:** `src/scenic/domains/racing/pass_commit_shield.py` — `pass_shield_step` layers on Phase 3 output; modes **COMMIT_PASS_LEFT/RIGHT**, **ABORT_PASS**, **EMERGENCY_AVOID**; configurable `PassShieldConfig`.
- **Behavior:** `FollowRacingLineMPCBehavior(..., pass_commit_shield_enabled=True)` requires `tactical_planner_enabled=True`. Logs `[Phase4Tactical]` alongside `[Phase3Tactical]`.
- **Tests:** `src/scenic/domains/racing/mpc/testing/test_pass_commit_shield.py`.
- **Benchmarks:** `phase4_runner` parses `[Phase4Tactical]` lines into `phase4_*` counts in `summary.csv` / digest.
- **Example:** `examples/racing/phase4_pass_shield/01_slower_opponent_pass_shield.scenic`.

### 2026-04 Safety hardening update

To address close-range corner/overlap failures (notably case `05`), Phase 4 now
adds stricter **Phase 4-only** guards in `pass_shield_step`:

- prevent releasing from committed shield state directly to free/follow behavior
  while overlap is still close (`release_overlap_guard`);
- abort committed pass on close `side_by_side` states even when scalar risk
  underestimates danger (`overlap_side_by_side`);
- abort setup before commit when overlap is already close (`setup_overlap`);
- require commit preconditions to be conservative (`clear_ahead` and minimum
  standoff distance) instead of committing while partially overlapped;
- add an emergency branch for overlap-driven geometry risk (`emergency_overlap`)
  so response is not only tied to the scalar risk threshold.

Compatibility intent:

- these rules are scoped to **Phase 4 pass/shield** only;
- Phase 3 tactical semantics remain the same;
- Phase 5 segment tactics can still override Phase 3 intent, but Phase 4 safety
  guardrails remain active on the effective mode.

## Exit Checklist

- [x] Commit and abort transitions are explicit and unit-tested (`test_pass_commit_shield.py`); on-track benchmarks optional.
- [x] Safety shield preempts unsafe tactical actions (emergency risk, setup risk, corridor/overlap while committed).
- [x] Dedicated overtake/abort **scenario bank** and sign-off on dSPACE (seven scenarios under `examples/racing/phase4_pass_shield/`, executed by `phase4_runner`).
- [x] Collision and boundary safety targets are met under Phase 4-specific stress cases (latest sign-off run reported `all_return_codes_zero=true`, `any_collision=false`, `any_off_track=false`).

## Validated Benchmarks (dSPACE)

- Sign-off run: `phase4_20260410_100121` (`python -m scenic.domains.racing.benchmarks.phase4_runner --time 2000`).
- Coverage: `00`-`06` scenario bank in `examples/racing/phase4_pass_shield/`.
- Aggregate outcomes: all scenarios completed, no collision/off-track, and expected Phase 4 tactical events (`commit_pass`, `shield_release`, plus `emergency_avoid` in the close-corner stress case).
