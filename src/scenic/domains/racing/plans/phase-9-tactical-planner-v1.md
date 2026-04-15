# Phase 9: Tactical Planner v1

## Prerequisites (handoff from Phase 8)

Phase 8 provides stable tactical facts (`fellow_relation`, corridor openness, `gap_ok`)
and dynamic safe-gap inputs.

Phase 9 introduces the first explicit tactical state machine on top of those facts.

This phase **adds** setup-pass tactical choices and speed-cap shaping; it does **not**
yet add hard commit/abort pass states.

## Current Status

**Complete as tactical baseline** — Phase 9 tactical planner path is wired with
decision-reason telemetry and benchmark runner support, with structural safety latches added
during hardening. Phase 9 is now treated as a frozen tactical baseline for Phase 10/11 work.

Latest validation snapshots (user-run digests, 15 s windows):

- `phase9_20260415_200726` (`F2/F4/F7`): tactical intent and setup behavior are present,
  but standalone Phase 9 can still fail sudden-stop disturbance safety (`F4` collision),
  which is expected to be handled by Phase 10 guard policy.
- `phase10_20260415_202041` + `phase10_20260415_204406` (`F2/F4/F5/F6/F7`): with Phase 10
  active on top of Phase 9 tactical output, all targeted scenarios were collision-free and
  off-track-free at 15 s.
- Therefore: **Phase 9 tactical layer is accepted as baseline**, while disturbance containment
  is owned by Phase 10 and pass commit/abort remains Phase 11 scope.

Handoff note:

- Team decision: keep Phase 9 tactical behavior frozen unless Phase 10/11 integration reveals
  a clear tactical-regression root cause.
- New overtake completion lifecycle logic should land in Phase 11, not as ad-hoc Phase 9 tuning.

- Architecture source: `src/scenic/domains/racing/restrcture_plan.md`
- Detailed transitions intent: `src/scenic/domains/racing/phase6-12.md`
- Master chain: `src/scenic/domains/racing/plans/phase-6-12-master-rollout.md`

Starting point in code:

- Existing conservative tactical logic is available in
  `src/scenic/domains/racing/tactical_planner.py` (Phase 3 style
  `FREE_RUN` / `FOLLOW` / `SETUP_LEFT` / `SETUP_RIGHT`).
- Phase 9 extends this into a Phase-8-informed planner with canonical state naming
  and stronger decision explainability, while still stopping short of commit/abort.
- Runner implemented: `src/scenic/domains/racing/benchmarks/phase9_runner.py`
  (`F0`, `F1`, `F2`, `F6`, `F7`) with `[Phase9Planner]` parsing in
  `phase_run_common.py`.

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

Design constraints:

- Avoid large parameter-search loops; favor explicit state-transition logic.
- Keep commit/abort behavior out of Phase 9 (Phase 11 responsibility).
- Keep low-level executor and guard interfaces stable.

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
- deterministic rerun of unchanged code/config should produce same tactical-state
  sequences for the same scenario.

Additional acceptance interpretation:

- For `F2`/`F4`, relation may switch after contact/overtake. Evaluate tactical
  decisions in timestamp windows (pre-contact and post-pass), not only aggregate counts.

## Required Telemetry (Phase 9)

- `planner_state`
- `chosen_ttl`
- `target_speed_cap`
- `decision_reason`
- `assessment_relation`
- `assessment_gap_ok`
- `assessment_optimal_open`
- `assessment_left_open`
- `assessment_right_open`
- optional transition diagnostics:
  - `state_change_count`
  - `state_dwell_time`

## Benchmark / Scenario Guidance

Recommended set:

- `F0`, `F1`, `F2`, `F6`, `F7`

Runner guidance:

```bash
python -m scenic.domains.racing.benchmarks.phase9_runner --time 1000
```

Benchmark runtime policy for this phase:

- Use **10 s default** (`--time 1000`) for iteration.
- Use **15 s max** (`--time 1500`) for confirmation.
- Do not run 20 s+ unless there is an explicit documented reason.
- Wall-clock planning note: on this setup, expect about **11 s wall-clock per 1 s simulated**
  (roughly **11x**), so a 20 s run is about 220 s wall-clock.
- Workflow rule: assistant provides benchmark commands; user runs them and returns digest/logs.
  Assistant should not run simulation directly.

Determinism guidance:

- For unchanged code/config, do not rerun same scenarios expecting different outcomes.
- Reruns are meaningful only after code/config/environment change.

Example acceptable sequences:

- `F2`: `FREE_RUN -> FOLLOW`
- `F6`: `FOLLOW -> SETUP_PASS_RIGHT`
- `F7`: `FOLLOW -> SETUP_PASS_LEFT`

## Implementation (code)

Primary targets:

- `src/scenic/domains/racing/tactical_planner.py` (Phase 9 state machine update)
- planner transition guards and reason taxonomy in tactical planner module
- behavior wiring where planner outputs TTL and speed-cap decisions
- parser/summary updates for Phase 9 state and reason metrics

Structural updates added during Phase 9 hardening:

- **Protected follow envelope (item 1):**
  - Added state latch `protected_follow_active` so tactical logic cannot drop to
    `FREE_RUN` while safety pressure is active (`gap_ok=False`, closing, elevated emergency
    risk, or tight gap).
  - Added explicit release condition using consecutive clear cycles
    (`protected_follow_release_cycles`) instead of one-frame toggles.
  - Added **opening-release path**: while fellow is still ahead, protected follow can now
    release into setup if a side opening stays clear for consecutive cycles
    (`gap_ok=True`, no closing, low emergency risk, open side, no proximity hazard).
    This aligns with racing overlap etiquette and autonomous gap-acceptance practice:
    do not force pass attempts under hazard, but do not block safe, sustained openings.
  - Added **hazard-vs-opening split**:
    - Keep strict contact/tight-gap hazard handling for safety latches.
    - Treat asymmetric side openings as a separate decision channel so setup can proceed
      without waiting for fully "free-run-like" conditions.
  - Unified emergency-risk usage with planner pass-safety envelope (`pass_safe_risk_max`)
    so risk gating is structurally consistent across latch, setup eligibility, and
    pressure-hold logic.
  - Added **setup-to-commit layer** for setup-pass continuity:
    - New commit arming path while already in setup: requires sustained side-consistent
      opening, adequate closing trend, and pass-safe conditions.
    - New commit hold window (`setup_commit_*`) keeps setup TTL active through brief
      moderate pressure oscillations to reduce `FOLLOW <-> SETUP` chatter that blocks
      pass completion.
    - Commit hold is cancelled immediately on hard hazards (contact-like overlap,
      tight gap, recovery hold, or high emergency risk), preserving safety priority.
  - Added **pass-intent commit latch** ahead of setup entry:
    - Planner can now arm pass intent while still in `FOLLOW` when a side-consistent
      opening remains stable.
    - Armed intent promotes directly into setup-commit hold, reducing dependence on
      fragile one-window setup entry timing.
    - This addresses the observed pattern where setup opportunities existed but were
      repeatedly lost to follow-pressure/protected-follow re-entry before pass progression.
  - Added **lateral-path lock split (path vs longitudinal safety)**:
    - During active pass-intent/setup-commit windows, lateral path choice is held
      (`left`/`right`) for a minimum lock duration to prevent TTL flicker.
    - Safety pressure can still act longitudinally (throttle/brake authority) without
      immediately revoking lateral setup intent.
    - Lateral lock is dropped on hard hazards only (overlap/tight-gap/recovery/high risk),
      preserving safety-first behavior while reducing right-left-right oscillation.
  - Added **speed-relative tactical gating (relative motion over fixed meters)**:
    - Replaced key fixed-gap triggers with headway/TTC-style checks derived from ego/opponent
      speed (`follow_tight_headway_s`, `blocked_headway_s`, `setup_min_headway_s`, `hard_ttc_s`).
    - Setup eligibility now requires positive relative closing margin (`pass_min_relative_speed_mps`)
      so setup is tied to actual overtake capability, not static spacing alone.
    - Follow pressure and hard hazard decisions now scale with current speed and relative
      closing dynamics, improving consistency across low/high-speed sections.
- **Contact recovery hold (item 3):**
  - Added recovery latch `recovery_hold_until_s`, triggered by contact-like overlap geometry
    (`partial_overlap` or `side_by_side`).
  - During hold, planner forces `FOLLOW` with reason `contact_recovery_hold`, blocking
    setup re-entry and allowing separation rebuild before normal transitions.

Legacy cleanup and retirement (what we tried):

- **Phase 9 authority cutover in behavior wiring:**
  - We observed that legacy runtime paths could still log/act as `FREE_RUN` during overlap
    windows while Phase 9 tactical output remained `FOLLOW`.
  - Implemented authority cutover so effective executor-facing state/reason/TTL is sourced
    from the tactical path when Phase 9 is active.
- **Retired legacy tactical overlays in Phase 9 mode:**
  - When Phase 8 assessment + tactical planner are active, legacy Phase 4 shield and
    Phase 5 segment-tactics overlays are forcibly disabled in behavior wiring to prevent
    hidden overrides.
- **Removed Phase 6 cap influence on Phase 9 decisions:**
  - In Phase 9 mode, Phase 6 planner no longer injects speed cap into tactical execution.
  - Phase 6 remains as observability/logging shell, while tactical/assessment path owns
    effective mode and cap.
- **Added structural hazard-follow longitudinal authority:**
  - New behavior-level `Phase9Hazard` gate enforces throttle suppression plus brake-floor
    authority when tactical state is `FOLLOW` with safety reasons and Phase 8 still indicates
    unsafe overlap/closing/gap pressure.
  - This is an architectural safety layer (not parameter-tuning): tactical intent can now
    directly impose minimum longitudinal deceleration during critical proximity windows.
  - Trial result (10 s targeted check, `F2` + `F7`): collision and near-contact counts dropped
    to zero in run `phase9_20260415_093910`; keep broader validation pending before sign-off.

State naming migration note:

- Canonical Phase 9 plan names are `SETUP_PASS_LEFT` / `SETUP_PASS_RIGHT`.
- If implementation keeps internal `SETUP_LEFT` / `SETUP_RIGHT` for compatibility,
  logs and benchmark-facing fields should expose canonical aliases.

## State Machine Contract (Phase 9)

Minimum transition intent:

- `FREE_RUN -> FOLLOW`: fellow ahead + blocking or unsafe/closed corridor.
- `FOLLOW -> SETUP_PASS_LEFT/RIGHT`: one side open and favored by occupancy signal.
- `SETUP_PASS_* -> FOLLOW`: corridor closes / risk rises / setup no longer safe.
- `SETUP_PASS_LEFT <-> SETUP_PASS_RIGHT`: allowed only with cooldown + reason.
- Any state -> `FREE_RUN`: no relevant opponent or opponent clearly behind.

Reason taxonomy (minimum set):

- `no_opponent`
- `opponent_behind_free_run`
- `ahead_blocking_follow`
- `gap_not_ok_follow`
- `setup_left_open`
- `setup_right_open`
- `setup_cancelled_risk`
- `setup_flip_cooldown_hold`
- `protected_follow_envelope`
- `contact_recovery_hold`
- `setup_commit_left_hold`
- `setup_commit_right_hold`
- `lateral_path_lock_left_hold`
- `lateral_path_lock_right_hold`

## Exit Checklist

- [x] State machine emits only supported Phase 9 states.
- [x] Decision telemetry is present and parseable for all scenarios.
- [x] `F6`/`F7` occupancy symmetry is reflected in setup-side decisions.
- [x] Follow behavior in `F2` remains safe and stable.
- [x] No new chatter pattern is introduced relative to prior phase baseline.
- [x] Deterministic replay of unchanged code/config reproduces tactical-state traces.

## Handoff to Phase 10

Proceed to [Phase 10: Stability guard and emergency policy](./phase-10-stability-guard-and-emergency-policy.md):
enforce anti-chaotic limits on brake/steer/switch behavior and add explicit
`EMERGENCY_STABLE` enforcement pathways.
