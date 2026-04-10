# Phase 3 tactical benchmark bank

Scenarios here use **`tactical_planner_enabled=True`** on `FollowRacingLineMPCBehavior`.

The layout set matches **`examples/racing/phase0_benchmark/`** (`00`–`06`): no opponent, slower opponent on optimal / left / right TTL, weaving, short gap into a corner, and side-by-side style start. This is the **full** Phase 3 regression bank (not a minimal smoke folder).

The tactical layer (Phase 3) chooses among:

- **FREE_RUN** — `optimal` TTL when no relevant opponent ahead
- **FOLLOW** — stay on `optimal`, cap speed vs a slower car ahead when a pass is not considered safe
- **SETUP_LEFT** / **SETUP_RIGHT** — switch to `left` / `right` TTL on long straights when risk is low (positioning only; no pass commitment)

**Mutually exclusive with Phase 1 scripted schedule:** if both `planner_enabled` and `tactical_planner_enabled` are true, the scripted schedule is ignored.

## Run (full bank)

From repo root:

```bash
python -m scenic.domains.racing.benchmarks.phase3_runner --inter-run-delay-s 0
```

Backward-compatible entry (same scenarios and KPIs; `run_id_prefix=phase3_on_phase0`):

```bash
python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0
```

Single scenario (example):

```bash
scenic examples/racing/phase3_tactical/01_slower_opponent_optimal.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1 --time 3000
```

Default **`--time`** for the runner is **3000** steps (~30 s sim at 0.01 s/step). Use a larger `--time` only if a scenario needs more time for a full lap.

Log markers:

- `[Phase3Tactical] ... ttl_switch ...`
- `[Phase3Tactical] ... mode=... ttl=... cap=...` (periodic)

## Validation record

A full-bank run on dSPACE at default **3000** simulation steps per scenario completed with **no collisions, no off-track, no near-miss events** in aggregate, all scenarios exit code **0**, laps **completed**. Tighter geometry (e.g. slower opponent on **right** TTL) produced the smallest reported **min_opponent_distance_m** in the digest; **weaving** produced more `[Phase3Tactical]` TTL switches than stable-opponent cases, as expected.

## Tests

```bash
python -m pytest src/scenic/domains/racing/mpc/testing/test_tactical_planner.py -q
```

See `src/scenic/domains/racing/plans/phase-3-smart-follow-and-stable-ttl.md`.
