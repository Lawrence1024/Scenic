# Phase 3: Smart follow and stable TTL

Scenarios here enable **`tactical_planner_enabled=True`** on `FollowRacingLineMPCBehavior`.

The tactical layer (Phase 3) chooses among:

- **FREE_RUN** — `optimal` TTL when no relevant opponent ahead
- **FOLLOW** — stay on `optimal`, cap speed vs a slower car ahead when a pass is not considered safe
- **SETUP_LEFT** / **SETUP_RIGHT** — switch to `left` / `right` TTL on long straights when risk is low (positioning only; no pass commitment)

**Mutually exclusive with Phase 1 scripted schedule:** if both `planner_enabled` and `tactical_planner_enabled` are true, the scripted schedule is ignored.

## Run

```bash
scenic examples/racing/phase3_tactical/01_slower_opponent_tactical.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1 --time 3000
```

Log markers:

- `[Phase3Tactical] ... ttl_switch ...`
- `[Phase3Tactical] ... mode=... ttl=... cap=...` (periodic)

## Tests

```bash
python -m pytest src/scenic/domains/racing/mpc/testing/test_tactical_planner.py -q
```

See `src/scenic/domains/racing/plans/phase-3-smart-follow-and-stable-ttl.md`.
