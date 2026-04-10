# Phase 2: Situation assessment

Phase 2 adds **race-semantics features** for the nearest opponent (one opponent), computed in
`scenic.domains.racing.situation_assessment` and logged from `FollowRacingLineMPCBehavior` on the
same cadence as the Phase 0 summary line (every 50 behavior steps).

## Log line

Look for:

```text
[Phase2] t=... ahead=... delta_s_m=...(polyline|heading_proxy) lat_rel=... closing_mps=...
overlap=... risk_01=... seg_ctx=... dist_m=... lon_m=... lat_m=...
```

- **`delta_s_m`**: wrapped along-lap meters when TTL waypoints + ego progress are available (`polyline`);
  otherwise longitudinal distance along ego heading (`heading_proxy`, same family as Phase 0 `nearest_opp_ds`).
- **`overlap`**: `side_by_side`, `partial_overlap`, `clear_ahead`, `clear_behind`, `closing_behind` with
  light hysteresis to reduce label flicker.
- **`seg_ctx`**: `straight` | `corner_entry` | `corner_body` | `corner_exit` from segment name + waypoint
  progress + curvature lookahead.

## Snapshot tests (no simulator)

```bash
python -m pytest src/scenic/domains/racing/mpc/testing/test_situation_assessment.py -q
```

## Try in sim

Any Phase 0 benchmark scenario with an opponent will emit `[Phase2]` lines, e.g.:

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 2000 --scenario 01_slower_opponent_optimal.scenic
```

The same `[Phase2]` lines appear when running **`phase3_runner`** on `examples/racing/phase3_tactical/` (Phase 0–aligned bank with **Phase 3** tactical enabled on ego); see `examples/racing/README.md`.

See `src/scenic/domains/racing/plans/phase-2-situation-assessment.md` for goals and exit checklist.
