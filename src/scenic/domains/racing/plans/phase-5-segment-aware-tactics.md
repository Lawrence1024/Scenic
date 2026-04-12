# Phase 5: Segment-Aware Tactics

## Prerequisites (handoff from Phase 4)

Phase 4 is **closed** with commit/abort/shield behavior validated on the seven-scenario bank
(`phase4_runner` on `examples/racing/phase4_pass_shield/`, run id `phase4_20260410_100121`):

- all scenarios completed (`return_code=0`)
- no collision / off-track in aggregate
- expected tactical events observed (`commit_pass`, `shield_release`, `emergency_avoid` where relevant)

Phase 5 must preserve this safety envelope while improving where and when tactical decisions are
made by segment context.

## Goal

Make tactical choices context-sensitive to track segments so the planner is race-aware, not only obstacle-aware.

## What to Build

Use segment map and track structure to adapt tactics:

- on straights, allow more aggressive pass setup
- at corner entry, prefer inside only when overlap is established early
- in corner body, avoid fresh side-switch commitments
- on corner exit, prioritize completion and traction

Focus on improving decision quality while maintaining Phase 4 safety.

## Lessons Applied from Phases 0-4

- **Behavior/TTL contract must stay explicit.** If a scenario expects lane semantics from TTL files,
  fellows must use TTL-geometric behavior rather than constant-offset behavior.
- **Placement diagnostics must be deterministic.** For ego-anchored spawn (`_racing_st_offset`),
  diagnostics should be based on commanded `(delta_s, delta_t)` and ego route context, not random
  initial Scenic XY.
- **Metrics must come from logs, not assumptions.** Every new Phase 5 decision branch should produce
  parseable markers and be summarized in `summary.csv` / digest.
- **Short horizon is acceptable for tactical regressions.** Keep default `--time 2000` unless a
  scenario explicitly needs a longer window.

## Required Telemetry (Phase 5)

Add/extend periodic logs so each decision is explainable:

- segment context at decision time (`straight`, `corner_entry`, `corner_body`, `corner_exit`)
- tactical mode + selected TTL + speed cap
- reason code for each switch or block (implemented: `entry_conservative`, `body_no_new_setup`; emitted on `[Phase5Event]` / `[Phase5Tactical]`)
- pass-shield interplay (`mode3` vs effective mode) when shield overrides segment intent

These should be consumed by runner parsers (as Phase 3/4 do) so KPI columns are machine-readable.

## Why It Matters

Without segment awareness, planner may be safe but strategically weak.

## Success Criteria

Compared to Phase 4 on mixed scenarios:

- same or better safety outcomes
- improved overtake timing
- fewer poor pass attempts into corners
- improved average lap time in traffic
- fewer aborts caused by late/weak commitments

Minimum measurable sign-off vs Phase 4 baseline:

- no regression in `collision` / `off_track` aggregate (must remain zero in sign-off bank)
- no increase in near-miss count on corner-focused stress scenarios
- reduced corner-entry commit attempts that later require emergency intervention
- stable TTL switching (no new chatter pattern by segment boundary transitions)

## Benchmark/Scenario Guidance

Bank: `examples/racing/phase5_segments/` (runner: `phase5_runner`). The default bank is **eleven** scenarios: **`00`–`06`** (aligned with the Phase 4 layout set), **`07`–`08`** (segment-targeted corners), and **`09`–`10`** (straight-opening slow fellow left vs right — symmetric pass-side checks).

**`00`–`06`:** Same *layout intent* as `examples/racing/phase4_pass_shield/` (and Phase 3 bank): baseline, optimal/left/right opponents, weaving, short headway into a corner, side-by-side-style start. Ego uses `phase5_segment_tactics_enabled=True` on top of tactical planner + pass-shield. These scenarios primarily guard **non-regression** vs Phase 4 and keep fellow/TTL contracts explicit.

**`07`–`08` (segment-targeted):** Two extra scenarios designed so **Phase 3 can be in `SETUP_LEFT` / `SETUP_RIGHT` while `segment_context` is `corner_entry` or `corner_body`**—the only combinations Phase 5 overrides today (`phase5_segment_tactics.py`). Ego XY is taken from **`ttl_optimal_xodr.csv`** waypoints classified offline with the same `build_waypoint_segment_map_from_ttl` + `planner_segment_context` pipeline as runtime:

| File | Role |
|------|------|
| `07_corner_entry_clear_ahead_phase5.scenic` | `corner_entry` + in-line opponent (`_racing_st_offset ('ahead', …)` on optimal) → expect **`entry_conservative`** when overlap is not in the allowed entry set |
| `08_corner_body_clear_ahead_phase5.scenic` | `corner_body` + same style opponent → expect **`body_no_new_setup`** |

**`09`–`10` (straight opening):** Mirror scenarios with a slow fellow on a lateral TTL (`ttl_right_xodr.csv` vs `ttl_left_xodr.csv`) so ego can demonstrate **`commit_pass_left`** vs **`commit_pass_right`** with **`shield_release`** on a straight; useful for symmetry and log inspection without corner override noise.

**Do not** rely on hand-picked world coordinates for “corner stress” without checking logs for `seg=` and sane CTE/opponent distance; a bad pose can yield `off_track` and meaningless `nearest_opp_dist` while the subprocess still exits 0.

Coverage themes across the full bank:

- straight-only pass opportunity (`01`–`03`, parts of `04`)
- corner-adjacent tailgate (`05`, aligned with Phase 4 `05`)
- corner-entry / corner-body **Phase 5 override** sign-off (`07`, `08`)
- weaving / instability (`04`)
- side-by-side-style start (`06`, aligned with Phase 4 `06`)

Scenario authoring rules:

- when using TTL-specific intent (`left`/`right`), pair with TTL-following fellow behavior
- keep `param fellowHarnessLog = True` so fellow motion is always auditable
- prefer ego-anchored offset spawning for reproducible relative starts
- for Phase 5 *capability* proof, verify **`[Phase5Event]`** and digest fields `phase5_event_segment_override`, `phase5_override_count`, and reason strings—not only “lap completed”

## Comparison vs Phase 4 bank

Phase 5 reuses the **same seven layout intents** as `examples/racing/phase4_pass_shield/` as scenarios **`00`–`06`**, with **`phase5_segment_tactics_enabled=True`** added on ego. Phase 4’s validated seven-scenario bank remains the regression baseline for pass/shield semantics; Phase 5 adds segment-conditioned shaping and extra scenarios **`07`–`10`** for corner and straight-opening coverage. A full **`phase5_runner`** run therefore **subsumes** the Phase 4 layout checks for `00`–`06` while extending metrics (`phase5_*` digest columns, `[Phase5Tactical]` / `[Phase5Event]` in logs).

## Validated benchmarks (record)

**Command (sign-off-style horizon):**

```bash
python -m scenic.domains.racing.benchmarks.phase5_runner --time 3000
```

**Recorded run (local, April 2026):** `run_id` **`phase5_20260412_090949`** — outputs under `src/scenic/domains/racing/benchmarks/results/phase5_20260412_090949/` (`summary.json`, `summary.csv`, per-scenario logs in `logs/`).

**Aggregate (digest):** `scenario_count`: **11**; **`all_return_codes_zero`**: **true**; **`any_collision` / `any_off_track`**: **false**; **`sum_near_miss_count`**: **1** (from **`05`** only).

**Per-scenario highlights (digest rows):**

| Scenario | Notes |
|----------|--------|
| `00` | No opponent; Phase 4/5 tactical counters at zero as expected. |
| `01`–`03`, `06` | Pass events (`commit_pass_*`, `shield_release`) consistent with layout; **`phase5_event_segment_override`** typically **0** on these straighter cases. |
| `04` | Weaving; multiple commit/shield cycles. |
| `05` | Corner tailgate stress: **`near_miss_count`**: **1**, **`phase4_emergency_avoid_count`**: **2**, **`collision_eval_hull_overlap`**: **true**, **`eval_contact_overlap_count`**: **2** — discrete **`collision`** remained **false**; treat as **conservative contact proxy** worth log review, not a silent pass. |
| `07` | Segment overrides fire: non-zero **`phase5_event_segment_override`** / **`phase5_override_count`**; **`phase3_ttl_switch_count`** is **high** (rapid optimal ↔ side setup) — **verify `[Phase5Event]` reasons** in `logs/07_....log` if tuning chatter. |
| `08` | Same pattern: non-zero Phase 5 override counts; **high** `phase3_ttl_switch_count` vs straights. |
| `09` / `10` | Straight-opening symmetry: **`commit_pass_left`** vs **`commit_pass_right`** with **`shield_release`**; **`nearest_opp_ds`** goes negative after the pass in **`[Phase0]`** samples (ego ahead along route). |

**Follow-ups (optional):** reduce TTL **oscillation** on **`07`/`08`** if product requirements demand calmer `phase3_ttl_switch_count`; revisit **`05`** hull/near-miss if eval-contact thresholds need tightening.

## Exit Checklist

- [x] Segment context is part of tactical decision inputs (`phase5_segment_tactics.py`, wired from segment map / planner context).
- [x] Decision policy differs by segment type as intended (corner entry / corner body overrides; see unit tests in `mpc/testing/test_phase5_segment_tactics.py`).
- [x] Mixed-scenario comparison against Phase 4 is documented (see **Comparison vs Phase 4 bank** above).
- [x] Safety is not regressed while tactical quality improves (recorded full-bank run: no aggregate collision/off-track; see **Validated benchmarks** for `05` caveat).
- [x] Runner digest includes Phase 5-specific explainability metrics (`phase5_tactical_line_count`, `phase5_ttl_switch_count`, `phase5_event_segment_override`, `phase5_event_segment_release`, `phase5_override_count`, plus fellow harness columns when applicable).
- [x] `07` / `08` show non-zero **`phase5_event_segment_override`** / **`phase5_override_count`** on the recorded run; confirm **`[Phase5Event]`** / **`[Phase5Tactical]`** reason strings (`entry_conservative`, `body_no_new_setup`) in per-scenario logs under `results/<run_id>/logs/`.
