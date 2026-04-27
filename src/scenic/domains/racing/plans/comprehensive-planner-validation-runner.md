# Comprehensive ego planner validation & stress-test campaign

## Purpose

Phase 0–5 development used **targeted** benchmark banks per feature. This document defines a **second layer**: broad, repeatable testing whose goal is to **find weaknesses** in the full Scenic ego stack—**MPC line following**, **tactical TTL choice**, **pass / abort / shield**, **segment-aware shaping**, and **instrumentation**—not to add new features.

**Scope:** One dynamic opponent (plus ego), consistent with [Deferred scope](./deferred-scope.md) and `overall_plan.md`.

## Goals

1. **Coverage:** Exercise as many distinct interaction modes, track regions, and fellow behaviors as practical in simulation.
2. **Regression safety:** Any change to `behaviors.scenic`, MPC config, or planner Python should be run against this suite before merge when touching racing control.
3. **Weakness discovery:** Surface instability (TTL chatter, projection stuckness, emergency spikes), borderline safety (near-miss, hull overlap), and performance cliffs (lap time, abort storms).
4. **Reporting:** One **unified** run id with `summary.json` / digest and per-scenario logs, comparable across runs.

## What “full stack” means here

| Layer | What we stress |
|--------|----------------|
| **Control / MPC** | CTE, reference continuity, curvature-speed gate, gear logic, pit vs main segment limits |
| **Phase 2** | Situation assessment stability (`overlap`, `seg_ctx`, flicker) |
| **Phase 3** | Tactical modes, TTL switches, follow caps |
| **Phase 4** | Commit / abort / shield, `EvalContact` / hull proxy |
| **Phase 5** | Segment overrides, `entry_conservative` / `body_no_new_setup` where applicable |
| **Fellow harness** | Placement correctness, speed profile sanity (`fellow_*` digest fields) |

## Relationship to existing banks

Do **not** duplicate phase logic in ad-hoc scenarios without reason. Prefer:

1. **Compose** the validation run from **existing directories** in sequence (Phase 0 → 5 banks, fellow smoke, placement debug as needed).
2. **Add** only scenarios that are **not** already represented—see [Scenario inventory](#suggested-scenario-inventory) below.

The deliverable “comprehensive runner” should either:

- **Option A — Orchestrator:** A Python module (e.g. `validation_full_stack_runner.py`) that invokes existing `phaseN_runner` modules or `run_phase_main` with fixed scenario dirs and merged output; or  
- **Option B — Unified folder:** `examples/racing/validation_full_stack/` containing symlinks or thin wrappers—usually worse for maintenance.

**Recommendation:** **Option A** — one runner that sequences calls and merges digests, so phase banks stay the single source of truth.

## Suggested scenario inventory

Below is a **checklist of test *ideas***. Items marked **(bank)** are already covered by an existing phase folder—include them by running that runner. Others are **candidates** for new `.scenic` files under a dedicated `examples/racing/validation_extra/` (or similar) if gaps appear.

### A. Ego-only / control sanity

| ID | Intent | Notes |
|----|--------|--------|
| A1 | Ego lap, no opponent | Phase 0 `00` **(bank)** |
| A2 | Long horizon ego-only | Same scenario, `--time` 4000+ to stress MPC over distance |
| A3 | Pit road excursion (if scene supports) | Optional; pit yield behavior |

### B. Opponent placement & route contract

| ID | Intent | Notes |
|----|--------|--------|
| B1 | Slower on optimal | Phase 0/3/4/5 `01` **(bank)** |
| B2 | Slower on left TTL | `02` **(bank)** |
| B3 | Slower on right TTL | `03` **(bank)** |
| B4 | Weaving | `04` **(bank)** |
| B5 | Just ahead into corner | `05` **(bank)** |
| B6 | Side-by-side start | `06` **(bank)** |
| B7 | Fellow **TTL-geometric** vs **constant offset** | fellow_smoke / harness scenarios |
| B8 | Extreme `ahead` / `behind` spacing | placement debug matrix **(bank)** |
| B9 | Lateral offset only (`left`/`right` spawn) | Phase 5 `09`/`10` style **(bank)** |

### C. Speed & closing dynamics

| ID | Intent | Notes |
|----|--------|--------|
| C1 | Much slower fellow (pass opportunity) | Covered indirectly by `01`–`03`; add **extra** with very low `speed_mph` if needed |
| C2 | Similar speed (weak pass) | New scenario: fellow ~ ego target |
| C3 | Faster closing from behind | New scenario: behind + positive relative speed (if supported) |
| C4 | Sudden deceleration fellow | fellow_smoke patterns if available |

### D. Segment context (Phase 5)

| ID | Intent | Notes |
|----|--------|--------|
| D1 | Corner entry + opponent | `07` **(bank)** |
| D2 | Corner body + opponent | `08` **(bank)** |
| D3 | Straight opening + lateral fellow | `09`/`10` **(bank)** |

### E. Safety & shield (Phase 4)

| ID | Intent | Notes |
|----|--------|--------|
| E1 | Clean pass + shield release | Typical `01`–`03` |
| E2 | Abort / emergency paths | Stress `05`, weaving `04` |
| E3 | `collision_eval_hull_overlap` without discrete collision | Review digest + `[EvalContact]` |

### F. Stability & “bad smells” (weakness hunting)

| ID | What to look for | Log / digest |
|----|------------------|--------------|
| F1 | TTL **chatter** | `phase3_ttl_switch_count`, `phase5_ttl_switch_count` very high vs peers |
| F2 | **Near-miss** / hull near | `near_miss_count`, `eval_contact_*`, `collision_eval_hull_overlap` |
| F3 | **Emergency avoid** storms | `phase4_emergency_avoid_count` |
| F4 | **Projection stuck** (MPC) | `[FollowRacingLineMPC]` ref_log / `projection_check STUCK?` |
| F5 | **Off-track** | `off_track`, `[Phase0Event] off_track` |
| F6 | Fellow **harness** anomalies | `fellow_t_out_of_band`, `fellow_speed_stuck_near_zero` |

### G. Reproducibility

| ID | Intent |
|----|--------|
| G1 | Same scenario, two runs—compare digest (lap time band, switch counts) |
| G2 | Seed sensitivity (if Scenic exposes seed)—optional |

### H. Python unit tests (fast, not dSPACE)

Already present: tactical, pass-shield, phase5 segment tactics, log metric parsers. **Extend** when new failure modes appear.

## Expected outcomes per scenario (`validation_full_stack_runner --suite all`)

Use this section **before** a hardware run to set expectations, and **after** a run to diff the digest / logs against predictions. Numbers are **order-of-magnitude** from scenario files (Laguna Seca, ego `target_speed` 60 mph unless noted). Useful conversion: **\(v[\mathrm{m/s}] \approx 0.447 \times v[\mathrm{mph}]\)** (e.g. 60 mph ≈ 26.8 m/s; 5 mph ≈ 2.2 m/s closing).

**Not a pass/fail spec:** predictions describe *likely* behavior given geometry and relative motion; stochastic seeding, MPC transients, and dSPACE timing can shift a run from “usually clean” to “borderline contact” without implying a broken stack.

### `baseline_runner` (`examples/racing/phase0_benchmark/`)

| File | Geometry / relative motion | Predicted outcome |
|------|------------------------------|-------------------|
| `00_no_opponent.scenic` | Ego only, optimal TTL | **`collision` false**, **`off_track` false**, **`waypoint_hits`** high (~full lap), **`ttl_switch_count`** 0. |
| `01_slower_opponent_optimal.scenic` | Fellow **ahead 40 m**, same TTL `ttl_optimal`, both **`FellowFollowTTLGeometricBehavior(speed_mph=60)`** vs ego 60 mph | Nominal **zero longitudinal closing** if both track at 60 mph; gap stays O(40 m). **`min_opponent_distance_m`** should stay well above 1 m unless a pass or lateral excursion occurs; **hull / discrete collision should be rare** (if repeat shows overlap, treat as **soft-fail** per digest). |
| `02_slower_opponent_left.scenic` | Ego optimal TTL vs fellow **left TTL**, ahead 40 m, 60 mph | **Parallel lanes**: large **lateral** separation; longitudinal gap ~40 m. **`collision` false** expected; **`min_opponent_distance_m`** moderate (several–tens of m) depending on lane spacing. |
| `03_slower_opponent_right.scenic` | Same as `02` but **right TTL** | Same as `02`. |
| `04_opponent_weaving_lightly.scenic` | Fellow **ahead 35 m**, **`FellowSwerveOutOfControlBehavior`** (±1.5 m lateral, 8 s period) | Fellow crosses laterally in front of ego’s corridor: **higher** `near_miss` / CTE risk than `01`; **`off_track`** or low **`waypoint_hits`** possible if ego evades aggressively. |
| `05_opponent_just_ahead_corner.scenic` | Fellow **ahead 12 m**, **55 mph** vs ego **60 mph** | Longitudinal closing ≈ **5 mph (~2.2 m/s)**. Over 30 s simulated horizon, closure **~66 m** if unconstrained—so ego must **brake or follow** in a short gap into a corner: **`off_track`** or follow stress **more likely** than in `01`; **`min_opponent_distance_m`** can become small under heavy braking. |
| `06_side_by_side_start.scenic` | Fellow **ahead 8 m**, fellow **`ttl_left`** at **62 mph** vs ego optimal **60 mph** | **Tight** initial gap; fellow slightly faster on a **different** TTL → **side-by-side / overlap** dynamics. Expect **low `waypoint_hits`** or **`off_track`** more often than `01–03`; **`collision` false** if lateral separation holds, but **borderline** contacts are more plausible than in `01`. |

### `scripted_runner` (`examples/racing/phase1_planner/`)

All three use **`planner_enabled=True`** and a **single scripted TTL switch at simulation time 10 s** (`ttl_schedule`). Ego starts on different TTLs; the switch forces a **large lateral transition** on a ~30 s horizon.

| File | Schedule | Predicted outcome |
|------|----------|-------------------|
| `01_optimal_to_left.scenic` | `10:left` from optimal | **`phase1_switch_observed` true**, **`ttl_switch_count`** ≥ 1. During/after the transition, **CTE / `off_track` flags are common** (digest often shows **`off_track` true** with modest **`waypoint_hits`**)—that is **consistent with a lane-change test**, not necessarily a regression. |
| `02_left_to_right.scenic` | `10:right` (ego starts **`ttl_left`**) | Same pattern as `01`. |
| `03_right_to_optimal.scenic` | `10:optimal` (ego starts **`ttl_right`**) | Same pattern as `01`. |

### `opponent_runner` (`examples/racing/phase2_assessment/`)

| File | Notes | Predicted outcome |
|------|-------|-------------------|
| `01_smoke_opponent.scenic` | Same layout as phase 0 `01` (ahead 40 m, geometric 60 mph); **Phase 2 assessment** logging | **`phase2_line_count` / overlap / seg_ctx** non-zero and consistent across **`phase0_samples`**; **`phase2_assess_errors`** 0. **`off_track`** may still appear (same ego path as phase 0 bank). |

### `tactical_runner` (`examples/racing/phase3_tactical/`)

Mirrors phase 0 bank but ego has **`tactical_planner_enabled=True`** (no pass/shield unless noted in file).

| File | Predicted outcome (vs phase 0 analog) |
|------|----------------------------------------|
| `00_no_opponent.scenic` | Like phase 0 `00`; **tactical** may log **`phase3_tactical_status_count`** / **`phase3_ttl_switch_count`** at low levels if idle. |
| `01`–`06` | Same geometry as phase 0 `01`–`06`. Expect **non-zero tactical / TTL activity** when overlap and pass band justify it; **`phase3_ttl_switch_count`** may exceed phase 0 for multi-lane cases. **Closing / weaving / corner** cases remain **stressful** in the same order as phase 0 (`05`, `06`, `04` hardest). |

### `phase4_runner` (`examples/racing/phase4_pass_shield/`)

Ego uses **`tactical_planner_enabled=True`** and **`pass_commit_shield_enabled=True`**. Same scenario names as phase 0; expect **Phase 4 log lines** and digest fields **`phase4_*`** (commit / abort / shield / emergency).

| File | Predicted outcome |
|------|-------------------|
| `00_no_opponent_pass_shield.scenic` | Baseline: **no** pass/commit storms; **`phase4_emergency_avoid_count`** ~0. |
| `01`–`03` | **Pass band** scenarios: some **`phase4_commit_pass_count`** or **`phase4_event_*`** possible when ego commits to a pass; **`phase4_emergency_avoid_count`** low if behavior is smooth. |
| `04_opponent_weaving_lightly_pass_shield.scenic` | **Weaving** → higher chance of **`phase4_abort_pass_count`** / **emergency** spikes vs `01`. |
| `05_opponent_just_ahead_corner_pass_shield.scenic` | Short gap + corner → **strong follow / abort / shield** activity vs `01`; **more** emergency or abort events than straight scenarios. |
| `06_side_by_side_start_pass_shield.scenic` | Side-by-side → **shield / commit** logic exercised; overlap classification may drive **tactical + Phase 4** lines. |

### `phase5_runner` (`examples/racing/phase5_segments/`)

Ego adds **`phase5_segment_tactics_enabled=True`**. Corner scenarios (`07`, `08`) use **spawn poses** on specific track segments (see file headers). **`09` / `10`**: ego optimal vs fellow **right/left TTL** at **20 mph**, **ahead 45 m** → large closing speed (~40 mph ≈ **18 m/s**), so **expect segment overrides / bypass-style tactics** and non-zero **`phase5_*`** counts when segment context matches.

| File | Predicted outcome |
|------|-------------------|
| `00_no_opponent_segment_tactics.scenic` | Phase 5 machinery may stay **quiet** (`phase5_event_*` ~0) if no segment-triggering opponent interaction. |
| `01`–`06` | Analogous to phase 3/0 geometry with **segment-aware** reasons (`entry_conservative`, `body_no_new_setup`, etc.) appearing in logs when segment + overlap match. |
| `07_corner_entry_clear_ahead_phase5.scenic` | Spawn at corner entry; fellow **ahead 20 m**, **52 mph** vs ego **60 mph** → **~8 mph (~3.6 m/s)** closing; **`phase5_event_segment_override`** / conservative corner behavior **likely** if assessment says corner entry. |
| `08_corner_body_clear_ahead_phase5.scenic` | Corner **body** context; expect **different** Phase 5 reason mix than `07`. |
| `09_straight_slow_fellow_right_opening_phase5.scenic` | **Straight**, fellow on **right TTL** at **20 mph** → ego should have **room to bypass left**; **`phase5_ttl_switch_count`** or override events **plausible**; **`collision` false** if pass completes. |
| `10_straight_slow_fellow_left_opening_phase5.scenic` | Mirror of `09` (fellow left, opening on opposite side). |

### `fellow_runner` (`examples/racing/fellow_smoke/`)

| File | Predicted outcome |
|------|-------------------|
| `00_ego_only_baseline.scenic` | No fellow harness pressure; clean baseline. |
| `01_fellow_ahead_constant_offset.scenic` | Constant lateral offset behavior; **`fellow_*`** digest fields populated; **`fellow_t_out_of_band` false**. |
| `02_fellow_behind_constant_offset.scenic` | Fellow **`behind` 30 m**; ego opens gap; **no** forward conflict. |
| `03` / `04` | **`left` / `right` 3 m** lateral spawn; placement and harness lines **observed**. |
| `05_fellow_ttl_geometric.scenic` | Same class as phase 0 `01` (ahead 40 m, geometric 60 mph). |
| `06_fellow_weaving.scenic` | Same weaving family as phase 0 `04`; **more** dynamic fellow than `05`. |
| `07_fellow_sudden_stop_interval.scenic` | **Periodic braking** (10 s interval, 3 s hold): **high risk** of **`near_miss`**, **`phase4_emergency_avoid_count`**, or strong decel—**not** expected to look like a calm `01`. |

### `fellow_placement_debug_runner` (`examples/racing/fellow_placement_debug/`)

Focus: **`fellow_placement_from_ego_offset_observed`**, **`[Fellow s,t]`** consistency, **`fellow_t_out_of_band` false**. Variants probe **ahead / behind / lateral** and **pit-adjacent** spawns.

| File | Predicted outcome |
|------|-------------------|
| `00_no_opponent_baseline.scenic` | No opponent; fellow fields null or unused. |
| `01_ahead_40_main_straight.scenic` | Canonical **ahead 40 m** on optimal; **small** placement error. |
| `02_behind_40_main_straight.scenic` | **Behind** 40 m; ego pulls away; harness still logs consistently. |
| `03_left_3p5_main_straight.scenic` / `04_right_3p5_main_straight.scenic` | **3.5 m** lateral; **`fellow_t0`** near **±** expected lateral band. |
| `05_ahead_40_near_pit_entry.scenic` / `06_behind_40_near_pit_exit.scenic` | Pit-adjacent route points: **watch** for larger **`fellow_position_range_m`** or outliers vs `01`. |
| `07_side_by_side_near_boundary.scenic` | Stress placement near **boundary**; **`fellow_t_out_of_band`** should remain **false** in a healthy harness. |
| `08_seed_stability_ahead_40.scenic` | Repeated concept: **stable** `fellow_s0` / `fellow_t0` across repeats if seeding is deterministic. |

## KPIs and pass / fail gates

Default **hard fail** (must fix before release of planner change):

- `return_code != 0`
- `collision == true` (discrete collision flag)
- `off_track == true` (if policy is zero tolerance)

**Soft fail / review** (investigate; may accept with waiver):

- `near_miss_count > 0`
- `collision_eval_hull_overlap == true`
- `phase4_emergency_avoid_count` high vs baseline for same scenario
- `phase3_ttl_switch_count` or `phase5_ttl_switch_count` **orders of magnitude** above sibling scenarios

**Baseline:** Store a reference digest (path + date) for “known good” after a clean campaign.

## Runner design (implementation spec)

### CLI

Mirror existing benchmarks:

- `--time`, `--time-step-s`, `--inter-run-delay-s`, `--out-dir`, `--scenario` filters where applicable.
- `--suite` optional: `all` | `phases_only` | `fellow_only` | `minimal_smoke` to shorten CI.

### Behavior

1. Run ordered list of sub-runners (see below).
2. Merge each child’s `summary.json` **results** into one array with a **`suite`** / **`source_runner`** field per row.
3. Emit one **`BENCHMARK_AI_DIGEST`** block and one merged `summary.json` at top-level `validation_<timestamp>/`.
4. Print per-sub-runner **Log file** hints or pointers to child `run_dir` for debugging.

### Proposed default sequence

1. `baseline_runner`  
2. `scripted_runner` (scripted switches—sanity)  
3. `opponent_runner`  
4. `tactical_runner`  
5. `phase4_runner`  
6. `phase5_runner`  
7. `fellow_runner` (subset if `--suite minimal`)  
8. Optional: `fellow_placement_debug_runner` with low `--repeats`

Implement via `subprocess` like `run_all_benchmarks_so_far.py`, but **merge artifacts** instead of stopping at first failure—or make **fail-fast** a flag (`--fail-fast`).

### Code locations (implemented)

- **Module:** `src/scenic/domains/racing/benchmarks/validation_full_stack_runner.py` — run with  
  `python -m scenic.domains.racing.benchmarks.validation_full_stack_runner`  
  (options: `--suite`, `--out-dir`, `--continue-on-failure`, `--skip-placement`; other flags forwarded to each child).
- Reuses: `build_benchmark_ai_digest_payload`, `print_benchmark_ai_digest`, `repo_root()`.
- Docs: `examples/racing/README.md` (**Validation full-stack runner**).

## Exit checklist (this plan)

- [x] `validation_full_stack_runner` implemented and documented.
- [ ] At least one **full** campaign completed; `summary.json` archived with run id.
- [ ] **Baseline** digest referenced in this doc or in `results/` README note.
- [ ] Gaps from [Scenario inventory](#suggested-scenario-inventory) triaged (new scenarios vs waived).
- [ ] Team agreement on **hard vs soft** gates for CI (if integrated into CI).

## Notes

- Longer simulation time (`--time 4000+`) is expensive; use for **nightly** or **manual** runs first.
- dSPACE / VEOS availability may limit automation; local policy may run **subset** on developer machines and **full** on hardware lab nights.
- Pit-heavy scenarios remain lower priority unless pit-aware planner work starts—see deferred scope.

## References

- [Phase 5 validated record](./phase-5-segment-aware-tactics.md#validated-benchmarks-record) — example of sign-off style.
- [Success definition](./success-definition.md)
- `examples/racing/README.md` — runner index
- `run_all_benchmarks_so_far.py` — sequencing pattern
