# Racing examples

Scenarios for the racing domain using the dSPACE racing simulator. All examples use `scenic.simulators.dspace.racing_model` and the Laguna Seca map unless noted.

| Example | Description |
|--------|-------------|
| **ego_mpc_behavior.scenic** | Ego follows the TTL with `FollowRacingLineMPCBehavior` (MPC; recommended). Uses TTL waypoints from `LS_ENU_TTL_CSV` (e.g. `ttl_main_road.csv` or `ttl_pitlane.csv`) and 100 Hz sim / 20 Hz control. Ego route (Lap vs Pit) is chosen by TTL distance; when similar, main road is preferred. |
| **ego_fixed_placing.scenic** | Single ego at fixed coordinates (no behavior). |
| **fellow_fixed_placing.scenic** | Ego + fellows at fixed waypoint positions along the track. |
| **three_segments.scenic** | Vehicles on `mainRacingRoad` vs `pitLaneRoad`; route assignment from OpenDRIVE. |
| **test_relative.scenic** | Fellows placed relatively (ahead/behind). |
| **ego_calibration_accel_decel.scenic** | Throttle/brake calibration for MPC tuning; prints acceleration/deceleration for `vehicle_mpc.yaml`. |
| **decision_tree_example.scenic** | Decision-tree behaviors: `FlagBasedSpeedBehavior`, `LaneSelectionBehavior`, `StopBehavior`, `FollowModeBehavior`. |
| **phase0_benchmark/** | Phase 0 baseline scenario bank + runner-oriented set (no opponent, slower opponent variants, weaving, corner approach, side-by-side). |
| **fellow_smoke/** | Fellow / traffic harness (not an ego phase): placement + optional `[FellowHarness]` readback; `fellow_runner` (default **2000** steps ≈ **20 s**). |
| **fellow_placement_debug/** | Focused placement repro bank: commanded `_racing_st_offset` vs observed spawn/road projection; `fellow_placement_debug_runner` (supports `--repeats`). |
| **phase1_planner/** | Phase 1 planner-to-MPC scripted handoff tests (optimal->left, left->right, right->optimal). |
| **phase2_assessment/** | Phase 2 situation-assessment smoke scenarios + `phase2_runner` metrics (`[Phase2]` logs). |
| **phase3_tactical/** | Phase 3 tactical planner — **full bank** (same `00`–`06` layouts as **phase0_benchmark**); `phase3_runner`. Optional alias: `phase3_on_phase0_runner` (same scenarios/KPIs). |
| **phase4_pass_shield/** | Phase 4 pass commit / abort / shield (`pass_commit_shield_enabled=True`); `phase4_runner`. Seven scenarios (`00`–`06`), same layouts as **phase0_benchmark** with tactical + pass-commit shield on ego. |
| **phase5_segments/** | Phase 5 segment-aware tactics (`phase5_segment_tactics_enabled=True`); bank **`00`–`06`** mirrors Phase 4 layouts, plus **`07`–`08`** (TTL-derived **corner_entry** / **corner_body** poses) and **`09`–`10`** (straight-opening slow-fellow left/right symmetry); `phase5_runner`. |
| **f_shared/** | Shared F-scenario bank (`F0`..`F8`) for post-Phase-5 work; avoids cloning nearly identical phase test folders and is reusable across runners. |

Run with the racing model, e.g.:

```bash
scenic examples/racing/ego_mpc_behavior.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1
```

Run the full Phase 0 benchmark bank:

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner
```

(`--time` defaults to **2000** simulation steps ≈ **20 s** at 0.01 s/step; pass `--time 3000` (~30 s) or any `N` to override. Default `--inter-run-delay-s` is **15**; use `--scenario` / `--scenario-glob` to run a subset.)

Run the full implemented stack in one go (same flags forwarded to each runner):

```bash
python -m scenic.domains.racing.benchmarks.run_all_benchmarks_so_far
```

Skip the fellow harnesses and Phase 0; run **Phase 1 through Phase 6** only (same forwarded flags, e.g. `--time 2000`):

```bash
python -m scenic.domains.racing.benchmarks.run_all_benchmarks_so_far --from phase1 --time 2000
```

(`--from` accepts `fellow_smoke`, `fellow_placement`, `phase0` … `phase6`, plus aliases `smoke` and `placement`.)

This combined runner now executes: `fellow_runner`, `fellow_placement_debug_runner`, and `phase0_runner` through `phase6_runner` (in sequence).

**Validation full-stack runner** (post–Phase 5 stress / regression campaign): runs **phase0 → phase5 → fellow** in one parent results folder, merges every child `summary.json` into **`merged_summary.json`**, and prints a **single** combined `BENCHMARK_AI_DIGEST_*` (rows include `source_child` and `child_run_id`). Plan: `src/scenic/domains/racing/plans/comprehensive-planner-validation-runner.md`.

```bash
python -m scenic.domains.racing.benchmarks.validation_full_stack_runner
python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --time 3000
python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --suite phases_only
python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --suite minimal --time 2000
python -m scenic.domains.racing.benchmarks.validation_full_stack_runner --continue-on-failure --skip-placement
```

- **`--suite`:** `all` (default: phases + fellow_smoke + fellow_placement), `phases_only`, `minimal` (phase0 + phase5 + fellow_smoke), `fellow_only`.
- Other flags (`--time`, `--inter-run-delay-s`, `--scenario`, **`--repeats`**, …) are **forwarded** to each child runner. **`--repeats 3`** runs every scenario three times (phase0/phase1 now support this; phase2+ already did).
- Output: `benchmarks/results/validation_full_stack_<timestamp>/merged_summary.json` plus per-child subfolders.

### Known hard regression cases (priority stress set)

Recent full-stack campaigns have repeatedly exposed a small set of high-risk
corner/overlap interactions that should be treated as **must-check** regression
cases whenever tactical or safety logic changes:

- `phase3_tactical/05_opponent_just_ahead_corner.scenic`
- `phase4_pass_shield/05_opponent_just_ahead_corner_pass_shield.scenic`
- `phase5_segments/05_opponent_just_ahead_corner_segment_tactics.scenic`
- `phase5_segments/08_corner_body_clear_ahead_phase5.scenic`

Why these matter:

- they combine short headway + corner geometry where overlap can emerge quickly;
- they stress mode transitions (`SETUP_*`, `FOLLOW`, shield emergency/abort);
- they are the most likely to reveal TTL switch chatter or late shield release.

Recommended targeted loop before broad reruns:

```bash
python -m scenic.domains.racing.benchmarks.phase3_runner --scenario 05_opponent_just_ahead_corner.scenic --time 2000
python -m scenic.domains.racing.benchmarks.phase4_runner --scenario 05_opponent_just_ahead_corner_pass_shield.scenic --time 2000
python -m scenic.domains.racing.benchmarks.phase5_runner --scenario 05_opponent_just_ahead_corner_segment_tactics.scenic --time 2000
python -m scenic.domains.racing.benchmarks.phase5_runner --scenario 08_corner_body_clear_ahead_phase5.scenic --time 2000
```

Use these as acceptance checks in addition to aggregate `--suite phases_only`
validation runs.

**Fellow vs TTL:** Scenario files set ``param fellowHarnessLog = True`` so logs can include ``[FellowHarness]`` readback alongside ego ``[Phase0]`` / ``[Phase2]``. ``ttlFileName`` on a fellow attaches route/polyline — it does not by itself mean the fellow "follows optimal vs left vs right TTL" as a planner; see ``examples/racing/fellow_smoke/README.md`` (**TTL files and fellow behaviors**).

Run the **fellow / traffic harness** (fellow placement + optional `[FellowHarness]` metrics; not a numbered ego phase):

```bash
python -m scenic.domains.racing.benchmarks.fellow_runner
```

Default **`--time` is 2000** steps (~**20 s** at 0.01 s/step). Same CLI flags as other phase runners (`--scenario-dir`, `--scenario`, `--scenario-glob`, `--time`, `--inter-run-delay-s`, `--out-dir`). See `examples/racing/fellow_smoke/README.md`.

- **Fellow placement debug runner (repro-focused):**
```bash
python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner
python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner --repeats 5
```

Run all Phase 1 scripted TTL-switch validation scenarios:

```bash
python -m scenic.domains.racing.benchmarks.phase1_runner
```

(Same delay and filtering flags as Phase 0; default **`--time` is 2000** steps. See `examples/racing/phase1_planner/README.md`.)

Phase 2–6 use the same CLI pattern (`--scenario-dir`, `--scenario`, `--scenario-glob`, `--time`, `--inter-run-delay-s`, `--out-dir`):

```bash
python -m scenic.domains.racing.benchmarks.phase2_runner
python -m scenic.domains.racing.benchmarks.phase3_runner
python -m scenic.domains.racing.benchmarks.phase4_runner
python -m scenic.domains.racing.benchmarks.phase5_runner
python -m scenic.domains.racing.benchmarks.phase6_runner
```

`phase3_on_phase0_runner` is a backward-compatible alias (same bank and KPIs as `phase3_runner`; legacy `run_id_prefix`). Prefer **`phase3_runner`** for new scripts.

**Default horizon:** phase runners use **2000** steps (~**20 s** at 0.01 s/step) unless you pass `--time`. Use **`--time 3000`** (~30 s) when you need a longer run (e.g. closer to a full lap or parity with older sign-off runs).

### Phases 4–5: scenarios vs runner code

Each phase runner uses the matching folder in this table as its default `--scenario-dir`. **Every `*.scenic` file in that folder is run automatically** (sorted by name). If you add or generate a new example (for example `examples/racing/phase4_pass_shield/02_my_case.scenic`), you do **not** need to edit `phase4_runner.py` to “register” the filename—the next full bank run will include it.

**Debugging runs:** If a scenario is hard to interpret from aggregate KPIs alone, add temporary `print` lines in the relevant behavior or Python helper (for example `[Phase4Event]` / `[Phase4Tactical]` in `FollowRacingLineMPCBehavior`), re-run the same `phaseN_runner` command, and inspect the per-scenario log under `benchmarks/results/<run_id>/logs/`. Remove or gate noisy prints before merging if they spam every control tick.

When you **implement** a phase and introduce new log lines or KPIs, **do** go back and update:

1. **`src/scenic/domains/racing/benchmarks/phaseN_runner.py`** — adjust `PhaseRunnerSpec`: `csv_fields`, and flags such as `phase1_switches` / `phase2_lines` / `phase3_tactical` if parsers should change.
2. **`src/scenic/domains/racing/benchmarks/phase_run_common.py`** — extend `collect_metrics_from_log` (and regexes) so new tags produce summary/CSV columns you care about.

Optional: in CI or docs, pin a subset with `--scenario file.scenic` for a stable smoke set; the full bank remains “all files in the phase directory.”

### Sharing benchmark output (terminal / AI / logs)

After **phase0**, **phase1**, **phase2–5**, or **phase3** runners finish, the terminal prints a single JSON line between **`BENCHMARK_AI_DIGEST_BEGIN`** and **`BENCHMARK_AI_DIGEST_END`**. That object has `schema: "benchmark_ai_digest_v1"`, `aggregate` rollups, and per-scenario `rows` (flat KPIs). Copy that whole block when sharing results, or attach `summary.json` under the path printed as `paths.run_dir` in the digest.

During the run, after each scenario’s one-line summary, the runner also prints **`Log file:`** with the **absolute path** to that scenario’s captured log (`benchmarks/results/<run_id>/logs/<scenario_stem>.log`), so you can open it directly while debugging.

- **Phase 3 tactical (full bank, same seven layouts as `phase0_benchmark/`, ego with `tactical_planner_enabled=True`):**  
  `python -m scenic.domains.racing.benchmarks.phase3_runner`  
  (Default **`--time` is 2000** steps; use **`--time 3000`** or higher if a scenario needs more simulated time for a full lap.)

**Phase 3 sign-off (record):** A full-bank dSPACE run at **3000** steps (longer horizon than the current default) reported **no collisions, no off-track, no near-miss** in aggregate; see `src/scenic/domains/racing/plans/phase-3-smart-follow-and-stable-ttl.md` (**Validated benchmarks**).

**Phase 5 sign-off (record):** Full bank at **3000** steps (`phase5_runner`), **11** scenarios (`00`–`10`), recorded run id **`phase5_20260412_090949`** — all scenarios completed with **no collision / off-track** in aggregate; see `src/scenic/domains/racing/plans/phase-5-segment-aware-tactics.md` (**Validated benchmarks (record)**) for digest highlights and caveats (`05` near-miss / hull overlap proxy; high `phase3_ttl_switch_count` on `07`/`08`).
