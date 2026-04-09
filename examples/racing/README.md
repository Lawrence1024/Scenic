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
| **phase1_planner/** | Phase 1 planner-to-MPC scripted handoff tests (optimal->left, left->right, right->optimal). |
| **phase2_assessment/** | Phase 2 situation-assessment smoke scenarios + `phase2_runner` metrics (`[Phase2]` logs). |
| **phase3_tactical/** | Phase 3 tactical planner (`tactical_planner_enabled`, FOLLOW / SETUP_*); `phase3_runner`. |
| **phase3_on_phase0_bank/** | Same cases as Phase 0 bank but ego has `tactical_planner_enabled=True`; `phase3_on_phase0_runner` (Phase 3 × Phase 0 cross-check). |
| **phase4_pass_shield/** | Phase 4 pass/shield (placeholder until implemented); `phase4_runner`. |
| **phase5_segments/** | Phase 5 segment planning (placeholder); `phase5_runner`. |
| **phase6_multi/** | Phase 6 multi-car (placeholder); `phase6_runner`. |

Run with the racing model, e.g.:

```bash
scenic examples/racing/ego_mpc_behavior.scenic --2d --model scenic.simulators.dspace.racing_model --simulate -b --count 1
```

Run the full Phase 0 benchmark bank:

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner
```

(`--time` defaults to **3000** simulation steps ≈ **30 s** at 0.01 s/step; pass `--time N` to override. Default `--inter-run-delay-s` is **15**; use `--scenario` / `--scenario-glob` to run a subset.)

Run all Phase 1 scripted TTL-switch validation scenarios:

```bash
python -m scenic.domains.racing.benchmarks.phase1_runner --time 3000
```

(Same delay and filtering flags as Phase 0; see `examples/racing/phase1_planner/README.md`.)

Phase 2–6 use the same CLI pattern (`--scenario-dir`, `--scenario`, `--scenario-glob`, `--time`, `--inter-run-delay-s`, `--out-dir`):

```bash
python -m scenic.domains.racing.benchmarks.phase2_runner --time 3000
python -m scenic.domains.racing.benchmarks.phase3_runner --time 3000
python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0
python -m scenic.domains.racing.benchmarks.phase4_runner --time 3000
python -m scenic.domains.racing.benchmarks.phase5_runner --time 3000
python -m scenic.domains.racing.benchmarks.phase6_runner --time 3000
```

All of these use **3000** steps by default except where you pass `--time`; `phase0_runner` and `phase3_on_phase0_runner` match that default so you can omit `--time` for the standard ~30 s simulated horizon.

### Phases 4–6 (not implemented yet): scenarios vs runner code

Each phase runner uses the matching folder in this table as its default `--scenario-dir`. **Every `*.scenic` file in that folder is run automatically** (sorted by name). If you add or generate a new example (for example `examples/racing/phase4_pass_shield/02_my_case.scenic`), you do **not** need to edit `phase4_runner.py` to “register” the filename—the next full bank run will include it.

When you **implement** a phase and introduce new log lines or KPIs, **do** go back and update:

1. **`src/scenic/domains/racing/benchmarks/phaseN_runner.py`** — adjust `PhaseRunnerSpec`: `csv_fields`, and flags such as `phase1_switches` / `phase2_lines` / `phase3_tactical` if parsers should change.
2. **`src/scenic/domains/racing/benchmarks/phase_run_common.py`** — extend `collect_metrics_from_log` (and regexes) so new tags produce summary/CSV columns you care about.

Optional: in CI or docs, pin a subset with `--scenario file.scenic` for a stable smoke set; the full bank remains “all files in the phase directory.”

### Sharing benchmark output (terminal / AI / logs)

After **phase0**, **phase1**, **phase2–6**, or **phase3_on_phase0** runners finish, the terminal prints a single JSON line between **`BENCHMARK_AI_DIGEST_BEGIN`** and **`BENCHMARK_AI_DIGEST_END`**. That object has `schema: "benchmark_ai_digest_v1"`, `aggregate` rollups, and per-scenario `rows` (flat KPIs). Copy that whole block when sharing results, or attach `summary.json` under the path printed as `paths.run_dir` in the digest.

- **Phase 3 only (one tactical scenario):**  
  `python -m scenic.domains.racing.benchmarks.phase3_runner --time 3000`
- **Phase 3 on full Phase 0 bank (tactical on ego for all Phase 0 layouts):**  
  `python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0`  
  (Default **`--time` is 3000** steps; increase if a scenario needs more simulated time for a full lap. This cross-check is the same seven scenarios as `phase0_benchmark/`, with **`tactical_planner_enabled=True`** on ego.)

**Phase 3 sign-off (record):** A full-bank dSPACE run at **3000** steps reported **no collisions, no off-track, no near-miss** in aggregate; see `src/scenic/domains/racing/plans/phase-3-smart-follow-and-stable-ttl.md` (**Validated benchmarks**).
