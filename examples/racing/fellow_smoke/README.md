# Fellow / traffic harness smoke bank

This folder is **not** a numbered ego phase (Phases 0–6). It exists to validate **fellow placement** (`[Placement]`, `[Fellow s,t]`) and optional **`[FellowHarness]`** readback lines from dSPACE, using the dedicated runner:

```bash
python -m scenic.domains.racing.benchmarks.fellow_runner
```

## Default simulated horizon

The runner defaults to **`--time 2000`** simulation steps. At **`time_step = 0.01` s**, that is **~20 s** of simulated time per scenario (same default as **Phase 0–4** benchmark runners). Pass **`--time 3000`** (~30 s) for a longer horizon when needed.

## Scene parameter: `fellowHarnessLog`

Scenarios here set **`param fellowHarnessLog = True`** so the simulator emits periodic **`[FellowHarness] t=…s idx=… speed_mps=… x=… y=…`** lines (throttled in the fellow readback path). Normal racing scenarios leave this **off** by default to avoid log noise.

Fellow behaviors use a consistent cruise speed (**60 mph**) where applicable so speed-based heuristics in the harness summary are interpretable. The sudden-stop scenario (**07**) intentionally drives commanded speed down during the stop phase; **`fellow_speed_stuck_near_zero`** may flag there even when behavior is correct—treat it in context.

## TTL files and fellow behaviors (avoid a common mistake)

**`ttlFileName` / `ttlFolder` on a fellow are not “which racing line the fellow obeys” in the ego-planner sense.**

- They attach the **TTL polyline / waypoints** used for **route**, **(s,t) projection**, segment context, and behaviors that **read TTL geometry** (for example lateral **d** from δ(s)).
- **`FellowConstantSpeedTrackOffsetBehavior`** commands the **(v, d) plant** with **v** from **`speed_mph`** and **d** from **placement / track-offset geometry** — it does **not** switch the fellow among optimal / left / right TTL CSVs as a tactical planner. Assigning `ttl_optimal_xodr.csv` to the opponent does **not** by itself mean “the fellow drives the optimal racing line” in the same way ego’s MPC does; it means that polyline is available for projection and for plant behaviors that use it.
- To command lateral **d** from **TTL geometry** (feedforward δ(s) on the main line), use **`FellowFollowTTLGeometricBehavior`** (see **`05_fellow_ttl_geometric.scenic`** vs **`01_…`** with constant offset).

Phase benchmark scenarios under **`examples/racing/phase0_benchmark/`** … **`phase4_pass_shield/`** repeat this pattern: match **behavior** to what you want to test; use **`param fellowHarnessLog = True`** when you need **`[FellowHarness]`** in logs for readback evidence. **Ego** reaction is visible via **`[Phase0]`**, **`[Phase2]`**, and benchmark **`summary.json`**; fellows do not report collisions — infer interaction from **relative pose** and ego-side metrics.

## On-disk results (same contract as other phase runners)

Each run writes under `src/scenic/domains/racing/benchmarks/results/<run_id>/`:

| Path | Contents |
|------|----------|
| `logs/<scenario_stem>.log` | Full stdout (Scenic, placement, optional `[FellowHarness]`, ego `[Phase0]` / `[Phase2]` if present) |
| `summary.json` | Per-scenario rows with parsed KPIs |
| `summary.csv` | Same columns for spreadsheets |

The terminal also prints **`BENCHMARK_AI_DIGEST_BEGIN` … `END`** (single-line JSON).

## Sharing results for analysis

1. Run the harness; optionally capture a **screenshot** of the terminal showing the digest block and the **“Wrote … summary.json”** line.
2. The digest includes **`paths.run_dir`** (see `print_benchmark_ai_digest` in `phase_run_common.py`).
3. For detailed review, use that directory (or synced copies of **`summary.json`** and **`logs/*.log`**). Analysis can reference numeric fields and **grep** per-scenario logs—not only the screenshot.

## Scenario table

| File | Fellow | Notes |
|------|--------|--------|
| `00_ego_only_baseline.scenic` | None | Baseline: no fellow readback; harness should not invent placement or harness samples. |
| `01_fellow_ahead_constant_offset.scenic` | `FellowConstantSpeedTrackOffsetBehavior` | `('ahead', 40)`, both `ttl_optimal`. |
| `02_fellow_behind_constant_offset.scenic` | Constant offset | `('behind', 30)`. |
| `03_fellow_left_lateral_offset.scenic` | Constant offset | `('left', 3)`. |
| `04_fellow_right_lateral_offset.scenic` | Constant offset | `('right', 3)`. |
| `05_fellow_ttl_geometric.scenic` | `FellowFollowTTLGeometricBehavior` | Same layout as 01; δ(s) lateral vs plant **d**. |
| `06_fellow_weaving.scenic` | `FellowSwerveOutOfControlBehavior` | Stress / lateral motion vs plant. |
| `07_fellow_sudden_stop_interval.scenic` | `FellowSuddenStopIntervalBehavior` | Speed profile changes (cruise / stop). |

**Baseline `00`:** The harness does **not** subtract or calibrate other scenarios against `00`; each log is parsed on its own. Scenario `00` is a **negative control** (no opponent): you should see no fellow placement lines and no `[FellowHarness]` samples from a fellow. With no fellow in the scene, the simulator does not run fellow readback for a traffic car, so spare slots in dSPACE fellow arrays are not what produces those metrics—if something shows up anyway, treat it as unexpected (e.g. stray prints or a parsing bug).
