# Falsification campaign results — smart-ego development log

A running record of 30-sample CE falsification campaigns and the
stack changes between them. Two scenarios are tracked:

- **S1** (`S1_falsify.scenic`) — fellow gap is the only knob;
  fellow always on left TTL at 20 mph. Used through SD-26/SD-27.
- **S2** (`S2_falsify.scenic`) — adds two more knobs: fellow side
  (left/right TTL) and fellow cruise speed (mph). Used from
  attempt 1 onward as the more comprehensive falsification target.

Each attempt entry follows the same structure so the deltas are
easy to read and the table can be lifted into a development-history
report.

The smart-ego racing planner is the unit under test — Scenic samples
the per-scenario knobs, the dSPACE+VEOS cosim plays the scenario,
the safety monitor records collision / off-track outcomes, and the
verifai runner aggregates per-sample telemetry.

---

## Methodology

All 30-sample campaigns use the same fixed inputs so attempt-to-attempt
deltas reflect stack changes only, not sampling noise:

| Knob | Value |
|---|---|
| Scenario | `examples/racing/falsifiable/S1_falsify.scenic` |
| Sampler | Cross-entropy (`--sampler ce`) |
| Monitor | `safety` |
| Sample count | 30 |
| Seed | 42 |
| Per-sample horizon | 3000 control ticks (≈30 s wall) |

Run command:

```powershell
python src/scenic/domains/racing/benchmarks/verifai_runner.py `
    examples/racing/falsifiable/S1_falsify.scenic `
    --sampler ce --monitor safety --count 30 --seed 42 --time 3000 --quiet `
    *>attempt_N.log
```

Per-sample output: `src/scenic/domains/racing/benchmarks/results/verifai_<TIMESTAMP>/`
with `summary.csv`, `summary.txt`, and `logs/sample_NNN.log`.

The verifai-runner summary fields used below:

- `collision` — set by the safety monitor on OBB overlap (eval-only geometry).
- `off_track` — set by the safety monitor when the centroid leaves the drivable polygon.
- `bbox_gap_m_min` — minimum OBB edge-to-edge gap observed across the run (the per-sample worst case; **not** the strategy-simulator's prediction).
- `commit_pass_*_count` — how many ticks the planner was inside `COMMIT_PASS_*` mode.
- `commit_pass_success_count` — passes that completed (ego cleared `post_pass_buffer_m` ahead before lifecycle ended).
- `commit_abort_pass_count` — passes that triggered `ABORT_PASS`.
- `selected_*` — strategy-selector picks per tick (telemetry from `simulate_strategy` + `select_strategy`).

The first row of the table also helps anchor *what improvement looks like*:
fewer collisions, fewer off-tracks, more successful passes, fewer aborts,
and a higher `bbox_gap_m_min` (worst-case clearance) all point in the
same direction.

---

## Headline trend

### S1 (gap-only)

| Attempt | Date | Stack | Collisions | Off-track | Successful passes | Worst OBB gap | tick p50 |
|---|---|---|---:|---:|---:|---:|---:|
| 1 | 2026-04-28 | pre-SD-25 baseline | **13 / 30** | 1 | 4 | 0.00 m | 13.7 ms |
| 2 | 2026-04-29 | SD-26 + SD-27a + SD-27b | **2 / 30** | 0 | 21 | 1.79 m | 18.7 ms |

Net change S1 attempt 1 → attempt 2: collisions ↓ 85%, successful passes ↑ 5×,
worst-case clearance moved from full overlap (0 m) to 1.79 m.

### S2 (gap + side + speed)

| Attempt | Date | Stack | Collisions | Off-track | Successful passes | Worst OBB gap | tick p50 |
|---|---|---|---:|---:|---:|---:|---:|
| 1 | 2026-04-29 | SD-26 + SD-27a + SD-27b (post S1-attempt2) | **6 / 30** | 3 | 16 | 0.00 m | 26.7 ms |

S2 attempt 1 is the **reference baseline before the planned per-tick
runtime cuts**. Any cut that drops `tick p50` below 26.7 ms without
regressing the four behavioral metrics (collisions ≤ 6, off-track ≤ 3,
successful passes ≥ 16, worst gap ≥ 0) is a net win. Worse on any of
those is a regression.

---

## Attempt 1 — pre-SD-25 baseline

**Date:** 2026-04-28 16:52
**Run dir:** `src/scenic/domains/racing/benchmarks/results/verifai_20260428_165255/`
**Stack at this run:** smart-ego with the SD-11 strategy pipeline (simulator + selector + authority) but **none** of SD-25 / SD-26 / SD-27 fixes applied.

### Headline numbers

| Metric | Value |
|---|---:|
| Collisions | 13 / 30 |
| Off-track | 1 |
| Successful passes (commit_pass_success) | 4 |
| Pass attempts (left / right / aborted) | 898 / 438 / 473 |
| Strategy picks (stay / follow / pL / pR) | 17321 / 17 / 381 / 281 |
| Worst per-sample `bbox_gap_m_min` | 0.00 m (full overlap) |

### Diagnosed pathologies (set the SD-26 / SD-27 agenda)

1. **Strategy simulator placed ego at full lateral offset on the side polyline at `t=0+`.** The simulator's "predicted clearance" for `pass_left` came from a geometry the MPC could never actually reach in the first 5–8 s. Sample #8 of this campaign: simulator predicted 7.73 m clearance for `pass_left`, actual closest approach 0.94 m → collision.
2. **Fellow trajectory propagated as cartesian constant-velocity.** On curving sections, fellow's predicted xy drifted off whatever line the fellow was actually following, opening fictional gaps.
3. **Clearance metric was centroid-to-centroid.** With IAC Dallaras at 4.88 m × 1.93 m, a 3.95 m centroid distance during end-to-end approach is already overlapping. The 2.5 m hard filter never got the chance to reject these geometries because the input value was wrong.
4. **Aborts dominated.** 473 abort_pass commits across 30 samples — the planner kept committing to passes the predictor said were safe, then aborting as the actual geometry tightened.

### What this run pointed at

Two design directions came out of this campaign:

- The strategy simulator was lying about pass clearance. The fix had to live inside the simulator (correct geometry + realistic ego dynamics + extended-object clearance), not in the selector or downstream gates.
- Fellow prediction needed to be at least as good as the actual fellow's racing-line behaviour, but **without** baking in a "fellow follows the racing line" assumption (a malfunctioning fellow has to be predictable too).

---

## Attempt 2 — post SD-26 + SD-27

**Date:** 2026-04-29 10:48
**Run dir:** `src/scenic/domains/racing/benchmarks/results/verifai_20260429_104834/`
**Stack at this run:** SD-26 + SD-27a + SD-27b applied on top of the attempt-1 baseline.

### Headline numbers (vs attempt 1)

| Metric | Attempt 1 | Attempt 2 | Change |
|---|---:|---:|---|
| Collisions | 13 / 30 | **2 / 30** | ↓ 85% |
| Off-track | 1 | 0 | ↓ |
| Successful passes | 4 | **21** | × 5.25 |
| Pass attempts (L / R / aborted) | 898 / 438 / 473 | 37 / 3173 / 33 | left attempts ↓ 96%, aborts ↓ 93% |
| Strategy picks (stay / follow / pL / pR) | 17321 / 17 / 381 / 281 | 16536 / 157 / 486 / 821 | follow ↑ 9×, pR selections ↑ 3× |
| Worst per-sample `bbox_gap_m_min` | 0.00 m | **1.79 m** | full overlap → safe gap |

### What changed in the stack

| Code change | File:lines | Mechanism |
|---|---|---|
| **SD-26 lane-change blending** | `src/scenic/domains/racing/prediction/strategy_simulator.py` (`_blend_alpha`, integration loop) | Ego trajectory in pass_* simulations is `α(t)·side + (1−α(t))·optimal` with `α = 1 − exp(−t/τ)`, τ = 2.5 s. Replaces the instantaneous teleport-to-side-polyline with the MPC's actual lateral convergence shape. |
| **SD-27a CTR fellow prediction** | `src/scenic/domains/racing/prediction/fellow_predictor.py` (`_yaw_rate_from_history`, `trajectory`) | `FellowPredictor.trajectory()` estimates yaw rate from heading history (recency-weighted regression), propagates fellow on a circular arc when turning. Falls back to CV when straight. **No racing-line assumption** — pure observation-based. |
| **SD-27b OBB-aware clearance** | `src/scenic/domains/racing/prediction/strategy_simulator.py` (per-tick clearance), reuses `eval_geometry.obb_separation_distance_m` | `min_clearance_m` is now true OBB edge-to-edge gap between IAC Dallaras (4.88 m × 1.93 m). Heading from finite-difference of the previous tick's xy. Pre-SD-27 was centroid-to-centroid. |
| **SD-27b reverse-blend in merge_back** | `src/scenic/domains/racing/prediction/strategy_simulator.py` (merge_back branch) | Symmetric to the forward blend — α decays back toward 0 over τ during merge_back instead of snapping to optimal. Without this, OBB clearance collapsed to ~0.1 m artificially during the post-pass merge tick. |
| **SD-27b threshold recalibration** | `src/scenic/domains/racing/planner/strategy_selector.py` (defaults), `src/scenic/domains/racing/tactical_planner.py` (`TacticalPlannerConfig.strategy_min_clearance_m`, `strategy_soft_clearance_m`) | Hard filter 2.5 → **0.5 m**, soft 1.5 → **0.2 m**. Necessary because the metric's *meaning* changed (centroid → edge-to-edge); the new numbers are calibrated to "physical bumper-to-bumper gap". |

Offline regression bank (`src/scenic/domains/racing/benchmarks/sd26_simulator_unit_bank.py`)
holds the math: 11 cases covering blend curve, tau-zero backward-compat,
pass-left / pass-right symmetry, stay-optimal invariance, merge-back
reverse-blend, far-fellow sanity, CTR-on-straight (collapses to CV),
CTR-on-arc (10× lower error than CV), OBB lateral pass (3.07 m gap),
and OBB full overlap (0 m, fails 0.5 m filter).

### Behavioural shift visible in the telemetry

The most striking number is `commit_pass_left` 898 → 37 and
`commit_pass_right` 438 → 3173. In `S1_falsify.scenic` the sampled
fellow gap puts fellow on the **left** racing line. Pre-SD-27 the
simulator (cartesian-CV smearing fellow off-line + centroid metric)
reported `pass_left` ≈ 3.95 m clearance — looked safe, often selected,
collided. Post-SD-27 fellow's predicted trajectory stays on the left
line (CTR), and the OBB metric correctly reports `pass_left` ≈ 0 m —
filter rejects, selector picks `pass_right` instead.

Aborts collapsed from 473 to 33 because the planner stopped committing
to passes that were never going to clear in the first place.

`follow_fellow` selections rose from 17 to 157 — the chicken-out path
now activates when both passes look unsafe, instead of defaulting to a
borderline `stay_optimal` that rear-ends fellow.

### Remaining failures (samples 4 and 19)

Both are different design problems from what SD-26 / SD-27 attack:

- **Sample 4 (seed 45):** strategy selector flip-flops between `pass_left` and `pass_right` every other tick — neither stable. 37 left commits made via the snapshot-fallback path, all aborted, eventually a tight geometry becomes a contact. **Root cause:** the 2-cycle hysteresis on the strategy authority is too short for a noisy alternating selector, and the snapshot fallback path doesn't apply the same hysteresis. Worth tracking as a future cycle.
- **Sample 19 (seed 60):** zero commits — ego stayed on `stay_optimal` the whole time and rear-ended fellow. The simulator alternates between predicting `stay_optimal` ≈ 9 m on even ticks and ≈ 1 m on odd ticks (per-tick fellow-pose oscillation upstream of the predictor). Selector picks `stay_optimal` whenever it clears the 0.5 m threshold; never proactively switches to `follow_fellow` when stay_optimal is consistently marginal. **Root cause:** input oscillation + selector having no "marginal-but-not-failing" escape hatch.

Neither failure invalidates SD-26 / SD-27 — they expose new design questions (selector stability, marginal-stay-optimal handling) that the prediction-correctness work surfaced because the dominant pre-SD-27 failure mode is gone.

---

## S2 — Attempt 1 — comprehensive scenario baseline (3 knobs)

**Date:** 2026-04-29 12:29
**Run dir:** `src/scenic/domains/racing/benchmarks/results/verifai_20260429_122917/`
**Stack at this run:** identical to S1 attempt 2 (SD-26 + SD-27a + SD-27b). No code changes between this run and S1 attempt 2 — only the scenario changed.

### Scenario change (vs S1)

| Knob | S1 | S2 |
|---|---|---|
| `gap_m` | `VerifaiRange(20, 60)` | same |
| Fellow side (L/R TTL) | hardcoded **left** | **`VerifaiDiscreteRange(0, 1)`** routed through paired distributionFunction helpers (lat offset and TTL filename stay synchronized) |
| Fellow speed | hardcoded **20 mph** | **`param fellow_speed_mph = VerifaiRange(15, 35)`** read at first activation by a thin wrapper behavior |

The wrapper-behavior detour is needed because Scenic doesn't auto-resolve Distribution kwargs to behavior constructors at scene-sample time. Object properties (like `_racing_st_offset`) DO get resolved, hence the per-property + Function-Distribution pattern for the side knob.

### Headline numbers

| Metric | Value | vs S1 attempt 2 |
|---|---:|---|
| Collisions | **6 / 30** | 2 → 6 (CE finds new failure modes that S1 doesn't expose) |
| Off-track | 3 | 0 → 3 |
| Successful passes | 16 | 21 → 16 |
| Pass attempts (L / R / aborted) | 1180 / 1954 / 97 | 37 / 3173 / 33 → much more balanced (fellow now varies sides) |
| Strategy picks (stay / follow / pL / pR) | 16573 / 184 / 684 / 559 | similar shape |
| Worst per-sample `bbox_gap_m_min` | **0.00 m** | full overlap reappears in the new failure modes |
| Mean tick_ms_p50 | **26.7 ms** | 18.7 → 26.7 ms (more knobs → more strategy work per tick) |

### Why S2 is harder

Three things compound:

1. **Both side TTLs get loaded and exercised.** The strategy simulator runs identical work, but the broader range of fellow positions surfaces failure modes that didn't exist in S1's left-only setup.
2. **Variable fellow speed** lets CE find combinations where the closing rate is borderline — slow enough that ego closes within horizon, fast enough that the pass dynamics get tight.
3. **The 26.7 ms p50** vs S1's 18.7 ms is partly extra polyline-loading and partly the planner running fuller paths because fellow is now at varying lateral positions instead of always left.

### Reference baseline before runtime cuts

S2 attempt 1 is the **frozen reference point** for the upcoming per-tick runtime-cut work (OBB early-exit, per-strategy early-exit on overlap, throttled telemetry, `path_collision_predicted` retirement, etc.). Acceptance criteria for any cut:

- `tick_ms_p50` strictly lower than 26.7 ms
- Collisions ≤ 6 / 30
- Off-track ≤ 3 / 30
- Successful passes ≥ 16
- Worst `bbox_gap_m_min` not worse than 0.00 m (i.e., no new full-overlap modes)

If a cut violates any of those, roll it back and revisit. The commit at this point in history is the rollback target.

### Out-of-scope for this baseline

- The fellow-speed range was bumped from 15-35 mph to 15-65 mph after this run completed; the next S2 attempt will exercise the wider range.
- The two new failure modes (S2 attempt 1's 6 collisions + 3 off-tracks) are tracked but not yet root-caused — we want the runtime work to land first so we have headroom to iterate without re-running heavy campaigns at every step.

---

## Template for future attempts

```markdown
## Attempt N — <one-line stack summary>

**Date:** YYYY-MM-DD
**Run dir:** `src/scenic/domains/racing/benchmarks/results/verifai_<TIMESTAMP>/`
**Stack at this run:** SD-XX, SD-YY, ...

### Headline numbers (vs attempt N-1)

| Metric | Prev | This | Change |
|---|---:|---:|---|
| Collisions | | | |
| Off-track | | | |
| Successful passes | | | |
| Pass attempts (L / R / aborted) | | | |
| Strategy picks (stay / follow / pL / pR) | | | |
| Worst `bbox_gap_m_min` | | | |

### What changed in the stack

| Code change | File:lines | Mechanism |
|---|---|---|

### Behavioural shift

(What's visible in the per-sample logs that wasn't last time.)

### Remaining failures

(Per-sample notes for any collisions / off-tracks. Each one tagged with
a candidate root cause and whether it's "in-scope" for the next cycle
or a different design problem.)
```
