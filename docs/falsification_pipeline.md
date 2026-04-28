# Falsification pipeline for the racing domain

**Status (SD-16):** Phase 1 (region-based placement), Phase 2 (sampled
test bank + subprocess runner), and Phase 3 (VerifAI active falsification)
are landed. The full pipeline runs end-to-end against the cosim bridge
without external orchestration.

## What "falsifiable" means here

The F-bank scenarios under `examples/racing/f_shared/` are point tests:
ego and fellow are placed at hardcoded `(x, y)` coordinates calibrated for
specific repeatable failure modes. They're regression tests, not coverage
tests.

The sampled bank under `examples/racing/sampled/` is the inverse:
the scenarios are written with Scenic distributions
(`Range(...)`, `on mainTrack`, etc.) so each invocation samples a fresh
starting layout. Run N samples and you get N distinct scenes from the same
scenario template — a coverage tool, not a regression tool.

A **falsifier** then reframes that coverage as an optimization problem:
> "Find the parameter values within the Scenic distributions that cause
> the smart ego to fail."

That's the role VerifAI plays.

---

## What we have today (post-SD-15)

### Region exposure
`mainTrack` and `pitTrack` are already PolygonalRegions in
`src/scenic/domains/racing/model.scenic:91-92`. Verified with
`examples/racing/sampled/smoke_on_mainTrack.scenic`:

```bash
scenic examples/racing/sampled/smoke_on_mainTrack.scenic --2d \
    --model scenic.simulators.dspace.racing_model \
    --count 5 --seed 42
```

→ 5 distinct scenes generated, all with ego inside mainTrack, none in
pit lane.

### Sampled scenario
`examples/racing/sampled/S1_fellow_left_ahead.scenic`:
- ego placed on its own TTL (`ttl_optimal_xodr.csv`) via the per-vehicle
  `position: new Point on ttlRegion(self.ttlFileName)` default on
  `RacingCar` (see `racing/model.scenic:164`). No `on R` specifier needed.
- fellow placed via `_racing_st_offset (Range(20, 60), 5)` — variable gap
  in [20, 60] m ahead, fixed +5 m left of ego (so fellow rides the left TTL)
- both vehicles run the full SD-13 strategy-driven planner

The integrated "on ttl" placement means each car's initial sampling region
follows ITS OWN `ttlFileName` attribute -- fellow with `ttl_left_xodr.csv`
samples on the left racing line, ego with `ttl_optimal_xodr.csv` samples
on the optimal racing line. Explicit `at (x,y)` or `on mainTrack` still
override the default the same as before.

### Batch runner
`src/scenic/domains/racing/benchmarks/sampled_runner.py`:

```bash
python src/scenic/domains/racing/benchmarks/sampled_runner.py \
    examples/racing/sampled/S1_fellow_left_ahead.scenic \
    --count 10 --seed 42 --time 3000
```

Output mirrors `full_stack_<timestamp>/`:

```
benchmarks/results/sampled_<TIMESTAMP>/
    S1_fellow_left_ahead.scenic       # snapshot of the file used
    logs/sample_001.log
    logs/sample_002.log
    ...
    summary.csv                        # one row per sample
    summary.txt                        # human-readable digest
```

Sample i uses `seed = base_seed + i`, so any single failing sample can
be re-run alone with the same seed.

### Determinism
Holding `--seed` fixed makes the entire sampled bank reproducible run-to-run.
Vary the seed (or the per-sample offset rule) to widen coverage.

### What the summary columns actually mean

`summary.csv`/`summary.txt` are parsed straight from the per-sample logs.
The signal-to-column mapping is non-obvious enough to be worth pinning
down -- earlier versions of the runner had two parser bugs (now fixed) that
made the summaries silently wrong:

| Column | Source line in log | Notes |
|---|---|---|
| `collision` | `[EvalEvent] type=eval_contact` | TRUE OBB-overlap event from the eval pipeline. Trustworthy. (Falls back to `bbox_gap_m<0` only if no eval_contact lines are present.) |
| `ego_start_xy` | first `[Ego debug] xy=(x, y) -> ...` | Resolved ego (x, y) at simulation start. |
| `opp_start_xy` | `[Placement] Fellow_0: ... -> s=<s>, t=<t>` | Race-frame coords; the ego-relative `_racing_st_offset` is reproducible across runs even if the absolute (x, y) varies. |
| `sampled_gap_m` | first paren in same `[Placement] Fellow_0: ... + (gap, lat) -> ...` | The float Scenic sampled out of `Range(20, 60)` for this run. |
| `commit_pass_left_count` / `_right_count` | `decision_reason=commit_pass_*_hold` and `=strategy_pass_*` | TICK count, not maneuver count -- a single 2 s overtake yields ~40 ticks. Use this as "did the planner attempt this side?" not "how many overtakes." |
| `commit_pass_success_count` | `pass_success=1` field on `[Commit]` lines | DISCRETE count -- the lifecycle clears the flag after one tick, so this is the number of completed overtakes. |
| `commit_abort_pass_count` | `decision_reason=abort_*` (pass / hold / commit_invalidated / recover_follow) | TICK count again. |
| `selected_*` | `[Strategy] t=... selected=<name>` | TICK count of the strategy selector's choice (the policy-side, before lifecycle execution). |

**Interpretation rule of thumb:** if `commit_pass_left_count` is high but
`commit_pass_success_count` is 0, the planner kept TRYING to overtake on
the left and never succeeded -- usually because `commit_abort_pass_count`
is also high (the SD-4 emergency-brake gate or the lifecycle's
`commit_invalidated_hazard` keeps killing the maneuver). That's a
falsification signal: the layout is reachable by the planner's intent but
not survivable by its execution.

### Two parser bugs that previously hid real failures

Both fixed in 2026-04-27; if you see results from before that date, treat
them as suspect:

1. **Encoding mismatch.** The runner captures the child subprocess's stdout
   straight off the pipe (UTF-8 / ASCII). The parser was decoding those
   bytes as UTF-16-LE -- which Python's `errors="replace"` accepts silently,
   yielding garbage with `?` placeholders that no regex matches. Result:
   every numeric metric came back zero and the summary printed `gap=?
   lap=? p50_ms=?` for every sample. Fix: BOM-sniff first, then default
   to UTF-8 (`_decode_log` in `sampled_runner.py`).

2. **Signal regexes wrong for the SD-13 planner.** The original parser was
   written against pre-SD-13 log conventions:
   - It looked for `decision_reason=pass_success_free_run`, but the SD-13
     planner emits `pass_success=1` as a field on `[Commit]` lines instead.
   - It looked for `[Placement] ... resolved ... (x, y)` but the actual
     line is `[Placement] Fellow_0: racing (s,t) from ego + (gap, lat) -> s=..., t=...`.
   - It looked for `[Ego] set position xy=(...)` but the actual line is
     `[Ego debug] xy=(...) -> ...`.
   Result: collision and overtake counts were both zero in the summary
   even when they had clearly happened in the log. Fix: regexes updated
   to match the SD-13 log format.

If you re-derive metrics from logs by hand (or write a new analysis
script), key off the source lines in the table above, not the legacy
patterns.

---

## VerifAI integration (Phase 3)

[VerifAI](https://verifai.readthedocs.io/) is a Scenic-companion library
that turns a Scenic scenario into an active falsification loop:

1. The scenario declares the parameter space using `VerifaiRange(...)` /
   `VerifaiOptions(...)` in place of plain `Range(...)`. Scenic's compiler
   auto-promotes any scenario containing those to use `VerifaiSampler`
   (`scenic.core.external_params.VerifaiSampler`).
2. The outer loop (our `verifai_runner.py`) samples those parameters
   using a **failure-biased sampler** -- Halton (deterministic
   quasi-random), cross-entropy (active), Bayesian opt (active),
   random -- each swappable via `--sampler {halton,ce,bo,random}`.
3. For each sample the simulation runs once; afterwards a **monitor**
   reads the canonical `SampleMetrics` and returns a robustness scalar
   (lower = closer to violation; negative = violated).
4. The sampler updates its surrogate on each feedback and biases the
   next sample toward parameter regions where robustness was lowest --
   targeted edge-case discovery rather than uniform coverage.

### Quickstart

Prerequisites: VEOS + ModelDesk + ControlDesk launched externally
(the Scenic process spawns its own IPC client and connects). VerifAI
installed (`pip install verifai`; already listed under the `test-full`
extra in `pyproject.toml`).

```bash
# 1. Import smoke (no simulator, no VEOS).
python -c "from scenic.core.external_params import VerifaiSampler, VerifaiRange; print('ok')"

# 2. Halton smoke -- deterministic, no learning, proves end-to-end wiring.
#    Bridge cold on sample 1, warm on samples 2+.
python src/scenic/domains/racing/benchmarks/verifai_runner.py \
    examples/racing/falsifiable/S1_falsify.scenic \
    --sampler halton --monitor min --count 3 --seed 42 --time 1500

# 3. Cross-entropy wiring test (small budget; proves feedback is threaded).
#    Use `--monitor safety` so CE has a continuous gradient on BOTH the
#    collision and off-track specs (see "Monitor reference" below).
python src/scenic/domains/racing/benchmarks/verifai_runner.py \
    examples/racing/falsifiable/S1_falsify.scenic \
    --sampler ce --monitor safety --count 10 --seed 42 --time 1500

# 4. Real falsification campaign (overnight; ~1 min/sample with warm bridge).
python src/scenic/domains/racing/benchmarks/verifai_runner.py \
    examples/racing/falsifiable/S1_falsify.scenic \
    --sampler ce --monitor safety --count 200 --seed 42 --time 3000

# 5. Inspect violations -- ranked by rho ascending.
cat src/scenic/domains/racing/benchmarks/results/verifai_*/error_table.csv
```

Each row in `error_table.csv` records the VerifAI-sampled parameter
values (e.g. `{"param0": 28.4}`) plus the parsed `SampleMetrics`. To
reproduce a single failure: copy the parameter value into the .scenic
file (or run `sampled_runner.py` with the failing iteration's seed).

### Output layout

```
src/scenic/domains/racing/benchmarks/results/verifai_<TIMESTAMP>/
    S1_falsify.scenic           # snapshot of the file used
    logs/sample_001.log         # captured stdout per iteration
    logs/sample_002.log
    ...
    summary.csv                 # one row per sample (same schema as sampled_runner)
    summary.txt                 # human-readable digest
    error_table.csv             # violations only, ranked by robustness ascending
```

### Monitor reference

All monitors read the parsed `SampleMetrics` from `sampled_runner.py`'s
`parse_sample`. Convention: **lower value = closer to violation,
negative = violated.** The runner picks one via `--monitor NAME`.

| Name | Signal | Robustness gradient |
|---|---|---|
| `collision` | `bbox_gap_m_min` (continuous min over `[EvalGT]`/`[EvalEvent]`) | meters of clearance; 0 = touch; <0 = overlap |
| `track` | `track_clearance_m` (signed distance to nearest edge from `[BoundsCheck]`) | +X = X meters of margin; -X = X meters past the boundary |
| `offtrack` | `[BoundsCheck] in_track=0` (BOOLEAN) | -1 / +1 (no gradient -- prefer `track` for CE/BO) |
| `overtake` | `commit_pass_success_count`, `commit_abort_pass_count` | +1 if any overtake completed; -(aborts) otherwise; 0 if no attempt |
| `brake` | `guard_emergency_stable_count` | -(emergency-brake tick count) |
| `safety` | min over the two SAFETY specs | `min(collision, track)` -- continuous on both sides; recommended for "find any safety violation" |
| `min` | composite | `min` over collision/overtake/brake/offtrack -- "find ANY violation" (includes the boolean `offtrack`, which can saturate the floor) |
| `all` | multi-objective tuple | `(collision, track, overtake, brake)` for `mab`-style samplers |

**Recommended defaults:**
- For pure safety falsification (collision OR off-track): `--monitor safety`. Both components are continuous, CE/BO converge cleanly.
- For collision-only campaigns: `--monitor collision`.
- For coverage-style "find ANY weakness": `--monitor min` (broader but the boolean `offtrack` can dominate the floor).

The split between `track` (continuous) and `offtrack` (boolean) exists
because the boolean version was implemented first; both read the same
`[BoundsCheck]` stream, but `track` carries the depth-of-excursion signal
that active samplers need to converge.

### Operational notes

- **Sampler choice:** start with `halton` for any new scenario --
  deterministic, no learning, exposes wiring/parser bugs cheaply.
  Move to `ce` for active falsification once the smoke run is clean.
  `bo` is more sample-efficient for very expensive simulations but
  takes more bookkeeping to converge.
- **Budget:** CE typically wants 100-500 samples to converge. At
  ~1 min/sample with the warm cosim bridge, a 200-sample run is
  ~3.5 hours of wall time -- run overnight.
- **Bridge warmth:** the in-process driver compiles the scenario once
  and keeps the IPC client connection alive across ALL samples. The
  first sample pays the ~10-38 s cosim cold-start tax; subsequent
  samples pay only the simulation time. The legacy
  `sampled_runner.py` (subprocess per sample) pays cold-start every
  time -- use it only when you specifically want per-sample isolation.
- **Two runners, one parser:** both `sampled_runner.py` and
  `verifai_runner.py` produce `summary.csv` rows via the same
  `parse_sample` function. The schemas are identical (verifai_runner
  adds `error_table.csv` on the side). Analysis tooling that reads
  `summary.csv` works on both.
- **Compounding RNG state:** non-VerifAI distributions in the .scenic
  file (e.g. `Uniform` over the ego TTL position) advance per Scenic's
  global RNG, which is seeded once at compile time. Iteration N's ego
  start therefore depends on N. By design -- VerifAI controls only
  parameters wrapped in `VerifaiRange` / `VerifaiOptions` / etc.

### Collision detection: two independent signals

The cosim stack contains TWO collision detectors that compute their
verdict independently, on different geometry, in different processes:

| Detector | Where it lives | What it sees | Where it surfaces |
|---|---|---|---|
| **Scenic / `eval_contact`** | `src/scenic/domains/racing/eval_geometry.py::classify_eval_contact` | Oriented bounding boxes at the IAC Dallara dimensions (1.93 x 4.88 m) | `[EvalEvent] type=eval_contact` log lines -> `summary.csv collision`, `bbox_gap_m_min` |
| **ASM_Traffic / `Out1[3542]`** | The vendor Simulink model running inside VEOS | Whatever shape the ASM_Traffic model uses internally | The green/red light in ControlDesk dashboards -- not surfaced to Scenic at all |

The ASM_Traffic detector runs inside the dSPACE-supplied model
(`Platform()://ASM_Traffic/Model Root/Environment/SignalInterface/SignalInterface/ASMSignalInterface/signal_structure/SignalFilterGain/Out1[3542]`)
and we don't have a public mapping of which Simulink block chain feeds
it; tracing it would require opening the model in ModelDesk.

For falsification we trust Scenic's `bbox_gap_m_min` as the source of
truth. It's continuous (gives CE/BO a real gradient), it's logged
deterministically, and it ships in `summary.csv` for every run. The
ControlDesk light remains a useful visual cross-check during live
sessions: if it disagrees with `summary.csv collision` for a specific
sample, that's a signal worth investigating (geometry mismatch, dSPACE
sensor model, etc.) -- but it doesn't currently feed back into the
falsifier loop.

---

## File map

```
examples/racing/sampled/
    smoke_on_mainTrack.scenic             # Phase 1 verification (no sim)
    S1_fellow_left_ahead.scenic           # Phase 2 sampled scenario (Range)

examples/racing/falsifiable/
    S1_falsify.scenic                     # Phase 3 active-falsification target (VerifaiRange)

src/scenic/domains/racing/benchmarks/
    sampled_runner.py                     # Phase 2 subprocess runner (uniform/halton)
    verifai_runner.py                     # Phase 3 in-process active-falsification driver
    monitors.py                           # Phase 3 robustness functions

docs/
    falsification_pipeline.md             # This file
```
