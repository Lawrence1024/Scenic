# Falsification pipeline for the racing domain

**Status (SD-16/SD-20/SD-21):** the full pipeline runs end-to-end against
the cosim bridge without external orchestration. SD-20 routed every eval
signal through Scenic's ``simulation.records`` channel; SD-21 deleted the
log-parsing path entirely so monitors are no longer coupled to stdout
format. The single in-process driver is ``verifai_runner.py``.

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

### Sampled-bank runs are just a verifai_runner mode

The earlier subprocess-style ``sampled_runner.py`` was deleted in SD-21.
For uniform-style coverage runs (no active falsifier), point
``verifai_runner.py`` at a sampled scenario with ``--sampler halton`` or
``--sampler random``:

```bash
python src/scenic/domains/racing/benchmarks/verifai_runner.py \
    examples/racing/sampled/S1_fellow_left_ahead.scenic \
    --sampler halton --monitor min --count 10 --seed 42 --time 3000
```

Output layout:

```
benchmarks/results/verifai_<TIMESTAMP>/
    S1_fellow_left_ahead.scenic       # snapshot of the file used
    logs/sample_001.log               # captured stdout (debug only; not parsed)
    logs/sample_002.log
    ...
    summary.csv                        # one row per sample
    summary.txt                        # human-readable digest
    error_table.csv                    # samples with rho <= violation_threshold
```

Sample i uses ``seed = base_seed + i`` for log/csv labelling, so any single
failing sample can be re-run alone (the actual Scenic RNG state is
seeded once at compile time; see "Compounding RNG state" below).

### Determinism
Holding ``--seed`` fixed makes the entire campaign reproducible
run-to-run. Vary the seed (or the per-sample offset rule) to widen
coverage.

### Where each `summary.csv` column comes from

Every column is derived structurally from ``simulation.result.records`` —
no log parsing. The records are populated by ``_record_event(tag, payload)``
calls in ``behaviors.scenic`` (alongside the existing prints) and by
direct ``self.records[...].append(...)`` in the dSPACE simulator and
placement modules.

| Column | Record tag | Notes |
|---|---|---|
| `collision` | `EvalEvent` (type=eval_contact) | TRUE OBB-overlap event. Falls back to `bbox_gap_m<0` from `EvalGT` if no eval_contact entry. |
| `bbox_gap_m_min` | `EvalGT` | Continuous min over OBB clearances. |
| `track_clearance_m` | `BoundsCheck` | Signed distance to nearest geofence; +X = inside, -X = past edge. |
| `off_track` | `BoundsCheck` (any in_track=False) | Boolean. |
| `ego_start_xy` | `EgoStart` (one entry, scene setup) | Resolved ego (x, y) at sim start. |
| `opp_start_xy` | `FellowPlacement` | Race-frame `(s, t)`; ego-anchored `_racing_st_offset` is reproducible. |
| `sampled_gap_m` | `FellowPlacement` (gap_m) | The float Scenic sampled out of `Range(20, 60)` for this run. |
| `commit_pass_left_count` / `_right_count` | `Commit` (decision_reason=commit_pass_*_hold ∪ strategy_pass_*) | TICK count, not maneuver count — a single 2 s overtake yields ~40 ticks. Use as "did the planner attempt this side?" |
| `commit_pass_success_count` | `Commit` (pass_success=True) | DISCRETE count — the lifecycle clears the flag after one tick. |
| `commit_abort_pass_count` | `Commit` (decision_reason=abort_*) | TICK count again. |
| `guard_emergency_stable_count` | `Guard` (emergency_stable_mode=True) | TICK count of emergency-stable mode. |
| `selected_*` | `Strategy` (selected=<name>) | TICK count of the strategy selector's choice. |
| `tick_count` / `tick_ms_p50` | `TickTime` | Behaviour-tick wallclock perf. |

**Interpretation rule of thumb:** if `commit_pass_left_count` is high but
`commit_pass_success_count` is 0, the planner kept TRYING to overtake on
the left and never succeeded — usually because `commit_abort_pass_count`
is also high. That's a falsification signal: the layout is reachable by
the planner's intent but not survivable by its execution.

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
values (e.g. `{"param0": 28.4}`) plus the resulting `SampleMetrics`. To
reproduce a single failure: copy the parameter value into the .scenic
file and re-run with the same seed.

### Output layout

```
src/scenic/domains/racing/benchmarks/results/verifai_<TIMESTAMP>/
    S1_falsify.scenic           # snapshot of the file used
    logs/sample_001.log         # captured stdout per iteration (debug only)
    logs/sample_002.log
    ...
    summary.csv                 # one row per sample
    summary.txt                 # human-readable digest
    error_table.csv             # violations only, ranked by robustness ascending
```

### Monitor reference

All monitors read `SampleMetrics`, which is built from
`simulation.result.records` by `metrics.parse_sample`. Convention:
**lower value = closer to violation, negative = violated.** The runner
picks one via `--monitor NAME`.

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
  samples pay only the simulation time.
- **Records, not logs:** monitors read `SampleMetrics` built from
  `simulation.result.records`. The per-sample stdout is captured to
  `logs/sample_NNN.log` for human debugging only — nothing in the
  metric pipeline parses it. Adding a new metric means adding a
  `_record_event(tag, payload)` call alongside the existing print and
  a corresponding extractor in
  `src/scenic/domains/racing/benchmarks/metrics.py::_records_extract`.
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
    verifai_runner.py                     # In-process driver (sampler ∈ {halton, random, ce, bo})
    metrics.py                            # SampleMetrics + records-driven parse_sample + summary writers
    monitors.py                           # Robustness functions consumed by verifai_runner

docs/
    falsification_pipeline.md             # This file
```
