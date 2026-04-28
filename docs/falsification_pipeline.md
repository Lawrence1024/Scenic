# Falsification pipeline for the racing domain

**Status (SD-15):** Phase 1 (region-based placement) and Phase 2 (sampled
test bank + batch runner) are landed. Phase 3 (VerifAI integration) is
documented here but deferred until baseline overtake quality is high enough
that finding edge cases is the actual problem.

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

## VerifAI integration (Phase 3, deferred)

[VerifAI](https://verifai.readthedocs.io/) is a Scenic-companion library
that turns a Scenic scenario into an active falsification loop:

1. The scenario declares the parameter space (the same `Range(...)` and
   `on mainTrack` distributions we already have).
2. VerifAI's outer loop samples those parameters using a
   **failure-biased sampler** (Bayesian Optimization, cross-entropy,
   simulated annealing, or random — each is a swappable backend).
3. For each sample VerifAI runs the simulation, reads a **monitor**
   (a function or specification that returns "satisfied" or
   "violated" + a real-valued robustness score).
4. The sampler updates its surrogate model and biases the next batch
   of samples toward the parameter regions where robustness was lowest
   (most likely to violate).

The result is targeted edge-case discovery rather than uniform coverage:
VerifAI converges on the corners of the parameter space that break the
ego, not the easy ones it already passes.

### What we'd need to add to integrate

**(a) Monitor blocks.** Define falsification specifications. For SD-15's
S1 scenario the natural ones are:

```python
# Pseudo-syntax — exact API depends on VerifAI version
monitor no_collision:
    while True:
        require min_opponent_distance > 0.0    # bbox_gap_m > 0

monitor overtake_completes:
    eventually within 25.0:
        commit_pass_success_count >= 1

monitor no_emergency_stable:
    require guard_emergency_stable_count == 0
```

These are read from the Scenic model's runtime state (the same `[EvalGT]`,
`[Commit]`, and `[Guard]` log signals our `sampled_runner.py` already
parses for `summary.csv`).

**(b) VerifAI invocation.** Wrap the existing
`sampled_runner.run_one_sample` with a VerifAI sampler:

```python
from verifai.scenic_server import ScenicServer
from verifai.samplers import HaltonSampler, CrossEntropySampler, BayesOptSampler

server = ScenicServer(
    scenic_file="examples/racing/sampled/S1_fellow_left_ahead.scenic",
    sampler=BayesOptSampler(...),
    monitor="no_collision",
    max_iterations=100,
)
server.run()
```

VerifAI hands each sample to the simulator (via the sampled_runner's
subprocess invocation), reads the monitor's robustness, and decides what
to sample next.

**(c) Output integration.** VerifAI's `error_table` would be the
falsification version of `summary.csv`: the list of parameter samples
that violated each monitor, ranked by severity. Drop it into the same
`benchmarks/results/sampled_<TIMESTAMP>/` directory under
`error_table.csv`.

### Why we're not implementing now

1. **Baseline quality.** SD-13 reduced F-bank collisions from 4 to 1, but
   F8 (corner-close-up) and F3R-style "ignoring fellow" perceptions are
   still open. Falsification is most useful when the baseline is "passes
   all easy cases, occasionally trips on hard ones" — once we're there,
   VerifAI directs attention to the actual corner cases.
2. **Scenario coverage.** We have one sampled scenario (S1). Falsification
   over a single scenario family is much less interesting than over a
   bank — `S2_fellow_right_ahead`, `S3_fellow_optimal_slower`,
   `S4_corner_entry_overtake` etc. should land before VerifAI is wired
   in, so the falsifier can budget across families.
3. **Compute budget.** VerifAI's failure-biased sampling typically needs
   50–500 simulation runs to converge. At ~1 minute per run with cosim
   warm-start, that's hours per scenario. Worth doing only after the
   scenarios are stable enough not to invalidate the budget mid-run.

### Concrete next steps when we're ready

1. Add 3–5 more sampled scenarios (one per F-bank family) to
   `examples/racing/sampled/`.
2. Write a small `monitors.py` module with the `no_collision`,
   `overtake_completes`, and `no_emergency_stable` predicates that read
   from the sampled_runner output format.
3. Add `verifai_sampled_runner.py` that wraps the existing
   `sampled_runner.run_one_sample` in a VerifAI sampler loop.
4. Run on the cosim warm-start path overnight on the failure-biased
   sampler with `max_iterations=200`. Read `error_table.csv` next morning.

Until then, the current `sampled_runner.py` with seeded uniform sampling
gives us reproducible coverage — enough to find new failure modes
manually as we improve the planner.

---

## File map

```
examples/racing/sampled/
    smoke_on_mainTrack.scenic             # Phase 1 verification (no sim)
    S1_fellow_left_ahead.scenic           # Phase 2 sampled scenario

src/scenic/domains/racing/benchmarks/
    sampled_runner.py                     # Phase 2 batch runner

docs/
    falsification_pipeline.md             # This file (Phase 3 documentation)
```
