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
- ego placed `on mainTrack` (uniform over the main racing region)
- fellow placed via `_racing_st_offset (Range(20, 60), 5)` — variable gap
  in [20, 60] m ahead, fixed +5 m left of ego (so fellow rides the left TTL)
- both vehicles run the full SD-13 strategy-driven planner

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
