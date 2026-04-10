# Phase 0 Benchmark Scenario Bank

This folder contains the baseline Phase 0 scenario set for repeatable opponent-aware planner benchmarking:

- `00_no_opponent.scenic`
- `01_slower_opponent_optimal.scenic`
- `02_slower_opponent_left.scenic`
- `03_slower_opponent_right.scenic`
- `04_opponent_weaving_lightly.scenic`
- `05_opponent_just_ahead_corner.scenic`
- `06_side_by_side_start.scenic`

## Run the full benchmark set

From repository root (default **`--time` is 2000**; shown explicitly below for clarity):

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 2000
```

Run one specific scenario (still writes automatic logs + summary):

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 2000 --scenario 02_slower_opponent_left.scenic
```

Run a subset by glob:

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 2000 --scenario-glob "0[0-2]_*.scenic"
```

Run with inter-scenario delay (helps dSPACE reset/teardown settle between cases; **default is 15 s**):

```bash
python -m scenic.domains.racing.benchmarks.phase0_runner --time 2000 --inter-run-delay-s 15
```

Outputs are written under:

- `src/scenic/domains/racing/benchmarks/results/<run_id>/logs/*.log`
- `src/scenic/domains/racing/benchmarks/results/<run_id>/summary.json`
- `src/scenic/domains/racing/benchmarks/results/<run_id>/summary.csv`

## Reported core metrics

- lap completion status
- lap time
- number of TTL switches
- minimum opponent distance
- collision yes/no
- off-track yes/no

Notes:

- `collision` / `off_track` are driven by Phase 0 event thresholds emitted by `FollowRacingLineMPCBehavior`.
- Existing control behavior is unchanged; this phase is for visibility and benchmarking.
- **Phase 0 exit checklist** (plans) is satisfied when the full bank runs non-interactively with stable completion and generated reports; see `src/scenic/domains/racing/plans/phase-0-baseline-and-visibility.md`.
