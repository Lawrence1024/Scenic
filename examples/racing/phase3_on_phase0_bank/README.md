# Phase 3 tactical × Phase 0 scenario bank

Same scenarios as `examples/racing/phase0_benchmark/`, but ego uses
`FollowRacingLineMPCBehavior(..., tactical_planner_enabled=True)` so the tactical
planner is exercised on the full Phase 0 layout set.

## Validation record

A full-bank run on dSPACE at default **3000** simulation steps per scenario (~30 s sim at 0.01 s/step) completed with **no collisions, no off-track, no near-miss events** in `summary.json` aggregate, all scenarios exit code **0**, laps **completed**. Tighter geometry (e.g. slower opponent on **right** TTL) produced the smallest reported **min_opponent_distance_m** in the digest; **weaving** produced more `[Phase3Tactical]` TTL switches than stable-opponent cases, as expected. **Multi-lap** stress is still an optional follow-up (see Phase 3 plan).

Run (from repo root):

```bash
python -m scenic.domains.racing.benchmarks.phase3_on_phase0_runner --inter-run-delay-s 0
```

Use a larger `--time` (e.g. **4500**) only if some scenarios need more simulated time for a full lap. After the run, copy the terminal block between
`BENCHMARK_AI_DIGEST_BEGIN` and `BENCHMARK_AI_DIGEST_END`, or share
`summary.json` under the printed `run_dir`.

Adding a new `.scenic` here follows the same rules as the main racing README: new
files are picked up automatically by `phase3_on_phase0_runner`.
