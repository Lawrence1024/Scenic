# Fellow placement debug bank

Use this bank to reproduce and diagnose fellow placement mismatches (commanded `_racing_st_offset` vs observed spawn pose/road projection), independent of tactical phase behavior.

## Run

From repo root:

```bash
python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner
```

Repeatability mode (same scenarios, N repeats each):

```bash
python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner --repeats 5
```

Optional shorter smoke horizon:

```bash
python -m scenic.domains.racing.benchmarks.fellow_placement_debug_runner --time 500
```

## Output contract

Like other runners, this writes:

- `src/scenic/domains/racing/benchmarks/results/<run_id>/logs/*.log`
- `src/scenic/domains/racing/benchmarks/results/<run_id>/summary.json`
- `src/scenic/domains/racing/benchmarks/results/<run_id>/summary.csv`
- terminal digest block: `BENCHMARK_AI_DIGEST_BEGIN ... END`

## Key placement metrics

- `requested_delta_s_m`, `requested_delta_t_m`
- `observed_delta_s_m`, `observed_delta_t_m`
- `placement_s_error_m`, `placement_t_error_m`
- `ego_road_id`, `fellow_road_id`, `road_id_mismatch`
- `unexpected_pit_projection`

These are parsed from `[Placement]`, `[Ego debug]`, and `[Fellow s,t]` lines.

## Determinism notes

- Ego-anchor scenarios in this bank intentionally avoid explicit opponent `at` ego-pose to prevent Scenic compile-time overlap rejection.
- Relative placement intent is carried by `_racing_st_offset` and resolved in the dSPACE placement path at runtime.
