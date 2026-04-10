# Fellow Placement Debug Matrix and Metrics

## Purpose

Define a focused benchmark harness to reproduce and diagnose fellow placement errors (especially route/segment mismatches such as Lap vs Pit projection), independent of higher-level ego tactical logic.

This plan is intentionally narrow:
- validate placement correctness at spawn and early rollout,
- quantify drift and route consistency,
- produce deterministic pass/fail signals for regressions.

## Scope

In scope:
- A dedicated placement-debug runner (`fellow_placement_debug_runner.py`).
- A compact scenario bank under `examples/racing/fellow_placement_debug/`.
- A metric schema specialized for placement correctness.
- Digest/CSV fields suitable for CI comparison.

Out of scope:
- Phase 0-4 tactical policy validation.
- Pass-shield policy scoring.
- Long horizon race performance.

## Scenario Matrix (Draft)

All scenarios should set:
- `param fellowHarnessLog = True`
- `param time_step = 0.01`
- default runtime `--time 2000` (with optional `--time 500` short mode for rapid repro)

Each case should run with a single fellow and fixed random seed unless scenario explicitly tests seed sensitivity.

| ID | Scenario Name | Intent | Spawn Relation | Route Context | Expected |
|---|---|---|---|---|---|
| P00 | no_opponent_baseline | Negative control (parser + no fellow safety) | none | Lap | No fellow metrics, no false positives |
| P01 | ahead_40_main_straight | Canonical ahead placement | `('ahead', 40)` | Lap/main straight | s error small, same track family |
| P02 | behind_40_main_straight | Canonical behind placement | `('behind', 40)` | Lap/main straight | s error small, same track family |
| P03 | left_3p5_main_straight | Lateral offset left | `('left', 3.5)` | Lap/main straight | t delta near +3.5 m equivalent |
| P04 | right_3p5_main_straight | Lateral offset right | `('right', 3.5)` | Lap/main straight | t delta near -3.5 m equivalent |
| P05 | ahead_40_near_pit_entry | Stress route ambiguity near pit merge/split | `('ahead', 40)` | Lap near pit entry | Must remain on Lap projection set |
| P06 | behind_40_near_pit_exit | Stress route ambiguity near pit exit | `('behind', 40)` | Lap near pit exit | Must remain on Lap projection set |
| P07 | side_by_side_near_segment_boundary | Boundary robustness | combined s/t offset | Lap near segment boundary | No route-flip at spawn |
| P08 | seed_stability_ahead_40 | Repeatability across N runs | `('ahead', 40)` | Lap | Distribution remains within bounds |

Notes:
- P05/P06/P07 are the highest-value repro candidates for the observed misplacement.
- P08 should execute the same scenario N times (recommended N=5) and aggregate placement error statistics.

## Metric Schema (Draft)

The runner should compute and emit metrics from:
- `[Placement] ... racing (s,t) from ego ...`
- `[Fellow s,t] ...`
- `[FellowHarness] ...`
- optional projection tags that include road identifiers where available.

### Core placement metrics

- `placement_command_observed` (bool): placement-from-ego line exists.
- `fellow_st_log_present` (bool): fellow s,t line exists.
- `ego_s0` (float|null): ego s at first placement context (if parsable).
- `ego_t0` (float|null): ego t at first placement context (if parsable).
- `fellow_s0` (float|null): fellow s from first fellow s,t line.
- `fellow_t0` (float|null): fellow t from first fellow s,t line.
- `requested_delta_s_m` (float|null): commanded longitudinal offset.
- `requested_delta_t_m` (float|null): commanded lateral offset.
- `observed_delta_s_m` (float|null): `fellow_s0 - ego_s0` in normalized track frame.
- `observed_delta_t_m` (float|null): `fellow_t0 - ego_t0`.
- `placement_s_error_m` (float|null): `abs(observed_delta_s_m - requested_delta_s_m)`.
- `placement_t_error_m` (float|null): `abs(observed_delta_t_m - requested_delta_t_m)`.

### Route/segment consistency metrics

- `ego_route_label` (str|null): parsed route token near placement.
- `fellow_route_label` (str|null): parsed fellow route token.
- `ego_projected_road_id` (int|null)
- `fellow_projected_road_id` (int|null)
- `route_consistent_at_spawn` (bool|null): route labels are compatible.
- `road_consistent_at_spawn` (bool|null): same expected road family (configurable Lap-only check).
- `unexpected_pit_projection` (bool): true when Lap-intended scenario projects fellow to pit at spawn.

### Early rollout stability metrics (first K harness points, recommended K=10)

- `fellow_harness_line_count` (int)
- `fellow_speed_min_mps` (float|null)
- `fellow_speed_max_mps` (float|null)
- `fellow_speed_jump_max_mps` (float|null)
- `fellow_position_range_m` (float|null)
- `early_rollout_outlier` (bool): large discontinuity suggesting spawn/readback glitch.

### Repeatability metrics (for repeated scenarios like P08)

- `replicate_count` (int)
- `placement_s_error_mean_m` (float|null)
- `placement_s_error_p95_m` (float|null)
- `placement_t_error_mean_m` (float|null)
- `placement_t_error_p95_m` (float|null)
- `unexpected_pit_projection_count` (int)

## Pass/Fail Contract (Draft)

Per-scenario hard fail:
- missing placement evidence (`placement_command_observed == false` or `fellow_st_log_present == false`),
- `placement_s_error_m > 10.0`,
- `placement_t_error_m > 2.0` for lateral-offset scenarios,
- `unexpected_pit_projection == true` in Lap-intended scenarios.

Soft warning:
- high speed/position discontinuity in first K harness points,
- replicate p95 drift above configured threshold.

## Runner Output Contract

The new runner should write:
- `results/<run_id>/logs/*.log`
- `results/<run_id>/summary.json`
- `results/<run_id>/summary.csv`
- `BENCHMARK_AI_DIGEST_BEGIN/END` block

Suggested digest key group:
- all core placement metrics,
- consistency metrics,
- replicate summary metrics where applicable.

## Implementation Order

1. Add scenario folder and P00-P08 scenarios.
2. Add parser helpers for command/observed delta extraction.
3. Implement runner + CSV schema.
4. Add repeat-mode support (`--repeats N`) for P08-like cases.
5. Add synthetic parser tests for new placement metrics.
6. Dry-run with `--time 500`, then signoff with `--time 2000`.

