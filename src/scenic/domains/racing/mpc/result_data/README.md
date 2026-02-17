# Racing Run Result Data & Analysis

This folder holds **post-processing for MPC racing runs**: log analysis, stored results per run, and comparison across runs. Everything analysis-related lives here.

---

## Layout

```
result_data/
├── README.md                    # This file
├── __init__.py
├── analyze_racing_log.py        # Parse logs → segments, events, CTE; write per-run outputs
├── compare_racing_results.py   # Compare multiple runs (time, waypoint hits, CTE per segment)
├── ttl_racing_line_xodr/        # One run (from log with TTL ttl_racing_line_xodr.csv)
│   ├── summary.json
│   ├── segments.csv
│   ├── events.csv
│   └── mpc.csv
├── ttl_fellow_test_xodr_all/    # Another run (example)
│   └── ...
└── <run_id>/                    # One subfolder per run (name = TTL stem or log stem)
    ├── summary.json
    ├── segments.csv
    ├── events.csv
    └── mpc.csv
```

Each **run** is a subfolder whose name comes from the log’s TTL (e.g. `ttl_racing_line_xodr` from `ttl_racing_line_xodr.csv`). The analysis scripts live alongside these run folders.

---

## 1. `analyze_racing_log.py` — How It Works

### What it reads

The script parses Scenic/dSPACE racing logs that contain:

- **`[FollowRacingLineMPC] t=X.Xs Step N: ... speed=...m/s CTE=...m ... segment N name`**  
  Used for: time `t`, segment id/name, speed, and **CTE (cross-track error)** at each logged MPC step (e.g. every 50 steps).
- **`[FollowRacingLineMPCBehavior] t=X.Xs WAYPOINT HIT: index i -> j at (x,y), distance=...m segment N name`**  
  Used for: time `t`, segment id/name, and waypoint progress.
- **`[RacingRun] TTL=... run_timestamp=...`** or **`[TTL] Assigned TTL PolylineRegion to ego (...)`**  
  Used for: run identifier (TTL name) so results are written under the correct `result_data/<run_id>/`.

Encoding: tries UTF-8 (with BOM) first; if no matching lines are found, retries with UTF-16 (for some Windows logs).

### What it computes

1. **Segment times**  
   From waypoint-hit events: time between consecutive hits is attributed to the segment of the *previous* event. So you get `time_s` and `pct` per segment.

2. **Waypoint hits per segment**  
   Count of WAYPOINT HIT events in each segment (excluding the synthetic first event at t=0).

3. **Per-segment CTE (when MPC lines exist)**  
   From each `[FollowRacingLineMPC] ... CTE=...m ... segment N name` line, the script records `(t, segment_id, segment_name, speed_mps, cte_m)`. It then aggregates by segment:
   - **mean_abs_cte_m**: mean of |CTE| in that segment  
   - **max_abs_cte_m**: max of |CTE| in that segment  

   Segments with no MPC sample in the log get empty CTE fields.

4. **Run-level stats**  
   In `summary.json`: `total_time_s`, `t_end`, `n_waypoints`, `n_mpc_samples`, and (if MPC samples exist) `mean_abs_cte_m` and `max_abs_cte_m` over the whole run.

### Outputs written to `result_data/<run_id>/`

| File | Contents |
|------|----------|
| **summary.json** | Run identifier (TTL, timestamp), total_time_s, t_end, n_waypoints, n_mpc_samples, mean_abs_cte_m, max_abs_cte_m (when available). |
| **segments.csv** | One row per segment: segment_id, segment_name, time_s, pct, waypoint_hits, mean_abs_cte_m, max_abs_cte_m. |
| **events.csv** | One row per waypoint hit: t, segment_id, segment_name. |
| **mpc.csv** | One row per parsed MPC step: t, segment_id, segment_name, speed_mps, cte_m. |

`<run_id>` is the **stem** of the TTL filename from the log (e.g. `ttl_racing_line_xodr` from `ttl_racing_line_xodr.csv`), or the log file stem if no TTL is found.

### Usage

From the **repository root**:

```bash
# Analyze run.log (default) and write into result_data/<run_id>/
python -m scenic.domains.racing.mpc.result_data.analyze_racing_log

# Specify log path
python -m scenic.domains.racing.mpc.result_data.analyze_racing_log --log path/to/run.log

# Print segment table as CSV (no result_data write with --no-result-dir)
python -m scenic.domains.racing.mpc.result_data.analyze_racing_log --log run.log --csv --no-result-dir
```

**Programmatic:**

```python
from scenic.domains.racing.mpc.result_data.analyze_racing_log import run_analysis

result = run_analysis("run.log")  # or None for default run.log
# result.segments_df  -> pandas DataFrame (segment_id, time_s, pct, waypoint_hits, mean_abs_cte_m, max_abs_cte_m)
# result.events_df    -> waypoint events (t, segment_id, segment_name)
# result.mpc_df       -> MPC samples (t, segment_id, segment_name, speed_mps, cte_m)
# result.summary       -> dict with total_time_s, n_waypoints, mean_abs_cte_m, max_abs_cte_m, etc.
```

---

## 2. `compare_racing_results.py` — How It Works

### What it reads

- **Directory**: by default this folder (`result_data/`). Every **subfolder** that contains a `summary.json` is treated as one run.
- **Per run**: `summary.json` and `segments.csv` (if present). CTE columns in `segments.csv` (mean_abs_cte_m, max_abs_cte_m) are used when available.

### What it does

1. **Run summary table**  
   For each run: run name, TTL, edit note, total time, waypoint hits, MPC sample count, t_end.

2. **Per-segment comparison**  
   Builds a table of segment_id, segment_name, and for each run: time_s, waypoint_hits, mean_abs_cte_m, max_abs_cte_m. So you can compare time and CTE per segment across runs.

3. **Optional CSV export**  
   With `--output FILE`, writes the segment-comparison table to a CSV file.

### Usage

From the **repository root**:

```bash
# Compare all runs in result_data/ (default)
python -m scenic.domains.racing.mpc.result_data.compare_racing_results

# Write comparison to CSV
python -m scenic.domains.racing.mpc.result_data.compare_racing_results --output comparison.csv

# Use another folder as result_data
python -m scenic.domains.racing.mpc.result_data.compare_racing_results --results-dir /path/to/result_data
```

---

## Interpreting the results

- **time_s / pct**: How long the car spent in each segment (from waypoint-hit timing). Useful to see which segments dominate lap time.
- **waypoint_hits**: How many waypoints were passed in that segment; reflects progress and density.
- **mean_abs_cte_m / max_abs_cte_m**: Lateral error (distance from reference line). High values in a segment (e.g. curves with max |CTE| > 5 m) indicate where the controller struggled or where the car was close to leaving the track. Comparing these across runs (e.g. different TTLs or tunings) shows where behavior improved or regressed.

Runs that “ran smooth” typically have moderate max |CTE| (e.g. &lt; 5 m) in most segments and no single segment with very large CTE. Heavy braking or run-off in the log often show up as high CTE in the corresponding segment in `segments.csv` and in the run’s `summary.json` (e.g. high max_abs_cte_m).
