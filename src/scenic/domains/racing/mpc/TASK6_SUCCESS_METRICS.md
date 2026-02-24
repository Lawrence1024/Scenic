# Task 6 — Success metrics and run protocol (To-Do 3 measurable)

## Experiment protocol

Run **exactly** these two experiments (e.g. same track, same scenario, 60 s each):

1. **Baseline (commit off)**  
   - Set `commit_enabled: false` in `vehicle_mpc.yaml` (or pass config that disables commit).  
   - Run for **60 seconds**.  
   - Save logs (stdout or log file).

2. **Commit on**  
   - Set `commit_enabled: true` (default).  
   - Run for **60 seconds** on the same scenario.  
   - Save logs.

## Metrics to compute from logs

For each run, from the **2-second window before each curve entry** (define “curve entry” e.g. when `approaching_curve` or `curv_ahead_filt` first exceeds threshold in a segment), compute:

| Metric | Description |
|--------|-------------|
| **# segment_id switches** | Number of times `segment_id` (or `seg_id_used`/`seg_id_raw`) changes in the 2 s pre-curve window. |
| **std(ds_ref)** | Standard deviation of `ds_ref` in that same 2 s window. |
| **max \|CTE\|** | Maximum absolute cross-track error in that 2 s window. |
| **Jerkiness proxy** | `max \|delta_cmd(t) - delta_cmd(t-1)\|` in that window (max step-to-step change in steering command). |

## Deliverable

Paste the **four numbers for both runs** (Baseline vs Commit on) so that To-Do 3 impact is measurable:

- Fewer segment_id switches and lower std(ds_ref) with commit on → reference stability improved.
- Lower max |CTE| and lower jerkiness with commit on → better tracking and smoother steering.

## Optional: log parser

A small Python script is provided to parse your log format and compute these metrics: `result_data/parse_commit_metrics.py`. Adjust the regex/line format inside the script to match your actual log lines (e.g. `[FollowRacingLineMPC] commit_log ...`, `ref_log ...`, `ff_log ...`).
