# Plan: Migrate Lateral MPC to MPCC-Style Control

This document outlines a path from the current **trajectory-tracking MPC** to a **Model Predictive Contouring Control (MPCC)**-style formulation for smoother, more “expert” autonomous racing behavior.

---

## For AI agents

- **Purpose:** Executable migration plan. Implement phases in order (Phase 1 → 2 → 3). Phase 0 is already done.
- **Scope:** Lateral MPC in `src/scenic/domains/racing/mpc/` (mpc_lateral.py, reference_builder.py, config). Longitudinal/behavior in racing domain may need changes for velocity profile (see Observed behaviors and Implementation notes).
- **Key files:** `mpc_lateral.py`, `reference_builder.py`, `config.py`, `vehicle_mpc.yaml`.
- **Success:** Observed issues (see Observed behaviors) are reduced or eliminated; contouring + progress cost in place; velocity profile smooth and anticipatory where added.

---

## 0. Observed behaviors (from run logs and visualization)

These are the behaviors this plan aims to fix or improve. Evidence comes from `run.log` and in-sim visualization.

### Throttle/brake oscillation

- **What:** Controller flips between full throttle (e.g. throttle=0.65) and braking (brake=0.1) every few steps.
- **Log evidence:** Repeated pattern: `throttle=0.650, brake=0.000` for several steps, then `[Speed Limit] Speed 35.84m/s exceeds limit 35.8m/s, applying brake=0.100`, then `throttle=0.000, brake=0.100`, then back to full throttle.
- **Cause:** Speed target is a hard limit (e.g. 35.8 m/s). As soon as speed exceeds it, throttle is cut and brake applied; when speed drops below, full throttle resumes. No deadband or smooth transition.
- **Goal:** Smooth, anticipatory speed control (e.g. curvature-based velocity profile, slow-in before bends) so we do not ride the limit and oscillate.

### Speed over 100 km/h when entering a turn

- **What:** Vehicle enters a turn at very high speed (e.g. 35.5–35.9 m/s, about 129 km/h).
- **Log evidence:** `Speed: 35.84 m/s` (and similar) for many consecutive steps on approach to bends; segment_heading about -104 deg with very small steering (-0.001 to -0.008).
- **Cause:** No anticipatory speed reduction; speed is held at a constant cap until already in the turn. No slow-in (reduce speed before the turn).
- **Goal:** Velocity profile that reduces speed before the turn (curvature-based or progress-based), so we are not at 100+ km/h when steering is required.

### Did not steer left in time (missed turn)

- **What:** Vehicle fails to turn left enough and runs wide to the right of the path.
- **Log evidence:** CTE about -10 to -12 m (RIGHT); vehicle heading about -73 deg, path segment heading about -49 deg; position far right of path. Speed has dropped to about 17–18 m/s with heavy braking (brake=0.35) after missing the turn.
- **Cause:** Entered the turn too fast; lateral controller did not (or could not) apply enough steering in time. Combination of (1) speed too high for the curvature and (2) reactive rather than anticipatory control.
- **Goal:** Slow down before the turn (velocity profile) and follow path (contouring); avoid too fast, miss turn, then brake hard.

### Lateral oscillation (CTE) — analysis and fixes

- **What (from run.log):** CTE oscillates small (±0.18 m), then drifts to one side (-1.0 to -1.3 m), then overcorrects to the other side (+3.3 → +5.2 m). Steering flips and grows (e.g. steer 0.098 then -0.47).
- **Root causes:** (1) No CTE deadzone — small e_y is penalized so the controller constantly corrects. (2) Very low lateral weight on straights (w_ey_low_curv = 0.01) allows drift. (3) Abrupt switch at curvature thresholds (low/mid/high) causes sudden weight jump and overcorrection. (4) CTE multiplier (1.5x–3x when off-track) can make recovery too aggressive and overshoot.
- **Fix plan (Phase 3+ oscillation fixes):**
  1. **CTE deadzone:** Config `cte_deadzone` (e.g. 0.2 m). Before building MPC state, clamp e_y to 0 when |e_y| < deadzone so the MPC does not correct for tiny errors.
  2. **Cap CTE multiplier:** Config `cte_multiplier_max` (e.g. 1.5). When off-track, limit the tracking-weight multiplier to avoid over-aggressive recovery.
  3. **Raise w_ey_low_curv:** Increase from 0.01 to ~0.05–0.1 so the car does not drift as much on straights.
  4. **Smooth curvature weight blending:** Interpolate weights (e.g. w_ey) between low/mid/high using curvature instead of step switches at thresholds.
- **Status:** [x] Done (implemented). Re-run and check run.log to confirm behavior.

### Trajectory-consistent control (smooth turns, avoid overshoot)
- **Goal:** Controller sees the whole trajectory so we don’t over-brake then throttle, or over-steer left then correct right.
- **Implementation:** Behavior builds a single speed profile `v_ref_profile` (curvature + CTE limits) and passes it to both longitudinal MPC and lateral MPC. Reference builder accepts optional `v_ref_profile`; when provided, `v_ref` and `s_horizon` are built from it so lateral progress/lag cost matches the planned speed. Lateral and longitudinal thus share the same trajectory.
- **Softer CTE speed limits:** 2–3 m CTE → 7 m/s (was 5 m/s), 3–5 m CTE → 6 m/s (was 4 m/s) to avoid crawling in turns while still staying cautious off-line.
- **Status:** [x] Done (reference_builder.py, mpc_lateral.run_step, behaviors.scenic).

### Run-off track fix (Laguna Seca)
- **Issue:** Car ran off at ~7% lap (sharp turn): waypoint distances grew to 4.5 m, full brake/steer recovery. Cause: entered turn at ~35 m/s with curvature 0.037 (needs ~15 m/s); lookahead and slew were too small to slow in time.
- **Fixes in behaviors.scenic:** (1) **Longer curvature lookahead:** minimum 85 m when speed > 15 m/s so sharp turns (Corkscrew, hairpins) are seen early. (2) **Faster slew-down:** slew_down_ms 4.0 → 7.0 so speed reference can drop in time. (3) **Curvature speed margin:** v_max in turns = 88% of theoretical (curvature_speed_margin = 0.88) for run-off margin. Same margin applied in v_ref_profile build.
- **Status:** [x] Done.

### How do we make sharp turns?

**Question:** How do we avoid “too fast, didn’t turn in time” (high speed, then late steer/brake, CTE and waypoint distance growing)?

**Principle (from literature):** Autonomous racing uses **curvature-integrated velocity profiling** and **slow-in, fast-out**: the reference speed is reduced *before* the turn based on curvature ahead, so the vehicle is already at a safe speed when the bend tightens. Methods like CiMPCC and VPMPCC map centerline curvature to a reference velocity and encode it in the optimization; we achieve a similar effect in the behavior layer by computing a curvature-based speed limit over a long lookahead and applying a stricter margin when any significant curvature is ahead.

**What we do (behaviors.scenic):**

1. **See the turn early (lookahead):**
   - Minimum lookahead **85 m** when speed > 15 m/s (so we see sharp turns like the Corkscrew).
   - At **high speed** (e.g. > 25 m/s), minimum lookahead **120 m**.
   - At **very high speed** (e.g. > 40 m/s, 140 mph cap), minimum lookahead **250 m** so we see sharp turns (e.g. κ≈0.1) in time to brake without lowering max speed (braking from 46 m/s at 7 m/s/s needs ~5.5 s ≈ 250 m).

2. **Curvature-based speed limit:**
   - Along the lookahead we sample curvature (3-point method) and compute v_max at each point as `margin × sqrt(a_y_max / κ)`.
   - **Base margin:** 88% of theoretical (curvature_speed_margin = 0.88) for general run-off margin.
   - **Slow-in:** When max curvature in lookahead > 0.015 we apply a stricter margin (82%); when **κ > 0.05** (very sharp) we use **75%** so we slow enough for sharp bends without capping max speed on straights.

3. **Slew and sharing:**
   - **slew_down_ms = 7.0** so the speed reference can drop quickly when the limit drops (brake in time).
   - Single **v_ref_profile** (curvature + CTE limits) is passed to both lateral and longitudinal MPC so both “see” the same planned slowdown.

**Summary:** We make sharp turns by (1) looking far enough ahead to see the sharp part of the bend, (2) limiting speed by curvature with a safety margin, (3) using a stricter margin when any bend is ahead (slow-in), and (4) allowing the reference to slew down fast and sharing it with both controllers.

### Smoothness (steering oscillation + sharp brake/throttle)
- **Observed (run.log):** (1) Steering: large step changes and sign flips (e.g. steer -0.001 → 0.595 in 50 steps near end of lap; 0.22 → 0.39 → -0.05 earlier). (2) Brake/throttle: sharp brake (e.g. 0.25) then within 50 steps back to full throttle (0.65), or speed limit brake 0.1 then throttle; no deadband so controller flips at exactly 35.8 m/s.
- **Goal:** Smoother steering (less left–right oscillation) and smoother brake–throttle transitions (no sharp brake then sharp throttle).
- **Implemented (done):**
  1. **Speed limit deadband + hysteresis (behaviors.scenic):** `SPEED_LIMIT_DEADBAND = 0.5` m/s. Brake only when speed > limit + 0.5; once braking, stay in limit until speed < limit − 0.5. Reduces flip-flop at 35.8 m/s.
  2. **Steering rate weight (vehicle_mpc.yaml):** `w_du` 1.8 → 2.2 so MPC penalizes steering rate more (smoother steer).
  3. **Steering LPF (vehicle_mpc.yaml):** `steering_lpf_cutoff_hz` 2.0 → 1.5 for smoother steering output.
  4. **Speed reference slew-up (behaviors.scenic):** `slew_up_ms` 6.0 → 5.0 so after a turn the speed reference ramps up slightly slower; throttle recovery is less abrupt.
- **Status:** [x] Done. Re-run and check run.log for reduced oscillation and smoother pedal transitions.
- **If still needed (to tune):** Increase `w_du` or lower LPF further; widen speed deadband (e.g. 0.7 m/s); or add explicit throttle ramp after brake release in behavior.
- **Over-braking on new TTL:** If run.log shows brake too much → throttle to compensate, consider: curvature_speed_margin or slow-in margins (82%/75%) for racing-line TTL, or throttle ramp after brake release.

### Racing-line TTL (ttl_racing_line_xodr.csv) — status and new behavior

- **TTL:** Curvature-capped racing line (κ ≤ 0.097 1/m) in `assets/ttls/.../transformed/ttl_racing_line_xodr.csv`. Path is feasible; no run-off from infeasible curvature.
- **Comparison vs centerline (ttl_fellow_test_xodr_all.csv):** With the centerline TTL, lateral following and speed profile are the baseline. With the racing-line TTL, lateral following remains good (path is feasible). The **new** observed issue is **over-braking then throttle to compensate**: curvature-based slow-in often reduces speed too much (e.g. 42→8 m/s, 35→10 m/s); the controller then applies throttle to recover. Pattern: brake=0.25 for many steps → speed drops to 8–15 m/s → throttle 0.05–1.0. Sometimes with large CTE (4–10 m) so throttle remains limited. Log evidence (with timestamps in run.log): e.g. Step 200→250 (brake then full throttle), 1250→1300 (36→13 m/s then low throttle), 1950→2100 (42→8 m/s, CTE 6.9 m), 2850→2900 (35→10 m/s, CTE 9.7 m).
- **Goal:** Reduce over-braking so we don’t overslow into turns, then need to throttle hard to compensate.
- **Status:** [x] Tuned (general fixes applicable to both centerline and racing-line TTL): (1) curvature_speed_margin 88%→90%, slow-in 82%/75%→85%/78%; (2) throttle ramp after brake release (10 steps, cap 0.25→1.0).

### ttl_racing_line_xodr run (post run-off fix) — high CTE in curves and end-of-straight

- **Run (result_data/ttl_racing_line_xodr):** 70.6% progress (2536 waypoints), 148.4 s, no run-off; CTE-based braking triggered in segment 44 (~4.5 m) and car slowed. The generic CTE/speed fix prevented run-off but lateral error remained high in places.
- **Evidence (segments.csv):** Segment **43 (curve)** mean |CTE| 7.99 m, **max 11.98 m** (worst). Segment **44 (straight)** max 4.56 m (heavy brake here). Other curves with 5–6 m: 19, 29, 45, 7.
- **Latest run (run.log added to result_data):** `analyze_racing_log --log run.log` was run; results written to `result_data/ttl_racing_line_xodr/`. Summary: **148.45 s**, **2512 waypoint hits** (~69.9% of 3591), **max |CTE| 11.77 m**. Segment-level CTE (from sampled MPC steps in log): **Segment 43** mean 7.68 m, max **11.77 m**; **Segment 44** mean 1.10 m, max **4.49 m** (heavy brake at end of run); **Segment 27** max 8.54 m; **Segment 45** max 6.10 m; **Segment 37** 4.48 m; **Segment 19** 3.97 m; **Segment 32** (straight) 3.98 m. Comparison with `ttl_fellow_test_xodr_all` (centerline): racing-line run is ~3 s faster (148 vs 151 s); segment 43/44 max CTE are similar (11.77 vs 11.97, 4.49 vs 4.49). Some segments (4, 7, 18, 19) show higher CTE on racing-line; others (12, 29) are better. Use `compare_racing_results` for full segment-by-segment comparison.
- **Run with high_curvature_threshold=0.06 (edit_note in log → summary.json):** Run.log from 2026-02-17 had `[RacingRun] ... edit_note=high_curvature_threshold=0.06`. Analyzer now prefers `[RacingRun]` over `[TTL] Assigned` so edit_note and run_timestamp are stored in `result_data/ttl_racing_line_xodr/summary.json` and shown in `compare_racing_results`. **Findings for this run:** 148.45 s, **2537 waypoint hits** (70.6%), **max |CTE| 12.02 m**. Segment **43 (curve)** mean 7.92 m, max **12.02 m** (worst); **44 (straight)** max **4.49 m** (heavy brake at end); **27 (curve)** mean 6.16 m, max **10.56 m**; **45** max 5.96 m; **37** max 4.33 m; **8 (straight)** max 4.34 m; **18** max 4.57 m. **Comparison vs run with threshold 0.07:** With 0.06, segment 43 max CTE increased slightly (12.02 vs 11.77 m) and segment 27 worsened (10.56 vs 8.54 m); segment 44 unchanged (4.49 m). So lowering the threshold did not improve the worst segments and may have expanded “high curvature” into more segments (e.g. 27, 8, 10, 12) with mixed effect. **Comparison vs centerline (ttl_fellow_test_xodr_all):** Racing-line (0.06) is better in segments 4 (0.26 vs 3.26 m), 19 (0.69 vs 0.61), 29 (2.15 vs 5.92), 38 (0.03 vs 0.12); worse in 8 (4.34 vs 0.99), 10 (2.44 vs 0.17), 12 (3.48 vs 3.14), 27 (10.56 vs 5.91), 43 (12.02 vs 11.97), 44 (4.49 vs 4.49).
- **Implications for MPC:**
  1. **Sharp curves (e.g. segment 43):** Either we still enter too fast (curvature speed / slow-in) or the lateral controller cannot correct in time (preview, contouring weight, or horizon). Improve by: (a) stricter slow-in for very high curvature (κ > 0.08); (b) higher contouring weight in high curvature so MPC prioritizes reducing e_y in tight bends; (c) longer effective preview when speed is high so the bend is seen earlier.
  2. **End-of-straight (segment 44):** High CTE (4.5 m) and heavy brake suggest either carry-over from segment 43 (exiting 43 with large error) or speed still too high at 43→44 transition. Improve by: tighter curvature-based speed at exit of the preceding sharp curve (stricter slow-in) so we enter 44 with lower speed and smaller error.
  3. **Data-driven tuning (generic):** Use `result_data/<run_id>/segments.csv` (mean_abs_cte_m, max_abs_cte_m per segment) and `compare_racing_results` to find which segments consistently have high CTE across runs. Apply only **generic** fixes: curvature-based speed limits, curvature-dependent MPC weights, preview/lookahead. No segment-ID or TTL-specific logic in the MPC.
- **Goal:** Reduce max |CTE| in curves (e.g. segment 43 < 6 m) and in segment 44 (< 3 m) while keeping no run-off; smooth speed and steering through 43→44.
- **Status:** [x] Done (code changes). (1) **Behaviors:** Stricter slow-in for κ > 0.08 (margin 74%); minimum lookahead 120 m when speed > 20 m/s so sharp bends are fully seen. (2) **MPC config:** high_curvature_threshold 0.1 → 0.07, then **0.06** (with run_edit_note in vehicle_mpc.yaml so analyze_racing_log tags results). w_ey_high_curv 18, w_epsi_high_curv 10. Run with 0.06 analyzed; segment 43/27 did not improve (see above). Next: try weight increases or revert threshold.
- **Potential MPC improvements (from run.log analysis and 0.06 run):**
  1. **High curvature (after trying 0.06):** Lowering threshold to 0.06 did not reduce max CTE in segment 43/27; segment 27 got worse. **Proposal:** Revert `high_curvature_threshold` to **0.07** and instead increase **w_ey_high_curv** (e.g. 18 → 22) and **w_epsi_high_curv** (e.g. 10 → 12) so contouring is stronger only where we already classify as high curvature, without expanding that zone.
  2. **Preview/horizon:** In high curvature, use a longer effective horizon or more path points so the MPC sees more of the bend ahead (reduces late correction in segment 43).
  3. **Slower entry into sharp curves:** Stricter slow-in for very high κ (e.g. margin 70% for κ > 0.09) or earlier trigger so segment-43-type curves are taken slower.
  4. **Segment 44 (end-of-straight):** Ensure speed is reduced before the 43→44 transition (stricter slow-in at exit of 43); optionally cap speed when |CTE| is already high to avoid heavy CTE-based braking.
  5. **Logging:** Increase MPC step log frequency (or log every step to a separate file) so `analyze_racing_log` gets full per-segment CTE statistics instead of ~61 samples per run.
  6. **Analyzer (edit_note):** Fixed so that when `[RacingRun]` appears after `[TTL] Assigned` in the log, the analyzer still captures **edit_note** and **run_timestamp** and writes them to summary.json (and compare_racing_results shows them). Update `run_edit_note` in vehicle_mpc.yaml for each tuning change so runs are tagged.

### What we already fixed (and what they do not fix)

- **Segment blending + hysteresis:** Address lateral steering oscillation at segment boundaries (left–right–left). Do not fix throttle/brake oscillation or too fast into turn.
- **Quick fixes (w_du, w_ddu, LPF, w_du_lon, w_a):** Smooth reaction (less aggressive steering and pedal changes). Do not fix the binary speed limit that causes throttle/brake flip or anticipation (slow-in, steer earlier).
- **ReferenceBuilder spline `dz` bug:** With 2D position `(x, y)` and a 3D spline, `dz` was never set but was used in gradient/hessian in `project_to_spline()`, causing "cannot access local variable 'dz'" and forcing linear fallback every step (worsening L–R oscillation). Fixed by using `dz` only when `len(position) >= 3` and `len(point) >= 3` (and `len(deriv)`, `len(deriv2)` for hessian). Spline path is now used when possible; Phase 2 contouring+progress will help further.
- **Log cleanup:** Noisy per-step prints removed or throttled in MPC, behavior, dSPACE model, and readback (spline fallback logged once per run; step summary every 50 steps; WAYPOINT HIT kept).
- **Log timestamps:** Run logs include simulation time `t=Xs` (e.g. `t=2.50s`) so runs with different TTLs or configs can be compared systematically. Time is step × dt with dt=0.05 s (scenario time_step).

### Summary: what is done vs what to fix/tune

| Item | Status | Where |
|------|--------|--------|
| Run-off track (Laguna Seca) | Done | Lookahead 85 m, slew_down 7, curvature margin 0.88 |
| **Sharp turns (slow-in, see turn early)** | Done | Lookahead 120 m when speed>20 m/s, 250 m when >40 m/s; slow-in 74% (κ>0.08), 82% (κ>0.05), 88% (κ>0.015) (behaviors.scenic) |
| Trajectory-consistent control (v_ref_profile shared) | Done | reference_builder, mpc_lateral, behaviors |
| Lateral oscillation (CTE deadzone, weight blend) | Done | config + mpc_lateral |
| **Smoothness: steering oscillation** | Done | w_du 2.2, steering_lpf 1.5 Hz |
| **Smoothness: sharp brake then throttle** | Done | Speed limit deadband 0.5 m/s + hysteresis, slew_up 5.0 |
| **End-of-lap throttle (TTL wrap)** | Done | Curvature lookahead wraps waypoints so we see straight after loop; TTL gap first/last ~0.68 m (doc) |
| **Racing-line TTL (feasible path)** | Done | Curvature cap in generate_racing_line.py; ttl_racing_line_xodr.csv |
| **Over-braking then throttle** | Done | curvature_speed_margin 90%, slow-in 85%/78%; throttle ramp after brake (behaviors.scenic) |
| **High CTE in curves (seg 43, 19, 29, 45) and segment 44 (ttl_racing_line_xodr)** | In progress | Stricter slow-in κ>0.08 (74%); lookahead ≥120 m; high_curvature_threshold 0.06 tried → seg 43/27 no improvement (max 12.02 m, 10.56 m). Next: revert to 0.07 and try w_ey_high_curv 22, w_epsi_high_curv 12; use run_edit_note to tag runs. |
| Further tuning if needed | Optional | Widen deadband; throttle ramp after brake; increase w_du_lon |

### TTL loop and end-of-lap throttle (why we didn’t throttle hard on the “straight” at the end)

- **TTL CSV (ttl_fellow_test_xodr_all.csv):** The file has 3591 waypoints (indices 0–3590). The **first** point is (55.766, 88.269) and the **last** is (56.156, 88.827). The distance between them is **~0.68 m** — so the TTL does **not** form a perfect closed loop; there is a small gap between the end and the start.
- **Why we didn’t throttle hard towards the end of the log:** The waypoint list was **not wrapped**. When `wp_last_idx` is in the 3570s, the curvature lookahead runs out at index 3590 after only the remaining segments (~14 segments, ~6 m). So (1) we never “see” the straight after the loop (waypoints 0, 1, 2, …), and (2) the short stretch we do see is the final curve into the start, so `curvature_ahead_max` can be non-zero and the slow-in cap keeps the speed limit low. The controller therefore never gets a “long straight ahead” signal and doesn’t command full speed.
- **Fix (done):** Curvature lookahead in `behaviors.scenic` now **wraps** waypoint indices: when we reach the end of the list we continue with indices 0, 1, 2, … (and stop after one full lap to avoid infinite loop). Curvature is computed with modulo indexing so the segment 3590→0 is included. Near the end of the lap we now see the straight after the loop and can throttle up.
- **Optional:** To make the TTL a perfect loop, append the first point to the CSV or adjust the last point so it coincides with the first (within tolerance).

---

## 1. Current vs MPCC-Style

| Aspect | Current (trajectory-tracking MPC) | MPCC-style |
|--------|-----------------------------------|------------|
| **Objective** | Minimize CTE (e_y) and heading error (e_ψ) along a fixed reference path. | Minimize **contouring error** (lateral deviation) and **lag error** (progress along path) while **maximizing progress** (or minimizing time). |
| **Path coupling** | Track a precomputed path; reference is (x,y,ψ,κ) along path. | Path is parameterized (e.g. arc length s); controller optimizes **progress s** and **lateral deviation** jointly. |
| **Smoothness** | Achieved by cost weights (w_du, w_ddu) and reference blending. | Built-in: **input rate penalties** and contouring cost naturally favor smooth steering and progress. |
| **Reference** | Single reference path; we blend ψ at segment boundaries. | Path is a **continuously parameterized curve** τ(s); state includes **progress s** (or θ) as an integrator. |

---

## 2. MPCC Concepts (from literature)

- **Contouring error**: Lateral distance from the reference path (similar to our e_y).
- **Lag error**: Difference between desired progress and actual progress along the path (encourages moving along the path, not just being close to it).
- **Progress state**: Augment dynamics with a state (e.g. θ or s) that evolves with speed and path geometry; the cost encourages increasing progress while keeping contouring error small.
- **Cost**: Typically something like  
  `Q_contour * e_y^2 + Q_lag * (s_ref - s)^2 - Q_progress * progress_increment`  
  plus **input and input-rate penalties** (steering, acceleration).
- **Constraints**: Same as now (track bounds, steering/accel limits, steer rate).

References: ETH Zurich MPCC (e.g. [Liniger MPCC](https://github.com/alexliniger/MPCC)), “Nonlinear Model-Predictive Contouring Controller” (IEEE), MURDriverless/mpcc.

---

## 3. Phased Migration Plan

### Phase 0: Done / quick fixes (current)

- [x] Reference blend at segment boundaries (ψ_ref, Option A).
- [x] Segment selection hysteresis (no advance cap).
- [x] Increased steering rate and acceleration penalties (w_du, w_ddu) and lower LPF cutoffs for smoother output.
- [x] Increased longitudinal smoothness (w_a, w_du_lon, throttle/brake LPF).

### Phase 1: Path parameterization and progress state (pre-MPCC) — **DONE**

**Goal:** Introduce arc-length parameterization and a “progress” notion so we can later add lag/progress terms.

- [x] **Path as τ(s):** Reference path is a smooth function of arc length s (spline or linear). ReferenceBuilder exposes s via `project_to_spline` and s_cumulative.
- [x] **Current progress s_0:** At each step, s_0 from projection onto path; returned from `build_reference()` and stored as `_last_s_0` on the lateral controller.
- [x] **Preview in s:** Horizon s_k = s_0 + speed·(k+1)·dt; reference sampled at those s; `build_reference()` returns `s_horizon`.
- [x] **No cost change:** Cost unchanged (e_y, e_ψ, u, du, ddu).

- **Files touched:** `reference_builder.py`, `mpc_lateral.py`; tests in `test_reference_builder.py`, `test_mpc_lateral.py`.
- **Success criteria:** Met: ref returns (..., s_0, s_horizon); MPC stores `_last_s_0`, `_last_s_horizon`; behavior unchanged.

### After Phase 1: What to visualize

- **Progress s_0 over time:** Log or plot `lat_controller._last_s_0` each step (e.g. in run.log or a small CSV). It should increase monotonically along the lap and reset or wrap when the path is closed.
- **Horizon in s:** Log `lat_controller._last_s_horizon` (array of length horizon_steps). Check that s_horizon[k] ≈ s_0 + speed·(k+1)·dt and that the spacing is consistent.
- **Reference consistency:** Overlay the reference points (x(s_k), y(s_k)) for the current horizon on the track map; they should lie on the path and advance with the vehicle.
- **Behavior parity:** Compare a short run (e.g. one lap) before vs after Phase 1: steering, throttle/brake, and lap time should be effectively unchanged (no cost change).

### Phase 2: Add lag error and progress incentive — **DONE**

**Goal:** Move cost toward MPCC: penalize “falling behind” and reward “making progress.”

- [x] **Lag error:** Cost term `Q_lag * (s_ref_k - s_k)^2` at each step (s_ref from s_horizon; s_ref_0 = s_0).
- [x] **Progress term:** Terminal cost `-Q_progress * (s_N - s_0)` (linear in q so optimizer prefers advancing).
- [x] **Config:** `Q_lag`, `Q_progress` in config.py and vehicle_mpc.yaml; default 0.0 for backward compat / A-B test.
- [x] **Balance:** Contouring (e_y, e_psi) and input/rate penalties unchanged; tune Q_lag and Q_progress (e.g. 0.01–0.1 lag, 0.001–0.01 progress) for smooth behavior.

- **Files touched:** `mpc_lateral.py` (cost, state n_x=4, dynamics, run_step), `config.py`, `vehicle_mpc.yaml`.
- **Success criteria:** Met: cost includes lag and progress; zero weights = trajectory-tracking only; tests pass.

### Phase 3: Full MPCC-style formulation — **DONE**

- **Goal:** Align with standard MPCC: progress as part of dynamics, contouring + lag + progress in cost.
- [x] **Augmented state:** State [e_y, e_psi, delta, s]; progress dynamics s_{k+1} = s_k + v_ref_k*dt (linearized; full MPCC s_dot = v*cos(e_psi)).
- [x] **Contouring + lag + progress cost:** Contouring (e_y, e_psi), lag Q_lag*(s_ref - s)^2, progress reward -Q_progress*(s_N - s_0). All active by default.
- [x] **Solver:** QP with linearized progress; OSQP with relaxed tolerances and accept "solved inaccurate" (Phase 2 fix).
- [x] **Defaults:** Q_lag=0.02, Q_progress=0.005 in config and YAML so MPCC cost is on by default; set both to 0 for trajectory-tracking only.
- **Files touched:** `mpc_lateral.py` (docstrings, progress comment), `config.py`, `vehicle_mpc.yaml`.
- **Success criteria:** Met: MPCC-style lateral controller; contouring+lag+progress cost active by default; same tracks usable; analyze behaviors and tune as integrated project.

---

## 4. Implementation Notes

- **Config:** Add `Q_lag`, `Q_progress` (and optionally contour weight names) to `MPCConfig` and YAML when moving to Phase 2.
- **Reference builder:** Refactor to output reference as a function of s (and optionally time) so both current MPC and future MPCC can share the same path representation.
- **Backward compatibility:** Keep current trajectory-tracking cost as an option (e.g. flag or zero Q_progress/Q_lag) so we can A/B test.
- **Longitudinal:** MPCC often couples velocity to curvature (slow-in). Keep or extend curvature-based speed limit and smooth velocity profile in the behavior/longitudinal layer.

---

## 5. References

- ETH Zurich MPCC: [alexliniger/MPCC](https://github.com/alexliniger/MPCC), [MURDriverless/mpcc](https://github.com/MURDriverless/mpcc).
- “A Nonlinear Model-Predictive Contouring Controller for Shared Control Driving Assistance in High-Performance Scenarios” (IEEE).
- “Learning Model Predictive Control with Error Dynamics Regression for Autonomous Racing” (arXiv 2309.10716) — LMPC for robustness and smoothness.
- “Towards time-optimal race car driving using nonlinear MPC” (IEEE) — input rate penalties and cost tuning.

---

## 6. Quick reference: what changed in quick fixes

- **Lateral:** `w_du` 0.75 → 1.8; `w_ddu` 0.000005 → 0.00003; `w_ddu_high_curv` increased; `steering_lpf_cutoff_hz` 3.0 → 2.0.
- **Longitudinal:** `w_a` 0.1 → 0.25; `w_du_lon` 1.0 → 2.0; throttle/brake LPF 5.0 → 3.5 Hz.

These favor smoother steering and throttle/brake and reduce “beginner-style” overcorrect; they are independent of the MPCC migration.

---

## 7. Quick reference for agents (phase → files → check)

| Phase | Main files | What to change | After edit, verify |
|-------|------------|----------------|--------------------|
| 0 | (done) | — | — |
| 1 | `reference_builder.py`, `mpc_lateral.py` | (done) Expose s; compute s_0; build ref in s; no cost change | Ref returns s_0, s_horizon; MPC stores them; behavior same |
| 2 | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | (done) Add progress state s, Q_lag, Q_progress; config keys | Cost terms present; Q_lag/Q_progress=0 for A/B |
| 3 | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | (done) MPCC cost on by default; docstrings; Q_lag/Q_progress defaults | Contouring+lag+progress active; set to 0 for tracking-only |
| 3+ | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | (done) Oscillation: CTE deadzone, cte_multiplier_max, w_ey_low_curv, smooth curvature blend | Re-run and verify via run.log |
