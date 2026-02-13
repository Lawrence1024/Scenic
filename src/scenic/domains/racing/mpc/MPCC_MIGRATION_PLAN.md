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

### What we already fixed (and what they do not fix)

- **Segment blending + hysteresis:** Address lateral steering oscillation at segment boundaries (left–right–left). Do not fix throttle/brake oscillation or too fast into turn.
- **Quick fixes (w_du, w_ddu, LPF, w_du_lon, w_a):** Smooth reaction (less aggressive steering and pedal changes). Do not fix the binary speed limit that causes throttle/brake flip or anticipation (slow-in, steer earlier).

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

### Phase 1: Path parameterization and progress state (pre-MPCC)

**Goal:** Introduce arc-length parameterization and a “progress” notion so we can later add lag/progress terms.

1. **Path as τ(s):** Ensure the reference path is available as a smooth function of arc length s (spline or resampled waypoints with s). ReferenceBuilder already uses splines; expose or compute s along the path.
2. **Current progress s_0:** At each step, compute current progress s_0 (e.g. projection onto path, or s at closest point). Pass s_0 into the controller.
3. **Preview in s:** Build reference (ψ_ref, κ_ref, v_ref) as a function of **s** (not just waypoint index), so the horizon is “s_0, s_0 + Δs_1, …” with consistent spacing in arc length (or time with v_ref).
4. **No cost change yet:** Keep current cost (e_y, e_ψ, u, du, ddu). This phase is mainly refactor for s-based reference.

- **Files to touch:** `reference_builder.py`, `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` (if new params).
- **Success criteria:** Reference builder and lateral MPC use s; state or ref_builder output includes s_0 and s along horizon; existing behavior unchanged (no cost change).

### Phase 2: Add lag error and progress incentive

**Goal:** Move cost toward MPCC: penalize “falling behind” and reward “making progress.”

1. **Lag error:** Define desired progress s_ref(t) or s_ref(k) (e.g. from speed profile). Add term to cost: `Q_lag * (s_ref - s)^2` at each step (or only at terminal).
2. **Progress term:** Add a negative cost (or reward) for progress made over the horizon, e.g. `-Q_progress * (s_N - s_0)` so the optimizer prefers advancing along the path. Tune Q_progress vs Q_contour so we don’t sacrifice tracking.
3. **Balance:** Keep contouring (e_y, e_ψ) and input/rate penalties; tune Q_lag and Q_progress so behavior is smooth and not overly aggressive.

- **Files to touch:** `mpc_lateral.py` (cost), `config.py`, `vehicle_mpc.yaml` (Q_lag, Q_progress).
- **Success criteria:** Cost function includes lag and progress terms; tuning guidelines in config or comments; backward-compat option (e.g. zero Q_progress/Q_lag) for A/B test.

### Phase 3: Full MPCC-style formulation (optional)

- **Goal:** Align with standard MPCC: progress as part of dynamics, contouring + lag + progress in cost.
- **Files to touch:** `mpc_lateral.py` (dynamics, cost, possibly solver), `reference_builder.py`, `config.py`, `vehicle_mpc.yaml`.
- **Concrete steps:**
  1. Augmented state: Add progress s (or θ) as a state; dynamics: s_{k+1} = s_k + f(v, ψ, path) (e.g. from path tangent and speed). May require path curvature κ(s) and heading ψ(s) along path.
  2. Contouring + lag cost: Standard MPCC cost: contouring error (e_y), lag error (s_ref - s), and progress reward.
  3. Solver: If dynamics become nonlinear in s, may need NMPC or linearization; current QP may suffice if we keep a linearized progress update.
  4. References: Use ETH/MPCC papers and open-source code for exact cost and dynamics formulations.
- **Success criteria:** MPCC-style lateral controller with progress state and contouring/lag/progress cost; validation on same tracks as current MPC.

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
| 1 | `reference_builder.py`, `mpc_lateral.py`, `config.py` | Expose s along path; compute s_0; build ref in s; no cost change | Ref has s_0 and s on horizon; behavior same as before |
| 2 | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | Add Q_lag, Q_progress to cost; config keys Q_lag, Q_progress | Cost terms present; optional zero weights for A/B |
| 3 | `mpc_lateral.py`, `reference_builder.py`, `config.py`, `vehicle_mpc.yaml` | Progress in dynamics; full MPCC cost; solver if needed | Progress state; contouring+lag+progress cost; same tracks |
