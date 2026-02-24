Got it. I’ll do two things:

1. **“Note what we’ve already tried”** (from your MPCC migration plan file), so I don’t re-suggest the same knobs. 
2. Propose a **recommended to-do sequence** focused on your real pain point: **back-to-back bad planning when approaching a curve** (not post-corner brake/throttle switching), plus your idea about **waypoint distance thresholds / CTE deadzone**—but implemented in a way that *won’t* recreate the “recenter while still far off” bug you already fought.

Also: the system tells me **some earlier uploads in this chat expired**. If you want me to re-open any older logs or prior plan versions beyond the current one you just uploaded, please re-upload them. (Your current plan file is available.)

---

## A. What you’ve already tried (so we don’t repeat)

From the plan, you’ve done a *lot* of “outer-loop stabilization” work already , including:

### Lateral MPCC / reference / behavior

* **Weight balancing to reduce “recenter then steer again”** (reduced `w_epsi_high_curv`, also velocity-scaled heading weights in curves; later added *near-line heading scaling* when |CTE| small in high curvature). 
* **Anti-cross / don’t cross TTL** via `w_anticross` (and even stronger in high curvature) + behavior “approach zone” scaling and slew caps in a narrow CTE band. 
* **Reference smoothing at segment boundaries**: smoothing κ_ref (window increased) and ψ_ref smoothing with angle-wrap safety. 
* **Horizon / preview changes**: increased lateral horizon (35→45) to “see the whole bend”. 
* **Lots of actuator/output smoothing attempts**: higher `w_du`, higher `w_ddu_high_curv` (several rounds), and lowering steering LPF cutoff (down to ~0.7 Hz). 
* **Slew logic changes**: you experimented with *tightening slew when CTE is large* (counterintuitive but correct for “steer a little over longer time”), plus *additional tighter slew in curves when off-line*. 
* **Steer caps**: fixed cap (too low → understeer), then dynamic `f(speed, curvature)` cap, then max-in-curves caps (0.6 → 0.5), and other cap variants. 
* **Segment/reference switching defenses**:

  * segment hysteresis + “stick to segment when far off” by increasing `segment_stick_cte_m` (e.g., 1.5 → 3.5 m) because you saw cases where MPC e_y looked small while behavior CTE was large. 
* **CTE deadzone tweaks** (made smaller), then **skipping deadzone when far off** (CTE > 2m) because deadzone was zeroing e_y at exactly the wrong time. 
* **Safety / recovery**: clamp / safe defaults when CTE becomes insane (>50m), skip MPCC call in that case. 
* **Diagnostics**: you added dense “steer pipeline” logs to separate raw MPC steer vs post-processing effects. 

### Longitudinal (relevant only insofar as it affects curve entry)

* Curvature-based speed limit margins, min corner speed floor, straight speed floor to avoid gear/downshift issues, deadband/hysteresis and ramps. 

**Key takeaway:** You’ve already tried most “tuning + smoothing + caps” levers. The remaining leverage is mostly about **reference/waypoint continuity and MPCC decision consistency at curve approach**, not yet another cap/LPF tweak.

---

## B. Recommended to-do steps (targeting bad planning when approaching curves)

Below is a sequence where each step is:

* **Concrete to implement**
* **Local in scope**
* Has a **clear expected behavior change**
* And won’t fight the stuff you already added

### To-Do 1 — Add a “reference continuity gate” on waypoint/segment selection (distance threshold + progress consistency)

You specifically asked about “distance threshold on our waypoint.” I agree — but the *right* version is **not** a simple “if far then deadzone”; it’s: **don’t let the reference jump to a different local minimum when approaching a curve.**

**Implement**

* When selecting the closest waypoint / segment for MPCC reference:

  1. Compute the best candidate as usual.
  2. If the candidate is “too far” OR would imply **non-monotonic progress** relative to last step (s decreases / jumps backwards / jumps forward too much), **reject it** and keep the previous segment (or constrain search to a forward window around previous `s`).
* Add two thresholds (start conservative):

  * `MAX_WP_MATCH_DIST_M` (e.g. 2–4 m; tune by track width)
  * `MAX_S_JUMP_M` (e.g. 3–6 m per tick, depends on speed and dt)
* This is basically a “data association gate” like tracking: don’t swap targets when association is weak.

**Expected behavior**

* On curve entry, you should stop seeing the pathology: **behavior CTE is large but MPCC e_y becomes tiny** (because reference snapped somewhere else).
* Fewer “back-to-back bad plans” at the start of the bend; steering should be more consistent across 5–15 ticks as you approach the corner.

**What to measure**

* Count events where `|CTE_behavior| > 2m` but `|e_y_mpc| < 0.2m`. Those should drop sharply.

---

### To-Do 2 — Replace “CTE deadzone” idea with a *conditional* deadzone that only applies when association is good

A pure CTE deadzone is dangerous (you already saw it can zero e_y while far off). The safe version is:

**Implement**

* Only apply deadzone if **all** are true:

  * `|CTE| < dz_cte` (small)
  * `wp_match_dist < dist_ok` (association is good)
  * and (optionally) `curvature` is low-to-mid (not high)
* Else: do **not** deadzone e_y.

This is very aligned with what you already learned (“skip deadzone when far off”), but now it’s tied to the waypoint-distance gate.

**Expected behavior**

* Near the line on straights: less micro-correction chatter.
* Approaching a curve or when the reference is uncertain: MPCC keeps correcting (no “recenter while still wrong”).

**What to measure**

* Reduced small-amplitude steer noise on straights **without** increasing curve-entry oscillation.

---

### To-Do 3 — Add a “curve-approach commitment” mode for 1–2 seconds before turn-in

Your issue statement: “constant back to back bad planning when approaching the curve.”
That often happens because the optimizer is re-solving each tick with slightly different curvature preview and keeps changing its mind.

**Implement**

* Detect “approaching a curve” using curvature ahead (you already have `curvature_ahead_max`).
* When entering this regime, latch a small set of planning knobs for a short window (e.g. 1.0–1.5s):

  * Freeze (or heavily low-pass) the **reference segment index** / `s_ref` anchor
  * Keep curvature regime “mid/high” weights stable (don’t blend low↔mid↔low every tick due to threshold jitter)
  * Optionally clamp how much the reference heading/curvature can change per tick

This is *not* slew limiting the steer; it’s stabilizing the **plan inputs**.

**Expected behavior**

* Instead of “plan A, plan B, plan C” in consecutive ticks right before the corner, you get **one coherent plan** that evolves smoothly.
* Steering should build more smoothly and not “abort” mid-approach.

**What to measure**

* Reduced variance of `curv_regime`/weights and reduced segment switches in the ~2s window before the curve.

---

### To-Do 4 — Add curvature feedforward steering (the one big thing you haven’t tried in the plan)

This is still my strongest recommendation, but I’m listing it *after* the reference continuity gates because you explicitly want to focus on approach planning.

**Implement**

* Compute `delta_ff = atan(L * kappa_ref)` along the horizon.
* Optimize `delta = delta_ff + delta_fb` (optimize delta_fb), or add `(delta - delta_ff)^2` cost.

**Expected behavior**

* On curve approach: MPCC stops “discovering” the turn late; it starts with the right steering bias.
* Less need for anti-cross hacks and extreme LPF because the base plan is correct earlier.

**What to measure**

* Fewer large steering sign flips in the first half of the bend.
* Lower peak |CTE| during curve entry.

---

### To-Do 5 — Keep your “segment stick” but make it depend on *match quality*, not only |prev_e_y|

You already raised `segment_stick_cte_m`, which helped some failure modes but can also “stick to a bad association.”

**Implement**

* Stick if either:

  * `|prev_e_y| > stick_m` **OR**
  * `wp_match_dist > dist_ok` (association uncertain)
* But allow switching when:

  * `wp_match_dist` is small and progress is consistent

**Expected behavior**

* Less “snap to a different segment” right when approaching the curve.
* But you don’t get trapped sticking forever when you actually should switch.

---

## C. Suggested implementation order (fastest feedback)

1. **To-Do 1 (waypoint/segment gating)**
2. **To-Do 2 (conditional deadzone)**
3. **To-Do 3 (curve-approach commitment window)**
4. **To-Do 4 (curvature feedforward)**
5. **To-Do 5 (stick depends on match quality)**

This order gives you quick wins *without* re-tuning 30 weights.

---

## D. What “good” should look like after each step

* After **To-Do 1**: your logs should stop showing “MPC thinks e_y≈0” while behavior says CTE≈3–6m when entering a curve.
* After **To-Do 2**: straights get calmer; curve entry does **not** get more recentering.
* After **To-Do 3**: you stop seeing back-to-back conflicting steering decisions in the 1–2 seconds before turn-in.
* After **To-Do 4**: turn-in becomes earlier and smoother; fewer TTL crossings.
* After **To-Do 5**: fewer “reference snaps,” but also fewer “stuck on bad segment” events.

---

## E. Implemented: Single-segment consistency (steering wobble fix)

**Problem:** At curve exits, feedforward (delta_ff, kappa_ref) and feedback (e_y, e_psi → delta_fb) were built from **different segment indices**. The reference builder used “nearest waypoint by node distance”; lateral errors used “best segment by perpendicular distance” (with gate, hysteresis, stick). When they disagreed, kappa_ref said “turn left” (curve segment) while e_y said “you’re left of path” (exit segment) so delta_fb commanded “turn right,” causing **steering sign flips** (left → right → left) and FF TRIPWIRE logs.

**Fix:** Use one segment index for both reference and errors in each step.

1. **`mpc_lateral.run_step`**  
   *Compute errors first.* Call `_compute_errors(...)` at the start of the step to get `(e_y, e_psi, mpc_segment_idx)`. Then call `ref_builder.build_reference(..., reference_segment_idx=mpc_segment_idx)` so the reference (kappa_ref, s_0, horizon) is built from the **same** segment as e_y/e_psi.

2. **`reference_builder.build_reference`**  
   New optional argument **`reference_segment_idx: Optional[int] = None`**. When provided and valid, use it as `nearest_idx` (and update `_last_nearest_idx`) instead of calling `find_nearest_waypoint`. Spline and polyline branches already use `nearest_idx` for the reference window and s_0, so kappa_ref and s_0 now align with the lateral MPC segment.

**Files:**  
* `src/scenic/domains/racing/mpc/mpc_lateral.py`: run_step reordered; single call to _compute_errors; build_reference(..., reference_segment_idx=mpc_segment_idx).  
* `src/scenic/domains/racing/mpc/reference_builder.py`: build_reference(..., reference_segment_idx=...); when set, nearest_idx = reference_segment_idx.

**Expected behavior:**  
Feedforward and feedback refer to the same path segment every step, so no structural ff/fb opposition at curve exits. Steering sign flips (left→right→left) and “FF TRIPWIRE 1” from delta_fb opposing delta_ff should drop or disappear when the cause was segment mismatch.

---

If you tell me (a) where waypoint selection happens (file/function) and (b) whether your MPCC uses a projected `s` state internally or only segment indices, I can write you **exact pseudocode** for the gating logic in To-Do 1/2/5 in the style of your current stack.
