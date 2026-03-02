Got it — **kinematic bicycle**, (N=35), (\Delta t=0.05) → **1.75 s preview**.

That combo *will* complain (or saturate and still miss) if the reference path requires curvature above your steering-limited curvature **or** if your speed controller lets you arrive too fast for that curvature. With a kinematic bicycle, you don’t have “slip” to save you.

Below are the most important fixes and concrete MPC structure improvements.

---

## Implemented (MPC improvements for sharp turns)

The following items from this document have been implemented in the codebase:

1. **§0 — Reference curvature cap**  
   In `mpc_lateral.py`, after `build_reference()` returns, we now clip `kappa_ref` to the kinematic limit:  
   `kappa_max = tan(max_steer_angle) / wheel_base` (≈ 0.09734 1/m),  
   so the reference is always feasible for the kinematic bicycle.

2. **§2B — Explicit curvature feedforward**  
   In `mpc_lateral.py`, we compute  
   `delta_ff = arctan(L * kappa_ref[0])`  
   and apply `delta_cmd = clip(delta_ff + delta_fb_mpc, -delta_max, delta_max)`.  
   The MPC optimizes the feedback part; feedforward handles the nominal steering for the curve.

3. **§2C — Steering rate constraint in the QP**  
   In `mpc_lateral.py` we added one constraint per prediction step:  
   `|u_k - delta_k| <= steer_rate_lim * tau`  
   (so `|delta_dot| <= steer_rate_lim` via the first-order actuator model).  
   Uses existing `steer_rate_lim` from config (e.g. 6.98 rad/s).

4. **Segment-selection smoothing**  
   In `mpc_lateral.py` we smooth which waypoint segment is used for lateral control:  
   - **Hysteresis only:** Keep the previous segment when the new “best” segment’s score is within `segment_hysteresis_m` (default 0.4 m) of the previous segment’s score, so we only switch when clearly better.  
   No advance cap (a +1 cap caused segment lag at high speed and the car ran off the road).  
   This reduces steering flips at waypoint boundaries while letting the segment index track the vehicle. Config: `segment_hysteresis_m` in `config.py` / YAML.

5. **Reference blend at segment boundaries (Option A)**  
   In `mpc_lateral.py` we blend the reference heading (ψ_ref) toward the next segment when the vehicle is near the end of the current segment (`u_proj >= segment_blend_u_start`, default 0.7). So ψ_ref = (1−α) ψ_cur + α ψ_next (angle blend via unit vector), with α ramping from 0 to 1 as `u_proj` goes from 0.7 to 1.0. This makes the reference (and thus e_ψ and steering) continuous across waypoints and removes the left–right–left oscillation. Config: `segment_blend_u_start` in `config.py` / YAML.

6. **Smoothness quick fixes (less “beginner” overcorrect)**  
   In `vehicle_mpc.yaml` and `config.py`: increased steering rate and acceleration penalties (`w_du`, `w_ddu`, `w_ddu_high_curv`), lower steering LPF cutoff (2 Hz), and increased longitudinal smoothness (`w_a`, `w_du_lon`, throttle/brake LPF 3.5 Hz). Reduces left–right overcorrect and “full throttle then hard brake” feel.

7. **MPCC-style migration plan**  
   See `src/scenic/domains/racing/mpc/MPCC_MIGRATION_PLAN.md` for a phased plan to evolve the lateral controller toward Model Predictive Contouring Control (contouring + lag + progress cost, path parameterization, input rate penalties).

**Not implemented (optional / already present):**  
- **§1** — Speed vs curvature: already enforced in the behavior via curvature-based `v_ref` cap.  
- **§2A** — Frenet (e_y, e_psi, δ): already used.  
- **§2D** — Slack variables: deferred.  
- **§3** — Longer or distance-based horizon: can be tried by increasing `mpc_prediction_horizon` in YAML.  
- **§4** — Coupled feasibility: κ cap (done) + speed cap (already in behavior).

---

## 0) First sanity check: is your reference curvature feasible?

With your params:

* Wheelbase (L = 2.9718) m
* Max steer (\delta_{\max}=0.2816) rad

Kinematic curvature limit:

[
\kappa_{\max}=\tan(\delta_{\max})/L \approx 0.09734 , (1/m)
]

If your TTL (or even a short segment) has (|\kappa| > 0.09734), then **your MPC is correct**: it can saturate steering and still not follow.

Earlier, we found **temp-based** paths can exceed this in places. That’s consistent with “another MPC works” if that other controller is *not* strictly kinematic, or it uses different vehicle params / relaxed constraints.

✅ Action:

* Enforce (|\kappa_{\text{ref}}|\le \kappa_{\max}) when generating TTL **or**
* Increase allowable steer angle in your model to match the other system (if your sim/vehicle actually can).

**→ Done:** We clip `kappa_ref` to κ_max in the lateral MPC after building the reference (§ Implemented above).

---

## 1) The real reason you run wide: speed isn’t consistent with curvature

Even if (|\kappa|\le \kappa_{\max}), you still need:

[
a_y = v^2 , |\kappa| \le a_{y,\max}
\Rightarrow v \le \sqrt{a_{y,\max}/|\kappa|}
]

If your longitudinal controller isn’t respecting that, the lateral MPC will “do everything” and still understeer (in the model it’s still bounded by steering).

✅ Action:

* Use the speed profile we generated (`v_profile_mps`) as a **hard cap** or strong soft constraint in your longitudinal planning.
* If you don’t have coupled planning: at minimum apply
  [
  v_{\text{cmd}}(s) = \min(v_{\text{desired}}, \sqrt{a_y/|\kappa_{\text{ref}}(s)|})
  ]

This single change usually stops the “full left and still misses”.

---

## 2) Improve your lateral MPC formulation (kinematic bicycle best practices)

### A) Track in Frenet coordinates, not raw XY (if you aren’t already)

States like:

* (e_y) (lateral error)
* (e_\psi) (heading error)
* (\delta) (steer)

Inputs:

* (\dot{\delta}) (steer rate) or (\Delta\delta)

This reduces geometry issues and stabilizes tuning.

### B) Add feedforward steering from reference curvature

Compute reference curvature (\kappa_{\text{ref}}(s)) and set feedforward:

[
\delta_{\text{ff}} = \arctan(L \kappa_{\text{ref}})
]

Then MPC only tracks the residual:

[
\delta = \delta_{\text{ff}} + \delta_{\text{fb}}
]

This helps a lot for sharp corners because otherwise MPC has to “discover” the steering angle purely from error.

**→ Done:** We add δ_ff = arctan(L·κ_ref[0]) to the MPC output and clip the total command (§ Implemented above).

### C) Constrain steering **rate**, not just steering angle

Real systems and most stable MPCs need:

* (|\delta| \le \delta_{\max})
* (|\dot{\delta}| \le \dot{\delta}_{\max})

Without a steer-rate constraint, MPC can become aggressive/oscillatory; with too tight a rate, it can be physically impossible to reach the needed (\delta) in time.

If you don’t know (\dot{\delta}_{\max}), start with something like **0.4–0.7 rad/s** as a tuning knob.

**→ Done:** We enforce |u_k - delta_k| ≤ steer_rate_lim·tau in the QP (uses config `steer_rate_lim`, e.g. 6.98 rad/s) (§ Implemented above).

### D) Add slack (soft constraints) on lateral bounds / tracking

If you have hard track boundaries, add slack variables so infeasible steps don’t explode the solver:

* Penalize slack heavily, but don’t make it impossible.

---

## 3) Your horizon is time-based; sharp turns prefer distance-based preview

You have 1.75 s horizon.

At different speeds, you get different spatial lookahead:

* at 20 m/s → 35 m preview
* at 30 m/s → 52.5 m preview

For corners like the “sharp left” you mentioned, what matters is **meters of preview**, not seconds.

✅ Improvements:

* Increase horizon to **50–70 steps** if computation allows (2.5–3.5 s), OR
* Keep horizon but reduce speed entering (again: speed profile), OR
* Use **curvature-aware reference selection** (ensure the reference segment includes the upcoming turn early enough).

---

## 4) A key structural improvement: couple lateral feasibility into speed planning

Since your lateral model is kinematic, you can enforce a simple feasibility constraint:

[
|\kappa_{\text{ref}}| \le \kappa_{\max}
\quad \text{and} \quad
v^2|\kappa_{\text{ref}}| \le a_y
]

In practice: generate TTL + speed profile that satisfies both, then MPC becomes easy.

---

## 5) What I recommend you run next (most likely to fix the complaint)

1. ~~Use the TTL that is **centerline-referenced** (our `v2`) **but** ensure curvature is capped to (\kappa_{\max}).~~ **Done:** κ_ref is now capped in the lateral MPC.
2. Use `v_profile_mps` as a speed cap (or reference) in longitudinal control. (Already present in behavior via curvature-based v_ref.)
3. ~~Add (\delta_{\text{ff}}=\arctan(L\kappa)) feedforward.~~ **Done.** 
4. ~~Add steer rate limits~~ **Done** (in QP). Slack: optional / deferred.
