# Fix log — MPC/MPCC and steering changes

This document records the changes made in response to the recommendations and questions in the original fix/debugging discussion. It is intended as a single reference for what was changed, where, and why.

---

## 1. Recommendation #1 (do this first): Remove `heading_flipped` logic — do not flip `e_y`

**Rationale:** The >90° reference flip was the single most dangerous part of the stack and matched the observed failure: spin → heading crosses 90° → reference frame flips → controller whipsaws → 180°. The flip condition used `heading`, which becomes unreliable exactly when a spin starts; it introduced a hard discontinuity (`e_y → -e_y`, `psi_ref → psi_ref + π`) in one timestep and produced "steer the opposite direction NOW" impulses that could cause or lock in a spin.

### Code removed

- **File:** `mpc_lateral.py`, `_compute_errors`
- **Removed block:**  
  - Computation of `heading_diff = psi_ref - heading` (wrapped to [-π, π]).  
  - If `abs(heading_diff) > π/2`: set `psi_ref = wrap(psi_ref + π)` and `heading_flipped = True`.  
  - After computing `e_y_raw`, the block that did `if heading_flipped: e_y = -e_y`.
- **Removed from segment-boundary blend:** The same >90° check and 180° flip for `psi_ref_next` when blending toward the next segment (so the next-segment heading is no longer flipped).

### Code kept

- **Reference heading:** `psi_ref = atan2(seg_dy, seg_dx)` only (no conditional flip).
- **Segment blend:** `psi_ref_next = atan2(seg_dy_next, seg_dx_next)` with no flip; blending between current and next segment heading unchanged.
- **Lateral error:** `nx = -seg_dy/seg_len`, `ny = seg_dx/seg_len`, and **`e_y = (px - proj_x)*nx + (py - proj_y)*ny`** with **no** sign flip.
- **Heading error:** `e_psi = wrap(heading - psi_ref)` as before.

### Documentation

- **mpc/README.md:** Key Conventions, Wiring (C) reference/CTE row, and (D) note updated to state that the >90° flip and e_y flip were removed (Recommendation #1) and to remove the "trap" wording that referred to that logic.

---

## 2. Recommendation A (must-do): Fix steering sign at WRITE (actuation boundary)

**Problem:** End-of-log physics showed delta_cmd_rad negative while actual yaw-rate indicated positive curvature — i.e. the actuator expects the opposite sign.

**Change:** Add a single constant in the write path so that the sign of the command sent to the actuator can be reversed.

### Code changed

- **File:** `src/scenic/simulators/dspace/steer_io.py`
- **Added:** `STEER_CMD_SIGN = -1.0` (comment: try -1 first; if car turns right for +delta_rad keep -1, if left use +1).
- **Conversion:** `theta_sw_deg = STEER_CMD_SIGN * delta_road_rad * R * 180.0 / math.pi` (previously no sign factor).
- **Startup log:** `log_startup_once()` now prints `STEER_CMD_SIGN` so the active sign is visible in logs.

### Verification

- **Deterministic test:** Hold `delta_road_rad = +0.02 rad` for 1 s at low speed.  
  - If the car turns **right**, the sign is wrong → keep `STEER_CMD_SIGN = -1.0`.  
  - If it turns **left**, use `STEER_CMD_SIGN = 1.0`.

### Post-run fix (run.log)

- **run.log** showed: MPC commanded **positive** `delta_cmd_rad` (left), but the car turned **right** (negative `kappa_meas`). With `STEER_CMD_SIGN = -1` we were sending negative deg for positive rad, and the actuator produced a right turn.  
- **Fix applied:** Set `STEER_CMD_SIGN = 1.0` in `steer_io.py` so that positive delta_rad is sent as positive steering-wheel deg and the car turns **left** when MPC commands left.

---

## 3. Recommendation B (must-do): When gate rejects, do not keep controlling off the stale segment (recover mode)

**Problem:** When `gate_accept=False` and `match_dist_m` was large (e.g. 16.7 m), the controller still kept `segment_id` equal to the previous segment. That is acceptable for tiny glitches but dangerous when actually off track: it leads to stale reference + saturated steering.

**Change:** Replace "reject → always hold last_seg" with a recover mode: if the gate rejects the candidate **and** the best match distance is above a hard-fail threshold, **force re-association** to the nearest segment instead of keeping the previous one.

### Code changed

- **File:** `mpc_lateral.py`, `_compute_errors`
- **Logic:** After the existing gate logic (reject_candidate True when too_far / backward / s_jump):
  - **New constant:** `hard_fail_dist_m` (from config `gate_hard_fail_dist_m`, default 6.0 m).
  - **New behavior:** If `reject_candidate` and **`best_match_dist > hard_fail_dist_m`**, do **not** set `best_segment_idx = last_seg`; keep `best_segment_idx` as the candidate (force re-association).
  - If `reject_candidate` and `best_match_dist <= hard_fail_dist_m`, behavior unchanged: `best_segment_idx = last_seg`.
- **Config:** `mpc_lateral.py` __init__: `self._gate_hard_fail_dist_m = getattr(config, 'gate_hard_fail_dist_m', 6.0)`.
- **config.py:** `self.gate_hard_fail_dist_m = config_dict.get('gate_hard_fail_dist_m', 6.0)`.
- **vehicle_mpc.yaml:** `gate_hard_fail_dist_m: 6.0` with comment "(m) when gate rejects and match_dist > this, force re-association (Rec B: recover mode)".

So when the vehicle is far off (e.g. >6 m from the best match), the controller re-associates to the nearest segment instead of continuing to control from the stale one.

---

## 4. Recommendation C (important): Apply stickiness only when association is good

**Problem:** The rule "stick to last segment when `abs(prev_e_y) >= stick_m`" was harmful when the car was already diverging: it locked onto a bad segment and prevented reacquisition. When far away, the system should prefer reacquisition over sticking.

**Change:** Allow stickiness only when **both** the gate accepted the current best segment **and** the match distance is below a threshold (association is good).

### Code changed

- **File:** `mpc_lateral.py`, `_compute_errors`
- **New constants:** `stick_dist_ok_m` from config `stick_association_ok_m` (default 2.0 m); `gate_accept = (gate_reason is None)`.
- **Stick condition (updated):**  
  Previously: stick if `prev_e_y` not None and `abs(prev_e_y) >= stick_m` and `last_seg` valid.  
  Now: same **and** `gate_accept` **and** `best_match_dist < stick_dist_ok_m`.
- **Config:** `_stick_association_ok_m` in `mpc_lateral.py` (default 2.0); `stick_association_ok_m` in `config.py` and `vehicle_mpc.yaml` with comment "(m) only stick to segment when match_dist < this (Rec C)".

So we only stick when the gate accepted and the vehicle is within 2 m of the best segment; otherwise we allow a segment switch (reacquisition).

---

## 5. Recommendation D: Make STEER_SIGN_SANITY use yaw-rate curvature

**Problem:** The existing STEER_SIGN_SANITY check used path curvature (`kappa_at_proj`) as "measured" curvature, which could be misleading at the exact moment of failure (e.g. wrong segment or wrong side of track).

**Change:** Use curvature from motion, **kappa_meas = yaw_rate / v**, for the sanity check (same as STEER_CAL kappa). Compare **sign(delta_cmd_rad)** vs **sign(yaw_rate / v)** so the log directly reflects whether the actuation sign is inverted in real time.

### Code changed

- **File:** `mpc_lateral.py`, `run_step`
- **State extraction:** Added `yaw_rate = vehicle_state.get('yaw_rate', None)` (rad/s) where other state is extracted.
- **STEER_SIGN_SANITY block:**  
  - If `speed > 0.15` and `yaw_rate` is not None: **`kappa_meas = yaw_rate / speed`**.  
  - Else: `kappa_meas = kappa_at_proj` (fallback to path curvature).  
  - Sign comparison and mismatch detection unchanged except they now use this `kappa_meas`.  
  - Log message updated to indicate "(yaw_rate/v)" and "actuation sign inversion" when a mismatch is detected.

The behavior already passes `yaw_rate` in `vehicle_state` when available (e.g. from `angularVelocity.z`), so no behavior change was required for that.

---

## 6. README and wiring documentation (from original fix questions)

The original fix discussion asked for a clear "wiring diagram" (signals, signs, frames, units, saturation). The following was added or updated in **mpc/README.md** so those questions are answered in one place:

- **A) Control pipeline:** Table describing MPC output (rad, not normalized), mapping to front wheel angle, rad → ControlDesk (steering wheel deg ±240, STEER_CMD_SIGN), negations (none on write; readback can use STEER_ACTUAL_SIGN), scales/rate/LPF/saturation, and logged names (delta_cmd, steer_write, steer_readback). Description of the I/O write path (SetSteerAction(rad) → simulator → road_rad_to_dspace_value).
- **B) State estimation:** Heading source (ControlDesk, wrap to [-π, π]), yaw rate (actor.angvel.z / vehicle_state['yaw_rate']), speed source, e_psi computation (heading - psi_ref then wrap), frame (world, typically ENU).
- **C) Reference and segment selection:** Best-segment selection, gate (too_far / backward / s_jump) and stick logic; note that reference heading and CTE use no flip (Recommendation #1). kappa_ref sign (spline and linear). CTE (e_y) from waypoint projection, no flip.
- **D) Wheelbase and kinematics:** wheel_base 2.9718 m, delta_ff = atan(L * kappa_ref), max_steer_angle = front wheel (rad). Note that the previous >90° flip and e_y flip were removed (Recommendation #1).

Related Docs in the README now point to **fix.md** as the source of those questions and to this document for a record of what was changed.

---

## Summary table

| Item | File(s) | What changed |
|------|--------|---------------|
| Rec #1: no heading flip | `mpc_lateral.py` | Removed >90° psi_ref flip and e_y sign flip; kept psi_ref = atan2(seg_dy, seg_dx), e_y = projection, no flip in blend. |
| Rec #1: docs | `mpc/README.md` | Conventions and wiring (C)(D) updated; trap note removed. |
| Rec A: steer sign at write | `steer_io.py` | STEER_CMD_SIGN = -1.0; theta_sw_deg = STEER_CMD_SIGN * delta_road_rad * R * 180/π; startup log. |
| Rec B: recover mode | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | When gate rejects and best_match_dist > gate_hard_fail_dist_m (6 m), keep best_segment_idx (force re-assoc). New config gate_hard_fail_dist_m. |
| Rec C: stick when association good | `mpc_lateral.py`, `config.py`, `vehicle_mpc.yaml` | Stick only if gate_accept and best_match_dist < stick_association_ok_m (2 m). New config stick_association_ok_m. |
| Rec D: STEER_SIGN_SANITY | `mpc_lateral.py` | kappa_meas = yaw_rate/v when v>0.15 and yaw_rate present; else kappa_at_proj. Log text updated. |
| Wiring answers | `mpc/README.md` | Section "Wiring and debugging" added (A–D tables and note). |
| Single-segment consistency | `mpc_lateral.py`, `reference_builder.py`, `MPCC_IMPROVEMENT_PLAN.md` | run_step: _compute_errors first; build_reference(..., reference_segment_idx=mpc_segment_idx). reference_builder: optional reference_segment_idx; when set, nearest_idx = it. Doc: MPCC_IMPROVEMENT_PLAN.md §E. |

---

## Single-segment consistency (steering wobble at curve exits)

**Problem:** Feedforward (kappa_ref, delta_ff) and feedback (e_y, e_psi, delta_fb) used different segment indices: ref_builder used nearest waypoint by **node distance**, while _compute_errors used best segment by **perpendicular distance** (with gate/hysteresis/stick). At curve exits they could disagree, so delta_ff said "turn left" and delta_fb said "turn right" → steering sign flips (left→right→left) and FF TRIPWIRE logs.

**Fix:** Use one segment per step. In `run_step`, call `_compute_errors` first to get `(e_y, e_psi, mpc_segment_idx)`, then call `ref_builder.build_reference(..., reference_segment_idx=mpc_segment_idx)` so the reference is built from the same segment. In `reference_builder.build_reference`, new optional arg `reference_segment_idx`: when provided, use it as `nearest_idx` instead of `find_nearest_waypoint`. Documented in **MPCC_IMPROVEMENT_PLAN.md** section E.

---

All of the above changes are in the codebase and config as of this document; adjust STEER_CMD_SIGN (Rec A) and the two new MPC config parameters (Rec B, C) as needed for your vehicle and track.
