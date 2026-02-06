Got it. If **MPC decides the path is infeasible**, an **emergency stop** is a good safety behavior. Right now your logs show multiple bugs that prevent that from working consistently. Here’s a concrete “to-do list” to fix them.

> Note: I can’t re-open older expired uploads, but your recent `run.log` excerpts already show the failure patterns clearly (index resets to 0, throttle/brake mismatch, “brake latched” still throttling, etc.).

---

# Emergency-stop behavior: what it must guarantee

When infeasible is detected, the system must enforce **one invariant**:

✅ **If emergency_stop = True → throttle = 0, brake = BRAKE_MAX, and no other module can override it.**

Everything below is aimed at making that invariant true.

---

# A. Fix the infeasibility detection and make it stable

### A1) Compute vehicle max curvature correctly

You already log things like:

* `kappa_veh_max = tan(delta_max) / L`

Make sure:

* `delta_max` is the **actual physical steering limit used by both MPC and simulator**
* `L` is correct (wheelbase used by the MPC kinematic model)

### A2) Compute reference curvature consistently (avoid spikes from kinks)

Even with 1m waypoints, curvature estimates can spike at kinks. Use one of:

* spline curvature from derivatives (preferred), or
* 3-point circle curvature on **smoothed/resampled** points

### A3) Add hysteresis / debounce for infeasible flag

To avoid “infeasible flips on/off” at 20 Hz:

* Enter infeasible if `max_kappa > 1.05 * kappa_veh_max` for **M consecutive steps** (e.g. M=3)
* Exit infeasible only if `max_kappa < 0.95 * kappa_veh_max` for **N steps** (e.g. N=10)

This prevents emergency stop chattering.

---

# B. Fix progress + waypoint state bugs (these currently corrupt everything)

### B1) Stop “progress resets to 0” forever

Your log showed patterns like:

* `advancing XXXX -> 0 (global_s0=0.00m)`
* then sync back

Fix rules:

* **Never set global progress to 0** except at initialization.
* If projection fails, **keep last valid global_s0**.

### B2) Make “global_s0” truly global (not window-local)

If you project onto a window spline, `s0` is local. You must compute:

* `global_s0 = cum_s[start_idx] + s0_local`

And `cum_s[]` must be precomputed along the full track once.

### B3) Derive waypoint index from global_s0 (don’t do idx += 1)

Use bisect:

* `idx = bisect_right(cum_s, global_s0) - 1`

This prevents “index stuck” and prevents random jumps.

### B4) Ensure *one source of truth* for progress/index

Right now you have:

* behavior index
* MPC segment index
* ref_builder progress

Pick one and sync all others from it (recommended: **global_s0**).
If you keep `[Waypoint Sync]`, it should be a one-way sync from global_s0-derived index, not from “whatever MPC chose”.

---

# C. Fix longitudinal control conflicts (this is why “stop” doesn’t actually stop)

### C1) Make throttle/brake mutually exclusive and controlled in ONE place

Your logs showed:

* “mode=RECOVERY_BRAKE_LATCHED” but applied throttle was still high
* LongControl printed throttle≠Final controls

Fix:

* Have **exactly one function** assemble final `throttle, brake`.
* Everything else outputs a desired **acceleration** or a speed reference, not raw throttle/brake.

### C2) Emergency stop must override at the very last layer

Right before `setThrottle()`/`setBraking()`:

```python
if emergency_stop:
    throttle = 0.0
    brake = BRAKE_MAX
```

And **do not** modify throttle/brake after this.

### C3) Verify logs match applied commands

Add one log line that prints:

* computed final controls
* and immediately after, the values passed into the simulator

Success criterion:

* When emergency_stop triggers, you always see:

  * `Final controls: throttle=0.000 brake=BRAKE_MAX`
  * `setThrottle(0.0)`
  * `setBraking(BRAKE_MAX)`

### C4) Remove/disable any code that “adds throttle back”

Common culprits:

* traction / anti-stall “minimum throttle”
* speed controller that always outputs some throttle
* smoothing filter that blends previous throttle
* “throttle floor” logic

In emergency stop mode, all of these must be bypassed.

---

# D. Fix any remaining “waypoint increment” logic that can desync state

### D1) Disable radius-based waypoint increment when using splines/progress

Your log still showed `within_radius: advancing ...` which can cause skipping.
When `use_splines=True`, disable it entirely.

### D2) Make nearest_idx selection consistent

If your spline window is anchored on `nearest_idx`, ensure nearest_idx itself is computed from the **same reference** each step (same waypoint list, same coordinate frame).

---

# E. Add the emergency-stop trigger where it matters

### E1) Trigger based on feasibility margin

Recommended trigger:

* `if max_kappa > kappa_veh_max * (1 + margin)` where margin 5–10%

### E2) Optional: also trigger if “saturated steering + error diverging”

This covers cases where curvature estimates miss a corner:

* If `abs(steer) > 0.95` and `abs(cte_mpc)` increases for K steps (e.g. 5), then emergency stop.

---

# Minimal implementation order (fastest path to “it works”)

1. **Implement emergency_stop override at final actuator layer** (C2).
2. **Fix throttle/brake pipeline so it can’t be overwritten** (C1, C4).
3. **Fix global_s0 + index reset-to-0 bug** (B1–B3).
4. **Add feasibility hysteresis + trigger** (A3, E1).
5. **Disable radius-based increments when splines on** (D1).

That sequence will give you a reliable emergency stop even before you perfect the racing behavior.

---

If you paste the last ~40 lines of the control loop where you compute `Final controls` and call `setThrottle/setBraking`, I can tell you exactly where to insert the emergency-stop override and which lines to remove to prevent later overwrites.
