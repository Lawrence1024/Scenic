# Modulo wrap-around waypoint logic vs segment logic — could it cause the swerve?

## Short answer

**Unlikely to be the direct cause** of the ff/fb opposition and hard swerve at ~67% progress. The lateral MPC does **not** receive the behavior’s waypoint index, so the wrap logic does not directly choose which segment the MPC uses. It can still have indirect effects (curvature lookahead, legacy CTE, progress logging).

---

## 1. How segment choice actually works

- **Behavior** updates `wp_last_idx` with your modulo wrap logic (advance on progress, wrap to 0 at lap end, reset `cumulative_dist_to_wp`).
- **Lateral MPC** is called with:
  - `run_step(vehicle_state, waypoints_for_mpc, None, ...)`  
  i.e. **`current_waypoint_idx = None`** (see `behaviors.scenic` around 1225–1227).
- So the MPC **never** gets `wp_last_idx`. It does **not** use the behavior’s “current waypoint” to pick the segment.
- **Segment choice** is done inside the MPC in `_compute_errors()`:
  - When `current_waypoint_idx is None`, `search_start = 0`.
  - It searches **all** segments `0 .. n_wp-1` (with wrap: segment `i` is `(i, (i+1)%n)`).
  - It picks the segment with **best perpendicular distance** (plus “ahead” bias and hysteresis).
- **Reference** is then built with `reference_segment_idx = mpc_segment_idx` (the segment just chosen by `_compute_errors`), so ff and errors use the **same** segment (single-segment fix).

So: **the modulo wrap-around waypoint logic in the behavior does not directly change which segment the MPC uses.** Segment selection is purely MPC-internal (distance + hysteresis).

---

## 2. Where wrap *could* matter (indirectly)

1. **Curvature lookahead (speed limit)**  
   In `behaviors.scenic`, curvature ahead for speed is computed using `wp_last_idx` and wrap (`lookahead_idx`, `(lookahead_idx+1)%n_wp`, etc.). If the new wrap logic ever makes `wp_last_idx` “jump” (e.g. to 0 too early, or advance several segments in one tick), then:
   - `curvature_ahead_max` could be wrong (e.g. straight instead of curve).
   - That affects speed limit and **deadzone** (e.g. `curv_regime`, `BLOCKED_CURV_HIGH`).  
   So you could get wrong speed or wrong deadzone, which can change how much the car slows and how much CTE is used — but that’s indirect, not a direct ff/fb segment mismatch.

2. **Legacy CTE (mismatch check / speed)**  
   Legacy CTE search starts from `nearest_idx = wp_last_idx` in a window. If `wp_last_idx` is wrong due to wrap, the nearest waypoint in that window could be wrong and legacy CTE could be off. Again, this is for mismatch detection and speed, not for the lateral MPC’s segment.

3. **Failure at 67%**  
   The bad run showed the problem around segment 3515–3528 (~67% of 3591 waypoints), **not** near the lap boundary (3590 → 0). A bug that only appears when wrapping at the lap end is less likely to explain a failure in the middle of the lap — unless the new wrap logic introduced a **general** bug (e.g. advancing `wp_last_idx` too aggressively on some segments), which could then affect curvature lookahead or other uses of `wp_last_idx` even at 67%.

---

## 3. Could wrap + segment logic “create” this error?

- **Segment logic (MPC):** Single-segment consistency (compute errors → use that segment for reference) is in place. So ff and fb refer to the same segment. The opposition we see (large negative `delta_fb` vs positive `delta_ff`) is the QP reacting to large CTE with a correction that opposes the path direction; the same-sign clamp then zeros or limits steer.
- **Wrap logic (behavior):** Does not feed into which segment the MPC uses. So **by itself**, the modulo wrap-around waypoint change does not directly “create” the segment/ff-fb mismatch that caused the swerve.

So: **the wrap change is unlikely to be the root cause of the ff/fb opposition and swerve**, unless it introduced a bug that (a) makes `wp_last_idx` wrong in the middle of the lap, and (b) that wrong value affects curvature lookahead (or similar) enough to change speed/deadzone and push the controller into a bad state. The more direct cause remains: large CTE → QP commands strong opposite delta_fb → same-sign clamp (and its threshold) → under-steer then wrong-way steer when curvature dips below 0.02.

---

## 4. Recommendations

1. **Sanity-check wrap logic**  
   Add a log line when `wp_last_idx` wraps (e.g. when it becomes 0 or when `old_wp_idx` was > 3500 and new is < 100). Confirm that wrap only happens near the start/finish line and that `wp_last_idx` doesn’t jump in the middle of the lap.

2. **Optionally pass `wp_last_idx` into the MPC**  
   Pass `current_waypoint_idx=wp_last_idx` (or the behavior’s current index) into `run_step` so that `_compute_errors` uses `search_start = max(0, current_waypoint_idx - 5)`. That doesn’t override the segment choice (still by distance), but it makes the search order consistent with the behavior’s notion of progress and can avoid rare edge cases.

3. **Revert test**  
   Temporarily revert only the modulo wrap-around waypoint change and re-run the same scenario. If the run succeeds, that suggests the new wrap logic (or its interaction with curvature lookahead / logging) contributes. If it still fails, the cause is likely elsewhere (e.g. same-sign clamp threshold, QP weights, or segment hysteresis).

4. **Tune the safety clamp**  
   As in the previous analysis: lower `curvature_same_sign_clamp_min` (e.g. to 0.01) so the same-sign clamp stays active in shallow curvature and prevents wrong-way steer when curvature drops just below 0.02.
