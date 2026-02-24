
### 4) Stop printing from `setGear()` / `setClutch()` on every call in `dspace/model.scenic`

These methods currently print every invocation:

```python
print(f"[DSPACERacingCar.setGear] Called with gear={gear}")
```

This is surprisingly expensive in Python hot loops and pollutes logs.

### Fix

Gate it behind a debug flag, or just remove.

#### Suggested minimal change

```python
if getattr(self, '_debug_transmission', False):
    print(f"[DSPACERacingCar.setGear] Called with gear={gear}")
```

Same for `setClutch()` and other high-frequency setters if they’re called often.

---

### 5) Make COM timing logging optional (or aggregated) in `dspace/controldesk/connection.py`

Right now every `get_var` / `set_var` appends to `_timing_log`:

```python
self._timing_log.append((path, "get", duration))
```

That adds Python list/tuple allocation in the hottest I/O layer.

### Why this matters

You already know COM is expensive. Per-call instrumentation overhead can become nontrivial too.

### Low-risk improvement

Add a flag like `self._enable_timing_log = True` and only append when profiling.

#### Example

```python
self._enable_timing_log = False   # default for performance runs
```

and in `get_var` / `set_var`:

```python
if self._enable_timing_log:
    self._timing_log.append((path, "get", time.perf_counter() - t0))
```

If you still want summaries in production runs, use **aggregated counters** instead of storing every record.

---

## Next-tier optimization (bigger win, more work)

### 6) Lateral MPC: avoid `solver.setup(...)` every tick (OSQP persistent update)

This is likely your biggest remaining compute cost.

I checked `racing/mpc/mpc_lateral.py` and `mpc_longitudinal.py`:

* both do `self.solver.setup(...)` inside the run path (every step)
* even with `warm_start=True`, repeated setup is expensive

### Best long-term fix

* Setup OSQP **once**
* Keep matrix sparsity pattern fixed
* Use `solver.update(...)` each tick for:

  * `q`
  * bounds (`l`, `u`)
  * and matrix numeric values (`Px`, `Ax`) if needed

This can be a major speedup, but it’s not a “quick patch”.

### Practical in-between option (lower risk)

**Adaptive lateral MPC frequency**:

* On straights / low curvature / low CTE:

  * run lateral MPC every 2nd control tick
  * reuse last steering command between solves
* In turns / high CTE:

  * run every control tick

This is often a strong speed win with minimal driving impact if thresholds are conservative.

---

## COM-read reduction ideas (next after the above)

You flagged this already, and I agree it’s still a big bottleneck.

### 7) Cache or decimate `steer_actual` read in `racing/mpc/io_adapter.py`

`io_adapter.read_state_from_controldesk(...)` still calls `sim._cd.get_var(steer_path)`.

Even one extra COM read per control tick hurts if each read is ~6–7 ms.

#### Low-risk approach

Add a tiny per-step cache in `io_adapter.py`:

* key = `(sim.currentTime, obj_id)`
* store last `steer_actual`
* if same control step, reuse

(If it’s only called once per tick currently, this won’t help much. But it protects future duplicate calls.)

#### Slightly more aggressive approach

Read `steer_actual` every N control ticks (e.g. 2), and reuse cached value on the others.

---

## What I’d run next (minimal changes, best signal)

If you want the cleanest next test iteration, I’d do **just these 4 first**:

1. **Remove fast-path `SetGearAction`**
2. **Replace hardcoded `0.05` timestamps with runtime `control_dt`**
3. **Cache `waypoints_for_mpc` conversion**
4. **Disable/gate `setGear()` prints and COM per-call timing append (for performance run)**

These are all low-risk and should make the next log easier to interpret.

---

## What to verify in the next run log

After these edits, I’d expect:

* **No gear-related spam** except real shifts
* Cleaner log timestamps (`t=...`) matching actual control cadence
* Slightly lower `[LoopOther]` / behavior overhead
* Same driving behavior (or better) with less noise
* COM timing summary still available *if* you keep timing enabled

---

If you want, I can also give you a **patch-style diff** for the first 3 changes (fast-path gear removal, control_dt propagation, and `waypoints_for_mpc` caching) so you can paste them directly.
