# Priority 3 (high impact): Ensure **all heavy MPC work** only runs on control steps (not just command application)

You already have a gate in `racing/behaviors.scenic`:

```scenic
_run_full_control = (simulation().is_control_step if hasattr(simulation(), 'is_control_step') else True) ...
```

But the key is: **everything expensive must be inside that gate**, including:

* waypoint/speed grading
* speed profile generation
* lateral MPC solve
* longitudinal MPC solve

If any of those still runs outside the gate, you lose most of the 100/20 benefit.

---

## Patch target

### File

`racing/behaviors.scenic`

### Region

Around the block where `_run_full_control` is computed (you showed it around line ~574 in your version).

### What to do

Make sure the control loop looks like this pattern:

```scenic
if _run_full_control:
    # 1) read state (MPC adapter)
    # 2) waypoint/speed grade
    # 3) speed profile
    # 4) lateral MPC
    # 5) longitudinal MPC
    # 6) save final commands to self._last_...
else:
    # Reuse previous commands, do NOT recompute MPC / grading / speed profile
    final_steer = self._last_final_steer
    final_throttle = self._last_final_throttle
    final_brake = self._last_final_brake
```

### Specifically verify these calls are inside the gate

Search for these names and make sure they are only in the `if _run_full_control:` path:

* `read_state_from_controldesk(`
* `compute_speed_profile` / speed profile function
* `lateral_controller.run_step(`
* `longitudinal_controller.run_step(`
* waypoint grading / route speed grading function

If you want, I can help you do a line-by-line audit of this block next.

---

# Priority 4 (medium impact): Cut per-step logging overhead (especially at 100Hz)

At 100Hz, even “every 50 steps” logs become frequent, and string formatting / console I/O matters more than it looks.

---

## A) Add a single verbose flag in `DSpaceSimulation.__init__`

### File

`dspace/simulator.py`

### Function

`DSpaceSimulation.__init__(...)`

Add:

```python
self._perf_verbose = os.environ.get("SCENIC_DSPACE_PERF_VERBOSE", "0").strip().lower() in ("1", "true", "yes")
```

---

## B) Guard noisy prints in `executeActions` and `step`

### File

`dspace/simulator.py`

### Functions

* `executeActions(...)`
* `step(...)`

Examples of prints to guard:

* `[executeActions] t=...`
* `[step] t=... Advancing simulation...`
* `[step] [OK] Step completed`
* light-step repeated per-step logs (if not needed)

Change patterns like:

```python
if self._execute_count % 50 == 1:
    print(...)
```

to:

```python
if self._perf_verbose and self._execute_count % 50 == 1:
    print(...)
```

Same in `step(...)`.

Keep the summary timing prints (`[Timing] steps=... mean(...)`) **enabled** — those are useful.

---

## C) (Optional) Increase timing print interval

In `DSpaceSimulation.__init__`, if you’re running long 100Hz tests:

```python
self._timing_interval = 200
```

instead of `50`.

This reduces console overhead and log size.

---

# Priority 5 (diagnostic + optimization targeting): Split timing into **control-step** vs **non-control-step**

Right now your averages are over **all sim steps**, which hides where the time goes.

At 100Hz/20Hz:

* control steps happen every 5th step
* non-control steps are 4/5 of the run

You want to know separately:

* “control-step cost”
* “non-control-step cost”

This makes optimization obvious and prevents misleading comparisons.

---

## Patch `executeActions(...)` timing accumulation

### File

`dspace/simulator.py`

### Function

`DSpaceSimulation.__init__(...)`

Add two timing accumulators:

```python
self._timing_sums_ctrl = {k: 0.0 for k in self._timing_sums}
self._timing_sums_nonctrl = {k: 0.0 for k in self._timing_sums}
self._timing_n_ctrl = 0
self._timing_n_nonctrl = 0
```

---

### File

`dspace/simulator.py`

### Function

`DSpaceSimulation.executeActions(...)`

Inside the block where you flush `_timing_last` into sums (the one with `_timing_keys`), replace:

```python
for k in self._timing_sums:
    self._timing_sums[k] += _last.get(k, 0.0)
self._timing_n += 1
```

with:

```python
for k in self._timing_sums:
    self._timing_sums[k] += _last.get(k, 0.0)
self._timing_n += 1

# Split by control vs non-control for the *previous* step
prev_step_index = self.currentTime - 1
is_prev_ctrl = (prev_step_index % self._control_interval) == 0 if self._control_interval > 0 else True
target_sums = self._timing_sums_ctrl if is_prev_ctrl else self._timing_sums_nonctrl

for k in target_sums:
    target_sums[k] += _last.get(k, 0.0)

if is_prev_ctrl:
    self._timing_n_ctrl += 1
else:
    self._timing_n_nonctrl += 1
```

---

## Also print split summary in `destroy()`

### File

`dspace/simulator.py`

### Function

`destroy(...)`

Add after the existing final timing summary:

```python
def _print_split(label, sums, n):
    if n <= 0:
        return
    print(f"[Timing] FINAL {label} (steps={n}): "
          f"apply_actions={sums['apply_actions']/n:.4f} "
          f"com_writes={sums['com_writes']/n:.4f} "
          f"step_time={sums['step_time']/n:.4f} "
          f"com_reads={sums['com_reads']/n:.4f} "
          f"loop_other={sums['loop_other']/n:.4f} "
          f"get_properties={sums['get_properties']/n:.4f}")

_print_split("CONTROL", self._timing_sums_ctrl, self._timing_n_ctrl)
_print_split("NONCONTROL", self._timing_sums_nonctrl, self._timing_n_nonctrl)
```

This will immediately tell you:

* whether control decimation is working,
* what the irreducible 100Hz sim-step cost is (`step_time` on non-control steps).

---

## Expected impact (roughly)

From your current 100Hz/20Hz log behavior:

* **Priority 1** (MPC dt fix): mainly **stability/correctness**, likely helps control quality a lot.
* **Priority 2** (remove duplicate readback): likely noticeable reduction in control-step COM read cost.
* **Priority 3** (strict control-step gating): potentially big if any heavy functions still run every sim step.
* **Priority 4** (logging): modest but easy, especially for long runs.
* **Priority 5** (split timing): not a speedup itself, but makes the next optimization decisions obvious.

---

## Important reality check (so expectations are correct)

Even after all 5 patches, **100Hz sim / 20Hz control may still be much slower than 20/20** if:

* `cd.advance_simulation_step()` itself takes ~20–30 ms per call.

Because at 100Hz you are making **5× more step calls** than 20Hz.

That’s why your current visualization shows a huge gap.

So the optimization path is:

1. **Fix control decimation correctness (P1/P3)**
2. **Remove redundant COM readback (P2)**
3. **Measure split timings (P5)**
4. Then decide whether you need a **bigger architectural change** (e.g., asynchronous/free-running simulator, or a lower-overhead stepping API in dSPACE if available)

---

If you want, next I can give you a **very small concrete patch snippet** for **Priority 2 (state cache)** as a ready-to-paste diff (just those two files).
