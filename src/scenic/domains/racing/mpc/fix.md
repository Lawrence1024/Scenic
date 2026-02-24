## Priority 1 — Remove or decimate `ManeuverTime` read in performance runs (easy win)

This is a surprisingly expensive debug read:

* `ManeuverTime[s]/Out1`: **2.300s total**
* **800 calls**
* **2.87 ms/call**

That’s a big tax for a debug clock.

### Concrete recommendation

Add a flag like:

* `debug_read_maneuver_time = False` (default for perf runs)

Or sample it every N control ticks:

* every 10th or 20th control tick

### Expected gain

If disabled entirely during perf runs:

* save ~**2.3 s total wall**
* ~**0.58 ms per sim step**
* ~**2.87 ms per control tick**

That’s a meaningful free win.

---

## Priority 2 — Optimize `<ego_state_6>` MAPort read path (still the biggest I/O bottleneck)

This remains the largest hot I/O cost:

* **11.429s total**
* **14.25 ms per control tick**

### 2A) Add a dedicated `get_ego_state6_fast()` method in MAPort wrapper

Instead of generic `get_vars(paths)`:

* use pre-cached refs
* fixed tuple return
* avoid per-call path tuple iteration if possible
* avoid generic conversion branches

This reduces Python overhead in the hottest read path.

### 2B) Instrument `get_ego_state6_fast()` internally

Break down:

* ref fetch time
* MAPort `Read2` call time(s)
* conversion time
* tuple packing time

You need to know whether the 14.25 ms is:

* API call overhead
* conversion overhead
* Python overhead

### 2C) Try true batch read (if XIL MAPort API supports it)

If there’s a bulk read API:

* one .NET call for all 6 signals
* convert in Python once

This is likely the best long-term MAPort read optimization.

---

## Priority 3 — MPC is now the dominant compute cost (optimize after time-sync + easy I/O wins)

`mpc_total ≈ 31.4 ms / control tick` is the biggest compute bucket.

### 3A) Add MPC internal breakdown (must do before big changes)

Break `mpc_total` into:

* path/reference prep
* matrix build / linearization
* solver call
* postprocess

Without this, you risk optimizing the wrong thing.

### 3B) Warm-start the MPC solver

If your solver supports it:

* reuse previous solution / state
* huge win on sequential control problems

### 3C) Adaptive MPC frequency (after correctness is fixed)

If safe:

* full solve every 2nd control tick on straight segments / stable state
* reuse prior steering on skipped tick

Since you already classify path segments and log straight/curve, you have the signal to gate this.

---

## Priority 4 — Reduce steering write churn (safe-ish, good ROI)

Steering is now the dominant write path:

* `Const_steering_cmd/Value`: **618 writes**
* **1.460s total**
* **2.36 ms/write**

### 4A) Steering quantization before dedup

Try a small quantization (e.g., `0.05°` or `0.1°`) before `_maybe_write_cd(...)`.

This typically cuts jitter writes without hurting behavior.

### 4B) Slightly increase steering dedup epsilon

If your current steering epsilon is tiny, bump it a little (steering-only).

Re-test tracking quality after this.

---

## Priority 5 — `waypoint_speed_grade` is creeping up late-run (watch it)

At `steps=800`, it’s:

* **3.1 ms / control tick**

It’s not the top issue, but it’s growing.

### What to do

Add sub-buckets for:

* waypoint cache lookup
* speed target calc
* grade calc
* interpolation/allocation

You already have excellent cache hit stats; this likely means computation/allocation cost, not cache misses.

---

## Priority 6 — Reduce log spam in performance runs (small but real wall-time help)

Your log is huge and still contains a lot of repeated debug lines:

* `[Control] ManeuverTime[s]=...`
* `[MPC] STEER_SIGN_SANITY ...`
* frequent behavior progress/sanity prints

Console I/O (especially on Windows) can materially affect wall time.

### Recommendation

Use a `debug_level` or flags:

* **perf mode**: summary + periodic timing only
* **debug mode**: detailed sanity lines

At minimum:

* throttle `[MPC] STEER_SIGN_SANITY` to anomaly-only or every N ticks.

---

# Suggested next-step plan (best order)

## Path A (recommended): correctness + quick wins first

1. **Fix / diagnose step-time sync semantics** (highest correctness priority)
2. **Disable/decimate `ManeuverTime` debug read** in perf runs
3. **Optimize `<ego_state_6>` fast path** (`get_ego_state6_fast`)
4. Re-run benchmark
5. Then optimize MPC

## Path B (if you want speed numbers ASAP despite sync bug)

1. Disable `ManeuverTime` read
2. Optimize `<ego_state_6>`
3. Steering quantization
4. Re-run
5. Return to step-sync debugging separately

---

# Bottom line

### What’s good

* MAPort migration is working ✅
* COM hot-path overhead is gone ✅
* Gear spam fix holds ✅
* Dedup still working ✅

### What’s now blocking you

* **Correctness:** Scenic↔dSPACE time-base mismatch (still huge)
* **Performance:** `MPC (~31 ms/tick)` + `<ego_state_6> (~14 ms/tick)` + `ManeuverTime debug read (~2.9 ms/tick)`

---

If you want, next I can give you a **concrete patch checklist** for:

1. gating/removing `ManeuverTime` read in perf runs, and
2. adding `get_ego_state6_fast()` in `maport/connection.py` with minimal changes to `readback.py`.
