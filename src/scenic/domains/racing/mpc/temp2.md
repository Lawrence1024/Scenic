## 2) `simulator.py` — add explicit clock debug instrumentation (for visualization mismatch)

This is the patch you asked for earlier (“tiny instrumentation patch”), and it’s still missing in the uploaded `simulator.py`.

### A. Add clock-debug fields in `DSpaceSimulation.__init__`

Find the block where you set `_light_step` (around line ~98–103):

```python
_light = os.environ.get("SCENIC_DSPACE_LIGHT_STEP", "").strip().lower()
self._light_step = getattr(sim, "light_step", False) or _light in ("1", "true", "yes")
if self._light_step:
    self._light_step_times = []  # for per-step step_time logging
```

### Immediately after that, insert:

```python
# Clock-debug instrumentation (helps align log time vs visualization time)
_clk = os.environ.get("SCENIC_DSPACE_CLOCK_DEBUG", "").strip().lower()
self._clock_debug = _clk in ("1", "true", "yes")
try:
    self._clock_debug_interval = int(os.environ.get("SCENIC_DSPACE_CLOCK_DEBUG_INTERVAL", "50"))
except Exception:
    self._clock_debug_interval = 50
self._clock_debug_first_wall = None
self._clock_debug_first_step = None
```

---

### B. Print clock mapping inside `executeActions`

In `executeActions`, after:

```python
self._execute_count += 1
```

insert:

```python
# Optional clock-debug print: map Scenic step index -> sim time -> wall elapsed
if getattr(self, "_clock_debug", False):
    now_wall = time.perf_counter()
    if self._clock_debug_first_wall is None:
        self._clock_debug_first_wall = now_wall
        self._clock_debug_first_step = int(getattr(self, "currentTime", 0))

    if (self._execute_count % max(1, self._clock_debug_interval)) == 1:
        step_idx = int(getattr(self, "currentTime", 0))
        sim_t = step_idx * float(self.timestep)
        wall_elapsed = now_wall - self._clock_debug_first_wall
        step_from_first = step_idx - int(self._clock_debug_first_step or 0)
        sim_elapsed = step_from_first * float(self.timestep)
        ratio = (sim_elapsed / wall_elapsed) if wall_elapsed > 1e-9 else 0.0
        print(
            f"[ClockDebug] exec#{self._execute_count} "
            f"step={step_idx} sim_t={sim_t:.3f}s "
            f"sim_elapsed={sim_elapsed:.3f}s wall_elapsed={wall_elapsed:.3f}s "
            f"sim/wall={ratio:.3f}x control_step={self.is_control_step}"
        )
```

### C. Improve existing `[executeActions]` debug print so it uses `currentTime` explicitly

You currently have:

```python
if self._execute_count % 50 == 1:
    t_log = (self._execute_count - 1) * self.timestep
    print(f"[executeActions] t={t_log:.2f}s #{self._execute_count} Executing actions for {len(self.scene.objects)} objects")
```

### Replace with:

```python
if self._execute_count % 50 == 1:
    step_idx = int(getattr(self, "currentTime", 0))
    sim_t = step_idx * float(self.timestep)
    print(f"[executeActions] step={step_idx} sim_t={sim_t:.2f}s #{self._execute_count} Executing actions for {len(self.scene.objects)} objects")
```
