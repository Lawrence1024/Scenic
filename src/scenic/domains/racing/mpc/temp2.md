
### 2) Replace hardcoded `0.05` timing assumptions in `racing/behaviors.scenic`

I found multiple places still hardcoding `0.05` for logs/timestamps (e.g. `t_log = step * 0.05`).

That can absolutely cause the “wrong time base” confusion you mentioned earlier.

### Fix pattern

Use a single runtime-derived control dt everywhere for control-tick logs:

```python
sim = simulation()
ctrl_dt = getattr(sim, 'control_dt', None)
if ctrl_dt is None or ctrl_dt <= 0:
    # fallback: control_period if present, else timestep
    ctrl_dt = getattr(sim, 'control_period', None)
if ctrl_dt is None or ctrl_dt <= 0:
    ctrl_dt = float(getattr(sim, 'timestep', 0.05))
```

Then replace:

* `t_log = step_for_log * 0.05`
* `t_log = self._behavior_step_count * 0.05`
  with:
* `t_log = step_for_log * ctrl_dt`
* `t_log = self._behavior_step_count * ctrl_dt`

### Also set `self.control_dt` explicitly in simulator

In `dspace/simulator.py`, you compute control dt but don’t consistently store it.

Add once after `_control_interval` is known (or in `getRacingControllers()` right after computing `control_dt`):

```python
self.control_dt = self.timestep * max(1, self._control_interval)
```