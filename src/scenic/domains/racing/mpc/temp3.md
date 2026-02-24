## 3) `controller.py` — patch debug print so it shows both control-time and sim-time

You still have a hardcoded:

```python
t_log = obj._ego_control_count * 0.05
```

That’s not ideal for debugging.

Find this block (around line ~111):

```python
if obj._ego_control_count % 50 == 0:
    t_log = obj._ego_control_count * 0.05
```

### Replace with:

```python
if obj._ego_control_count % 50 == 0:
    # Control-time (based on actual configured control period)
    ctrl_period = getattr(self.simulation, "control_period", None)
    if ctrl_period is None or float(ctrl_period) <= 0:
        ctrl_period = float(getattr(self.simulation, "timestep", 0.0))
    else:
        ctrl_period = float(ctrl_period)
    t_ctrl = obj._ego_control_count * ctrl_period

    # Sim-time from simulation step index (for visualization alignment)
    sim_step_idx = int(getattr(self.simulation, "currentTime", 0))
    t_sim = sim_step_idx * float(getattr(self.simulation, "timestep", 0.0))
```

Then update the print line from:

```python
print(f"[EgoControl] t={t_log:.2f}s #{obj._ego_control_count} Writing: ...")
```

to:

```python
print(f"[EgoControl] t_ctrl={t_ctrl:.2f}s sim_t={t_sim:.2f}s step={sim_step_idx} #{obj._ego_control_count} Writing: throttle={throttle_scenic:.3f}->{throttle_scenic*100:.1f}, brake={brake_scenic:.3f}->{brake_scenic*100:.1f}, steer_rad={_delta_rad:.4f}->{steer_deg:.1f}deg")
```