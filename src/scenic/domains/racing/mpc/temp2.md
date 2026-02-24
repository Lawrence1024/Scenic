### 2) Dedup clutch one-shot writes too (same pattern as gear)

You already fixed gear one-shots well. Clutch one-shots still do a direct write:

* `vehicle/controller.py` → one-shot action handling
* `clutch` uses `self.cd.set_var(...)` directly, no dedup wrapper

This is not a blocker, but if a behavior queues clutch repeatedly, you could get unnecessary writes.

### Suggested change

Replace clutch one-shot write with `_maybe_write_cd(...)`:

```python
elif action_type == "clutch":
    clutch_pct = float(value * 100.0)
    if self._maybe_write_cd(self.KEY_CLUTCH, clutch_pct, 1e-6):
        print(f"[EgoControl] Setting clutch to {clutch_pct}%")
```