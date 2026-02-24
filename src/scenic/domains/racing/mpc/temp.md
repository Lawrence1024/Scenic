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