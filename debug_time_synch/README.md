# Time sync debug: Scenic t vs dSPACE ManeuverTime

Minimal "frozen controller" run to see what causes time discrepancy:

- **No** control logic, **no** MPC, **no** readback
- Only: connect → start maneuver → pause → repeatedly `advance_simulation_step()`
- After each step: read dSPACE `ManeuverTime[s]` and log vs expected Scenic time `(step_index + 1) * timestep`

**Prereqs:** ControlDesk running, experiment loaded, platform ready for online calibration.

**Run from Scenic repo root:**

```bash
cd Scenic
python debug_time_synch/run_frozen_controller.py --steps 60 --timestep 0.01
```

Output: per-step log of `step | Scenic_t | ManeuverTime | diff` and a short summary of drift.

---

## What the test showed (time discrepancy causes)

1. **Large initial offset**  
   When the frozen loop starts (first `advance_simulation_step()`), dSPACE `ManeuverTime` is already ~0.55s, while Scenic expects `0.01s` (one step). So **ManeuverTime includes time that ran before our step loop** (e.g. after `start_maneuver()` and before we started calling `SingleStep()`). Scenic’s `t` is “time since we started stepping”; dSPACE’s ManeuverTime is “time since maneuver start” (or similar). Different time origins → constant offset in the diff.

2. **ManeuverTime advances in chunks**  
   In the run, ManeuverTime stayed constant for several steps (e.g. 0.551s for steps 0–6, then 0.561s, 0.571s, …). So either the variable is sampled/updated at a lower rate than every step, or **SingleStepTime** in dSPACE is smaller than we think (e.g. 0.001s so that 10 steps ≈ 0.01s). That can look like a rate mismatch or “Scenic steps faster than ManeuverTime.”

3. **How to fix or live with it**  
   - If you need “Scenic t ≈ ManeuverTime”: record ManeuverTime at the first control step and subtract that offset when comparing or logging.  
   - If you need step-accurate sync: confirm **SingleStepTime** in the experiment matches `timestep` (e.g. 0.01), and whether ManeuverTime is driven by the same clock as the stepped simulation.
