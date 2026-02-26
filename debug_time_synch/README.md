# Time sync debug: wall-clock trigger vs dSPACE time variables

Minimal "frozen controller" run: **trigger one step every x seconds (wall clock)**, then record zeroed wall time, ManeuverTime, and SimulationTime.

- **No** control logic, **no** MPC, **no** readback
- Connect → set dSPACE SingleStepTime to the trigger interval → start maneuver → pause
- **Trigger loop:** every `--interval` seconds (wall time), call `advance_simulation_step()`, then read and log wall_t_0, ManeuverTime_0, SimTime_0 (all zeroed at step 0)

**ControlDesk variable paths (recorded for reference):**

| Variable         | Path |
|------------------|------|
| **ManeuverTime** | `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/DISP_Plant/ManeuverTime[s]/Out1` |
| **SimulationTime** | `Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime` |

**Prereqs:** ControlDesk running, experiment loaded, platform ready for online calibration.

**Run from Scenic repo root:**

```bash
cd Scenic
python debug_time_synch/run_frozen_controller.py --steps 60 --interval 0.5
```

Output: per-step log of `step | wall_t_0 | ManeuverTime_0 | SimTime_0 | dManeuver`. Compare the three zeroed times for drift.

---

## What the test showed (time discrepancy causes)

1. **Large initial offset**  
   When the frozen loop starts (first `advance_simulation_step()`), dSPACE `ManeuverTime` is already ~0.55s, while Scenic expects `0.01s` (one step). So **ManeuverTime includes time that ran before our step loop** (e.g. after `start_maneuver()` and before we started calling `SingleStep()`). Scenic’s `t` is “time since we started stepping”; dSPACE’s ManeuverTime is “time since maneuver start” (or similar). Different time origins → constant offset in the diff.

2. **ManeuverTime advances in chunks**  
   In the run, ManeuverTime stayed constant for several steps (e.g. 0.551s for steps 0–6, then 0.561s, 0.571s, …). So either the variable is sampled/updated at a lower rate than every step, or **SingleStepTime** in dSPACE is smaller than we think (e.g. 0.001s so that 10 steps ≈ 0.01s). That can look like a rate mismatch or “Scenic steps faster than ManeuverTime.”

3. **How to fix or live with it**  
   - If you need “Scenic t ≈ ManeuverTime”: record the dSPACE time (ManeuverTime or SimulationTime) at the first control step and subtract that offset when comparing or logging.  
   - If you need step-accurate sync: confirm **SingleStepTime** in the experiment matches `timestep` (e.g. 0.05 for 20Hz), and whether ManeuverTime/SimulationTime is driven by the same clock as the stepped simulation.
