# 5) How to enable the clock debug for the next run

## Git Bash

```bash
export SCENIC_DSPACE_CLOCK_DEBUG=1
export SCENIC_DSPACE_CLOCK_DEBUG_INTERVAL=50
```

## PowerShell

```powershell
$env:SCENIC_DSPACE_CLOCK_DEBUG="1"
$env:SCENIC_DSPACE_CLOCK_DEBUG_INTERVAL="50"
```

(Then run your normal Scenic command in the same terminal session.)

---

## What I’d like to see in your next log

After these patches, the log should include:

* `[ClockDebug] ... sim_t=... wall_elapsed=... sim/wall=...`
* `[executeActions] step=... sim_t=...`
* `[EgoControl] t_ctrl=... sim_t=... step=...`
* `[LoopOther] ... state_unpack=... path_progress=... speed_profile=... mpc_total=... waypoint_speed_grade=... cmd_post=...`

That will let us finally answer:

* whether 100Hz is slow because of **extra loop overhead**, **COM step contention**, or **visualization throttling**
* and whether the time mismatch is just **log timestamp semantics** vs actual sim timeline.

If you want, after your next run I can parse the new log and give you a **clean latency budget** (per 50-step window + % contribution by bucket).
