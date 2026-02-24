#### 0A) Add a startup “time-base sanity” check and fail fast

For the first N step calls (e.g., 50):

* read dSPACE time **before** and **after** each `advance_simulation_step()`
* compute `delta_dspace`
* compare against expected `timestep`

If `delta_dspace` is not close to expected (or mostly zeros), print a warning / raise in debug mode.

#### 0B) Log both clocks explicitly (no ambiguous `t=`)

Rename labels:

* `scenic_sim_t`
* `ctrl_sched_t`
* `dspace_maneuver_t`

#### 0C) Don’t use Scenic `t += timestep` as “truth” yet

For debug/validation runs, treat dSPACE time as source-of-truth until step semantics are fixed.

#### 0D) Investigate `set_simulation_step(...)` semantics in the ControlDesk wrapper

Given your frozen tests, the likely issue is:

* step API advances a different internal tick than you assume, or
* configured timestep isn’t actually applied.