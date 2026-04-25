# CoSim integration findings

Empirical notes from the Scenic ↔ VEOS CoSim investigation. Contents here are
pragmatic lessons about variable access, data types, and engine behaviour that
are not obvious from the dSPACE SDK docs — and that future integrators will
hit if they're not forewarned. Architecture lives in
[`veos_cosim_ipc_bridge/README.md`](veos_cosim_ipc_bridge/README.md).

---

## 1. Variable access backends — which works for what

Three paths to dSPACE runtime variables are relevant:

| Backend | Reads | Writes | Notes |
|---|---|---|---|
| **ControlDesk COM** (`ControlDeskApp.get_var/set_var`) | Works only for variables present in the ControlDesk variable collection (Platform_2's ActiveVariableDescription) | Same subset, mostly parameters | Many ASM_Traffic internals are **not** in the collection. Reads on measurement signals (`.../Out1`, `GPS_CALC/...`) typically fail with `"The device must be in online mode"` even when go-online succeeded. |
| **MAPort** (XIL API via pythonnet) | All variables reachable through the XIL model | All variables where model permits writes | The **reliable** backend under CoSim. Scenic's `_var_access` prefers MAPort when available. |
| **CoSim bus** (`VeosCoSim_IoReadMI`/`_IoWriteMI`) | Inports (VEOS → client) | Outports (client → VEOS) | The only path VEOS actually reads for command flow under CoSim (see §3). **External tools cannot write bus outports** — only the C++ CoSim client can. |

### Evidence

- ControlDesk COM reads of `Pos_x_Vehicle_CoorSys_E[m]/Out1`, `GPS_CALC/*`, `SimulationTime`
  all fail; MAPort reads of the same paths all succeed.
- Scanning the ControlDesk Variables collection (33,861 entries) finds none of
  `Pos_x_Vehicle_CoorSys_E`, `v_x_Vehicle_CoG`, `GPS_CALC`, or `SimulationTime` as
  substrings — they're literally not in the platform's tree.
- External MAPort/COM attempts to write `{_COSIM_OUT}/throttle_cmd` fail with
  `"Could not write variable"` across repeated runs.

### Implication for Scenic

Scenic's readback layer should route through MAPort (already the default in
`DSpaceSimulation._var_access`). Do **not** rely on ControlDesk COM for reads of
measurement signals — they'll silently fail.

---

## 2. Data type pitfalls with MAPort

MAPort's `set_var(path, value)` routes the Python type to an XIL type:

- Python `int` → `eUINT` (via `CreateUintValue`)
- Python `float` → `eFLOAT` (via `CreateFloatValue`)

If the underlying variable's declared type doesn't match, you get one of:

```
DataType missmatch. Got: eFLOAT, Expected: eUINT.
DataType missmatch. Got: eUINT, Expected: eFLOAT.
```

and the write is silently rejected. **These errors are the #1 cause of "my write
latched but nothing happened" in this session.**

Empirically observed types (from connection.py's `initialize_vesi_interface`
and direct probes):

| Path | Expected Python type |
|---|---|
| `VESIResultData_Manual/.../Const_enable_*_cmd/Value` | `int` (UINT) |
| `VESIResultData_Manual/.../Const_throttle_cmd/Value` | `float` |
| `VESIResultData_Manual/.../Const_brake_cmd_*/Value` | `float` |
| `VESIResultData_Manual/.../Const_steering_cmd/Value` | `float` |
| `VESIResultData_Manual/.../Const_gear_cmd/Value` | `int` |
| `VesiInterface/Sw_Manual_VESI_Overwrite[0|1]/Value` | `float` (despite `[0|1]` name!) |
| `VesiInterface/Sw_Activate_CLIF[0|1]/Value` | `float` (same caveat) |
| `RaceControl/Sw_RaceControl[0Intern|1Extern|2Orchestrator]/Value` | `float` |
| `RaceControl/race_control/Const_sys_state/Value` | `int` (`9` for "running") |
| `RaceControl/race_control/Const_track_flag/Value` | `int` |
| `RaceControl/race_control/Const_veh_flag/Value` | `int` |
| `RaceControl/Parameters/manual_mode` | `float` |
| `RaceControl/Parameters/track_flag_manual` | array of `float` |

**Rule of thumb:** names with `Const_enable_*` / `Const_*_state` / `Const_*_flag` are
UINT; names with `Sw_*` and `Const_*_cmd` (floats like throttle) are float. When in
doubt, try the write; the vendor exception tells you the expected type.

The name suffix `[0|1]` looks like it should mean UINT but doesn't — all three
`Sw_*[0|1]`/`Sw_*[0Intern|1Extern|2Orchestrator]` switches are FLOAT-typed.

---

## 3. Command flow under CoSim

**The VEOS plant under CoSim reads commands from the CoSim bus outports, not from
`VESIResultData_Manual` memory.** This is the single most important architectural
fact to know about Scenic + CoSim integration.

Evidence:
- Writing VESI throttle=1.0 via MAPort (and verifying readback = 1.0) leaves
  `OUT throttle_cmd` on the CoSim bus at 0.0 — the VESI memory isn't mirrored
  onto the bus by any dSPACE-internal wiring.
- The plant responds to whatever is on the bus outports. Since no dSPACE-side
  process writes VESI values onto the bus, the plant gets zeros regardless of
  VESI writes.

### Who writes the outports?

Only the **C++ CoSim client** (`VeosCoSimTestClientIpc.exe`) can write outports
via `VeosCoSim_IoWriteMI`. External MAPort and ControlDesk COM writes to outport
paths (`Platform()://CoSimServerScenic/.../Outports/.../throttle_cmd`) all fail
with `"Could not write variable"`.

**So Scenic must either:**

1. Have the C++ client mirror VESI → outports every tick, **or**
2. Have Scenic send per-tick command values over the IPC bridge and the client
   translates them into `IoWriteMI` calls.

**Current implementation chooses option 2** (see `veos_cosim_ipc_bridge/README.md`
§"Current step handshake"). The STEP reply Scenic sends back to the client carries
a JSON `outports` dict; the client parses, writes each named outport via
`VeosCoSim_IoWriteMI`, then calls `FinishCommandMI`. This keeps Python in control
of command semantics and avoids tangling the client in dSPACE variable paths.

### Signal catalog (from enumeration at connect)

- **Inports (17, `Direction_Read`)**: ego pose/velocity/GPS (scalars, length 1),
  fellow pose/GPS (length-30 arrays), fellow velocity (length-90 array).
- **Outports (13, `Direction_Write`)**:
  - Scalar Float64: `throttle_cmd`, `brake_cmd_front`, `brake_cmd_rear`,
    `steering_cmd_deg`, `gear_cmd`, `Pos_ClutchPedal`, `enable_throttle_cmd`,
    `enable_brake_cmd`, `enable_steering_cmd`, `enable_gear_cmd`
  - Array Float64 (length 30): `v_fellows_external_km_h`,
    `d_fellows_external_m`, `s_fellows_external_m`

The client's current implementation writes scalar Float64 outports only;
array outports (fellow command arrays) are a follow-up.

---

## 4. `initialize_vesi_interface()` writes do NOT persist under CoSim

`controldesk/connection.py::initialize_vesi_interface()` sets ~10 VESI and
race-control variables via ControlDesk COM `set_var`. Under CoSim, **the calls
succeed without exception but the values don't land** — a subsequent state dump
via MAPort shows every variable at 0 regardless of what init wrote.

Example — immediately after `initialize_vesi_interface()` returned:

```
Sw_Activate_CLIF              = 1.0   (init wrote 0.0)
Sw_Manual_VESI_Overwrite      = 0.0   (init wrote 1.0)
Sw_RaceControl                = 1.0   (init wrote 0.0)
Const_sys_state               = 0.0   (init wrote 9)
Const_track_flag              = 0.0   (init wrote 1)
VESI_enable_throttle          = 0.0   (init wrote 1)
VESI_enable_brake             = 0.0   (init wrote 1)
...
```

### Workaround

Re-apply the init via MAPort right before you need the values. MAPort writes do
persist. This duplicates `initialize_vesi_interface`'s logic; a cleaner
long-term fix is to refactor `initialize_vesi_interface` to use `_var_access`
(which prefers MAPort) instead of the ControlDesk app directly.

---

## 5. `SimulationTime` ≠ `ManeuverTime`

VEOS exposes two distinct clocks:

- `Platform()://ASM_Traffic/Simulation and RTOS/Simulation/SimulationTime` —
  ticks whenever VEOS ticks (whenever the CoSim bridge releases a TIME_TRIGGER).
  This advances even when the scenario is dormant.
- `Platform()://ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/DISP_Plant/ManeuverTime[s]/Out1` —
  advances only when the ModelDesk maneuver engine is actively running.

In this session's tests, `SimulationTime` advanced freely (hundreds of
sim-seconds) while `ManeuverTime` stayed at **0.0** the entire time. The
scenario never actually "started" in the ModelDesk sense, even though VEOS was
ticking and our JSON reply was writing outports correctly.

**So `SimulationTime` advancing is necessary but not sufficient.** When
diagnosing "ego isn't moving" under CoSim, check `ManeuverTime` — if it's 0,
the maneuver engine isn't running and no throttle will move the vehicle
regardless of where the command value lands.

---

## 6. The "Start" click mystery — SOLVED: pulse MANEUVER_START via MAPort

The programmatic equivalent of the ModelDesk "Start" button is a **MAPort pulse
on `MANEUVER_START`**:

```python
path = (
    "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
    "UserInterface/PAR_Plant/ManeuverControl/MANEUVER_START/MDLDCtrl_ManeuverStart"
)
mp.set_var(path, 1.0)
time.sleep(0.5)
mp.set_var(path, 0.0)
```

This is the same variable `cd.start_maneuver()` in `connection.py` pulses.
It must be called **after `Sw_Activate_CLIF=2.0` is written and VEOS is stepping
(gate in AUTO mode)**. Without CLIF=2.0 set first, the pulse has no effect.

### Required ordering (CLIF before pulse)

```
1. VEOS IPC client connects, gate enters AUTO mode (TIME_TRIGGERs flowing freely)
2. Read + capture Sw_Activate_CLIF baseline (typically 0.0)
3. Write Sw_Activate_CLIF = 2.0 via MAPort (COM write doesn't persist; see §4)
4. Pulse MANEUVER_START (1.0 → sleep 0.5s → 0.0) via MAPort
5. ManeuverTime begins advancing (~0.013s first read)
6. … run steps normally …
7. At teardown: write Sw_Activate_CLIF back to captured baseline (0.0)
   so the next run sees a clean 0 → 2 transition
```

Step 3 must precede step 4. Reversing the order leaves ManeuverTime at 0.0.

### Scenic integration

`simulator.py` step 9 now implements the above sequence automatically when
`launch_veos_ipc_client=True`. It stores the baseline in `self._clif_original`
and restores it in `destroy()`. No manual ModelDesk interaction is required.

### Confirmed step fidelity

After the MANEUVER_START pulse:
- ManeuverTime started at 0.013s
- After 500 manual gate steps: ManeuverTime = 5.021s
- Delta = 5.008s ≈ 500 × 0.01s (10ms) — **step size confirmed at 10ms, <0.2% error**

### What does NOT work for starting ManeuverTime

- `cd.start_simulation()` (RTA.Start) — no effect
- `exp.ManeuverControl.Start(0|1|True|False)` — no effect
- `exp.ManeuverControl.Reset() + Start(1)` — no effect
- `exp.ActivateTrafficScenario(...)` — no effect
- Writing any of the 15+ probed variables directly

**Do NOT call `exp.Test.Execute()`.** Off-limits; unknown side effects.

---

## 7. CoSim process constraints

Operational constraints that aren't obvious from reading the code:

- **At most one VEOS CoSim client may be connected at a time.** If
  `VeosCoSimTestClientIpc.exe` from a previous run is still alive, the new run
  will fail to connect. `python_listener/print_time_callbacks.py` auto-spawns
  and terminates the client, but crashes can leak the process — kill it
  manually before re-running.
- **VESI manual mode (`Sw_Manual_VESI_Overwrite=1`) and CoSim command flow
  compete.** Under pure CoSim, the plant should read outports only; the exact
  plant configuration to make this true has not been fully validated in this
  session (see §6).

---

## 8. Pitfalls / things to check when debugging "nothing is happening"

In order of frequency:

1. **The C++ client wasn't rebuilt** after a `.cpp` change. Symptoms: your new
   log lines don't appear, old behavior persists. Re-run `build_client.bat`
   from the x64 Native Tools prompt and verify the `.exe` timestamp updated.
2. **Wrong Python type for MAPort write** (UINT vs FLOAT). Check §2.
3. **You're probing a variable not in ControlDesk's collection.** Use MAPort
   directly; don't trust ControlDesk COM reads for measurement signals.
4. **State dump shows zeros where `initialize_vesi_interface` should have
   written non-zero.** That's §4 — init didn't persist, re-apply via MAPort.
5. **`ManeuverTime = 0.0`.** The scenario isn't running; sim_time alone is
   not enough (§5, §6).
6. **Outport is 0.0 on the bus.** Either Scenic's JSON reply doesn't include
   the outport, or the C++ client's outport map doesn't have the name (typo?
   name not in the `SIGNAL_ENUM_*` catalog?).

---

## 9. Gating (non-CoSim compatibility)

All CoSim-specific behaviour in `simulator.py` / `controldesk/session.py` /
`controldesk/connection.py` / `modeldesk/authoring.py` is gated on the
`launch_veos_ipc_client` Scenic parameter (default `True`). When `False`:

- No IPC bridge, no C++ client spawn, no JSON-envelope replies.
- `step()` uses the original ControlDesk `advance_simulation_step()` + poll
  path; no sync bridge branch is entered.
- `cd_session.pause()` in setup and `cd.start_simulation()` in destroy both
  run (original behaviour).
- `author_scenario()` downloads without the 30-second pauses added for CoSim
  (pauses only fire when `launch_veos_ipc_client=True`).
- Maneuver pulses go through `ControlDeskApp.set_var` by default (no
  `var_access` override); Scenic can opt into MAPort routing by passing
  `var_access=self._var_access`.

Running existing F-bank scenarios without the flag behaves exactly as it did
pre-integration.

---

## 10. Dennis's 2026-04 VEOS — new switches and VESI init fix

Dennis's updated VEOS adds (or surfaces) two new enum-suffixed switches and
renames the legacy `Sw_Manual_VESI_Overwrite[0|1]` path. The old path throws
"Index was outside the bounds of the array" on ControlDesk COM write — the
square-bracket legacy suffix was parsed as an array indexer on a scalar, and
that in turn aborted `initialize_vesi_interface()` via its outer try/except,
leaving all 14 downstream enable/zero writes unexecuted. Symptom: **ego
doesn't respond to MAPort Const_throttle_cmd despite writes succeeding at
the MAPort layer** — because the enable flags were never set.

**Fix (2026-04-23, confirmed working):**

- `controldesk/connection.py::initialize_vesi_interface` — switched to
  per-step try/except with `[VESI-INIT-FAIL]` diagnostic prints so path
  failures are localized instead of aborting the whole init.
- Updated the `Sw_Manual_VESI_Overwrite` path to the new
  `[0bridge|1extern|2scenic]/Value` suffix.
- `simulator.py` setup writes three switches post-init:
    * `Sw_Activate_CLIF[0|1]/Value`  → `2.0`
    * `Sw_MultiEgo_Fellows[0const|1race|2extern]/Sw_MultiEgo_Fellows` → `0.0` (const / MAPort)
    * `Sw_Manual_VESI_Overwrite[0bridge|1extern|2scenic]/Value` → `1.0` (extern / MAPort)
  All three captured at setup and restored at teardown.

### Why mode `0 const` for fellow

The reverted controller writes fellow velocity via MAPort
`Const_v_Fellows_External[km|h]/Value`. Mode `0 const` selects exactly that
backing path. Mode `2 extern` routes fellow to the CoSim bus
(`v_fellows_external_km_h[0]`), which requires the `stage_outports()` method
on `SyncStepBridge` — not present in the reverted code. Use 0 until CoSim-bus
fellow writes are restored.

### Why mode `1 extern` for ego (and the caveat)

Mode `1 extern` on Dennis's VEOS appears to consume the same MAPort
`Const_throttle_cmd` / `Const_steering_cmd` paths the controller writes —
**but only if `initialize_vesi_interface()` completed successfully** (all
enable flags set). Before the per-step try/except fix, init was silently
aborting at step 2 and the enable flags stayed at 0 → MAPort writes were
received but ignored downstream.

Mode `2 scenic` would route ego to CoSim-bus outports (`throttle_cmd`,
`brake_cmd_front/rear`, `steering_cmd_deg`, `gear_cmd` + `enable_*_cmd`),
confirmed via Port Topology inspection. This requires `stage_outports()` on
the bridge and per-tick CoSim ego writes in the controller — not currently
wired but is Dennis's architecturally preferred path.

### Docker stack requirement — CLIF keepalive

`ExternalControl` VPU's CLIF client must remain connected past the
`asmmaneuverstart` one-shot, or ManeuverTime freezes after 3s. In the
minimal Scenic stack (`dspace_scenic_stack.yml`), this is held by the
`asm_socketcan_bridge` sidecar which connects on VEOS-healthy and runs its
ROS 2 bridge-node permanently. Without it, VEOS halts before warmup
regardless of switch settings.

Requires virtual-CAN support in the WSL2 kernel (vcan module); Microsoft's
default WSL kernel doesn't ship it — rebuild from
`linux-msft-wsl-<version>` with `CONFIG_CAN_VCAN=y` and point
`.wslconfig` at the output `bzImage`.

### `DS_ROUTE_ID` env var must NOT be set when driving with Scenic

On Dennis's 2026-04 VEOS, if `DS_ROUTE_ID` is set as an env var on the
veos service in the compose yml, VEOS uses it to initialize ego pose at
every `MANEUVER_START` — *overriding* whatever `ts.Download()` wrote to
the ModelDesk scenario. Symptom: ego teleports from wherever Scenic
computed (e.g. `(-78.86, -112.41)` for R2 Lap s=439.236m) to the
route-baked hardcoded pose the moment MANEUVER_START fires (e.g.
`(-87.20, -148.26)` for `DS_ROUTE_ID=1` pit lane). The ModelDesk
sequence writes (StartPosition, Route.Activate, AdditionalLateralOffset,
VehicleOrientation) all land correctly and survive Download + Reset —
verified by reading back `seq.StartPosition` / `Route.ActiveElement` at
A=post-`place_ego`, B=post-`ts.Download()`, C=post-`ManeuverControl.Reset()`
— but ASM_Traffic's ego init on MANEUVER_START ignores them when the env
var is present.

Fix: leave `DS_ROUTE_ID` commented out in `dspace_scenic_stack.yml`. With
no env override, Scenic's `place_ego()` writes to the ModelDesk sequence
and `ts.Download()` becomes the authoritative source of the initial ego
pose on MANEUVER_START. Confirmed 2026-04-23: ego spawns at
`(-79.68, -111.85)` — within 1 m of the requested `(-78.86, -112.41)`.

### `Sw_Manual_VESI_Overwrite` is mode-dependent — Scenic-ego vs ART-ego

`Sw_Manual_VESI_Overwrite` routes ego VESI inputs to one of three sources.
Two switch-path suffixes exist depending on which VEOS OSA is loaded:

| Suffix | Where | Allowed values |
|---|---|---|
| `[0bridge\|1extern\|2scenic]` | Dennis's 2026-04 CoSim VEOS only | `0` bridge, `1` extern, `2` scenic CoSim bus (last not yet wired) |
| `[0\|1]` (legacy) | Pre-Dennis VEOS (`/home/dspace/VEOS/ASM_Traffic.osa`) | `0` bridge, `1` extern (manual MAPort) |

Only one of the two paths exists on a given OSA. `simulator.py::setup()`
step 9b probes the new path first; if it TRC-misses (old VEOS), it falls
back to the legacy `[0|1]` path. The path that actually resolves is recorded
in `self._vesi_overwrite_path_used` so teardown restores via the same path.

The target value is selected from `scene.params["scenic_control"]`
(default `True`):

```python
_scenic_drives_ego = bool(params.get("scenic_control", True))
_vesi_overwrite_target = 1.0 if _scenic_drives_ego else 0.0
# 1.0 = extern/manual (Scenic MAPort drives ego)
# 0.0 = bridge       (raptor / asm_socketcan_bridge drives ego)
```

For ART-driven ego (`param scenic_control = False`,
`ARTStackControlBehavior`), the switch goes to `0.0` so the plant sees
raptor's CAN commands via the SocketCAN bridge. Fellow path is independent:
`Sw_MultiEgo_Fellows = 0.0` (const) regardless, since Scenic always writes
fellow MAPort arrays. Requires the full `dspace_art_stack.yml` stack up
(raptor running).

Important: COM cannot write paths whose suffix uses enum names (the
`[0bridge|1extern|2scenic]` form on the new VEOS) — COM parses the brackets
as an array indexer and chokes on the non-integer enum names. Both paths are
written via MAPort instead. `initialize_vesi_interface()` in
`controldesk/connection.py` deliberately omits the
`Sw_Manual_VESI_Overwrite` write because COM is its only access. Step 9b in
`simulator.py` is the single source of truth for this switch.

**2026-04-23 attempt status:**

- ✅ **Scenic-ego + Scenic-fellow** works on Dennis's 2026-04 CoSim VEOS.
  Confirmed by `run.log` with `[Setup] [CoSim] Set Sw_Manual_VESI_Overwrite=1.0
  (extern / Scenic-driven ego)` and ego spawning at (-79.68, -111.85) post
  MANEUVER_START.
- ❌ **ART-ego + Scenic-fellow** attempted with the Option B conditional switch
  in place — ego did not move. Root cause not yet identified; no log captured
  of the failure attempt.

The reverting-to-pre-CoSim workaround (`param launch_veos_ipc_client = False`)
is **no longer available**: Dennis's 2026-04 CoSim VEOS (the only build now
deployed) requires the new switch-writes pipeline regardless of whether the
IPC client is used for synchronous stepping. Fixing ART-ego must happen on
the live CoSim path, not by side-stepping it.

**2026-04-24 diagnosis — one root cause found and fixed:**

Hypothesis 3 (ART stack reset didn't land) turned out to be correct, but for a
different reason than "raptor wasn't ready". The `_call_art_stack_reset()`
helper was calling the `set_selected_ttl` service with a type name
(`race_decision_engine/srv/SetSelectedTtl`) that had been REMOVED from the
code base. The current race_common build advertises the service only as
`race_msgs/srv/SetSelectedTtl`. DDS type-hash matching silently failed and the
`ros2 service call` hung until the 15 s subprocess timeout, at which point
Scenic logged a warning and moved on — but raptor's TTL was never set, so the
decision engine stayed on whatever default it booted with and held throttle at
zero.

The `/vehicle_kinematic_state_node/reset_vks_state` service was also gone from
the current build, which explains why the first sub-call in the helper was
also failing silently.

Fixes applied 2026-04-24:
- Dropped the reset_vks_state call entirely (service removed upstream).
- Service type renamed in the set_selected_ttl call:
  `race_decision_engine/srv/SetSelectedTtl` → `race_msgs/srv/SetSelectedTtl`.
- Setup source changed from `/opt/race_common/install/setup.bash` (baked image,
  stale — missing the new race_msgs srv definitions) to
  `/race_common/install/setup.bash` (host-mounted rebuilt workspace, authoritative).
- Matching update in `ros2_bag/config.py::ART_STACK_DEFAULT_SETUP` so the
  bag-recorder sources the same fresh install and can deserialize custom
  race_msgs types.

End-to-end verification (via direct docker exec):
```
docker exec art_driving_stack bash -c "source /race_common/install/setup.bash \
  && ros2 service call /race_decision_engine_node/set_selected_ttl \
     race_msgs/srv/SetSelectedTtl '{selected_ttl: race}'"
# -> success=True, message='Queued TTL change to race'
```

**2026-04-24 — final root cause found and fixed (ART-ego now drives end-to-end):**

After the TTL-service fix above, ART-ego still didn't move. Live in-flight
diagnostics via MAPort (no Scenic running, just the docker stack +
ControlDesk-driven simulation) confirmed:

- raptor was correctly publishing `acc_pedal_cmd: 50.0` on
  `/raptor_dbw_interface/accelerator_cmd` (verified via `ros2 topic echo`).
- The `asm_socketcan_bridge` was healthy: V23 DBC matched, V-ESI connection
  established, "vehicle_inputs message received" / "to_raptor message received"
  logged on every tick.
- `Sw_Manual_VESI_Overwrite=0` (bridge MUX selected ✓), all four
  `Const_enable_*_cmd=0` (manual side off ✓), `Pos_ClutchPedal=100` (released
  ✓), `manual_mode=1` (drive-by-wire enabled ✓), `track_flag_manual[0]=1`
  (green at the RaceControl VPU level ✓).

Plant feedback (`/raptor_dbw_interface/pt_report_1`): `engine_speed_rpm=950`
(idle), `throttle_position=0.0`, `vehicle_speed_kmph=0.0` — VEOS was rejecting
all throttle.

**The deciding probe** was a manual MAPort overlay applied while raptor kept
publishing 50%:

```
Sw_RaceControl   1 -> 0   (extern -> intern)
Const_sys_state  0 -> 9   (off  -> running)
Const_track_flag 0 -> 1   (red  -> green)
Sw_Manual_VESI_Overwrite stays at 0  (bridge mode unchanged)
```

Within one tick, `throttle_position` went 0 → 50%, engine RPM rose from idle,
and the car accelerated 0 → 13.3 km/h over 6 s. Reverting `Sw_RaceControl=1`
immediately dropped throttle back to 0. So:

**The ART-ego throttle gate was the RaceControl state machine, not the bridge
routing.** With `Sw_RaceControl=1` (extern), the plant waits for an external
race-control source to deliver a green flag. Nothing in the dspace_art_stack
actually feeds that extern path with a green flag (raptor reads the flag from
VEOS, not the other way around), so the plant sat in pre-race state and gated
throttle from any source — bridge, manual MAPort, anything. The
`track_flag_manual[0]=1` write at the RaceControl VPU level apparently doesn't
propagate back to the ASM_Traffic plant's `Const_track_flag` when extern mode
is selected.

**Fix applied 2026-04-24** (`controldesk/connection.py::initialize_vesi_interface`):

The race-control values are now mode-INDEPENDENT (always intern + green):
```python
_race_ctrl  = 0.0   # always intern
_sys_state  = 9     # always running
_track_flag = 1     # always green
_veh_flag   = 0
```

Previously these were mode-dependent (0/9/1 for Scenic-ego, 1/0/0 for
ART-ego). The ART-ego defaults were a wrong assumption that "extern mode +
zeros" would let raptor's race-control take over; empirically that's not what
happens. raptor still gets its own race-flag readback from VEOS via the
bridge for its decision-making — that's a separate signal path independent
from how the ASM_Traffic plant gates throttle.

The mode-DEPENDENT values are unchanged:
```python
_enable = 1 if scenic_drives_ego else 0       # MUX selector (Scenic vs bridge)
_clif   = 0.0 if scenic_drives_ego else 1.0   # bridge mode default for ART
_clutch = 0.0 if scenic_drives_ego else 100.0 # ART runs with clutch released
```
…and `Sw_Manual_VESI_Overwrite` stays in `simulator.py` step 9b (1 for
Scenic, 0 for ART), written via MAPort because COM can't write its
`[0bridge|1extern|2scenic]` enum suffix.

**End-to-end greppable confirmations on a working ART-ego run:**
- `[ControlDesk] VesiInterface initialized: ART-driven ego (bridge path on, manual override off).`
- `[Setup] Set Sw_MultiEgo_Fellows=0.0 (const) via MAPort (...)`
- `[Setup] Set Sw_Manual_VESI_Overwrite=0.0 (bridge / ART-driven ego) via MAPort [<suffix>] (...)`
  where `<suffix>` is `[0bridge|1extern|2scenic]` on Dennis's CoSim VEOS or `[0|1]` on the old VEOS.
- `[Setup] ART set_selected_ttl(race|pit) OK.`
- raptor's `/raptor_dbw_interface/accelerator_cmd::acc_pedal_cmd` reflected
  in `/raptor_dbw_interface/pt_report_1::throttle_position` within one tick.

**End-to-end greppable confirmations on a working Scenic-ego run:**
- `[ControlDesk] VesiInterface initialized: Scenic-driven ego (manual override on).`
- `[Setup] Set Sw_Manual_VESI_Overwrite=1.0 (extern / Scenic-driven ego) via MAPort [<suffix>] (...)`
- raptor's `/raptor_dbw_interface/accelerator_cmd::acc_pedal_cmd` *diverges* from the
  plant's `pt_report_1::throttle_position` (raptor's bridge writes are correctly
  bypassed; Scenic's MAPort writes are what's driving). Verified live 2026-04-24
  on the old VEOS: raptor wanted ~38% throttle, plant was at 100% under Scenic MPC.

### Open verification gaps — fellow movement (2026-04-25)

Until 2026-04-25 we focused exclusively on *ego* motion in all four
(scenic_control, launch_veos_ipc_client) configurations. **Scenic-fellow motion
was never explicitly verified** in three of the four — partial observation only.
What we know vs. what we owe:

| Stack / VEOS | Ego mode | Fellow seen moving? | Status |
|---|---|---|---|
| `dspace_art_stack.yml` (old VEOS) | Scenic-ego | not explicitly verified | open backlog |
| `dspace_art_stack.yml` (old VEOS) | ART-ego (`art_fellow_combined.scenic`) | not explicitly verified | open backlog |
| `dspace_art_cosim_stack.yml` (new VEOS) | Scenic-ego (`F1_fellow_behind_optimal_cruise.scenic`) | **yes — observed 2026-04-25** | partially confirmed |
| `dspace_art_cosim_stack.yml` (new VEOS) | ART-ego (`art_fellow_combined.scenic`) | not yet tested | covered in current Phase B plan |

For the old VEOS, fellow code path uses `Const_*_Fellows_External` MAPort arrays
written by Scenic's controller every tick — if it ever stopped working we'd
likely have seen it during MPC runs. But the lack of an explicit
fellow-position-rising-over-time confirmation is a real gap. When we revisit
old-VEOS workflows, the test is to run `art_fellow_combined.scenic` (or any
F-bank scenario with a fellow) on `dspace_art_stack.yml` and tail
`run.log` for `[Fellow s,t]` lines showing the fellow's `s` advancing, OR
read the MAPort `FellowTrailer` x/y over time.

---

## 11. ART-ego on Dennis's CoSim VEOS — BLOCKED on `VESIResultData.h` (2026-04-25)

After Phase A confirmed Scenic-ego works end-to-end on the new CoSim VEOS
(`dspace_art_cosim_stack.yml`, locally-rebuilt bridges, `Cosim-VEOS/ASM_Traffic.osa`),
Phase B attempted ART-ego on the same stack and is currently blocked at the
bridge ↔ VEOS protocol layer. Documented here so the state of investigation isn't
only in conversation memory.

### Symptom

- Scenic setup completes cleanly: VESI init summary `0 failed`,
  `Sw_Manual_VESI_Overwrite=0.0 (bridge / ART-driven ego) via MAPort [0bridge|1extern|2scenic]`,
  `ART set_selected_ttl(race) OK`, ManeuverTime advances, ego placed correctly.
- raptor publishes non-zero `/raptor_dbw_interface/accelerator_cmd::acc_pedal_cmd`
  (e.g. 100%).
- The socketcan bridge correctly receives the matching CAN frame (0x579 with data
  `10 27 06 ...`, where `0x2710 = 10000 = 100%`), parses it via the V23 DBC, sets
  `feedbackCmd.vehicle_inputs.throttle_cmd = 1.0`,
  `enable_throttle_cmd = 1`, and on every tick calls
  `api.sendControlData(22222, &feedbackCmd, sizeof(feedbackCmd))`. No bridge errors.
- VEOS plant `pt_report_1::throttle_position` stays `0.0`, `engine_speed_rpm` stays
  at idle (~950), `vehicle_speed_kmph` stays `0.0`. No VEOS errors.

Same OSA accepts manual MAPort writes via `Sw_Manual_VESI_Overwrite=1` /
`Const_throttle_cmd` — ruling out a broader CoSim handshake or RaceControl
gating issue. Whatever is wrong is specific to the bridge → VEOS UDP path on
port 22222.

### What we tried

- Confirmed all the §10 race-control overrides land (intern + green +
  sys_state=9), so this is not a re-occurrence of the 2026-04-24 RaceControl
  gate issue.
- Verified raptor's CAN encode is correct end-to-end (ROS topic → DBC → frame
  bytes match the spec).
- Rebuilt `airacingtech/iac_asm_socketcan_bridge:latest` against
  `~/dSPACE-IAC-sut-te-bridge/dspace_bridge_ws/src/asm_socketcan_bridge/include/asm_socketcan_bridge/ASMBus.h`
  in both states:
  - **NEW** (`s_Preview_Lat_m[10]` array, the version Dennis sent): bridge
    runs cleanly, throttle still doesn't reach plant.
  - **OLD** (`s_Preview_Lat_m` scalar, git HEAD): bridge crashes on startup
    with `std::length_error: vector::_M_default_append`. Confirms the new
    layout is mandatory and is being loaded.
- Eliminated `race_common` (outside the bridge submodule) as a source of the
  protocol mismatch — no references to VESI/ASMBus/vehicle_inputs anywhere
  outside the bridge source tree.

### Hypothesis (current)

The packet sent by `api.sendControlData(22222, &feedbackCmd, sizeof(feedbackCmd))`
is a raw memory blit of the `VESIResultData` struct. VEOS reinterprets those
bytes against *its own* compiled definition. If the layouts differ by a single
field offset or a padding byte, `throttle_cmd` lands somewhere VEOS reads as 0
and `enable_throttle_cmd` may land on a byte VEOS reads as garbage — exactly
the symptom.

`VESIResultData.h` lives at
`~/dSPACE-IAC-sut-te-bridge/dspace_bridge_ws/src/asm_socketcan_bridge/include/asm_socketcan_bridge/VESIResultData.h`,
is byte-identical between the bridge source and the race_common submodule,
and Dennis only sent us `ASMBus.h` — not its sibling. Likely the same kind of
update he did for ASMBus.h, but for the file that defines the actual
bridge → VEOS payload.

### Action — request to Dennis (sent 2026-04-25)

Asked Dennis for, in priority order:
1. `VESIResultData.h` matching the new ASM_Bus build.
2. The rest of the auto-generated header bundle from the same build output
   (in case there's another sibling we'd hit next).
3. Struct packing/alignment config the OSA is built with — `#pragma pack`
   setting and whether `boolean_T` is 1 byte or 4 bytes — since even with
   the right header, a packing mismatch reproduces this exact symptom.

### Workaround until reply

Scenic-ego on `dspace_art_cosim_stack.yml` is fully working (verified Phase A
2026-04-25). Use `param scenic_control = True` on any scenario you want to
exercise on the new CoSim VEOS. ART-ego remains usable on the **old** VEOS via
`dspace_art_stack.yml` (verified 2026-04-24, §10).

### Reverse-engineering option (not pursued unless Dennis can't help)

If the canonical header isn't available, the layout can be probed: with
`Sw_Manual_VESI_Overwrite=0` (bridge MUX) and the bridge sending sentinel
patterns per field, watch which scalar in VEOS lights up (e.g. via MAPort
read of plant-side `throttle_position` / `steering_angle` etc.). Tedious
but mechanical. Risk: padding/alignment mismatches are invisible to this
method until they bite — `boolean_T` size in particular (1 vs 4 bytes) is
toolchain-dependent and silently shifts every field that follows.
