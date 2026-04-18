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
`launch_veos_ipc_client` Scenic parameter (default `False`). When `False`:

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
