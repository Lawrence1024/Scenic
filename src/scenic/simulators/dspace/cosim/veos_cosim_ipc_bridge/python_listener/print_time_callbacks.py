from __future__ import annotations

import json
import socket
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path

HOST = "127.0.0.1"
PORT = 50555
VEOS_HOST = "192.168.100.101"
VEOS_NAME = "CoSimServerScenic"
SCENARIO_SRC = "LagunaSeca_ExternalControl"
FELLOW_NAME = "F1"
MODELDESK_PRE_CONNECT_DELAY_S = 10.0
POST_SAVEAS_SETTLE_S = 0.2
PRE_DOWNLOAD_DELAY_S = 30.0
POST_MODELDESK_DOWNLOAD_DELAY_S = 30.0
TIMESTEP_S = 0.01

# Ego readback paths (MAPort-reachable; COM can't see these, so we use MAPort only).
_EGO_BASE = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant"
EGO_X_PATH  = f"{_EGO_BASE}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"
EGO_Y_PATH  = f"{_EGO_BASE}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"
EGO_VX_PATH = f"{_EGO_BASE}/Velocities/v_x_Vehicle_CoG[km|h]/Out1"
EGO_VY_PATH = f"{_EGO_BASE}/Velocities/v_y_Vehicle_CoG[km|h]/Out1"

# VESI command write paths (from vehicle/controller.py). ControlDesk COM writes OK here,
# MAPort writes OK too — we use MAPort for consistency with reads.
_VESI_INPUTS = "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs"
VESI_THROTTLE_CMD    = f"{_VESI_INPUTS}/Const_throttle_cmd/Value"
VESI_BRAKE_FRONT_CMD = f"{_VESI_INPUTS}/Const_brake_cmd_front/Value"
VESI_BRAKE_REAR_CMD  = f"{_VESI_INPUTS}/Const_brake_cmd_rear/Value"
VESI_STEERING_CMD    = f"{_VESI_INPUTS}/Const_steering_cmd/Value"
VESI_GEAR_CMD        = f"{_VESI_INPUTS}/Const_gear_cmd/Value"
VESI_ENABLE_THROTTLE = f"{_VESI_INPUTS}/Const_enable_throttle_cmd/Value"
VESI_ENABLE_BRAKE    = f"{_VESI_INPUTS}/Const_enable_brake_cmd/Value"
VESI_ENABLE_STEERING = f"{_VESI_INPUTS}/Const_enable_steering_cmd/Value"
VESI_ENABLE_GEAR     = f"{_VESI_INPUTS}/Const_enable_gear_cmd/Value"

# Race "go" signals (from simulator.py step 14). These unlock vehicle motion.
RACE_TRACK_FLAG_MANUAL = "Platform()://RaceControl/Model Root/Parameters/track_flag_manual"  # array, [0]=1
RACE_VEH_FLAG_MANUAL   = "Platform()://RaceControl/Model Root/Parameters/veh_flag_manual"    # array, [0]=0
RACE_MANUAL_MODE       = "Platform()://RaceControl/Model Root/Parameters/manual_mode"        # scalar = 1.0

# Drive test tuning.
DRIVE_STEPS_COUNT = 500           # 500 × 10ms = 5 sim-seconds of full-throttle run
DRIVE_STEPS_TIMEOUT_S = 60.0
EGO_MOVED_THRESHOLD_M = 0.1       # need > 10 cm for a real "moved" call

# CoSim server Bus Outports — the commands VEOS reads FROM the CoSim client. If we
# write VESIResultData_Manual but these stay at 0, it confirms the plant is taking
# inputs from the CoSim bus, not from VESI.
_COSIM_BUS_ROOT = (
    "Platform()://CoSimServerScenic/Signal Chain/IO Function View/Communication/"
    "Bus Manager/Bus Configuration/Model Root"
)
_COSIM_OUT = f"{_COSIM_BUS_ROOT}/Outports/CoSimServerScenic"
_COSIM_IN  = f"{_COSIM_BUS_ROOT}/Inports/CoSimServerScenic"
COSIM_OUTPORT_DIAGNOSTIC_PATHS = [
    ("throttle_cmd",       f"{_COSIM_OUT}/throttle_cmd"),
    ("brake_cmd_front",    f"{_COSIM_OUT}/brake_cmd_front"),
    ("brake_cmd_rear",     f"{_COSIM_OUT}/brake_cmd_rear"),
    ("steering_cmd_deg",   f"{_COSIM_OUT}/steering_cmd_deg"),
    ("gear_cmd",           f"{_COSIM_OUT}/gear_cmd"),
    ("enable_throttle_cmd",f"{_COSIM_OUT}/enable_throttle_cmd"),
    ("enable_brake_cmd",   f"{_COSIM_OUT}/enable_brake_cmd"),
    ("enable_steering_cmd",f"{_COSIM_OUT}/enable_steering_cmd"),
    ("enable_gear_cmd",    f"{_COSIM_OUT}/enable_gear_cmd"),
]
# One CoSim Inport to confirm data flows FROM VEOS (already known to work, but good to
# cross-check that the plant is live during this test run).
COSIM_INPORT_DIAGNOSTIC_PATHS = [
    ("ego_pos_x_m_bus",  f"{_COSIM_IN}/Pos_x_Vehicle_CoorSys_E_m"),
    ("ego_vx_m_s_bus",   f"{_COSIM_IN}/v_x_Vehicle_CoG_m_s"),
]


class StepGate:
    """Controls whether TIME_TRIGGER events are auto-released (default) or gated.

    In 'auto' mode the socket handler sends STEP immediately (the tester's original
    behavior). In 'manual' mode the handler blocks until ``step(n)`` releases a slot,
    mirroring SyncStepBridge.step() semantics so we can exercise the exact pattern
    that Scenic uses in production.
    """

    def __init__(self) -> None:
        self._cv = threading.Condition()
        self._mode = "auto"
        self._remaining = 0
        self._processed = 0

    def set_manual(self) -> None:
        with self._cv:
            self._mode = "manual"
            self._remaining = 0

    def set_auto(self) -> None:
        with self._cv:
            self._mode = "auto"
            self._cv.notify_all()

    def handler_wait(self) -> None:
        """Called by the TIME_TRIGGER handler. Blocks in manual mode until a slot is granted."""
        with self._cv:
            while self._mode == "manual" and self._remaining <= 0:
                self._cv.wait()
            if self._mode == "manual":
                self._remaining -= 1
            self._processed += 1
            self._cv.notify_all()

    def step(self, n: int = 1, timeout: float = 10.0) -> bool:
        """Release n steps and block until all n are processed. Returns True on success."""
        with self._cv:
            start = self._processed
            self._remaining += int(n)
            self._cv.notify_all()
            deadline = time.perf_counter() + float(timeout)
            while self._processed - start < n:
                remaining = deadline - time.perf_counter()
                if remaining <= 0.0:
                    return False
                self._cv.wait(timeout=remaining)
        return True

    @property
    def processed(self) -> int:
        with self._cv:
            return self._processed



def _default_ipc_client_exe() -> Path:
    return (
        Path(__file__).resolve().parent.parent
        / "client"
        / "build"
        / "VeosCoSimTestClientIpc.exe"
    )


def _ensure_src_on_syspath() -> None:
    """Add repo ``src`` to sys.path when running this file directly."""
    current = Path(__file__).resolve()
    for parent in current.parents:
        if parent.name == "src":
            src = str(parent)
            if src not in sys.path:
                sys.path.insert(0, src)
            return


def _check_modeldesk_connection():
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.modeldesk.connection import ModelDeskConnection

    return ModelDeskConnection().connect()


def _connect_controldesk_scenic_control():
    """Run Scenic's full connect_and_prepare path with scenic_control=True.

    This goes online, starts measurement, calls initialize_vesi_interface (which flips
    Sw_Manual_VESI_Overwrite=1 and enables throttle/brake/gear/steering channels), and
    sets the simulation step. Returns the connected ControlDeskApp on success, or None.
    """
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.controldesk import session as cd_session

    class _SimCfg:
        scenic_control = True

    class _SimShim:
        timestep = TIMESTEP_S
        sim = _SimCfg()

    return cd_session.connect_and_prepare(_SimShim())


def _connect_maport():
    """Connect MAPort (XIL API) for variable access. Returns MAPortApp or None.

    Uses the same MAPortConfigVEOS.xml that Scenic uses. start_if_needed=False because
    VEOS should already be running (ControlDesk went online earlier).
    """
    _ensure_src_on_syspath()
    try:
        from scenic.simulators.dspace.maport import session as maport_session
    except Exception as e:
        print(f"[MAPort] import failed: {type(e).__name__}: {e}", flush=True)
        return None
    try:
        return maport_session.connect_and_prepare_maport(sim=None, start_if_needed=False)
    except Exception as e:
        print(f"[MAPort] connect_and_prepare_maport threw: {type(e).__name__}: {e}", flush=True)
        return None


def _read_ego_state(mp) -> "tuple[float | None, float | None, float | None, float | None]":
    """Read ego (x, y, vx_kmh, vy_kmh) via MAPort. All four reads are known to work."""
    def _get(path):
        try:
            return float(mp.get_var(path))
        except Exception:
            return None
    return _get(EGO_X_PATH), _get(EGO_Y_PATH), _get(EGO_VX_PATH), _get(EGO_VY_PATH)


def _safe_set_verify(mp, label: str, path: str, value) -> None:
    """Write once, read back to verify. Log what actually latched. Crucial: pass Python
    ``int`` for UINT-typed variables (enable flags, gear) — MAPort.set_var routes Python
    int -> UINT and Python float -> FLOAT, and a float->UINT write fails silently on the
    wire with 'DataType missmatch. Got: eFLOAT, Expected: eUINT.'
    """
    try:
        mp.set_var(path, value)
    except Exception as e:
        msg = str(e)
        short = msg.split("\n")[0][:200]
        print(f"[DriveTest]   {label}: WRITE FAIL {type(e).__name__}: {short}", flush=True)
        return
    try:
        readback = mp.get_var(path)
    except Exception as e:
        print(f"[DriveTest]   {label}: wrote {value!r} (readback FAILED: {type(e).__name__}: {e})", flush=True)
        return
    print(f"[DriveTest]   {label}: wrote {value!r}  readback={readback!r}", flush=True)


def _safe_set_array_index0(mp, label: str, path: str, value) -> None:
    """Read the array, set index 0, write it back, read again to verify."""
    try:
        arr = list(mp.get_var(path) or [])
    except Exception as e:
        print(f"[DriveTest]   {label}: INITIAL READ FAIL {type(e).__name__}: {e}", flush=True)
        return
    if not arr:
        arr = [0.0]
    arr[0] = float(value)
    try:
        mp.set_var(path, arr)
    except Exception as e:
        msg = str(e)
        short = msg.split("\n")[0][:200]
        print(f"[DriveTest]   {label} (array[0]={value}): WRITE FAIL {type(e).__name__}: {short}", flush=True)
        return
    try:
        post = mp.get_var(path)
        head = list(post)[:3] if post is not None else None
        print(f"[DriveTest]   {label}: wrote array[0]={value!r}  readback head={head!r}", flush=True)
    except Exception as e:
        print(f"[DriveTest]   {label}: wrote array[0]={value!r}  (readback FAILED: {e})", flush=True)


def _test_drive_via_stepping(cd, mp, gate: StepGate) -> None:
    """The definitive experiment: apply VESI drive inputs via MAPort, release steps via
    the gate, and see if the ego accelerates. No ManeuverControl.Start(), no RTA.Start(),
    no pause_simulation(). The hypothesis: stepping IS the control, and the ModelDesk
    maneuver engine is irrelevant for external-control scenarios.

    Succeeds when ego_xy moves noticeably and/or ego_vx goes non-zero after we apply
    full throttle and release 500 steps (5 sim-seconds).
    """
    print("[DriveTest] ===== Drive via stepping (no Start/Stop calls) =====", flush=True)

    # Baseline — before any writes
    x0, y0, vx0, vy0 = _read_ego_state(mp)
    print(
        f"[DriveTest] baseline: ego_xy=({x0}, {y0})  ego_v_kmh=({vx0}, {vy0})",
        flush=True,
    )

    # Switch gate to MANUAL so we have deterministic control over time progression.
    gate.set_manual()
    print(f"[DriveTest] Gate MANUAL (processed={gate.processed}).", flush=True)

    # (1) Race-go signals (same pattern Scenic's simulator.py uses at step 14 of setup).
    print("[DriveTest] Applying race-go signals via MAPort ...", flush=True)
    _safe_set_array_index0(mp, "track_flag_manual", RACE_TRACK_FLAG_MANUAL, 1)
    _safe_set_array_index0(mp, "veh_flag_manual",   RACE_VEH_FLAG_MANUAL,   0)
    _safe_set_verify(mp, "manual_mode", RACE_MANUAL_MODE, 1.0)

    # (2) Enable every VESI channel we care about. IMPORTANT: these are UINT variables
    # in VEOS; passing Python float would fail with 'DataType missmatch Got: eFLOAT,
    # Expected: eUINT'. MAPort.set_var routes Python `int` to UINT correctly.
    print("[DriveTest] Enabling VESI channels (int values for UINT vars) ...", flush=True)
    _safe_set_verify(mp, "enable_throttle", VESI_ENABLE_THROTTLE, 1)
    _safe_set_verify(mp, "enable_brake",    VESI_ENABLE_BRAKE,    1)
    _safe_set_verify(mp, "enable_steering", VESI_ENABLE_STEERING, 1)
    _safe_set_verify(mp, "enable_gear",     VESI_ENABLE_GEAR,     1)

    # (3) Apply drive commands: 1st gear (UINT), full throttle (FLOAT), no brake,
    # neutral steering.
    print("[DriveTest] Writing VESI commands: gear=1, throttle=1.0, brake=0, steer=0 ...", flush=True)
    _safe_set_verify(mp, "gear",        VESI_GEAR_CMD,        1)        # int -> UINT
    _safe_set_verify(mp, "brake_front", VESI_BRAKE_FRONT_CMD, 0.0)
    _safe_set_verify(mp, "brake_rear",  VESI_BRAKE_REAR_CMD,  0.0)
    _safe_set_verify(mp, "steering",    VESI_STEERING_CMD,    0.0)
    _safe_set_verify(mp, "throttle",    VESI_THROTTLE_CMD,    1.0)

    # Short settle so writes propagate before we start stepping.
    time.sleep(0.2)

    # (4) Release steps under gate control. Each released step = one VEOS tick.
    print(
        f"[DriveTest] Releasing {DRIVE_STEPS_COUNT} steps "
        f"({DRIVE_STEPS_COUNT * TIMESTEP_S:.1f} sim-seconds) via gate ...",
        flush=True,
    )
    wall0 = time.perf_counter()
    ok = gate.step(n=DRIVE_STEPS_COUNT, timeout=DRIVE_STEPS_TIMEOUT_S)
    wall_elapsed = time.perf_counter() - wall0

    # (5) Read final ego state.
    x1, y1, vx1, vy1 = _read_ego_state(mp)
    dx = (x1 - x0) if (x0 is not None and x1 is not None) else None
    dy = (y1 - y0) if (y0 is not None and y1 is not None) else None
    ego_moved = (
        dx is not None and dy is not None
        and (abs(dx) > EGO_MOVED_THRESHOLD_M or abs(dy) > EGO_MOVED_THRESHOLD_M)
    )
    ego_has_velocity = (vx1 is not None and abs(vx1) > 0.1) or (vy1 is not None and abs(vy1) > 0.1)

    step_status = "OK" if ok else "TIMED_OUT"
    print(
        f"[DriveTest] Step release {step_status}  wall_elapsed={wall_elapsed:.2f}s",
        flush=True,
    )
    print(
        f"[DriveTest] final:    ego_xy=({x1}, {y1})  ego_v_kmh=({vx1}, {vy1})",
        flush=True,
    )
    print(
        f"[DriveTest] ego_delta=({dx}, {dy})  EGO_MOVED={ego_moved}  EGO_HAS_VELOCITY={ego_has_velocity}",
        flush=True,
    )

    # (6) Diagnostic: read the CoSim bus Outports. If these stay at 0 while we wrote
    # VESI throttle=1.0 above, VEOS is taking commands from the bus (populated by the
    # CoSim client) — not from VESIResultData_Manual.
    print("[DriveTest] --- Phase 1: CoSim outport diagnostic after VESI writes ---", flush=True)
    for label, path in COSIM_OUTPORT_DIAGNOSTIC_PATHS:
        try:
            val = mp.get_var(path)
            print(f"[DriveTest]   OUT {label:22s}  value={val!r}", flush=True)
        except Exception as e:
            msg = str(e).split("\n")[0][:150]
            print(f"[DriveTest]   OUT {label:22s}  READ_FAIL: {type(e).__name__}: {msg}", flush=True)

    # ================= Phase 2: write CoSim outports DIRECTLY =================
    # Earlier runs showed MAPort can't write CoSim outports (they're owned by the
    # CoSim client). Re-try with a fresh call — if the write succeeds and moves
    # the ego, Scenic's commanding surface under CoSim is the outport layer, not
    # VESIResultData_Manual.
    print("[DriveTest] --- Phase 2: try writing CoSim outports directly via MAPort ---", flush=True)

    def _try_outport_write(label: str, path: str, value) -> bool:
        """Attempt write+readback. Return True if write landed."""
        try:
            mp.set_var(path, value)
        except Exception as e:
            msg = str(e).split("\n")[0][:180]
            print(f"[DriveTest]   BUS_WRITE_FAIL {label:22s} value={value!r}: {type(e).__name__}: {msg}", flush=True)
            return False
        try:
            readback = mp.get_var(path)
            print(f"[DriveTest]   BUS_WRITE_OK   {label:22s} wrote={value!r}  readback={readback!r}", flush=True)
            return True
        except Exception as e:
            print(f"[DriveTest]   BUS_WRITE_OK   {label:22s} wrote={value!r}  (readback failed: {e})", flush=True)
            return True

    # Enables first (int→UINT). Then gear (int). Then float commands.
    wrote_any = False
    wrote_any |= _try_outport_write("enable_throttle_cmd", f"{_COSIM_OUT}/enable_throttle_cmd", 1)
    wrote_any |= _try_outport_write("enable_brake_cmd",    f"{_COSIM_OUT}/enable_brake_cmd",    1)
    wrote_any |= _try_outport_write("enable_steering_cmd", f"{_COSIM_OUT}/enable_steering_cmd", 1)
    wrote_any |= _try_outport_write("enable_gear_cmd",     f"{_COSIM_OUT}/enable_gear_cmd",     1)
    wrote_any |= _try_outport_write("gear_cmd",            f"{_COSIM_OUT}/gear_cmd",            1)
    wrote_any |= _try_outport_write("brake_cmd_front",     f"{_COSIM_OUT}/brake_cmd_front",     0.0)
    wrote_any |= _try_outport_write("brake_cmd_rear",      f"{_COSIM_OUT}/brake_cmd_rear",      0.0)
    wrote_any |= _try_outport_write("steering_cmd_deg",    f"{_COSIM_OUT}/steering_cmd_deg",    0.0)
    wrote_any |= _try_outport_write("throttle_cmd",        f"{_COSIM_OUT}/throttle_cmd",        1.0)

    if wrote_any:
        # Release another 500 steps so the plant can respond to bus-written commands.
        print(
            f"[DriveTest] Phase 2: at least one outport write landed. Releasing "
            f"{DRIVE_STEPS_COUNT} more steps to see if ego responds ...",
            flush=True,
        )
        time.sleep(0.2)
        ok2 = gate.step(n=DRIVE_STEPS_COUNT, timeout=DRIVE_STEPS_TIMEOUT_S)
        x2, y2, vx2, vy2 = _read_ego_state(mp)
        dx2 = (x2 - x1) if (x1 is not None and x2 is not None) else None
        dy2 = (y2 - y1) if (y1 is not None and y2 is not None) else None
        ego_moved_p2 = (
            dx2 is not None and dy2 is not None
            and (abs(dx2) > EGO_MOVED_THRESHOLD_M or abs(dy2) > EGO_MOVED_THRESHOLD_M)
        )
        ego_has_vel_p2 = (vx2 is not None and abs(vx2) > 0.1) or (vy2 is not None and abs(vy2) > 0.1)
        print(
            f"[DriveTest] Phase 2 final:  ego_xy=({x2}, {y2})  ego_v_kmh=({vx2}, {vy2})  "
            f"ego_delta_p2=({dx2}, {dy2})  EGO_MOVED_P2={ego_moved_p2}  "
            f"EGO_HAS_VELOCITY_P2={ego_has_vel_p2}",
            flush=True,
        )
        if ego_moved_p2 or ego_has_vel_p2:
            print(
                "[DriveTest] ===> PHASE 2 SUCCESS: writing CoSim outports directly moves the ego. "
                "Under CoSim, Scenic should write commands to CoSim bus outports, NOT to "
                "VESIResultData_Manual.",
                flush=True,
            )
        else:
            print(
                "[DriveTest] ===> PHASE 2 FAIL: ego still did not move. Writes landed on the bus "
                "but something downstream (plant gating, scenario activation, race state) is "
                "blocking command flow to the vehicle plant.",
                flush=True,
            )
    else:
        print(
            "[DriveTest] ===> PHASE 2 SKIPPED: no outport write landed. CoSim outports are "
            "read-only from external tools; the CoSim client (C++ side) must forward commands. "
            "Path forward: modify VeosCoSimTestClientIpc.cpp to read VESI and write outports "
            "per tick, OR extend the IPC protocol so Scenic tells the client what to write.",
            flush=True,
        )

    if ego_moved or ego_has_velocity:
        print(
            "[DriveTest] ===> SUCCESS: ego responded to VESI inputs under gate-controlled stepping. "
            "No ManeuverControl.Start / RTA.Start required.",
            flush=True,
        )
    else:
        print(
            "[DriveTest] ===> FAIL: ego did not move. VESI inputs are not reaching the plant, "
            "OR a precondition (race_control flags, enable channels, VESI manual mode) isn't latched.",
            flush=True,
        )

    # Cleanup: brake hard and zero throttle so the vehicle stops if we triggered motion.
    _safe_set_verify(mp, "cleanup_throttle",    VESI_THROTTLE_CMD,    0.0)
    _safe_set_verify(mp, "cleanup_brake_front", VESI_BRAKE_FRONT_CMD, 1.0)
    _safe_set_verify(mp, "cleanup_brake_rear",  VESI_BRAKE_REAR_CMD,  1.0)

    gate.set_auto()
    print(f"[DriveTest] Gate AUTO (processed={gate.processed}). done.", flush=True)



def _modeldesk_create_scenario_add_fellow_and_download(conn_md) -> tuple[bool, str]:
    """Create working scenario, add one fellow, and download to VEOS."""
    import pythoncom

    app = getattr(conn_md, "app", None)
    exp = getattr(conn_md, "exp", None)
    if app is None or exp is None:
        return False, "ModelDeskConnection missing app/experiment."

    ts0 = exp.TrafficScenario
    if ts0 is None:
        return False, "Active experiment has no TrafficScenario."

    name = time.strftime("Scenic_veos_test_%Y%m%d_%H%M%S")
    print(
        f"[ModelDeskDownload] Activate source='{SCENARIO_SRC}' -> SaveAs '{name}' "
        f"-> add fellow '{FELLOW_NAME}' -> Save/Download ...",
        flush=True,
    )

    try:
        exp.ActivateTrafficScenario(SCENARIO_SRC)
    except Exception as e:
        print(
            f"[ModelDeskDownload] ActivateTrafficScenario({SCENARIO_SRC!r}) ignored: {e}",
            flush=True,
        )

    try:
        exp.TrafficScenario.SaveAs(name, True)
    except Exception:
        editor = exp.EditTrafficScenario()
        try:
            editor.SaveAs(name, True)
        finally:
            try:
                editor.Close(False)
            except Exception:
                pass

    try:
        exp.ActivateTrafficScenario(name)
    except Exception as e:
        return False, f"ActivateTrafficScenario({name!r}) failed: {e}"

    pythoncom.PumpWaitingMessages()
    time.sleep(POST_SAVEAS_SETTLE_S)
    proj = app.ActiveProject
    if proj is None:
        return False, "ActiveProject is None after SaveAs."
    exp = proj.ActiveExperiment
    if exp is None:
        return False, "ActiveExperiment is None after SaveAs."
    ts = exp.TrafficScenario
    if ts is None:
        return False, "TrafficScenario is None after SaveAs/activate."

    try:
        fellow = ts.Fellows.Item(FELLOW_NAME)
        print(f"[ModelDeskDownload] Using existing fellow: {FELLOW_NAME}", flush=True)
    except Exception:
        fellow = ts.Fellows.Add()
        fellow.Name = FELLOW_NAME
        print(f"[ModelDeskDownload] Added fellow: {FELLOW_NAME}", flush=True)

    try:
        seqs = fellow.Sequences
        seq = seqs.Add() if seqs.Count == 0 else seqs.Item(1)
        route_sel = seq.Route
        available = [str(x) for x in list(route_sel.AvailableElements)]
        chosen = None
        non_pit = [n for n in available if "pit" not in n.lower()]
        if non_pit:
            chosen = non_pit[0]
        elif available:
            chosen = available[0]
        if chosen:
            route_sel.Activate(chosen)
            print(f"[ModelDeskDownload] Fellow route set: {chosen}", flush=True)
        route_sel.Direction = 1
        route_sel.UseExternal = True
    except Exception as e:
        print(f"[ModelDeskDownload] Fellow route setup warning: {e}", flush=True)

    try:
        from scenic.simulators.dspace.modeldesk.traffic_object import apply_fellow_traffic_object
        apply_fellow_traffic_object(fellow)
    except Exception as e:
        print(f"[ModelDeskDownload] Traffic object setup warning: {e}", flush=True)

    try:
        try:
            consistent = ts.CheckConsistency()
            print(f"[ModelDeskDownload] CheckConsistency() = {consistent}", flush=True)
        except Exception as e:
            print(f"[ModelDeskDownload] CheckConsistency() skipped: {e}", flush=True)

        ts.Save()
        print(
            f"[ModelDeskDownload] Pre-download pause {PRE_DOWNLOAD_DELAY_S:.1f}s ...",
            flush=True,
        )
        time.sleep(PRE_DOWNLOAD_DELAY_S)
        ok = ts.Download()
    except Exception as e:
        return False, f"Save/Download failed: {e}"

    if not ok:
        return False, "TrafficScenario.Download() returned False."

    try:
        exp.ManeuverControl.Reset()
        print("[ModelDeskDownload] ManeuverControl.Reset() OK.", flush=True)
    except Exception as e:
        print(f"[ModelDeskDownload] ManeuverControl.Reset() warning: {e}", flush=True)

    return True, f"Scenario '{name}' downloaded with fellow '{FELLOW_NAME}'."


def main() -> int:
    exe_path = _default_ipc_client_exe()
    client_proc = None
    gate = StepGate()

    print(f"Starting local IPC listener on {HOST}:{PORT} ...", flush=True)
    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((HOST, PORT))
            server.listen(1)

            if not exe_path.is_file():
                print(
                    f"ERROR: IPC client executable not found: {exe_path}",
                    file=sys.stderr,
                    flush=True,
                )
                return 1

            cmd = [
                str(exe_path),
                "--host", VEOS_HOST,
                "--name", VEOS_NAME,
                "--ipc-host", HOST,
                "--ipc-port", str(PORT),
            ]
            print(f"Auto-launching IPC client: {' '.join(cmd)}", flush=True)
            client_proc = subprocess.Popen(
                cmd,
                cwd=str(exe_path.parent),
                creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0,
            )

            print("Waiting for IPC bridge client to connect...", flush=True)
            conn, addr = server.accept()

            with conn:
                print(f"IPC bridge connected from {addr[0]}:{addr[1]}", flush=True)

                def _run_setup():
                    try:
                        print(
                            f"[ModelDeskCheck] Pre-connect pause {MODELDESK_PRE_CONNECT_DELAY_S:.1f}s ...",
                            flush=True,
                        )
                        time.sleep(MODELDESK_PRE_CONNECT_DELAY_S)
                        print('[ModelDeskCheck] COM dispatch starting: "ModelDesk.Application"', flush=True)
                        conn_md = _check_modeldesk_connection()
                        proj = getattr(conn_md, "proj", None)
                        exp = getattr(conn_md, "exp", None)
                        proj_name = getattr(proj, "Name", "?") if proj is not None else "?"
                        exp_name = getattr(exp, "Name", "?") if exp is not None else "?"
                        print(
                            f"[ModelDeskCheck] OK: connected (project={proj_name}, experiment={exp_name}).",
                            flush=True,
                        )

                        md_ok, md_msg = _modeldesk_create_scenario_add_fellow_and_download(conn_md)
                        if not md_ok:
                            print(f"[ModelDeskDownload] FAILED: {md_msg}", flush=True)
                            print("[Setup] ModelDesk step failed; aborting ControlDesk check.", flush=True)
                            return
                        print(f"[ModelDeskDownload] OK: {md_msg}", flush=True)

                        print(
                            f"[ControlDeskAfterModelDesk] Waiting {POST_MODELDESK_DOWNLOAD_DELAY_S:.1f}s "
                            "before ControlDesk connect + Scenic-control init ...",
                            flush=True,
                        )
                        time.sleep(POST_MODELDESK_DOWNLOAD_DELAY_S)
                        print(
                            "[ControlDeskAfterModelDesk] Running connect_and_prepare "
                            "(online + measurement + VESI Manual init + timestep) ...",
                            flush=True,
                        )
                        cd = _connect_controldesk_scenic_control()
                        if cd is None:
                            print("[ControlDeskAfterModelDesk] FAILED: connect_and_prepare returned None.", flush=True)
                            print("[Setup] ControlDesk step failed.", flush=True)
                            return
                        print(
                            "[ControlDeskAfterModelDesk] OK: online, measurement started, "
                            "VesiInterface initialized (scenic_control=True), "
                            f"timestep={TIMESTEP_S:.4f}s applied.",
                            flush=True,
                        )

                        print("[MAPortCheck] Attempting MAPort (XIL API) connect ...", flush=True)
                        mp = _connect_maport()
                        if mp is None:
                            print(
                                "[MAPortCheck] FAILED: MAPort connect returned None. "
                                "Check clr/pythonnet install, XIL API assemblies, and MAPortConfigVEOS.xml.",
                                flush=True,
                            )
                            mp_connected = False
                        else:
                            print("[MAPortCheck] OK: MAPort connected.", flush=True)
                            mp_connected = True
                            try:
                                _test_drive_via_stepping(cd, mp, gate)
                            except Exception as e:
                                print(f"[DriveTest] UNHANDLED ERROR: {type(e).__name__}: {e}", flush=True)
                                print(traceback.format_exc(), flush=True)
                            finally:
                                try:
                                    mp.dispose()
                                    print("[MAPortCheck] MAPort disposed.", flush=True)
                                except Exception as e:
                                    print(f"[MAPortCheck] MAPort dispose warning: {type(e).__name__}: {e}", flush=True)

                        print("[Setup] === ALL STEPS COMPLETED ===", flush=True)
                        print(f"[Setup]   MAPort connected: {mp_connected}", flush=True)
                        print("[Setup]   See [DriveTest] lines above for the verdict.", flush=True)
                    except Exception as e:
                        print(f"[Setup] ERROR: {e}", flush=True)
                        print(traceback.format_exc(), flush=True)
                    finally:
                        # Shut down the IPC socket so the main recv loop exits and the
                        # process ends cleanly — user no longer wants the bridge idling
                        # after the experiment completes.
                        print("[Setup] Shutting down IPC bridge to end the process.", flush=True)
                        try:
                            conn.shutdown(socket.SHUT_RDWR)
                        except Exception:
                            pass

                threading.Thread(target=_run_setup, name="SetupWorker", daemon=True).start()

                buf = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("IPC bridge disconnected.", flush=True)
                        break

                    buf += chunk

                    def _safe_send(data: bytes) -> bool:
                        """Send, but treat socket-shut-down errors as a signal to stop the loop."""
                        try:
                            conn.sendall(data)
                            return True
                        except OSError:
                            return False

                    send_failed = False
                    while b"\n" in buf:
                        line, buf = buf.split(b"\n", 1)
                        line = line.strip()
                        if not line:
                            continue

                        text = line.decode("utf-8", errors="replace")

                        try:
                            obj = json.loads(text)
                        except json.JSONDecodeError:
                            print(f"[RAW] {text}", flush=True)
                            if not _safe_send(b"ACK\n"):
                                send_failed = True
                                break
                            continue

                        event = obj.get("event", "UNKNOWN")
                        if event == "TIME_TRIGGER":
                            gate.handler_wait()
                            if not _safe_send(b"STEP\n"):
                                send_failed = True
                                break
                        else:
                            print(f"[{event}] {obj}", flush=True)
                            if not _safe_send(b"ACK\n"):
                                send_failed = True
                                break
                    if send_failed:
                        break

        return 0

    except KeyboardInterrupt:
        print("Interrupted; shutting down.", flush=True)
        return 130
    except OSError as exc:
        # socket-closed errors are expected on clean shutdown — demote to a quiet message.
        msg = str(exc)
        if "10058" in msg or "shut down" in msg.lower():
            print("IPC bridge socket closed cleanly.", flush=True)
            return 0
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1
    finally:
        if client_proc is not None:
            print("Terminating auto-launched IPC client process ...", flush=True)
            try:
                client_proc.terminate()
                client_proc.wait(timeout=8)
            except subprocess.TimeoutExpired:
                try:
                    client_proc.kill()
                except Exception:
                    pass
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
