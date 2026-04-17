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
VAR_TEST_SETTLE_S = 0.05

EGO_CMD_VARS = [
    ("throttle",    "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_throttle_cmd/Value",    0.25),
    ("brake_front", "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_front/Value", 0.10),
    ("brake_rear",  "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_brake_cmd_rear/Value",  0.10),
    ("steering",    "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_steering_cmd/Value",    0.05),
    ("gear",        "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/vehicle_inputs/Const_gear_cmd/Value",        1.0),
]

_EGO_BASE  = "Platform()://ASM_Traffic/Model Root/VehicleDynamics/Plant/UserInterface/DISP_Plant"
_GPS_BASE  = "Platform()://ASM_Traffic/Model Root/Environment/Road/PlantModel/GPS_POSITION/GPS_CALC"
EGO_READ_VARS = [
    ("ego_x_m",       f"{_EGO_BASE}/Positions/Pos_x_Vehicle_CoorSys_E[m]/Out1"),
    ("ego_y_m",       f"{_EGO_BASE}/Positions/Pos_y_Vehicle_CoorSys_E[m]/Out1"),
    ("ego_z_m",       f"{_EGO_BASE}/Positions/Pos_z_Vehicle_CoorSys_E[m]/Out1"),
    ("ego_yaw_deg",   f"{_EGO_BASE}/Positions/Angle_Yaw_Vehicle_CoorSys_E[deg]/Out1"),
    ("ego_vx_kmh",    f"{_EGO_BASE}/Velocities/v_x_Vehicle_CoG[km|h]/Out1"),
    ("ego_vy_kmh",    f"{_EGO_BASE}/Velocities/v_y_Vehicle_CoG[km|h]/Out1"),
    ("gps_lon_deg",   f"{_GPS_BASE}/Longitude_deg"),
    ("gps_lat_deg",   f"{_GPS_BASE}/Latitude_deg"),
    ("gps_hdg_deg",   f"{_GPS_BASE}/Heading_deg"),
]

_FELLOW_EXT_BASE = "Platform()://ASM_Traffic/Model Root/Environment/Traffic/PlantModel/FellowMovement/External_Signals"
FELLOW_BULK_VARS = [
    ("fellow_v_kmh_array", f"{_FELLOW_EXT_BASE}/Const_v_Fellows_External[km|h]/Value"),
    ("fellow_d_m_array",   f"{_FELLOW_EXT_BASE}/Const_d_Fellows_External[m]/Value"),
]


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


def _test_variable_access(cd, backend: str = "VarTest") -> tuple[int, int, list[str]]:
    """Read/write/restore every variable group Scenic touches, via the given backend.

    `cd` can be a ControlDeskApp (COM) or MAPortApp — both expose get_var/set_var.
    `backend` is a short label used in log prefixes so COM vs MAPort output is
    easy to tell apart.

    Never aborts on error: each variable's probe is wrapped in its own try/except so
    failures just print the label and move on. Returns (n_ok, n_fail, failed_labels).
    """
    tag = f"[{backend}]"
    n_ok = 0
    n_fail = 0
    failed: list[str] = []

    def _probe_rw_scalar(label: str, path: str, test_val: float) -> None:
        nonlocal n_ok, n_fail
        try:
            before = cd.get_var(path)
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (read): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)
            return
        try:
            cd.set_var(path, float(test_val))
            time.sleep(VAR_TEST_SETTLE_S)
            after = cd.get_var(path)
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (write/readback): {type(e).__name__}: {e} (before={before!r})", flush=True)
            n_fail += 1
            failed.append(label)
            try:
                cd.set_var(path, before)
            except Exception:
                pass
            return
        try:
            cd.set_var(path, before)
        except Exception as e:
            print(f"{tag} WARN {label:22s} restore failed: {type(e).__name__}: {e}", flush=True)
        print(f"{tag} OK   {label:22s} before={before!r} wrote={test_val!r} readback={after!r}", flush=True)
        n_ok += 1

    def _probe_read_only(label: str, path: str) -> None:
        nonlocal n_ok, n_fail
        try:
            val = cd.get_var(path)
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (read): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)
            return
        print(f"{tag} OK   {label:22s} value={val!r}", flush=True)
        n_ok += 1

    def _probe_rw_array(label: str, path: str) -> None:
        nonlocal n_ok, n_fail
        try:
            before = cd.get_var(path)
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (read): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)
            return
        try:
            before_list = list(before) if before is not None else []
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (value not iterable): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)
            return
        if not before_list:
            print(f"{tag} OK   {label:22s} (empty array; read succeeded, skipped write probe)", flush=True)
            n_ok += 1
            return
        try:
            modified = list(before_list)
            modified[0] = float(modified[0]) + 0.1
            cd.set_var(path, modified)
            time.sleep(VAR_TEST_SETTLE_S)
            after = cd.get_var(path)
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (write/readback): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)
            try:
                cd.set_var(path, before_list)
            except Exception:
                pass
            return
        try:
            cd.set_var(path, before_list)
        except Exception as e:
            print(f"{tag} WARN {label:22s} restore failed: {type(e).__name__}: {e}", flush=True)
        try:
            after0 = list(after)[0] if after is not None else None
        except Exception:
            after0 = None
        print(
            f"{tag} OK   {label:22s} len={len(before_list)} "
            f"before[0]={before_list[0]!r} wrote[0]={modified[0]!r} readback[0]={after0!r}",
            flush=True,
        )
        n_ok += 1

    def _safe(callable_, label: str) -> None:
        nonlocal n_fail
        try:
            callable_()
        except Exception as e:
            print(f"{tag} FAIL {label:22s} (unexpected): {type(e).__name__}: {e}", flush=True)
            n_fail += 1
            failed.append(label)

    print(f"{tag} --- Ego command channels (write/read/restore) ---", flush=True)
    for label, path, test_val in EGO_CMD_VARS:
        _safe(lambda l=label, p=path, v=test_val: _probe_rw_scalar(l, p, v), label)

    print(f"{tag} --- Ego state readback (read-only) ---", flush=True)
    for label, path in EGO_READ_VARS:
        _safe(lambda l=label, p=path: _probe_read_only(l, p), label)

    print(f"{tag} --- Fellow external signals (array read/write/restore) ---", flush=True)
    for label, path in FELLOW_BULK_VARS:
        _safe(lambda l=label, p=path: _probe_rw_array(l, p), label)

    return n_ok, n_fail, failed


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

                        com_ok, com_fail, com_failed = _test_variable_access(cd, backend="ControlDesk")

                        print("[MAPortCheck] Attempting MAPort (XIL API) connect ...", flush=True)
                        mp = _connect_maport()
                        if mp is None:
                            print(
                                "[MAPortCheck] FAILED: MAPort connect returned None. "
                                "Check clr/pythonnet install, XIL API assemblies, and MAPortConfigVEOS.xml.",
                                flush=True,
                            )
                            mp_ok = mp_fail = 0
                            mp_failed: list[str] = []
                            mp_connected = False
                        else:
                            print("[MAPortCheck] OK: MAPort connected.", flush=True)
                            mp_connected = True
                            try:
                                mp_ok, mp_fail, mp_failed = _test_variable_access(mp, backend="MAPort")
                            finally:
                                try:
                                    mp.dispose()
                                    print("[MAPortCheck] MAPort disposed.", flush=True)
                                except Exception as e:
                                    print(f"[MAPortCheck] MAPort dispose warning: {type(e).__name__}: {e}", flush=True)

                        print("[Setup] === ALL STEPS COMPLETED SUCCESSFULLY ===", flush=True)
                        print("[Setup]   ModelDesk scenario downloaded to VEOS.", flush=True)
                        print("[Setup]   ControlDesk online with scenic_control=True init.", flush=True)
                        print(f"[Setup]   ControlDesk variable test: {com_ok} OK, {com_fail} FAIL.", flush=True)
                        if com_failed:
                            print(f"[Setup]     Failed (ControlDesk): {', '.join(com_failed)}", flush=True)
                        if mp_connected:
                            print(f"[Setup]   MAPort variable test:     {mp_ok} OK, {mp_fail} FAIL.", flush=True)
                            if mp_failed:
                                print(f"[Setup]     Failed (MAPort):     {', '.join(mp_failed)}", flush=True)
                        else:
                            print("[Setup]   MAPort variable test:     SKIPPED (connect failed).", flush=True)
                        print("[Setup]   IPC bridge is releasing VEOS steps.", flush=True)
                    except Exception as e:
                        print(f"[Setup] ERROR: {e}", flush=True)
                        print(traceback.format_exc(), flush=True)

                threading.Thread(target=_run_setup, name="SetupWorker", daemon=True).start()

                buf = b""
                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("IPC bridge disconnected.", flush=True)
                        break

                    buf += chunk

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
                            conn.sendall(b"ACK\n")
                            continue

                        event = obj.get("event", "UNKNOWN")
                        if event == "TIME_TRIGGER":
                            conn.sendall(b"STEP\n")
                        else:
                            print(f"[{event}] {obj}", flush=True)
                            conn.sendall(b"ACK\n")

        return 0

    except KeyboardInterrupt:
        print("Interrupted; shutting down.", flush=True)
        return 130
    except OSError as exc:
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
