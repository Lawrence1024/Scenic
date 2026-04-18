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
PRE_DOWNLOAD_DELAY_S = 20.0
POST_MODELDESK_DOWNLOAD_DELAY_S = 20.0
TIMESTEP_S = 0.01

_CLIF_PATH = "Platform()://ASM_Traffic/Model Root/VesiInterface/Sw_Activate_CLIF[0|1]/Value"
_MANEUVER_START_PATH = (
    "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/"
    "UserInterface/PAR_Plant/ManeuverControl/MANEUVER_START/MDLDCtrl_ManeuverStart"
)
MANEUVER_TIME_PATH = (
    "Platform()://ASM_Traffic/Model Root/Environment/Maneuver/UserInterface/"
    "DISP_Plant/ManeuverTime[s]/Out1"
)

MANEUVER_LIFECYCLE_STEPS = 100   # ~100 × 10ms = 1 sim-second of controlled stepping
MANEUVER_LIFECYCLE_TIMEOUT_S = 30.0


class StepGate:
    """Controls whether TIME_TRIGGER events are auto-released (default) or gated.

    In 'auto' mode the socket handler sends STEP immediately so VEOS runs freely.
    In 'manual' mode each STEP is held until ``step(n)`` releases a slot.
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
    """Run Scenic's full connect_and_prepare path with scenic_control=True."""
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.controldesk import session as cd_session

    class _SimCfg:
        scenic_control = True

    class _SimShim:
        timestep = TIMESTEP_S
        sim = _SimCfg()

    return cd_session.connect_and_prepare(_SimShim())


def _connect_maport():
    """Connect MAPort (XIL API) for variable access. Returns MAPortApp or None."""
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


def _test_maneuver_lifecycle(cd, mp, gate: StepGate, conn_md=None) -> None:
    """Exercise the full maneuver start → controlled steps → stop → reset sequence.

    Gate must be in AUTO when this is called (VEOS stepping freely). Sequence:
      1. Start maneuver via ModelDesk COM ManeuverControl.Start() if conn_md available,
         else fall back to MAPort MANEUVER_START pulse.
         NOTE: MAPort pulse triggers ExternalControl CLIF blocking; COM path may not.
      2. Poll ManeuverTime > 0 (5s wall clock) — exit on failure
      3. Switch gate → MANUAL
      4. Run MANEUVER_LIFECYCLE_STEPS controlled steps
      5. Switch gate → AUTO
      6. Pulse MANEUVER_STOP, then MANEUVER_RESET via ControlDesk connection
      7. Poll ManeuverTime → 0 (3s)
    """
    _ensure_src_on_syspath()

    print("[ManeuverTest] ===== Maneuver lifecycle test =====", flush=True)

    # Read CLIF for diagnostic logging only (not changed here).
    try:
        clif_val = float(mp.get_var(_CLIF_PATH))
        print(f"[ManeuverTest] Sw_Activate_CLIF current value = {clif_val!r}", flush=True)
    except Exception as e:
        print(f"[ManeuverTest] [WARN] Could not read CLIF: {e}", flush=True)

    # Start the maneuver. Prefer ModelDesk COM (ManeuverControl.Start) which may
    # bypass ExternalControl CLIF blocking. Fall back to MAPort variable pulse.
    exp = getattr(conn_md, "exp", None) if conn_md is not None else None
    mc = getattr(exp, "ManeuverControl", None) if exp is not None else None
    if mc is not None:
        print("[ManeuverTest] Starting maneuver via ModelDesk COM ManeuverControl.Start(False) ...", flush=True)
        try:
            mc.Start(False)
            print("[ManeuverTest] ManeuverControl.Start(False) called.", flush=True)
        except Exception as e:
            print(f"[ManeuverTest] ManeuverControl.Start() error: {type(e).__name__}: {e}", flush=True)
    else:
        print("[ManeuverTest] No ModelDesk conn; pulsing MANEUVER_START via MAPort ...", flush=True)
        try:
            mp.set_var(_MANEUVER_START_PATH, 1.0)
            time.sleep(0.5)
            mp.set_var(_MANEUVER_START_PATH, 0.0)
            print("[ManeuverTest] MANEUVER_START pulsed (1.0 → 0.0).", flush=True)
        except Exception as e:
            print(f"[ManeuverTest] MANEUVER_START pulse error: {type(e).__name__}: {e}", flush=True)

    # 4. Poll ManeuverTime > 0
    print("[ManeuverTest] Polling ManeuverTime (up to 5s) ...", flush=True)
    mt_started = False
    deadline = time.perf_counter() + 5.0
    while time.perf_counter() < deadline:
        try:
            mt = float(mp.get_var(MANEUVER_TIME_PATH))
        except Exception:
            mt = 0.0
        if mt > 0.0:
            print(f"[ManeuverTest] ManeuverTime = {mt:.4f}s — maneuver is running.", flush=True)
            mt_started = True
            break
        time.sleep(0.2)

    if not mt_started:
        print(
            "[ManeuverTest] FATAL: ManeuverTime did not advance after maneuver start call.\n"
            "  Possible causes: VEOS not stepping, ExternalControl CLIF blocking, bridge not AUTO.",
            flush=True,
        )
        sys.exit(1)

    # 5. Switch gate → MANUAL
    gate.set_manual()
    print(f"[ManeuverTest] Gate → MANUAL (processed={gate.processed}).", flush=True)

    # 6. Run controlled steps
    print(
        f"[ManeuverTest] Releasing {MANEUVER_LIFECYCLE_STEPS} controlled steps "
        f"({MANEUVER_LIFECYCLE_STEPS * TIMESTEP_S:.2f}s sim time) ...",
        flush=True,
    )
    wall0 = time.perf_counter()
    ok = gate.step(n=MANEUVER_LIFECYCLE_STEPS, timeout=MANEUVER_LIFECYCLE_TIMEOUT_S)
    wall_elapsed = time.perf_counter() - wall0
    step_status = "OK" if ok else "TIMED_OUT"
    print(
        f"[ManeuverTest] Step release {step_status}  wall_elapsed={wall_elapsed:.2f}s  "
        f"processed={gate.processed}",
        flush=True,
    )

    try:
        mt_after = float(mp.get_var(MANEUVER_TIME_PATH))
        print(f"[ManeuverTest] ManeuverTime after steps = {mt_after:.4f}s", flush=True)
    except Exception as e:
        print(f"[ManeuverTest] ManeuverTime readback failed: {e}", flush=True)

    # 7. Switch gate → AUTO (VEOS must step freely to observe stop/reset pulses)
    gate.set_auto()
    print("[ManeuverTest] Gate → AUTO (stop/reset pulses need VEOS stepping).", flush=True)

    # 8. Pulse MANEUVER_STOP then MANEUVER_RESET via ControlDesk connection
    try:
        cd.stop_maneuver(var_access=mp)
        print("[ManeuverTest] MANEUVER_STOP pulsed.", flush=True)
    except Exception as e:
        print(f"[ManeuverTest] MANEUVER_STOP pulse error: {type(e).__name__}: {e}", flush=True)

    try:
        cd.reset_maneuver(var_access=mp)
        print("[ManeuverTest] MANEUVER_RESET pulsed.", flush=True)
    except Exception as e:
        print(f"[ManeuverTest] MANEUVER_RESET pulse error: {type(e).__name__}: {e}", flush=True)

    # 9. Poll ManeuverTime → 0
    print("[ManeuverTest] Polling ManeuverTime → 0 (up to 3s) ...", flush=True)
    mt_reset_ok = False
    deadline = time.perf_counter() + 3.0
    while time.perf_counter() < deadline:
        try:
            mt = float(mp.get_var(MANEUVER_TIME_PATH))
        except Exception:
            mt = -1.0
        if mt <= 0.0:
            print(f"[ManeuverTest] ManeuverTime reset to {mt:.4f}s — OK.", flush=True)
            mt_reset_ok = True
            break
        time.sleep(0.1)
    if not mt_reset_ok:
        print("[ManeuverTest] [WARN] ManeuverTime did not reach 0 within 3s after reset.", flush=True)

    print(
        f"[ManeuverTest] ===== lifecycle {'PASS' if mt_reset_ok else 'PARTIAL'} =====",
        flush=True,
    )


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
                            print("[Setup] ModelDesk step failed; aborting.", flush=True)
                            return
                        print(f"[ModelDeskDownload] OK: {md_msg}", flush=True)

                        print(
                            f"[ControlDeskAfterModelDesk] Waiting {POST_MODELDESK_DOWNLOAD_DELAY_S:.1f}s "
                            "before ControlDesk connect ...",
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
                                "[MAPortCheck] FAILED: MAPort connect returned None.",
                                flush=True,
                            )
                            return
                        print("[MAPortCheck] OK: MAPort connected.", flush=True)

                        run1_ok = False
                        run2_ok = False
                        try:
                            # --- Run 1 (sut_te_bridge connected; should always pass) ---
                            print("\n[Setup] ===== Lifecycle run 1 of 2 =====", flush=True)
                            _test_maneuver_lifecycle(cd, mp, gate, conn_md=conn_md)
                            run1_ok = True

                            # Brief settle between runs (VEOS still running; no IPC restart;
                            # sut_te_bridge stays connected — this is the key difference from
                            # the "restart tester twice" experiment).
                            print(
                                "\n[Setup] ===== Lifecycle run 2 of 2 (same socket, no IPC restart) =====",
                                flush=True,
                            )
                            print(
                                "[Setup] If BSC-restart is the root cause, run 2 should PASS here\n"
                                "        because sut_te_bridge is still connected (no disconnect happened).",
                                flush=True,
                            )
                            time.sleep(2.0)
                            _test_maneuver_lifecycle(cd, mp, gate, conn_md=conn_md)
                            run2_ok = True
                        except SystemExit:
                            raise
                        except Exception as e:
                            print(f"[ManeuverTest] UNHANDLED ERROR: {type(e).__name__}: {e}", flush=True)
                            print(traceback.format_exc(), flush=True)
                        finally:
                            try:
                                mp.dispose()
                                print("[MAPortCheck] MAPort disposed.", flush=True)
                            except Exception as e:
                                print(f"[MAPortCheck] MAPort dispose warning: {type(e).__name__}: {e}", flush=True)

                        if run1_ok and run2_ok:
                            print(
                                "\n[Setup] === ALL STEPS COMPLETED SUCCESSFULLY (run1=PASS, run2=PASS) ===",
                                flush=True,
                            )
                            print(
                                "[Setup] Both lifecycle runs passed in the same socket session.\n"
                                "        This confirms the root cause: BSC restart (from IPC client disconnect)\n"
                                "        is what blocks run 2 — NOT a ControlDesk or CLIF restore issue.",
                                flush=True,
                            )
                        else:
                            print(
                                f"\n[Setup] PARTIAL: run1={'PASS' if run1_ok else 'FAIL'} "
                                f"run2={'PASS' if run2_ok else 'FAIL'}",
                                flush=True,
                            )
                    except SystemExit:
                        raise
                    except Exception as e:
                        print(f"[Setup] ERROR: {e}", flush=True)
                        print(traceback.format_exc(), flush=True)
                    finally:
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
                client_proc.kill()
                client_proc.wait(timeout=3)
            except Exception:
                pass


if __name__ == "__main__":
    raise SystemExit(main())
