from __future__ import annotations

import argparse
import json
import socket
import subprocess
import sys
import threading
import time
import traceback
from pathlib import Path
from typing import Optional

TIME_RESOLUTION_PER_SECOND = 1e9
SCENIC_DEFAULT_SCENARIO_SRC = "LagunaSeca_ExternalControl"


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


def _check_controldesk_like_scenic(*, timestep: float, scenic_control: bool):
    """Run the same ControlDesk connection path Scenic uses."""
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.controldesk import session as cd_session

    class _SimCfg:
        def __init__(self, scenic_control_: bool):
            self.scenic_control = bool(scenic_control_)

    class _SimShim:
        def __init__(self, timestep_: float, scenic_control_: bool):
            self.timestep = float(timestep_)
            self.sim = _SimCfg(scenic_control_)

    shim = _SimShim(timestep, scenic_control)
    return cd_session.connect_and_prepare(shim)


def _check_modeldesk_connection():
    """Run a lightweight ModelDesk COM connection check."""
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.modeldesk.connection import ModelDeskConnection

    return ModelDeskConnection().connect()


def _connect_controldesk_go_online() -> tuple[bool, str]:
    """Connect to ControlDesk and request online calibration."""
    _ensure_src_on_syspath()
    from scenic.simulators.dspace.controldesk.connection import ControlDeskApp

    try:
        cd = ControlDeskApp().connect()
        cd.go_online()
        return True, "connected + go_online() succeeded"
    except Exception as e:
        return False, f"connect/go_online failed: {e}"


def _modeldesk_create_scenario_add_fellow_and_download(
    conn_md,
    *,
    scenario_src: Optional[str],
    scenario_name: Optional[str],
    fellow_name: str,
    maneuver_reset: bool,
) -> tuple[bool, str]:
    """Create working scenario, add one fellow, and download to VEOS."""
    import pythoncom

    app = getattr(conn_md, "app", None)
    exp = getattr(conn_md, "exp", None)
    if app is None or exp is None:
        return False, "ModelDeskConnection missing app/experiment."

    ts0 = exp.TrafficScenario
    if ts0 is None:
        return False, "Active experiment has no TrafficScenario."

    # Match Scenic setup default: simulator.py activates sim.scenario_src, whose model default
    # is "LagunaSeca_ExternalControl" in src/scenic/simulators/dspace/model.scenic.
    source = scenario_src if scenario_src else SCENIC_DEFAULT_SCENARIO_SRC
    name = scenario_name or time.strftime("Scenic_veos_test_%Y%m%d_%H%M%S")
    fellow_name = str(fellow_name or "F1")

    print(
        f"[ModelDeskDownload] Activate source='{source}' -> SaveAs '{name}' "
        f"-> add fellow '{fellow_name}' -> Save/Download ...",
        flush=True,
    )

    try:
        exp.ActivateTrafficScenario(source)
    except Exception as e:
        print(f"[ModelDeskDownload] ActivateTrafficScenario({source!r}) ignored: {e}", flush=True)

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
    time.sleep(0.2)
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
        fellow = ts.Fellows.Item(fellow_name)
        print(f"[ModelDeskDownload] Using existing fellow: {fellow_name}", flush=True)
    except Exception:
        fellow = ts.Fellows.Add()
        fellow.Name = fellow_name
        print(f"[ModelDeskDownload] Added fellow: {fellow_name}", flush=True)

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
        ok = ts.Download()
    except Exception as e:
        return False, f"Save/Download failed: {e}"

    if not ok:
        return False, "TrafficScenario.Download() returned False."

    if maneuver_reset:
        try:
            exp.ManeuverControl.Reset()
            print("[ModelDeskDownload] ManeuverControl.Reset() OK.", flush=True)
        except Exception as e:
            print(f"[ModelDeskDownload] ManeuverControl.Reset() warning: {e}", flush=True)

    return True, f"Scenario '{name}' downloaded with fellow '{fellow_name}'."


def _controldesk_rw_smoke(cd) -> bool:
    """Best-effort read/write/read/restore check on a VESI variable."""
    key_throttle = (
        "Platform()://ASM_Traffic/Model Root/VesiInterface/VESIResultData_Manual/"
        "vehicle_inputs/Const_throttle_cmd/Value"
    )
    try:
        before = cd.get_var(key_throttle)
        print(f"[ControlDeskRW] Read before: throttle={before!r}", flush=True)
    except Exception as e:
        print(f"[ControlDeskRW] FAILED reading throttle before write: {e}", flush=True)
        return False

    # Toggle between 0 and 1 where possible, then restore exact previous value.
    try:
        try:
            before_f = float(before)
            write_val = 0.0 if abs(before_f) > 0.5 else 1.0
        except Exception:
            write_val = 1.0

        cd.set_var(key_throttle, write_val)
        time.sleep(0.05)
        after = cd.get_var(key_throttle)
        print(
            f"[ControlDeskRW] Wrote throttle={write_val!r}; read after={after!r}",
            flush=True,
        )
        cd.set_var(key_throttle, before)
        time.sleep(0.05)
        restored = cd.get_var(key_throttle)
        print(f"[ControlDeskRW] Restored throttle={restored!r}", flush=True)
        return True
    except Exception as e:
        print(f"[ControlDeskRW] FAILED during write/restore: {e}", flush=True)
        return False


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Listen for timer-callback events from the VEOS IPC bridge client."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Local host to bind")
    parser.add_argument("--port", type=int, default=50555, help="Local TCP port to bind")
    parser.add_argument(
        "--time-trigger-ack-delay-s",
        type=float,
        default=0.0,
        help="Delay before replying ACK to TIME_TRIGGER (default: 0.0s).",
    )
    parser.add_argument(
        "--print-every-n",
        type=int,
        default=0,
        help="Print timing line every N TIME_TRIGGER events; 0 disables per-trigger logs (default: 0).",
    )
    parser.add_argument(
        "--no-auto-launch-client",
        action="store_true",
        help="Do not auto-launch VeosCoSimTestClientIpc.exe.",
    )
    parser.add_argument(
        "--ipc-client-exe",
        default=None,
        help="Path to VeosCoSimTestClientIpc.exe (default: ../client/build/VeosCoSimTestClientIpc.exe).",
    )
    parser.add_argument("--veos-host", default="192.168.100.101", help="VEOS host for IPC client.")
    parser.add_argument("--veos-name", default="CoSimServerScenic", help="VEOS CoSim server name.")
    parser.add_argument(
        "--check-controldesk",
        action="store_true",
        help="Try ControlDesk connect using Scenic's connect_and_prepare path.",
    )
    parser.add_argument(
        "--check-modeldesk",
        action="store_true",
        help="Try ModelDesk COM connect using scenic.simulators.dspace.modeldesk.connection.ModelDeskConnection.",
    )
    parser.add_argument(
        "--modeldesk-create-and-download",
        action="store_true",
        help="After ModelDesk connect, SaveAs a working scenario, add one fellow, and Download to VEOS.",
    )
    parser.add_argument(
        "--modeldesk-post-connect-delay-s",
        type=float,
        default=10.0,
        help="Wait time after ModelDesk connect and before scenario copy/download (default: 10.0s).",
    )
    parser.add_argument(
        "--post-modeldesk-download-delay-s",
        type=float,
        default=30.0,
        help="Wait time after successful ModelDesk download before ControlDesk connect/go_online (default: 30.0s).",
    )
    parser.add_argument(
        "--scenario-src",
        default=SCENIC_DEFAULT_SCENARIO_SRC,
        help=(
            "Source scenario name to activate before SaveAs "
            f"(default: Scenic model default '{SCENIC_DEFAULT_SCENARIO_SRC}')."
        ),
    )
    parser.add_argument(
        "--scenario-name",
        default=None,
        help="Name for SaveAs scenario (default: Scenic_veos_test_YYYYMMDD_HHMMSS).",
    )
    parser.add_argument(
        "--fellow-name",
        default="F1",
        help="Fellow name to add/configure in ModelDesk scenario (default: F1).",
    )
    parser.add_argument(
        "--no-maneuver-reset",
        action="store_true",
        help="Skip ManeuverControl.Reset() after successful Download.",
    )
    parser.add_argument(
        "--blocking-controldesk-check",
        action="store_true",
        help="Run ControlDesk check inside TIME_TRIGGER callback path (legacy blocking mode). Default is non-blocking background mode.",
    )
    parser.add_argument(
        "--first-trigger-controldesk-delay-s",
        type=float,
        default=10.0,
        help="When --check-controldesk is enabled, sleep this long before ControlDesk connect (default: 10.0s).",
    )
    parser.add_argument(
        "--post-controldesk-delay-s",
        type=float,
        default=0.0,
        help="When --check-controldesk is enabled, sleep this long after ControlDesk connect attempt before write test/timing (default: 0.0s).",
    )
    parser.add_argument(
        "--enable-controldesk-rw-test",
        action="store_true",
        help="When --check-controldesk is enabled, run read/write smoke test (default: disabled).",
    )
    parser.add_argument(
        "--post-rw-delay-s",
        type=float,
        default=0.0,
        help="When RW smoke test runs, wait this long afterwards before stress timing starts (default: 0.0s).",
    )
    parser.add_argument(
        "--external-control",
        action="store_true",
        help="Use scenic_control=False when checking ControlDesk (apply external baseline path).",
    )
    parser.add_argument(
        "--timestep",
        type=float,
        default=0.01,
        help="Timestep used in ControlDesk check via set_simulation_step (default: 0.01).",
    )
    args = parser.parse_args()
    ack_delay = max(0.0, float(args.time_trigger_ack_delay_s))
    print_every_n = int(args.print_every_n)
    first_trigger_cd_delay_s = max(0.0, float(args.first_trigger_controldesk_delay_s))
    post_controldesk_delay_s = max(0.0, float(args.post_controldesk_delay_s))
    post_rw_delay_s = max(0.0, float(args.post_rw_delay_s))
    auto_launch_client = not bool(args.no_auto_launch_client)
    client_proc = None

    print(f"Starting local IPC listener on {args.host}:{args.port} ...", flush=True)
    tick_count = 0
    last_wall_elapsed_s = 0.0
    last_sim_elapsed_s = 0.0
    saw_sim_time = False

    def _print_summary(reason: str):
        if tick_count <= 0:
            print(f"[STRESS_SUMMARY] reason={reason} ticks=0 (no TIME_TRIGGER samples)", flush=True)
            return
        wall_elapsed_s = max(0.0, float(last_wall_elapsed_s))
        sim_elapsed_s = max(0.0, float(last_sim_elapsed_s))
        avg_wall_tick_s = wall_elapsed_s / max(1, tick_count)
        avg_hz = (1.0 / avg_wall_tick_s) if avg_wall_tick_s > 0 else float("inf")
        sim_hz = (tick_count / sim_elapsed_s) if sim_elapsed_s > 0 else 0.0
        print(
            "[STRESS_SUMMARY] "
            f"reason={reason} ticks={tick_count} sim_elapsed_s={sim_elapsed_s:.6f} "
            f"wall_elapsed_s={wall_elapsed_s:.6f} "
            f"avg_wall_tick_s={avg_wall_tick_s:.6f} "
            f"avg_wall_tick_hz={avg_hz:.2f} "
            f"sim_tick_hz={sim_hz:.2f}",
            flush=True,
        )
        if saw_sim_time:
            print(
                "[STRESS_SUMMARY] Target 10ms timestep => expected sim_dt_s ~= 0.010000",
                flush=True,
            )
        else:
            print(
                "[STRESS_SUMMARY] sim_time was not present in callbacks; sim_elapsed_s stayed at 0.",
                flush=True,
            )

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)

            if auto_launch_client:
                exe_path = Path(args.ipc_client_exe) if args.ipc_client_exe else _default_ipc_client_exe()
                if not exe_path.is_file():
                    print(
                        f"ERROR: IPC client executable not found: {exe_path}",
                        file=sys.stderr,
                        flush=True,
                    )
                    return 1
                cmd = [
                    str(exe_path),
                    "--host",
                    str(args.veos_host),
                    "--name",
                    str(args.veos_name),
                    "--ipc-host",
                    str(args.host),
                    "--ipc-port",
                    str(args.port),
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
                print(
                    f"Stress test config: ack_delay={ack_delay:.3f}s, "
                    f"print_every_n={print_every_n}",
                    flush=True,
                )
                buf = b""
                base_wall_s = None
                base_sim_ns = None
                last_sim_ns = None
                controldesk_checked = False
                cd_workflow_done = threading.Event()

                def _run_controldesk_workflow():
                    nonlocal controldesk_checked
                    scenic_control = not bool(args.external_control)
                    try:
                        if first_trigger_cd_delay_s > 0.0:
                            print(
                                "[ControlDeskCheck] Pre-connect pause "
                                f"{first_trigger_cd_delay_s:.3f}s ...",
                                flush=True,
                            )
                            time.sleep(first_trigger_cd_delay_s)
                        print(
                            f"[ControlDeskCheck] Attempting Scenic-like ControlDesk connect "
                            f"(scenic_control={scenic_control}, timestep={float(args.timestep):.3f}) ...",
                            flush=True,
                        )
                        print(
                            '[ControlDeskCheck] COM dispatch starting: "ControlDeskNG.Application"',
                            flush=True,
                        )
                        cd = _check_controldesk_like_scenic(
                            timestep=float(args.timestep),
                            scenic_control=scenic_control,
                        )
                        if cd is None:
                            print("[ControlDeskCheck] FAILED (see messages above).", flush=True)
                        else:
                            print(
                                "[ControlDeskCheck] OK: connect_and_prepare succeeded "
                                "(online, measurement started, VESI/baseline, timestep applied).",
                                flush=True,
                            )
                        if post_controldesk_delay_s > 0.0:
                            print(
                                "[ControlDeskCheck] Post-connect pause "
                                f"{post_controldesk_delay_s:.3f}s before write test ...",
                                flush=True,
                            )
                            time.sleep(post_controldesk_delay_s)
                        if cd is not None and scenic_control and bool(args.enable_controldesk_rw_test):
                            print(
                                "[ControlDeskRW] Running Scenic-control read/write smoke test ...",
                                flush=True,
                            )
                            rw_ok = _controldesk_rw_smoke(cd)
                            print("[ControlDeskRW] OK" if rw_ok else "[ControlDeskRW] FAILED", flush=True)
                            if post_rw_delay_s > 0.0:
                                print(
                                    f"[ControlDeskRW] Post-write settle pause {post_rw_delay_s:.3f}s ...",
                                    flush=True,
                                )
                                time.sleep(post_rw_delay_s)
                        controldesk_checked = True
                        print("[ControlDeskCheck] Workflow complete.", flush=True)
                    except Exception as e:
                        print(f"[ControlDeskCheck] UNHANDLED ERROR: {e}", flush=True)
                        print(traceback.format_exc(), flush=True)
                    finally:
                        cd_workflow_done.set()

                if args.check_controldesk and not args.blocking_controldesk_check:
                    print(
                        "[ControlDeskCheck] Starting non-blocking background workflow "
                        "(TIME_TRIGGER callbacks continue).",
                        flush=True,
                    )
                    threading.Thread(
                        target=_run_controldesk_workflow,
                        name="ControlDeskCheckWorker",
                        daemon=True,
                    ).start()
                elif args.check_modeldesk:
                    def _run_modeldesk_workflow():
                        try:
                            if first_trigger_cd_delay_s > 0.0:
                                print(
                                    "[ModelDeskCheck] Pre-connect pause "
                                    f"{first_trigger_cd_delay_s:.3f}s ...",
                                    flush=True,
                                )
                                time.sleep(first_trigger_cd_delay_s)
                            print(
                                '[ModelDeskCheck] COM dispatch starting: "ModelDesk.Application"',
                                flush=True,
                            )
                            conn_md = _check_modeldesk_connection()
                            proj = getattr(conn_md, "proj", None)
                            exp = getattr(conn_md, "exp", None)
                            proj_name = getattr(proj, "Name", "?") if proj is not None else "?"
                            exp_name = getattr(exp, "Name", "?") if exp is not None else "?"
                            print(
                                f"[ModelDeskCheck] OK: connected (project={proj_name}, experiment={exp_name}).",
                                flush=True,
                            )
                            if bool(args.modeldesk_create_and_download):
                                delay_s = max(0.0, float(args.modeldesk_post_connect_delay_s))
                                if delay_s > 0.0:
                                    print(
                                        f"[ModelDeskDownload] Post-connect pause {delay_s:.3f}s before SaveAs/copy/download ...",
                                        flush=True,
                                    )
                                    time.sleep(delay_s)
                                ok, msg = _modeldesk_create_scenario_add_fellow_and_download(
                                    conn_md,
                                    scenario_src=args.scenario_src,
                                    scenario_name=args.scenario_name,
                                    fellow_name=args.fellow_name,
                                    maneuver_reset=not bool(args.no_maneuver_reset),
                                )
                                if ok:
                                    print(f"[ModelDeskDownload] OK: {msg}", flush=True)
                                    delay_after_download = max(0.0, float(args.post_modeldesk_download_delay_s))
                                    if delay_after_download > 0.0:
                                        print(
                                            "[ControlDeskAfterModelDesk] Waiting "
                                            f"{delay_after_download:.3f}s before ControlDesk connect/go_online ...",
                                            flush=True,
                                        )
                                        time.sleep(delay_after_download)
                                    ok_cd, msg_cd = _connect_controldesk_go_online()
                                    if ok_cd:
                                        print(f"[ControlDeskAfterModelDesk] OK: {msg_cd}", flush=True)
                                    else:
                                        print(f"[ControlDeskAfterModelDesk] FAILED: {msg_cd}", flush=True)
                                else:
                                    print(f"[ModelDeskDownload] FAILED: {msg}", flush=True)
                        except Exception as e:
                            print(f"[ModelDeskCheck] ERROR: {e}", flush=True)
                            print(traceback.format_exc(), flush=True)

                    print(
                        "[ModelDeskCheck] Starting non-blocking background workflow "
                        "(TIME_TRIGGER callbacks continue).",
                        flush=True,
                    )
                    threading.Thread(
                        target=_run_modeldesk_workflow,
                        name="ModelDeskCheckWorker",
                        daemon=True,
                    ).start()

                while True:
                    chunk = conn.recv(4096)
                    if not chunk:
                        print("IPC bridge disconnected.", flush=True)
                        _print_summary("disconnect_before_target")
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
                        sim_time: Optional[int] = obj.get("sim_time")
                        count = obj.get("count")
                        source = obj.get("source")

                        if event == "TIME_TRIGGER":
                            if args.check_controldesk and args.blocking_controldesk_check and not controldesk_checked:
                                scenic_control = not bool(args.external_control)
                                print(
                                    "[ControlDeskCheck] First TIME_TRIGGER received. "
                                    f"Pausing {first_trigger_cd_delay_s:.3f}s before Scenic-like ControlDesk connect ...",
                                    flush=True,
                                )
                                if first_trigger_cd_delay_s > 0.0:
                                    time.sleep(first_trigger_cd_delay_s)
                                print(
                                    f"[ControlDeskCheck] Attempting Scenic-like ControlDesk connect "
                                    f"(scenic_control={scenic_control}, timestep={float(args.timestep):.3f}) ...",
                                    flush=True,
                                )
                                print(
                                    '[ControlDeskCheck] COM dispatch starting: "ControlDeskNG.Application"',
                                    flush=True,
                                )
                                cd = _check_controldesk_like_scenic(
                                    timestep=float(args.timestep),
                                    scenic_control=scenic_control,
                                )
                                if cd is None:
                                    print("[ControlDeskCheck] FAILED (see messages above).", flush=True)
                                else:
                                    print(
                                        "[ControlDeskCheck] OK: connect_and_prepare succeeded "
                                        "(online, measurement started, VESI/baseline, timestep applied).",
                                        flush=True,
                                    )
                                if post_controldesk_delay_s > 0.0:
                                    print(
                                        "[ControlDeskCheck] Post-connect pause "
                                        f"{post_controldesk_delay_s:.3f}s before write test and stress timing ...",
                                        flush=True,
                                    )
                                    time.sleep(post_controldesk_delay_s)
                                if cd is not None and scenic_control and bool(args.enable_controldesk_rw_test):
                                    print(
                                        "[ControlDeskRW] Running Scenic-control read/write smoke test ...",
                                        flush=True,
                                    )
                                    rw_ok = _controldesk_rw_smoke(cd)
                                    if rw_ok:
                                        print("[ControlDeskRW] OK", flush=True)
                                    else:
                                        print("[ControlDeskRW] FAILED", flush=True)
                                    if post_rw_delay_s > 0.0:
                                        print(
                                            f"[ControlDeskRW] Post-write settle pause {post_rw_delay_s:.3f}s ...",
                                            flush=True,
                                        )
                                        time.sleep(post_rw_delay_s)
                                controldesk_checked = True
                                # Release this warmup trigger after pre/post delays and ControlDesk probe,
                                # but do not include it in timing statistics.
                                if ack_delay > 0.0:
                                    time.sleep(ack_delay)
                                conn.sendall(b"STEP\n")
                                print(
                                    "[ControlDeskCheck] Warmup complete; stress timing begins on next TIME_TRIGGER.",
                                    flush=True,
                                )
                                continue

                            tick_count += 1

                            now_wall_s = time.perf_counter()
                            sim_ns = None
                            if sim_time is not None:
                                try:
                                    sim_ns = int(sim_time)
                                    saw_sim_time = True
                                except (TypeError, ValueError):
                                    sim_ns = None

                            if base_wall_s is None:
                                base_wall_s = now_wall_s
                                if sim_ns is not None:
                                    base_sim_ns = sim_ns
                                last_sim_ns = sim_ns

                            wall_elapsed_s = (
                                (now_wall_s - base_wall_s) if base_wall_s is not None else 0.0
                            )
                            sim_elapsed_s = 0.0
                            sim_dt_s = None
                            if sim_ns is not None and base_sim_ns is not None:
                                sim_elapsed_s = (sim_ns - base_sim_ns) / TIME_RESOLUTION_PER_SECOND
                            if sim_ns is not None and last_sim_ns is not None:
                                sim_dt_s = (sim_ns - last_sim_ns) / TIME_RESOLUTION_PER_SECOND
                            last_sim_ns = sim_ns
                            last_wall_elapsed_s = wall_elapsed_s
                            last_sim_elapsed_s = sim_elapsed_s

                            if print_every_n > 0 and (tick_count % print_every_n) == 0:
                                if sim_dt_s is None:
                                    print(
                                        f"[TIME_TRIGGER] tick={tick_count} source={source} count={count} "
                                        f"sim_elapsed_s={sim_elapsed_s:.6f} wall_elapsed_s={wall_elapsed_s:.6f}",
                                        flush=True,
                                    )
                                else:
                                    print(
                                        f"[TIME_TRIGGER] tick={tick_count} source={source} count={count} "
                                        f"sim_elapsed_s={sim_elapsed_s:.6f} wall_elapsed_s={wall_elapsed_s:.6f} "
                                        f"sim_dt_s={sim_dt_s:.6f}",
                                        flush=True,
                                    )

                            if ack_delay > 0.0:
                                time.sleep(ack_delay)
                            conn.sendall(b"STEP\n")

                        else:
                            print(f"[{event}] {obj}", flush=True)
                            conn.sendall(b"ACK\n")

        return 0

    except KeyboardInterrupt:
        print("Interrupted; shutting down.", flush=True)
        _print_summary("keyboard_interrupt")
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