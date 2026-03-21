#!/usr/bin/env python3
"""Smoke test: optional CoSim IPC + ModelDesk COM (same paths as Scenic dSPACE).

**ModelDesk** (COM): ``Dispatch("ModelDesk.Application")`` → active project / experiment
(see ``ModelDeskConnection``).

**CoSim** (VEOS): In practice VEOS / ModelDesk may need the IPC-enabled client connected
before the stack is ready. That matches the flow in ``src/scenic/simulators/dspace/cosim/README.md``:

1. Start ``SyncStepBridge`` on localhost (same as ``DSpaceSimulation.setup()``).
2. Launch ``VeosCoSimTestClientIpc.exe`` — it connects to the bridge first, then to VEOS.

Run from repo root (Windows), with ``src`` on PYTHONPATH::

  set PYTHONPATH=src
  python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py --with-cosim

ModelDesk-only (no CoSim subprocess)::

  python src/scenic/simulators/dspace/modeldesk_connection_test/test_modeldesk_connection.py

Or as a module::

  python -m scenic.simulators.dspace.modeldesk_connection_test.test_modeldesk_connection --with-cosim

SaveAs + Download to VEOS (like ``DSpaceSimulation.setup()``)::

  python .../test_modeldesk_connection.py --with-cosim --test-veos-download
"""

from __future__ import annotations

import argparse
import subprocess
import sys
import threading
import time
from pathlib import Path

import pythoncom


def _ensure_import() -> None:
    try:
        import scenic.simulators.dspace.modeldesk.connection  # noqa: F401
        return
    except ImportError:
        pass
    repo_root = Path(__file__).resolve()
    for _ in range(4):
        repo_root = repo_root.parent
    src = repo_root / "src"
    if src.is_dir():
        sys.path.insert(0, str(src))


_ensure_import()

from scenic.simulators.dspace.modeldesk.connection import ModelDeskConnection


def _dspace_dir() -> Path:
    return Path(__file__).resolve().parent.parent


def _default_cosim_exe() -> Path:
    return (
        _dspace_dir()
        / "cosim"
        / "veos_cosim_ipc_bridge"
        / "client"
        / "build"
        / "VeosCoSimTestClientIpc.exe"
    )


def _safe_name(obj, default: str = "?") -> str:
    if obj is None:
        return default
    return str(getattr(obj, "Name", default))


def test_save_as_and_download_to_veos(
    conn: ModelDeskConnection,
    *,
    scenario_src: str | None,
    new_name: str | None,
    maneuver_reset: bool = True,
) -> tuple[bool, str]:
    """Mirror ``DSpaceSimulation.setup()`` traffic scenario + download to VEOS.

    Steps (see ``simulator.py`` and ``modeldesk/scenario.py``):
    1. ``ActivateTrafficScenario(scenario_src)`` (best-effort)
    2. ``SaveAs(new_name, True)`` (fallback via ``EditTrafficScenario``)
    3. ``ActivateTrafficScenario(new_name)``
    4. Rebind handles, ``PumpWaitingMessages`` + short sleep
    5. ``TrafficScenario.Save()`` then ``TrafficScenario.Download()``
    6. Optional ``ManeuverControl.Reset()`` like ``setup()`` after download
    """
    app = conn.app
    exp = conn.exp
    if app is None or exp is None:
        return False, "ModelDeskConnection has no app/experiment."

    ts0 = exp.TrafficScenario
    if ts0 is None:
        return False, "Active experiment has no TrafficScenario."

    source = scenario_src if scenario_src is not None else _safe_name(ts0, "Unknown")
    name = new_name or time.strftime("Scenic_veos_test_%Y%m%d_%H%M%S")

    print(
        f"[modeldesk_connection_test] VEOS download test: Activate source='{source}' "
        f"-> SaveAs -> Activate '{name}' -> Save/Download ..."
    )

    try:
        exp.ActivateTrafficScenario(source)
    except Exception as e:
        print(f"[modeldesk_connection_test]   ActivateTrafficScenario({source!r}) (ignored): {e}")

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

    print(f"[modeldesk_connection_test]   Active TrafficScenario: {_safe_name(ts)}")

    try:
        consistent = ts.CheckConsistency()
        print(f"[modeldesk_connection_test]   CheckConsistency() = {consistent}")
    except Exception as e:
        print(f"[modeldesk_connection_test]   CheckConsistency() (skipped): {e}")

    try:
        ts.Save()
        print("[modeldesk_connection_test]   Save() OK.")
    except Exception as e:
        return False, f"TrafficScenario.Save() failed: {e}"

    try:
        ok = ts.Download()
    except Exception as e:
        return False, f"TrafficScenario.Download() failed: {e}"

    if ok:
        print("[modeldesk_connection_test]   Download() returned True — scenario sent to VEOS.")
    else:
        print(
            "[modeldesk_connection_test]   Download() returned False — VEOS may not have updated.",
            flush=True,
        )
        return False, "TrafficScenario.Download() returned False"

    time.sleep(0.1)
    if maneuver_reset:
        try:
            exp.ManeuverControl.Reset()
            print("[modeldesk_connection_test]   ManeuverControl.Reset() OK (after download).")
        except Exception as e:
            print(f"[modeldesk_connection_test]   ManeuverControl.Reset() (warn): {e}")

    return True, f"Working copy '{name}' saved and downloaded."


def _run_cosim_stack(
    *,
    cosim_exe: Path,
    veos_host: str,
    veos_name: str,
    ipc_host: str,
    ipc_port: int,
    connect_timeout: float,
    auto_step: bool,
) -> tuple[object, subprocess.Popen | None, threading.Event | None, threading.Thread | None]:
    """Start SyncStepBridge, spawn IPC client, optionally pump STEPs in a daemon thread."""
    from scenic.simulators.dspace.cosim.veos_cosim_ipc_bridge.python_listener.sync_step_bridge import (
        SyncStepBridge,
    )

    bridge = SyncStepBridge(host=ipc_host, port=ipc_port)
    print(
        f"[modeldesk_connection_test] Starting SyncStepBridge on {ipc_host}:{ipc_port} "
        "(same as Scenic CoSim sync) ..."
    )
    bridge.start()
    time.sleep(0.05)

    if not cosim_exe.is_file():
        bridge.close()
        raise FileNotFoundError(
            f"IPC client not found: {cosim_exe}\n"
            "Build it with: cosim\\veos_cosim_ipc_bridge\\build_client.bat\n"
            "Or pass --cosim-exe PATH"
        )

    cmd = [
        str(cosim_exe),
        "--host",
        veos_host,
        "--name",
        veos_name,
        "--ipc-host",
        ipc_host,
        "--ipc-port",
        str(ipc_port),
    ]
    print(f"[modeldesk_connection_test] Launching: {' '.join(cmd)}")
    proc = subprocess.Popen(
        cmd,
        cwd=str(cosim_exe.parent),
        creationflags=subprocess.CREATE_NEW_CONSOLE if sys.platform == "win32" else 0,
    )

    print(
        f"[modeldesk_connection_test] Waiting for IPC client to connect (timeout {connect_timeout}s) ..."
    )
    try:
        bridge.wait_connected(timeout=connect_timeout)
    except Exception as e:
        proc.terminate()
        try:
            proc.wait(timeout=5)
        except Exception:
            pass
        bridge.close()
        raise RuntimeError(
            "IPC client did not connect to SyncStepBridge. "
            "Check VEOS host/name, firewall, and that the EXE matches the bridge protocol."
        ) from e

    print("[modeldesk_connection_test] IPC client connected to SyncStepBridge.")

    stop_step = None
    step_thread = None
    if auto_step:

        def _pump() -> None:
            assert stop_step is not None
            while not stop_step.is_set():
                try:
                    bridge.step(timeout=2.0)
                except TimeoutError:
                    continue
                except Exception as exc:
                    print(f"[modeldesk_connection_test] auto-step thread exit: {exc}", flush=True)
                    break

        stop_step = threading.Event()
        step_thread = threading.Thread(target=_pump, name="SyncStepPump", daemon=True)
        step_thread.start()
        print(
            "[modeldesk_connection_test] auto-step thread running (releases TIME_TRIGGER via bridge.step)."
        )

    return bridge, proc, stop_step, step_thread


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Test ModelDesk COM; optionally start CoSim IPC client + SyncStepBridge first."
    )
    parser.add_argument(
        "--with-cosim",
        action="store_true",
        help="Start SyncStepBridge and VeosCoSimTestClientIpc.exe before ModelDesk (recommended if VEOS needs the client).",
    )
    parser.add_argument(
        "--cosim-exe",
        type=Path,
        default=None,
        help=f"Path to VeosCoSimTestClientIpc.exe (default: {_default_cosim_exe()})",
    )
    parser.add_argument("--veos-host", default="192.168.100.101", help="VEOS / CoSim server host")
    parser.add_argument(
        "--veos-name",
        default="CoSimServerScenic",
        help="CoSim server name (must match VEOS / cosim_server_scenic.json)",
    )
    parser.add_argument("--ipc-host", default="127.0.0.1", help="SyncStepBridge bind address")
    parser.add_argument("--ipc-port", type=int, default=50555, help="SyncStepBridge port (Scenic default)")
    parser.add_argument(
        "--ipc-connect-timeout",
        type=float,
        default=120.0,
        help="Seconds to wait for the IPC client to connect to SyncStepBridge",
    )
    parser.add_argument(
        "--auto-step",
        action="store_true",
        help="Background thread: call bridge.step() so TIME_TRIGGER is released (use if simulation starts and would otherwise block).",
    )
    parser.add_argument(
        "--verbose",
        "-v",
        action="store_true",
        help="Print extra COM details (best-effort).",
    )
    parser.add_argument(
        "--test-veos-download",
        action="store_true",
        help=(
            "After COM connect: SaveAs a new working copy, activate it, Save + Download to VEOS "
            "(same pattern as DSpaceSimulation.setup())."
        ),
    )
    parser.add_argument(
        "--scenario-src",
        default=None,
        help="Traffic scenario to activate before SaveAs (default: current TrafficScenario name).",
    )
    parser.add_argument(
        "--scenario-name",
        default=None,
        help="New scenario name for SaveAs (default: Scenic_veos_test_YYYYMMDD_HHMMSS).",
    )
    parser.add_argument(
        "--no-maneuver-reset",
        action="store_true",
        help="Skip ManeuverControl.Reset() after Download (setup() normally calls it).",
    )
    args = parser.parse_args()

    cosim_exe = args.cosim_exe or _default_cosim_exe()
    bridge = None
    proc: subprocess.Popen | None = None
    stop_step: threading.Event | None = None
    step_thread: threading.Thread | None = None

    if args.with_cosim:
        try:
            bridge, proc, stop_step, step_thread = _run_cosim_stack(
                cosim_exe=cosim_exe,
                veos_host=args.veos_host,
                veos_name=args.veos_name,
                ipc_host=args.ipc_host,
                ipc_port=args.ipc_port,
                connect_timeout=args.ipc_connect_timeout,
                auto_step=args.auto_step,
            )
        except FileNotFoundError as e:
            print(f"[modeldesk_connection_test] FAILED: {e}", file=sys.stderr)
            return 1
        except RuntimeError as e:
            print(f"[modeldesk_connection_test] FAILED: {e}", file=sys.stderr)
            return 1

    print("[modeldesk_connection_test] Connecting via ModelDesk.Application ...")
    try:
        conn = ModelDeskConnection().connect()
    except RuntimeError as e:
        print(f"[modeldesk_connection_test] FAILED: {e}", file=sys.stderr)
        return 1
    except Exception as e:
        print(f"[modeldesk_connection_test] FAILED: {e!r}", file=sys.stderr)
        return 1

    proj = conn.proj
    exp = conn.exp
    print("[modeldesk_connection_test] OK — ModelDesk COM connection succeeded.")
    print(f"  Project:    {_safe_name(proj)}")
    print(f"  Experiment: {_safe_name(exp)}")

    try:
        ts = conn.get_traffic_scenario()
        if ts is None:
            print("  TrafficScenario: (none)")
        else:
            print(f"  TrafficScenario: {_safe_name(ts)}")
    except Exception as e:
        print(f"  TrafficScenario: (could not read: {e})")

    if args.verbose and proj is not None:
        try:
            ap = getattr(proj, "ActiveProject", None)
            print(f"  [verbose] proj.ActiveProject repr: {ap!r}")
        except Exception as e:
            print(f"  [verbose] ActiveProject: {e}")
        try:
            print(f"  [verbose] exp type: {type(exp)}")
        except Exception:
            pass

    exit_code = 0
    if args.test_veos_download:
        ok, msg = test_save_as_and_download_to_veos(
            conn,
            scenario_src=args.scenario_src,
            new_name=args.scenario_name,
            maneuver_reset=not args.no_maneuver_reset,
        )
        if ok:
            print(f"[modeldesk_connection_test] VEOS download test: {msg}")
        else:
            print(f"[modeldesk_connection_test] VEOS download test FAILED: {msg}", file=sys.stderr)
            exit_code = 2

    print("[modeldesk_connection_test] Done.")

    # Cleanup CoSim stack
    if stop_step is not None:
        stop_step.set()
    if bridge is not None:
        bridge.close()
    if proc is not None:
        print("[modeldesk_connection_test] Terminating IPC client process ...", flush=True)
        proc.terminate()
        try:
            proc.wait(timeout=8)
        except subprocess.TimeoutExpired:
            proc.kill()
    if step_thread is not None and step_thread.is_alive():
        step_thread.join(timeout=2.0)

    return exit_code


if __name__ == "__main__":
    sys.exit(main())
