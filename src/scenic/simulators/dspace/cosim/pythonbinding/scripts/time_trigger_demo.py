#!/usr/bin/env python3
"""
Manual demo: connect to a running VEOS co-simulation server and handle each simulation
step in a Python *time_trigger* callback (VeosCoSim_RunMI).

**Before running**, install the binding from this directory (so ``import veos_cosim`` works)::

    cd src/scenic/simulators/dspace/cosim/pythonbinding
    pip install -e .

Then start your VEOS co-simulation server (same host/name as your C++ test client).

**Run** from anywhere (not inside ``pythonbinding/`` alone, or the package folder can shadow
the built extension; the repo root is safe)::

    python src/scenic/simulators/dspace/cosim/pythonbinding/scripts/time_trigger_demo.py

Or with options::

    python .../time_trigger_demo.py --host 192.168.100.101 --name CoSimServerScenic

On Windows, the script adds the directory containing ``VeosCoSimAppl.dll`` to the DLL search
path (``os.add_dll_directory``). Override with ``--dll-dir`` if your layout differs.

Expected behavior (with a healthy server and reachable network)
-------------------------------------------------------------
1. The script connects and prints how many IO signals and bus channels were discovered.
2. When the simulation **starts**, you see ``Simulation start t=...``.
3. On **every** simulation tick (e.g. 10 ms if server sample time is 0.01 s), the time trigger
   runs; the script increments an internal step counter. **Only every 0.05 s** it also prints a
   ``[control]`` line (same decimation idea as Scenic’s 0.05 s control period).
4. When the simulation **stops**, you see ``Simulation stop t=...`` and the run returns.
5. The script prints the integer **VeosCoSim_Result** returned by ``VeosCoSim_RunMI``. Per dSPACE
   documentation bundled with the client, this value **may still indicate Error even when the run
   was normal**—use logs and whether callbacks ran as the real success signal.

If the server is down or the host is wrong, ``connect()`` raises with a connection failure.
"""

from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

# --- DLL path must be set before importing the native extension -----------------------------
_SCRIPT_DIR = Path(__file__).resolve().parent
_BINDING_ROOT = _SCRIPT_DIR.parent
_DEFAULT_DLL_DIR = (
    _BINDING_ROOT.parent / "VeosCoSim_Client" / "client" / "x64" / "Debug" / "lib"
)

TIME_RES_NS = 1_000_000_000
CONTROL_INTERVAL_NS = 50_000_000  # 0.05 s simulated time between "[control]" prints


def _add_dll_directory(path: Path) -> None:
    if not path.is_dir():
        print(f"ERROR: DLL directory does not exist: {path}", file=sys.stderr)
        print(
            "  Pass --dll-dir pointing to the folder that contains VeosCoSimAppl.dll.",
            file=sys.stderr,
        )
        sys.exit(2)
    if hasattr(os, "add_dll_directory"):
        os.add_dll_directory(str(path))
    else:
        print("WARNING: os.add_dll_directory unavailable; ensure VeosCoSimAppl.dll is on PATH.")


def main() -> None:
    parser = argparse.ArgumentParser(
        description="VeosCoSim Python demo: blocking run() with time_trigger callback.",
    )
    parser.add_argument(
        "--host",
        default="192.168.100.101",
        help="Co-sim server host (default: %(default)s)",
    )
    parser.add_argument(
        "--name",
        default="CoSimServerScenic",
        help="Co-sim server name (default: %(default)s)",
    )
    parser.add_argument(
        "--dll-dir",
        type=Path,
        default=None,
        help=f"Directory containing VeosCoSimAppl.dll (default: {_DEFAULT_DLL_DIR})",
    )
    parser.add_argument(
        "--remote-port",
        type=int,
        default=0,
        help="Explicit VeosCoSim server TCP port (0: resolve by name/port mapper).",
    )
    parser.add_argument(
        "--local-port",
        type=int,
        default=0,
        help="Optional local client TCP port for tunneled setups (0: auto).",
    )
    args = parser.parse_args()

    dll_dir = args.dll_dir or _DEFAULT_DLL_DIR
    _add_dll_directory(dll_dir.resolve())

    # Import after DLL path is configured
    from veos_cosim import CoSimClient

    client = CoSimClient()
    if args.remote_port or args.local_port:
        print(
            f"Connecting (connect2) to {args.host!r} / {args.name!r} "
            f"remote_port={args.remote_port} local_port={args.local_port} ..."
        )
        client.connect2(
            args.host,
            args.name,
            remote_port=int(args.remote_port),
            local_port=int(args.local_port),
        )
    else:
        print(f"Connecting to {args.host!r} / {args.name!r} ...")
        client.connect(args.host, args.name)
    print("Connected.\n")

    signals = client.io_signals()
    channels = client.channels()
    print(f"IO signals: {len(signals)}")
    for s in signals[:8]:
        print(f"  - {s['name']!r} id={s['id']} direction={s['direction']} length={s['length']}")
    if len(signals) > 8:
        print(f"  ... and {len(signals) - 8} more")
    print(f"Bus channels: {len(channels)}")
    for c in channels:
        print(f"  - {c['controller_name']!r} id={c['id']} protocol={c['bus_protocol']}")
    print()

    step_count = 0
    last_control_time_ns: int | None = None

    def on_start(sim_time_ns: int) -> None:
        print(f"Simulation start  t={sim_time_ns / TIME_RES_NS:.6f} s")

    def on_stop(sim_time_ns: int) -> None:
        print(f"Simulation stop   t={sim_time_ns / TIME_RES_NS:.6f} s")

    def on_time_trigger(sim_time_ns: int) -> None:
        nonlocal step_count, last_control_time_ns
        step_count += 1
        # Match Scenic-style 0.05 s control cadence on a typical 0.01 s co-sim step.
        if last_control_time_ns is None or (sim_time_ns - last_control_time_ns) >= CONTROL_INTERVAL_NS:
            last_control_time_ns = sim_time_ns
            print(
                f"[control] tick #{step_count}  t={sim_time_ns / TIME_RES_NS:.6f} s  "
                f"(time_trigger; co-sim waits until this returns)"
            )

    print("Starting VeosCoSim_RunMI (blocking until simulation ends)...\n")
    result_code = client.run(on_time_trigger, on_start=on_start, on_stop=on_stop)
    print()
    print(f"Run finished. VeosCoSim_RunMI returned result code: {result_code}")
    print(
        "(Per dSPACE docs, this may be Error even on success; see pythonbinding/README.md.)"
    )
    print(f"Total time_trigger invocations: {step_count}")


if __name__ == "__main__":
    main()
