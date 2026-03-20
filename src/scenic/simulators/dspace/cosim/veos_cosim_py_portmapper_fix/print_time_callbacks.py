from __future__ import annotations

import argparse
import os
import sys
from pathlib import Path

ROOT = Path(__file__).resolve().parent
SRC = ROOT / "src"
if str(SRC) not in sys.path:
    sys.path.insert(0, str(SRC))


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description="Print each VEOS time-trigger callback.")
    parser.add_argument("--backend", choices=["bridge", "direct"], default="bridge")
    parser.add_argument("--remote-ip", default="192.168.100.101")
    parser.add_argument("--server-name", default="CoSimServerScenic")
    parser.add_argument("--remote-port", type=int, default=None)
    parser.add_argument("--local-port", type=int, default=None)
    parser.add_argument("--portmapper-port", type=int, default=None,
                        help="Sets VEOSCOSIM_PORTMAPPER_PORT before connecting. Required when the VEOS port mapper does not use the default port 27072.")
    parser.add_argument("--bridge-dll", default=None,
                        help=r'Path to veos_cosim_bridge.dll. Default: .\bridge\build\Release\veos_cosim_bridge.dll')
    parser.add_argument("--lib-path", default=None,
                        help=r'Optional override for VeosCoSimAppl.dll when backend=direct')
    parser.add_argument("--idle-sleep", type=float, default=0.01)
    return parser


def main() -> int:
    parser = build_parser()
    args = parser.parse_args()

    if args.portmapper_port is not None:
        os.environ["VEOSCOSIM_PORTMAPPER_PORT"] = str(int(args.portmapper_port))

    from veos_cosim_py import VeosCoSimClient, Command

    count = 0

    def on_time_trigger(sim_time: int) -> None:
        nonlocal count
        count += 1
        print(f"[TIME_TRIGGER] count={count} sim_time={sim_time}", flush=True)

    print(
        f"Connecting with backend={args.backend!r}, remote_ip={args.remote_ip!r}, "
        f"server_name={args.server_name!r}, remote_port={args.remote_port!r}, "
        f"local_port={args.local_port!r}, portmapper_port={os.environ.get('VEOSCOSIM_PORTMAPPER_PORT')!r}, "
        f"lib_path={args.lib_path!r}",
        flush=True,
    )

    try:
        with VeosCoSimClient(
            backend=args.backend,
            remote_ip=args.remote_ip,
            server_name=args.server_name,
            remote_port=args.remote_port,
            local_port=args.local_port,
            portmapper_port=args.portmapper_port,
            bridge_dll=args.bridge_dll,
            lib_path=args.lib_path,
        ) as client:
            client.register_callbacks(
                on_start=lambda t: print(f"[START] sim_time={t}", flush=True),
                on_stop=lambda t: print(f"[STOP] sim_time={t}", flush=True),
                on_terminate=lambda t: print(f"[TERMINATE] sim_time={t}", flush=True),
                on_time_trigger=on_time_trigger,
            )
            client.connect()
            client.start_nonblocking()
            print("Connected. Waiting for commands...", flush=True)

            while True:
                sim_time, cmd = client.poll_once(timeout_sleep=args.idle_sleep)
                if cmd is None:
                    continue
                client.dispatch_command(sim_time, cmd)
                client.finish_command()
                if cmd == Command.TERMINATE:
                    break

    except KeyboardInterrupt:
        print("\nInterrupted by user.", flush=True)
        return 130
    except Exception as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1

    print("Exited cleanly.", flush=True)
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
