from __future__ import annotations

import argparse
import json
import socket
import sys
from typing import Optional


def main() -> int:
    parser = argparse.ArgumentParser(
        description="Listen for timer-callback events from the VEOS IPC bridge client."
    )
    parser.add_argument("--host", default="127.0.0.1", help="Local host to bind")
    parser.add_argument("--port", type=int, default=50555, help="Local TCP port to bind")
    args = parser.parse_args()

    print(f"Starting local IPC listener on {args.host}:{args.port} ...", flush=True)

    try:
        with socket.socket(socket.AF_INET, socket.SOCK_STREAM) as server:
            server.setsockopt(socket.SOL_SOCKET, socket.SO_REUSEADDR, 1)
            server.bind((args.host, args.port))
            server.listen(1)

            print("Waiting for IPC bridge client to connect...", flush=True)
            conn, addr = server.accept()

            with conn:
                print(f"IPC bridge connected from {addr[0]}:{addr[1]}", flush=True)
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
                            continue

                        event = obj.get("event", "UNKNOWN")
                        sim_time: Optional[int] = obj.get("sim_time")
                        count = obj.get("count")

                        if event == "TIME_TRIGGER":
                            if count is not None:
                                print(f"[TIME_TRIGGER] count={count} sim_time={sim_time}", flush=True)
                            else:
                                print(f"[TIME_TRIGGER] sim_time={sim_time}", flush=True)
                        else:
                            print(f"[{event}] {obj}", flush=True)

        return 0

    except OSError as exc:
        print(f"ERROR: {exc}", file=sys.stderr, flush=True)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())
