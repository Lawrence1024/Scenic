#!/usr/bin/env python3
"""
Talk to an already-running VeosCoSimTestClient bridge over localhost TCP.

Start C++ client first:
  VeosCoSimTestClient.exe --host 192.168.100.101 --name CoSimServerScenic --bridge-port 17071

Then run:
  python src/scenic/simulators/dspace/cosim/pythonbinding/scripts/cpp_client_bridge_demo.py --port 17071
"""

from __future__ import annotations

import argparse
import socket


def recv_line(sock: socket.socket) -> str:
    data = bytearray()
    while True:
        chunk = sock.recv(1)
        if not chunk:
            raise ConnectionError("bridge closed connection")
        if chunk == b"\n":
            return data.decode("utf-8", errors="replace").rstrip("\r")
        data.extend(chunk)


def send_line(sock: socket.socket, line: str) -> None:
    sock.sendall((line + "\n").encode("utf-8"))


def main() -> None:
    parser = argparse.ArgumentParser(description="Drive running C++ VeosCoSim client bridge.")
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=17071)
    parser.add_argument("--steps", type=int, default=10)
    args = parser.parse_args()

    with socket.create_connection((args.host, args.port), timeout=10.0) as sock:
        hello = recv_line(sock)
        print(hello)
        send_line(sock, "PING")
        print(recv_line(sock))
        print("Requesting simulation steps from running C++ client...")
        for i in range(args.steps):
            send_line(sock, "STEP")
            line = recv_line(sock)
            print(f"{i + 1:03d}: {line}")
        send_line(sock, "QUIT")
        print(recv_line(sock))


if __name__ == "__main__":
    main()

