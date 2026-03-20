from __future__ import annotations

import os
import struct
import time

from .backends import make_backend, VeosCoSimError
from .types import Command, Result, DataType, Direction, SizeKind, SCALAR_FORMATS


class VeosCoSimClient:
    def __init__(
        self,
        *,
        backend: str = "bridge",
        remote_ip: str,
        server_name: str | None = None,
        remote_port: int | None = None,
        local_port: int | None = None,
        bridge_dll: str | None = None,
        lib_path: str | None = None,
        portmapper_port: int | None = None,
    ):
        self.remote_ip = remote_ip
        self.server_name = server_name
        self.remote_port = remote_port
        self.local_port = local_port
        self.portmapper_port = portmapper_port

        if self.portmapper_port is not None:
            os.environ["VEOSCOSIM_PORTMAPPER_PORT"] = str(int(self.portmapper_port))

        self._backend = make_backend(backend, lib_path=lib_path, bridge_dll=bridge_dll)
        self._backend.create()
        self._connected = False
        self._on_start = None
        self._on_stop = None
        self._on_terminate = None
        self._on_time_trigger = None

    def __enter__(self):
        return self

    def __exit__(self, exc_type, exc, tb):
        self.close()

    def register_callbacks(self, *, on_start=None, on_stop=None, on_terminate=None, on_time_trigger=None):
        self._on_start = on_start
        self._on_stop = on_stop
        self._on_terminate = on_terminate
        self._on_time_trigger = on_time_trigger

    def connect(self) -> None:
        if self._connected:
            return
        self._backend.connect(self.remote_ip, self.server_name, self.remote_port, self.local_port)
        self._connected = True

    def disconnect(self) -> None:
        if self._connected:
            self._backend.disconnect()
            self._connected = False

    def close(self) -> None:
        try:
            self.disconnect()
        finally:
            self._backend.destroy()

    def start_nonblocking(self) -> None:
        if not self._connected:
            raise VeosCoSimError("connect() must be called before start_nonblocking()")
        self._backend.start_nonblocking()

    def poll_once(self, timeout_sleep: float = 0.01):
        result, sim_time, cmd = self._backend.get_next_command()
        if result == int(Result.EMPTY):
            time.sleep(timeout_sleep)
            return None, None
        if result != int(Result.OK):
            raise VeosCoSimError(f"get_next_command failed with result={result}")
        return sim_time, Command(cmd)

    def finish_command(self) -> None:
        self._backend.finish_command()

    def dispatch_command(self, sim_time: int, cmd: Command) -> None:
        if cmd == Command.START and self._on_start:
            self._on_start(sim_time)
        elif cmd == Command.STOP and self._on_stop:
            self._on_stop(sim_time)
        elif cmd == Command.TERMINATE and self._on_terminate:
            self._on_terminate(sim_time)
        elif cmd == Command.TIME_TRIGGER and self._on_time_trigger:
            self._on_time_trigger(sim_time)

    def list_signals(self):
        infos = self._backend.list_signals()
        out = []
        for s in infos:
            out.append({
                "id": int(s.id),
                "length": int(s.length),
                "dataType": DataType(int(s.dataType)),
                "direction": Direction(int(s.direction)),
                "sizeKind": SizeKind(int(s.sizeKind)),
                "name": s.name.decode("utf-8", errors="replace").rstrip("\x00"),
            })
        return out

    def read_bytes(self, signal_id: int, length: int) -> bytes:
        return self._backend.read_bytes(signal_id, length)

    def write_bytes(self, signal_id: int, data: bytes) -> None:
        self._backend.write_bytes(signal_id, data)

    def read_scalar(self, signal_id: int, data_type: DataType):
        fmt = SCALAR_FORMATS.get(DataType(data_type))
        if fmt is None:
            raise VeosCoSimError(f"Unsupported scalar data type: {data_type}")
        data = self.read_bytes(signal_id, struct.calcsize(fmt))
        return struct.unpack("<" + fmt, data)[0]

    def write_scalar(self, signal_id: int, value, data_type: DataType) -> None:
        fmt = SCALAR_FORMATS.get(DataType(data_type))
        if fmt is None:
            raise VeosCoSimError(f"Unsupported scalar data type: {data_type}")
        self.write_bytes(signal_id, struct.pack("<" + fmt, value))
