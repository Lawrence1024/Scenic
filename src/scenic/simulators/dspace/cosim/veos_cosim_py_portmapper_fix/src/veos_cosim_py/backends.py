from __future__ import annotations

import ctypes as C
import os
from pathlib import Path

from .types import Result, VeosCoSim_IoSignalInfo, VeosCoSim_GeneralInfo

LOG_CALLBACK = C.CFUNCTYPE(None, C.c_int32, C.c_char_p)

class VeosCoSimError(RuntimeError):
    pass

def _check(result: int, what: str) -> None:
    if int(result) != int(Result.OK):
        raise VeosCoSimError(f"{what} failed with result={int(result)}")

def _package_root_from_file(path: Path) -> Path:
    return path.resolve().parents[2]

def _vendor_root_candidates() -> list[Path]:
    pkg_root = _package_root_from_file(Path(__file__))
    parent = pkg_root.parent
    return [
        parent / 'VeosCoSim_Client' / 'client' / 'x64' / 'Release',
        parent / 'VeosCoSim_Client' / 'VeosCoSim_Client' / 'client' / 'x64' / 'Release',
        pkg_root / 'VeosCoSim_Client' / 'client' / 'x64' / 'Release',
        pkg_root / 'VeosCoSim_Client' / 'VeosCoSim_Client' / 'client' / 'x64' / 'Release',
    ]

def _resolve_vendor_root() -> Path:
    for cand in _vendor_root_candidates():
        if (cand / 'include' / 'VeosCoSim.h').exists() and ((cand / 'lib' / 'VeosCoSimAppl.dll').exists() or (cand / 'lib' / 'VeosCoSimApplStatic.lib').exists()):
            return cand
    searched = '\n'.join(str(p) for p in _vendor_root_candidates())
    raise VeosCoSimError('Could not locate sibling VeosCoSim_Client release folder. Searched:\n' + searched)

def default_vendor_dll() -> Path:
    return _resolve_vendor_root() / 'lib' / 'VeosCoSimAppl.dll'

def default_bridge_dll() -> Path:
    pkg_root = _package_root_from_file(Path(__file__))
    return pkg_root / 'bridge' / 'build' / 'Release' / 'veos_cosim_bridge.dll'

class DirectBackend:
    def __init__(self, lib_path: str | None = None):
        dll = Path(lib_path) if lib_path else default_vendor_dll()
        if not dll.exists():
            raise VeosCoSimError(f"Vendor DLL not found: {dll}")
        if os.name == 'nt':
            os.add_dll_directory(str(dll.parent))
            self.lib = C.WinDLL(str(dll))
        else:
            self.lib = C.CDLL(str(dll))
        self._handle = None
        self._log_cb = None
        self._bind()

    def _bind(self) -> None:
        lib = self.lib
        lib.VeosCoSim_CreateMI.restype = C.c_void_p
        lib.VeosCoSim_DestroyMI.argtypes = [C.c_void_p]
        lib.VeosCoSim_DestroyMI.restype = None
        lib.VeosCoSim_ConnectMI.argtypes = [C.c_void_p, C.c_char_p, C.c_char_p, LOG_CALLBACK]
        lib.VeosCoSim_ConnectMI.restype = C.c_int32

        class ConnectCfg(C.Structure):
            _fields_ = [
                ('remoteIpAddress', C.c_char_p),
                ('name', C.c_char_p),
                ('logCallback', LOG_CALLBACK),
                ('remotePort', C.c_uint16),
                ('localPort', C.c_uint16),
            ]
        self.ConnectCfg = ConnectCfg
        lib.VeosCoSim_ConnectMI2.argtypes = [C.c_void_p, ConnectCfg]
        lib.VeosCoSim_ConnectMI2.restype = C.c_int32
        lib.VeosCoSim_DisconnectMI.argtypes = [C.c_void_p]
        lib.VeosCoSim_DisconnectMI.restype = C.c_int32

        class RuntimeCfg(C.Structure):
            _fields_ = [
                ('startSimulationCallback', C.c_void_p),
                ('stopSimulationCallback', C.c_void_p),
                ('terminateSimulationCallback', C.c_void_p),
                ('timeTriggerCallback', C.c_void_p),
                ('ioReadCallback', C.c_void_p),
                ('canMessageReceivedCallback', C.c_void_p),
                ('linMessageReceivedCallback', C.c_void_p),
                ('ethMessageReceivedCallback', C.c_void_p),
                ('userData', C.c_void_p),
            ]
        self.RuntimeCfg = RuntimeCfg
        lib.VeosCoSim_StartNonBlockingMI.argtypes = [C.c_void_p, RuntimeCfg]
        lib.VeosCoSim_StartNonBlockingMI.restype = C.c_int32
        lib.VeosCoSim_GetNextCommandMI.argtypes = [C.c_void_p, C.POINTER(C.c_int64), C.POINTER(C.c_int32)]
        lib.VeosCoSim_GetNextCommandMI.restype = C.c_int32
        lib.VeosCoSim_FinishCommandMI.argtypes = [C.c_void_p]
        lib.VeosCoSim_FinishCommandMI.restype = C.c_int32
        lib.VeosCoSim_IoGetAvailableSignalsMI.argtypes = [C.c_void_p, C.POINTER(C.c_uint32), C.POINTER(C.POINTER(VeosCoSim_IoSignalInfo))]
        lib.VeosCoSim_IoGetAvailableSignalsMI.restype = C.c_int32
        lib.VeosCoSim_IoReadMI.argtypes = [C.c_void_p, C.c_uint32, C.POINTER(C.c_uint32), C.c_void_p]
        lib.VeosCoSim_IoReadMI.restype = C.c_int32
        lib.VeosCoSim_IoWriteMI.argtypes = [C.c_void_p, C.c_uint32, C.c_uint32, C.c_void_p]
        lib.VeosCoSim_IoWriteMI.restype = C.c_int32
        lib.VeosCoSim_GetGeneralInfoMI.argtypes = [C.c_void_p, C.POINTER(VeosCoSim_GeneralInfo)]
        lib.VeosCoSim_GetGeneralInfoMI.restype = C.c_int32

    def create(self) -> None:
        self._handle = self.lib.VeosCoSim_CreateMI()
        if not self._handle:
            raise VeosCoSimError('VeosCoSim_CreateMI returned null')

    def destroy(self) -> None:
        if self._handle:
            self.lib.VeosCoSim_DestroyMI(self._handle)
            self._handle = None

    def connect(self, remote_ip: str, server_name: str | None, remote_port: int | None, local_port: int | None) -> None:
        @LOG_CALLBACK
        def log_cb(severity, message):
            text = message.decode('utf-8', errors='replace') if message else ''
            print(f'[veos/{severity}] {text}', flush=True)
        self._log_cb = log_cb
        if remote_port is not None or local_port is not None:
            cfg = self.ConnectCfg(remote_ip.encode('utf-8'), server_name.encode('utf-8') if server_name else None, self._log_cb, int(remote_port or 0), int(local_port or 0))
            result = self.lib.VeosCoSim_ConnectMI2(self._handle, cfg)
            if int(result) != int(Result.OK):
                raise VeosCoSimError(f"connect (direct, MI2) failed with result={int(result)} remote_ip={remote_ip!r} server_name={server_name!r} remote_port={remote_port!r} local_port={local_port!r}")
            return
        if not server_name:
            raise VeosCoSimError('server_name is required for ConnectMI')
        result = self.lib.VeosCoSim_ConnectMI(self._handle, remote_ip.encode('utf-8'), server_name.encode('utf-8'), self._log_cb)
        if int(result) != int(Result.OK):
            raise VeosCoSimError(f"connect (direct) failed with result={int(result)} remote_ip={remote_ip!r} server_name={server_name!r}")

    def disconnect(self) -> None:
        _check(self.lib.VeosCoSim_DisconnectMI(self._handle), 'disconnect')
    def start_nonblocking(self) -> None:
        _check(self.lib.VeosCoSim_StartNonBlockingMI(self._handle, self.RuntimeCfg()), 'start_nonblocking')
    def get_next_command(self):
        sim_time = C.c_int64(0)
        cmd = C.c_int32(0)
        result = self.lib.VeosCoSim_GetNextCommandMI(self._handle, C.byref(sim_time), C.byref(cmd))
        return int(result), int(sim_time.value), int(cmd.value)
    def finish_command(self) -> None:
        _check(self.lib.VeosCoSim_FinishCommandMI(self._handle), 'finish_command')
    def list_signals(self):
        count = C.c_uint32(0)
        infos = C.POINTER(VeosCoSim_IoSignalInfo)()
        _check(self.lib.VeosCoSim_IoGetAvailableSignalsMI(self._handle, C.byref(count), C.byref(infos)), 'io_get_available_signals')
        return [infos[i] for i in range(count.value)]
    def read_bytes(self, signal_id: int, length: int) -> bytes:
        n = C.c_uint32(length)
        buf = (C.c_ubyte * length)()
        _check(self.lib.VeosCoSim_IoReadMI(self._handle, int(signal_id), C.byref(n), C.byref(buf)), 'io_read')
        return bytes(buf[: n.value])
    def write_bytes(self, signal_id: int, data: bytes) -> None:
        n = len(data)
        buf = (C.c_ubyte * n).from_buffer_copy(data)
        _check(self.lib.VeosCoSim_IoWriteMI(self._handle, int(signal_id), n, C.byref(buf)), 'io_write')
    def get_general_info(self):
        info = VeosCoSim_GeneralInfo()
        _check(self.lib.VeosCoSim_GetGeneralInfoMI(self._handle, C.byref(info)), 'get_general_info')
        return info

class BridgeBackend:
    def __init__(self, bridge_dll: str | None = None):
        dll = Path(bridge_dll) if bridge_dll else default_bridge_dll()
        if not dll.exists():
            raise VeosCoSimError(f"Bridge DLL not found: {dll}. Build it first with build_bridge.bat")
        if os.name == 'nt':
            os.add_dll_directory(str(dll.parent))
            self.lib = C.WinDLL(str(dll))
        else:
            self.lib = C.CDLL(str(dll))
        self._handle = None
        self._bind()

    def _bind(self) -> None:
        lib = self.lib
        lib.vcp_create.restype = C.c_void_p
        lib.vcp_destroy.argtypes = [C.c_void_p]
        lib.vcp_destroy.restype = None
        lib.vcp_connect.argtypes = [C.c_void_p, C.c_char_p, C.c_char_p]
        lib.vcp_connect.restype = C.c_int32
        lib.vcp_connect2.argtypes = [C.c_void_p, C.c_char_p, C.c_char_p, C.c_uint16, C.c_uint16]
        lib.vcp_connect2.restype = C.c_int32
        lib.vcp_disconnect.argtypes = [C.c_void_p]
        lib.vcp_disconnect.restype = C.c_int32
        lib.vcp_start_nonblocking.argtypes = [C.c_void_p]
        lib.vcp_start_nonblocking.restype = C.c_int32
        lib.vcp_get_next_command.argtypes = [C.c_void_p, C.POINTER(C.c_int64), C.POINTER(C.c_int32)]
        lib.vcp_get_next_command.restype = C.c_int32
        lib.vcp_finish_command.argtypes = [C.c_void_p]
        lib.vcp_finish_command.restype = C.c_int32
        lib.vcp_io_get_available_signals.argtypes = [C.c_void_p, C.POINTER(C.c_uint32), C.POINTER(C.POINTER(VeosCoSim_IoSignalInfo))]
        lib.vcp_io_get_available_signals.restype = C.c_int32
        lib.vcp_io_read.argtypes = [C.c_void_p, C.c_uint32, C.POINTER(C.c_uint32), C.c_void_p]
        lib.vcp_io_read.restype = C.c_int32
        lib.vcp_io_write.argtypes = [C.c_void_p, C.c_uint32, C.c_uint32, C.c_void_p]
        lib.vcp_io_write.restype = C.c_int32
        lib.vcp_get_general_info.argtypes = [C.c_void_p, C.POINTER(VeosCoSim_GeneralInfo)]
        lib.vcp_get_general_info.restype = C.c_int32

    def create(self) -> None:
        self._handle = self.lib.vcp_create()
        if not self._handle:
            raise VeosCoSimError('vcp_create returned null')
    def destroy(self) -> None:
        if self._handle:
            self.lib.vcp_destroy(self._handle)
            self._handle = None
    def connect(self, remote_ip: str, server_name: str | None, remote_port: int | None, local_port: int | None) -> None:
        if remote_port is not None or local_port is not None:
            result = self.lib.vcp_connect2(self._handle, remote_ip.encode('utf-8'), server_name.encode('utf-8') if server_name else None, int(remote_port or 0), int(local_port or 0))
            if int(result) != int(Result.OK):
                raise VeosCoSimError(f"connect (bridge, MI2) failed with result={int(result)} remote_ip={remote_ip!r} server_name={server_name!r} remote_port={remote_port!r} local_port={local_port!r}")
            return
        if not server_name:
            raise VeosCoSimError('server_name is required for bridge ConnectMI')
        result = self.lib.vcp_connect(self._handle, remote_ip.encode('utf-8'), server_name.encode('utf-8'))
        if int(result) != int(Result.OK):
            raise VeosCoSimError(f"connect (bridge) failed with result={int(result)} remote_ip={remote_ip!r} server_name={server_name!r}")
    def disconnect(self) -> None:
        _check(self.lib.vcp_disconnect(self._handle), 'disconnect')
    def start_nonblocking(self) -> None:
        _check(self.lib.vcp_start_nonblocking(self._handle), 'start_nonblocking')
    def get_next_command(self):
        sim_time = C.c_int64(0)
        cmd = C.c_int32(0)
        result = self.lib.vcp_get_next_command(self._handle, C.byref(sim_time), C.byref(cmd))
        return int(result), int(sim_time.value), int(cmd.value)
    def finish_command(self) -> None:
        _check(self.lib.vcp_finish_command(self._handle), 'finish_command')
    def list_signals(self):
        count = C.c_uint32(0)
        infos = C.POINTER(VeosCoSim_IoSignalInfo)()
        _check(self.lib.vcp_io_get_available_signals(self._handle, C.byref(count), C.byref(infos)), 'io_get_available_signals')
        return [infos[i] for i in range(count.value)]
    def read_bytes(self, signal_id: int, length: int) -> bytes:
        n = C.c_uint32(length)
        buf = (C.c_ubyte * length)()
        _check(self.lib.vcp_io_read(self._handle, int(signal_id), C.byref(n), C.byref(buf)), 'io_read')
        return bytes(buf[: n.value])
    def write_bytes(self, signal_id: int, data: bytes) -> None:
        n = len(data)
        buf = (C.c_ubyte * n).from_buffer_copy(data)
        _check(self.lib.vcp_io_write(self._handle, int(signal_id), n, C.byref(buf)), 'io_write')
    def get_general_info(self):
        info = VeosCoSim_GeneralInfo()
        _check(self.lib.vcp_get_general_info(self._handle, C.byref(info)), 'get_general_info')
        return info

def make_backend(name: str, *, lib_path: str | None = None, bridge_dll: str | None = None):
    lowered = name.strip().lower()
    if lowered == 'direct':
        return DirectBackend(lib_path=lib_path)
    if lowered == 'bridge':
        return BridgeBackend(bridge_dll=bridge_dll)
    raise VeosCoSimError(f'Unknown backend: {name}')
