import datetime
import os
from pathlib import Path
import platform
import sys
from collections.abc import Callable, Iterator
from contextlib import contextmanager
from ctypes import POINTER, byref, c_int32, c_int64, c_uint32, c_uint8
from typing import Iterable

import Bindings as c
from Bindings import (
    DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH,
    CanController,
    CanMessage,
    Command,
    ConnectConfig,
    ConnectionState,
    DLLConfiguration,
    EthController,
    EthMessage,
    Init_dll_callbacks,
    Io_Signal,
    LinController,
    LinMessage,
    Result,
    Severity,
    TerminateReason,
    VeosCoSimError,
    formatted_print,
    register_library,
)

lib_name = None
if sys.platform.startswith("win"):
    lib_name = "DsVeosCoSim.dll"
else:
    lib_name = "libDsVeosCoSim.so"

class CoSimClient:
    def __init__(self, lib_path: Path = None):
        if not lib_path:
            lib_path = Path(__file__).parent / lib_name

        self.lib = register_library(lib_path)

        self._tty_color_enabled = False
        self._controller_infos = {}
        self._io_signal_infos = None

        self.__handle = self.lib.DsVeosCoSim_Create()

    def __del__(self):
        self.lib.DsVeosCoSim_Destroy(self.__handle)

    @property
    def handle(self):
        return self.__handle

    @contextmanager
    def Connect(self, connectConfig: ConnectConfig):
        try:
            _log_callback = c._DLL_LogCallback_t(self.__LogCallback)
            self.lib.DsVeosCoSim_SetLogCallback(_log_callback)

            if self.lib.DsVeosCoSim_Connect(self.handle, connectConfig) == Result.ERROR:
                raise VeosCoSimError("VeosCoSim_Connect failed")

            yield CoSimConnection(self)
        finally:
            self.Disconnect()

    def __LogCallback(self, severity, msg):
        if sys.stderr.isatty():
            severity_format = c._SEVERITY_FMT_COLORS

            self._tty_color_enabled
            if not self._tty_color_enabled:
                if platform.system() == "Windows":
                    os.system("color")
                self._tty_color_enabled = True
        else:
            severity_format = c._SEVERITY_FMT_NOCOLORS

        ts = datetime.datetime.now().isoformat()
        svr = severity_format.get(severity, severity_format.get(-1))

        if isinstance(msg, bytes):
            msg = msg.decode("utf-8")
        else:
            msg = str(msg)
        sys.stdout.flush()

        print("{} {} {}".format(ts, svr, msg), file=sys.stderr)

    def GetConnectionState(self):
        connection_state = c_int32()
        self.lib.DsVeosCoSim_GetConnectionState(self.handle, byref(connection_state))

        return connection_state.value

    def Disconnect(self):
        self._controller_infos = None

        if self.GetConnectionState() == int(ConnectionState.DISCONNECTED):
            return Result.DISCONNECTED

        result = self.lib.DsVeosCoSim_Disconnect(self.handle)

        if result == Result.OK:
            formatted_print(Severity.INFO, "Successfully disconnected!")
        else:
            raise VeosCoSimError("VeosCoSim_Disconnect failed")


class CoSimConnection:
    def __init__(self, client: CoSimClient):
        self.client = client

    @property
    def handle(self):
        return self.client.handle

    @property
    def lib(self):
        return self.client.lib

    def SetNextSimulationTime(self, next_time):
        assert next_time >= 0

        result = self.lib.DsVeosCoSim_SetNextSimulationTime(self.handle, c_int64(next_time))

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_SetNextSimulationTime failed")
        return result

    def GetStepSize(self):
        step_size = c_int64()
        self.lib.DsVeosCoSim_GetStepSize(self.handle, byref(step_size))
        return step_size.value

    @staticmethod
    def _make_can_controller_info(info):
        canController = CanController()
        canController.id = int(info.id)
        canController.queue_size = int(info.queue_size)
        canController.bits_per_second = int(info.bits_per_second)
        canController.flexible_data_rate_bits_per_second = int(info.flexible_data_rate_bits_per_second)

        canController.name = info.name
        canController.channel_name = info.channel_name
        canController.cluster_name = info.cluster_name
        return canController

    def GetCanControllers(self):
        count = c_uint32()
        controller_infos = POINTER(CanController)()
        result = self.lib.DsVeosCoSim_GetCanControllers(self.handle, byref(count), byref(controller_infos))
        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_GetCanControllers failed")

        return [self._make_can_controller_info(controller_infos[i]) for i in range(count.value)]

    def GetEthControllers(self):
        count = c_uint32()
        controller_infos = POINTER(EthController)()
        result = self.lib.DsVeosCoSim_GetEthControllers(self.handle, byref(count), byref(controller_infos))
        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_GetEthControllers failed")

        return [self._make_eth_controller_info(controller_infos[i]) for i in range(count.value)]

    @staticmethod
    def _make_eth_controller_info(info):
        ethController = EthController()
        ethController.id = int(info.id)
        ethController.queue_size = int(info.queue_size)
        ethController.bits_per_second = int(info.bits_per_second)
        ethController.mac_address = info.mac_address
        ethController.name = info.name
        ethController.channel_name = info.channel_name
        ethController.cluster_name = info.cluster_name

        return ethController

    def GetLinControllers(self):
        count = c_uint32()
        controller_infos = POINTER(LinController)()
        result = self.lib.DsVeosCoSim_GetLinControllers(self.handle, byref(count), byref(controller_infos))
        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_GetLinControllers failed")

        return [self._make_lin_controller_info(controller_infos[i]) for i in range(count.value)]

    @staticmethod
    def _make_lin_controller_info(info):
        linController = LinController()
        linController.id = int(info.id)
        linController.queue_size = int(info.queue_size)
        linController.bits_per_second = int(info.bits_per_second)
        linController.type = int(info.type)
        linController.name = info.name
        linController.channel_name = info.channel_name
        linController.cluster_name = info.cluster_name
        return linController

    @staticmethod
    def _make_signals(dll_signal):
        inf = Io_Signal()
        inf.id = int(dll_signal.id)
        inf.length = int(dll_signal.length)
        inf.dataType = int(dll_signal.dataType)
        inf.sizeKind = int(dll_signal.sizeKind)
        inf.name = dll_signal.name

        return inf

    def GetIncomingSignals(self):
        count = c_uint32()
        incomingSignals = POINTER(Io_Signal)()

        result = self.lib.DsVeosCoSim_GetIncomingSignals(self.handle, byref(count), byref(incomingSignals))
        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_GetIncomingSignals failed")

        signals = {inf.id: inf for inf in (self._make_signals(incomingSignals[i]) for i in range(count.value))}
        return signals

    def GetOutgoingSignals(self):
        count = c_uint32()
        outgoingSignals = POINTER(Io_Signal)()

        result = self.lib.DsVeosCoSim_GetOutgoingSignals(self.handle, byref(count), byref(outgoingSignals))
        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_GetOutgoingSignals failed")

        signals = {inf.id: inf for inf in (self._make_signals(outgoingSignals[i]) for i in range(count.value))}
        return signals

    def RunCallbackBasedCoSimulation(self, config=DLLConfiguration()):
        self.dllcfg = Init_dll_callbacks(config)
        result = self.lib.DsVeosCoSim_RunCallbackBasedCoSimulation(self.handle, self.dllcfg)

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_RunCallbackBasedCoSimulation failed")

        return result

    def StartPollingBasedCoSimulation(self, config=DLLConfiguration()):
        self.dllcfg = Init_dll_callbacks(config)
        result = self.lib.DsVeosCoSim_StartPollingBasedCoSimulation(self.handle, self.dllcfg)

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_StartPollingBasedCoSimulation failed")

        return result

    def PollCommand(self):
        simulation_time = c_int64()
        command = c_int32(Command.TERMINATE.value)
        self.lib.DsVeosCoSim_PollCommand(self.handle, byref(simulation_time), byref(command))

        return simulation_time, command

    def FinishCommand(self):
        self.lib.DsVeosCoSim_FinishCommand(self.client.handle)

    def Commands(self) -> Iterator[(int, Command)]:
        while True:
            sim_time, command = self.PollCommand()
            command = Command(command.value)

            if command is Command.TERMINATE:
                return sim_time.value, command

            yield sim_time.value, command

    def AttachToSimulation(self, handle_command: Callable[[int, Command], None]):
        self.StartPollingBasedCoSimulation()
        session = CoSimSession(self)

        for rets in self.Commands():
            handle_command(session, *rets)
            self.FinishCommand()

    def StartSimulation(self):
        result = self.lib.DsVeosCoSim_StartSimulation(self.client.handle)
        if result != Result.OK:
            raise VeosCoSimError("Failed to start simulation.")

    def StopSimulation(self):
        result = self.lib.DsVeosCoSim_StopSimulation(self.client.handle)
        if result != Result.OK:
            raise VeosCoSimError("Failed to stop simulation.")

    def PauseSimulation(self):
        result = self.lib.DsVeosCoSim_PauseSimulation(self.client.handle)
        if result != Result.OK:
            raise VeosCoSimError("Failed to pause simulation.")

    def ContinueSimulation(self):
        result = self.lib.DsVeosCoSim_ContinueSimulation(self.client.handle)
        if result != Result.OK:
            raise VeosCoSimError("Failed to continue simulation.")

    def TerminateSimulation(self, terminate_reason=TerminateReason.FINISHED):
        result = self.lib.DsVeosCoSim_TerminateSimulation(self.handle, terminate_reason)
        if result != Result.OK:
            raise VeosCoSimError("Failed to terminate simulation.")


class CoSimSession:
    def __init__(self, connection: CoSimConnection):
        self.connection = connection

    @property
    def handle(self):
        return self.connection.client.handle

    @property
    def lib(self):
        return self.connection.client.lib
    
    @staticmethod
    def MakeCanMessage(data, id=0, flags=0) -> CanMessage:
        msg = CanMessage()
        msg.id = id
        msg.flags = flags

        msg.length = len(data)
        msg.data = (c_uint8 * msg.length)(*data)
        return msg


    def TransmitCanMessage(self, msg):
        result = self.lib.DsVeosCoSim_TransmitCanMessage(self.handle, byref(msg))

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_TransmitCanMessage failed")
        return result

    def ReceiveCanMessage(self):
        msg = CanMessage()
        result = self.lib.DsVeosCoSim_ReceiveCanMessage(self.handle, byref(msg))

        if result == Result.OK:
            return msg
        elif result == Result.EMPTY:
            return None

        raise VeosCoSimError("VeosCoSim_ReceiveCanMessage failed")

    def TransmitLinMessage(self, msg):
        result = self.lib.DsVeosCoSim_TransmitLinMessage(self.handle, byref(msg))

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_TransmitLinMessage failed")
        return result

    def ReceiveLinMessage(self):
        msg = LinMessage()
        result = self.lib.DsVeosCoSim_ReceiveLinMessage(self.handle, byref(msg))

        if result == Result.OK:
            return msg
        elif result == Result.EMPTY:
            return None

        raise VeosCoSimError("VeosCoSim_ReceiveLinMessage failed")

    @staticmethod
    def MakeEthMsg(controller: EthController, data: Iterable[bytes], flags=None) -> EthMessage:
        msg = EthMessage()
        msg.controller_id = controller.id

        if flags:
            msg.flags = flags

        assert len(data) <= DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH

        msg.length = len(data)
        msg.data = (c_uint8 * msg.length)(*data)

        return msg

    def TransmitEthMessage(self, msg):
        result = self.lib.DsVeosCoSim_TransmitEthMessage(self.handle, msg)

        if result == Result.ERROR:
            raise VeosCoSimError("VeosCoSim_TransmitEthMessage failed")
        return result

    def ReceiveEthMessage(self):
        msg = EthMessage()
        result = self.lib.DsVeosCoSim_ReceiveEthMessage(self.handle, byref(msg))

        if result == Result.OK:
            return msg
        elif result == Result.EMPTY:
            return None

        raise VeosCoSimError("VeosCoSim_ReceiveEthMessage failed")

    def ReadIncomingSignal(self, incomingSignalId) -> Iterable:
        length = c_uint32()
        inf = self.connection.GetIncomingSignals().get(incomingSignalId)

        if inf is None:
            raise VeosCoSimError("signal_id is invalid")

        arr = (c._DATATYPE_CTYPE_MAP.get(inf.dataType) * inf.length)()

        # Call the DLL function
        result = self.lib.DsVeosCoSim_ReadIncomingSignal(self.handle, incomingSignalId, byref(length), arr)

        if result != Result.OK:
            raise VeosCoSimError("VeosCoSim_ReadIncomingSignal failed")

        value = [arr[i] for i in range(length.value)]
        return value

    def WriteOutgoingSignal(self, signal_id, value) -> Result:
        inf = self.connection.GetOutgoingSignals().get(signal_id)

        if inf is None:
            raise VeosCoSimError("signal_id is invalid")

        if len(value) != inf.length:
            raise VeosCoSimError("Unexpected values length={} (expected={})".format(len(value), inf.length))

        arr = (c._DATATYPE_CTYPE_MAP.get(inf.dataType) * inf.length)(*value)

        result = self.lib.DsVeosCoSim_WriteOutgoingSignal(
            self.handle,
            c_uint32(signal_id),
            c_uint32(len(value)),
            arr,
        )
        if result != Result.OK:
            raise VeosCoSimError("VeosCoSim_WriteOutgoingSignal failed")

        return result
