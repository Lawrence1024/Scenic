import datetime
import os
import platform
import sys

from ctypes import (
    CDLL,
    CFUNCTYPE,
    POINTER,
    Structure,
    c_bool,
    c_char_p,
    c_double,
    c_float,
    c_int,
    c_int8,
    c_int16,
    c_int32,
    c_int64,
    c_uint8,
    c_uint16,
    c_uint32,
    c_uint64,
    c_void_p,
    cast,
)
from enum import Enum
from pathlib import Path


class VeosCoSimError(Exception):
    """Raised when a VeosComSim function returns Result.ERROR"""

    pass


def simulation_time_to_seconds(simulation_time):
    """
    Converts the given simulation time to a double in seconds.

    :param simulation_time: The simulation time to convert
    :return: The simulation time in seconds
    """
    return float(simulation_time) / _DLL_VeosCoSim_Simulation_Time_Resolution_Per_Second


def formatted_print(severity=2, *args):
    if sys.stderr.isatty():
        severity_format = _SEVERITY_FMT_COLORS
        if platform.system() == "Windows":
            os.system("color")
    else:
        severity_format = _SEVERITY_FMT_NOCOLORS

    ts = datetime.datetime.now().isoformat()
    svr = severity_format.get(severity, severity_format.get(-1))
    formatted_message = " ".join(map(str, args))

    sys.stdout.flush()
    print(f"{ts} {svr} {formatted_message}", file=sys.stderr)


_DLL_CoSimHandle = c_void_p
_DLL_CoSim_SimulationTime = c_int64
_DLL_VeosCoSim_Result = c_int32
_DLL_VeosCoSim_TerminateReason = c_int32
_DLL_IO_SIGNAL_ID = c_uint32
_DLL_VeosCoSim_Simulation_Time_Resolution_Per_Second = 1e9


class Result:
    OK = 0
    ERROR = 1
    EMPTY = 2
    FULL = 3
    INVALID_ARGUMENT = 4
    DISCONNECTED = 5
    # INT_MAX_SENTINEL_DO_NOT_USE = 2147483647 -- used in all enums


class Command(Enum):
    NONE = 0
    STEP = 1
    START = 2
    STOP = 3
    TERMINATE = 4
    PAUSE = 5
    CONTINUE = 6
    TERMINATE_FINISHED = 7
    PING = 8


class Severity:
    ERROR = 0
    WARNING = 1
    INFO = 2
    TRACE = 3


class TerminateReason:
    FINISHED = 0
    ERROR = 1


class ConnectionState:
    DISCONNECTED = 0
    CONNECTED = 1


class SimulationState:
    Unloaded = 0
    Stopped = 1
    Running = 2
    Paused = 3
    Terminated = 4


class Datatype:
    BOOL = 1
    INT8 = 2
    INT16 = 3
    INT32 = 4
    INT64 = 5
    UINT8 = 6
    UINT16 = 7
    UINT32 = 8
    UINT64 = 9
    FLOAT32 = 10
    FLOAT64 = 11


class SizeKind:
    FIXED = 1
    VARIABLE = 2


class CanMessageFlags:
    LOOPBACK = 1
    ERROR = 2
    DROP = 4
    EXTENDED_ID = 8
    BIT_RATE_SWITCH = 16
    FLEXIBLE_DATARATE_FORMAT = 32


class EthMessageFlags:
    LOOPBACK = 1
    ERROR = 2
    DROP = 4


class LinControllerType:
    RESPONDER = 1
    COMMANDER = 2


class LinMessageFlags:
    LOOPBACK = 1
    ERROR = 2
    DROP = 4
    HEADER = 8
    RESPONSE = 16
    WAKE_EVENT = 32
    SLEEP_EVENT = 64
    ENHANCED_CHECKSUM = 128
    TRANSFER_ONCE = 256
    PARITY_FAILURE = 512
    COLLISION = 1024
    NO_RESPONSE = 2048


class FrMessageFlags:
    LOOPBACK = 1
    ERROR = 2
    DROP = 4
    STARTUP = 8
    SYNC_FRAME = 16
    NULL_FRAME = 32
    PAYLOAD_PREAMBLE = 64
    TRANSFER_ONCE = 128
    CHANNEL_A = 256
    CHANNEL_B = 512

# struct DsVeosCoSim_IoSignal


class Io_Signal(Structure):
    _fields_ = [
        ("id", c_uint32),
        ("length", c_uint32),
        ("dataType", c_int32),
        ("sizeKind", c_int32),
        ("name", c_char_p),
    ]


# struct DsVeosCoSim_CanController


class CanController(Structure):
    _fields_ = [
        ("id", c_uint32),
        ("queue_size", c_uint32),
        ("bits_per_second", c_uint64),
        ("flexible_data_rate_bits_per_second", c_uint64),
        ("name", c_char_p),
        ("channel_name", c_char_p),
        ("cluster_name", c_char_p),
    ]


# struct DsVeosCoSim_CanMessage


class CanMessage(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", POINTER(c_uint8)),
    ]

    def FromController(self, controller: CanController):
        self.controller_id = controller.id

        if controller.flexible_data_rate_bits_per_second:
            self.flags = CanMessageFlags.BIT_RATE_SWITCH | CanMessageFlags.FLEXIBLE_DATARATE_FORMAT

        return self


# struct DsVeosCoSim_CanMessageContainer

DSVEOSCOSIM_CAN_MESSAGE_MAX_LENGTH = 64


class CanMessageContainer(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("reserved", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", (c_uint8 * DSVEOSCOSIM_CAN_MESSAGE_MAX_LENGTH)),
    ]


# struct DsVeosCoSim_EthController

DSVEOSCOSIM_ETH_ADDRESS_LENGTH = 6


class EthController(Structure):
    _fields_ = [
        ("id", c_uint32),
        ("queue_size", c_uint32),
        ("bits_per_second", c_uint64),
        ("mac_address", (c_uint8 * DSVEOSCOSIM_ETH_ADDRESS_LENGTH)),
        ("name", c_char_p),
        ("channel_name", c_char_p),
        ("cluster_name", c_char_p),
    ]


# struct DsVeosCoSim_EthMessage


class EthMessage(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("reserved", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", POINTER(c_uint8)),
    ]


# struct DsVeosCoSim_EthMessageContainer

DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH = 9018

class EthMessageContainer(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("reserved", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", (c_uint8 * DSVEOSCOSIM_ETH_MESSAGE_MAX_LENGTH)),
    ]


# struct DsVeosCoSim_LinController


class LinController(Structure):
    _fields_ = [
        ("id", c_uint32),
        ("queue_size", c_uint32),
        ("bits_per_second", c_uint64),
        ("type", c_int32),
        ("name", c_char_p),
        ("channel_name", c_char_p),
        ("cluster_name", c_char_p),
    ]


# struct DsVeosCoSim_LinMessage


class LinMessage(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", POINTER(c_uint8)),
    ]


# struct DsVeosCoSim_LinMessageContainer

DSVEOSCOSIM_LIN_MESSAGE_MAX_LENGTH = 8


class LinMessageContainer(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("reserved", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", (c_uint8 * DSVEOSCOSIM_LIN_MESSAGE_MAX_LENGTH)),
    ]


# struct DsVeosCoSim_FrController


class FrController(Structure):
    _fields_ = [
        ("id", c_uint32),
        ("queue_size", c_uint32),
        ("bits_per_second", c_uint64),
        ("name", c_char_p),
        ("channel_name", c_char_p),
        ("cluster_name", c_char_p),
    ]


# struct DsVeosCoSim_FrMessage


class FrMessage(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", POINTER(c_uint8)),
    ]



# struct DsVeosCoSim_FrMessageContainer

DSVEOSCOSIM_FLEXRAY_MESSAGE_MAX_LENGTH = 254


class FrMessageContainer(Structure):
    _fields_ = [
        ("timestamp", c_int64),
        ("controller_id", c_uint32),
        ("reserved", c_uint32),
        ("id", c_uint32),
        ("flags", c_uint32),
        ("length", c_uint32),
        ("data", (c_uint8 * DSVEOSCOSIM_FLEXRAY_MESSAGE_MAX_LENGTH)),
    ]


# typedef void (*DsVeosCoSim_LogCallback)(DsVeosCoSim_Severity severity, const char* logMessage);  // NOLINT
_DLL_LogCallback_t = CFUNCTYPE(None, c_int, c_char_p)

# typedef void (*DsVeosCoSim_SimulationCallback)(DsVeosCoSim_SimulationTime simulationTime, void* userData);  // NOLINT
_DLL_SimulationCallback_t = CFUNCTYPE(None, c_int64, c_void_p)

# typedef void (*DsVeosCoSim_SimulationTerminatedCallback)(DsVeosCoSim_SimulationTime simulationTime, DsVeosCoSim_TerminateReason reason, void* userData);
_DLL_SimulationTerminatedCallback_t = CFUNCTYPE(None, c_int64, c_int32, c_void_p)

# typedef void (*DsVeosCoSim_IncomingSignalChangedCallback)(DsVeosCoSim_SimulationTime simulationTime, const DsVeosCoSim_IoSignal* incomingSignal, uint32_t length, const void* value, void* userData);
_DLL_IncomingSignalChangedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_uint32, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_CanMessageReceivedCallback)(DsVeosCoSim_SimulationTime simulationTime,  // NOLINT const DsVeosCoSim_CanController* canController, const DsVeosCoSim_CanMessage* message, void* userData);
_DLL_CanMessageReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_EthMessageReceivedCallback)(DsVeosCoSim_SimulationTime simulationTime,  // NOLINT const DsVeosCoSim_EthController* ethController, const DsVeosCoSim_EthMessage* message, void* userData);
_DLL_EthMessageReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_LinMessageReceivedCallback)(DsVeosCoSim_SimulationTime simulationTime,  // NOLINT const DsVeosCoSim_LinController* linController, const DsVeosCoSim_LinMessage* message, void* userData);
_DLL_LinMessageReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_FrMessageReceivedCallback)(DsVeosCoSim_SimulationTime simulationTime,  // NOLINT const DsVeosCoSim_FrController* frController, const DsVeosCoSim_FrMessage* message, void* userData);
_DLL_FrMessageReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_CanMessageContainerReceivedCallback)(...)
_DLL_CanMessageContainerReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_EthMessageContainerReceivedCallback)(...)
_DLL_EthMessageContainerReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_LinMessageContainerReceivedCallback)(...)
_DLL_LinMessageContainerReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)

# typedef void (*DsVeosCoSim_FrMessageContainerReceivedCallback)(...)
_DLL_FrMessageContainerReceivedCallback_t = CFUNCTYPE(None, c_int64, c_void_p, c_void_p, c_void_p)


class _DLL_Callbacks(Structure):
    _fields_ = [
        ("simulationStartedCallback", _DLL_SimulationCallback_t),
        ("simulationStoppedCallback", _DLL_SimulationCallback_t),
        ("simulationTerminatedCallback", _DLL_SimulationTerminatedCallback_t),
        ("simulationPausedCallback", _DLL_SimulationCallback_t),
        ("simulationContinuedCallback", _DLL_SimulationCallback_t),
        ("simulationBeginStepCallback", _DLL_SimulationCallback_t),
        ("simulationEndStepCallback", _DLL_SimulationCallback_t),
        ("incomingSignalChangedCallback", _DLL_IncomingSignalChangedCallback_t),
        ("canMessageReceivedCallback", _DLL_CanMessageReceivedCallback_t),
        ("ethMessageReceivedCallback", _DLL_EthMessageReceivedCallback_t),
        ("linMessageReceivedCallback", _DLL_LinMessageReceivedCallback_t),
        ("userData", c_void_p),
        ("canMessageContainerReceivedCallback", _DLL_CanMessageContainerReceivedCallback_t),
        ("ethMessageContainerReceivedCallback", _DLL_EthMessageContainerReceivedCallback_t),
        ("linMessageContainerReceivedCallback", _DLL_LinMessageContainerReceivedCallback_t),
        ("frMessageContainerReceivedCallback", _DLL_FrMessageContainerReceivedCallback_t),
        ("frMessageReceivedCallback", _DLL_FrMessageReceivedCallback_t),
    ]


class DLLConfiguration(object):
    def __init__(self):
        self.on_start_simulation = None
        self.on_stop_simulation = None
        self.on_pause_simulation = None
        self.on_continue_simulation = None
        self.on_simulation_terminated = None
        self.on_incoming_signal_changed = None
        self.on_begin_step_simulation = None
        self.on_end_step_simulation = None
        self.on_can_message_received = None
        self.on_eth_message_received = None
        self.on_lin_message_received = None


class ConnectConfig(Structure):
    _fields_ = [
        (
            "remoteIpAddress",
            c_char_p,
        ),
        ("serverName", c_char_p),
        ("clientName", c_char_p),
        ("remotePort", c_uint16),
        ("localPort", c_uint16),
    ]

    def __init__(
        self,
        remoteIpAddress="",
        serverName="",
        clientName="Python extension",
        remotePort=None,
        localPort=None,
    ):
        valid_configs = (
            (remoteIpAddress == "" and serverName != ""),
            (remoteIpAddress != "" and remotePort),
            (remoteIpAddress != "" and serverName),
        )
        assert any(valid_configs)

        if remoteIpAddress:
            self.remoteIpAddress = remoteIpAddress.encode("utf-8")
        if serverName:
            self.serverName = serverName.encode("utf-8")
        if clientName:
            self.clientName = clientName.encode("utf-8")
        if remotePort:
            self.remotePort = remotePort
        if localPort:
            self.localPort = localPort


def register_library(absolute_path: Path) -> CDLL:
    _dll = CDLL(str(absolute_path))

    # DSVEOSCOSIM_DECL void DsVeosCoSim_SetLogCallback(DsVeosCoSim_LogCallback logCallback);
    _DLL_VeosCoSim_SetLogCallback = _dll.DsVeosCoSim_SetLogCallback
    _DLL_VeosCoSim_SetLogCallback.argtypes = [_DLL_LogCallback_t]
    _DLL_VeosCoSim_SetLogCallback.restype = None

    # DSVEOSCOSIM_DECL DsVeosCoSim_Handle DsVeosCoSim_Create(void);
    _DLL_Handle_Create = _dll.DsVeosCoSim_Create
    _DLL_Handle_Create.restype = c_void_p

    # DSVEOSCOSIM_DECL void DsVeosCoSim_Destroy(DsVeosCoSim_Handle handle);
    _DLL_Handle_Destroy = _dll.DsVeosCoSim_Destroy
    _DLL_Handle_Destroy.argtypes = [_DLL_CoSimHandle]

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_Connect(DsVeosCoSim_Handle handle, DsVeosCoSim_ConnectConfig connectConfig);
    _DLL_VeosCoSim_Connect = _dll.DsVeosCoSim_Connect
    _DLL_VeosCoSim_Connect.argtypes = [_DLL_CoSimHandle, ConnectConfig]
    _DLL_VeosCoSim_Connect.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_Disconnect(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_Disconnect = _dll.DsVeosCoSim_Disconnect
    _DLL_VeosCoSim_Disconnect.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_Disconnect.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetConnectionState(DsVeosCoSim_Handle handle, DsVeosCoSim_ConnectionState* connectionState);
    _DLL_VeosCoSim_GetConnectionState = _dll.DsVeosCoSim_GetConnectionState
    _DLL_VeosCoSim_GetConnectionState.argtypes = [_DLL_CoSimHandle, POINTER(c_int32)]
    _DLL_VeosCoSim_GetConnectionState.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_RunCallbackBasedCoSimulation(DsVeosCoSim_Handle handle, DsVeosCoSim_Callbacks callbacks);
    _DLL_VeosCoSim_RunCallbackBasedCoSimulation = _dll.DsVeosCoSim_RunCallbackBasedCoSimulation
    _DLL_VeosCoSim_RunCallbackBasedCoSimulation.argtypes = [
        _DLL_CoSimHandle,
        _DLL_Callbacks,
    ]
    _DLL_VeosCoSim_RunCallbackBasedCoSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_StartPollingBasedCoSimulation(DsVeosCoSim_Handle handle, DsVeosCoSim_Callbacks callbacks);
    _DLL_VeosCoSim_StartPollingBasedCoSimulation = _dll.DsVeosCoSim_StartPollingBasedCoSimulation
    _DLL_VeosCoSim_StartPollingBasedCoSimulation.argtypes = [
        _DLL_CoSimHandle,
        _DLL_Callbacks,
    ]
    _DLL_VeosCoSim_StartPollingBasedCoSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_PollCommand(DsVeosCoSim_Handle handle, DsVeosCoSim_SimulationTime* simulationTime, DsVeosCoSim_Command* command);
    _DLL_VeosCoSim_PollCommand = _dll.DsVeosCoSim_PollCommand
    _DLL_VeosCoSim_PollCommand.argtypes = [
        _DLL_CoSimHandle,
        POINTER(_DLL_CoSim_SimulationTime),
        POINTER(c_int32),
    ]
    _DLL_VeosCoSim_PollCommand.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_FinishCommand(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_FinishCommand = _dll.DsVeosCoSim_FinishCommand
    _DLL_VeosCoSim_FinishCommand.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_FinishCommand.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_SetNextSimulationTime(DsVeosCoSim_Handle handle, DsVeosCoSim_SimulationTime simulationTime);
    _DLL_VeosCoSim_SetNextSimulationTime = _dll.DsVeosCoSim_SetNextSimulationTime
    _DLL_VeosCoSim_SetNextSimulationTime.argtypes = [
        _DLL_CoSimHandle,
        _DLL_CoSim_SimulationTime,
    ]
    _DLL_VeosCoSim_SetNextSimulationTime.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetStepSize(DsVeosCoSim_Handle handle, DsVeosCoSim_SimulationTime* stepSize);
    _DLL_VeosCoSim_GetStepSize = _dll.DsVeosCoSim_GetStepSize
    _DLL_VeosCoSim_GetStepSize.argtypes = [
        _DLL_CoSimHandle,
        POINTER(_DLL_CoSim_SimulationTime),
    ]
    _DLL_VeosCoSim_GetStepSize.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetIncomingSignals(DsVeosCoSim_Handle handle, uint32_t* incomingSignalsCount, const DsVeosCoSim_IoSignal** incomingSignals);
    _DLL_VeosCoSim_GetIncomingSignals = _dll.DsVeosCoSim_GetIncomingSignals
    _DLL_VeosCoSim_GetIncomingSignals.argtypes = [
        _DLL_CoSimHandle,
        POINTER(c_uint32),
        POINTER(POINTER(Io_Signal)),
    ]
    _DLL_VeosCoSim_GetIncomingSignals.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReadIncomingSignal(DsVeosCoSim_Handle handle, DsVeosCoSim_IoSignalId incomingSignalId, uint32_t* length, void* value);
    _DLL_VeosCoSim_ReadIncomingSignal = _dll.DsVeosCoSim_ReadIncomingSignal
    _DLL_VeosCoSim_ReadIncomingSignal.argtypes = [
        _DLL_CoSimHandle,
        _DLL_IO_SIGNAL_ID,
        POINTER(c_uint32),
        c_void_p,
    ]
    _DLL_VeosCoSim_ReadIncomingSignal.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetOutgoingSignals(DsVeosCoSim_Handle handle, uint32_t* outgoingSignalsCount, const DsVeosCoSim_IoSignal** outgoingSignals);
    _DLL_VeosCoSim_GetOutgoingSignals = _dll.DsVeosCoSim_GetOutgoingSignals
    _DLL_VeosCoSim_GetOutgoingSignals.argtypes = [
        _DLL_CoSimHandle,
        POINTER(c_uint32),
        POINTER(POINTER(Io_Signal)),
    ]
    _DLL_VeosCoSim_GetOutgoingSignals.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_WriteOutgoingSignal(DsVeosCoSim_Handle handle, DsVeosCoSim_IoSignalId outgoingSignalId, uint32_t length, const void* value);
    _DLL_VeosCoSim_WriteOutgoingSignal = _dll.DsVeosCoSim_WriteOutgoingSignal
    _DLL_VeosCoSim_WriteOutgoingSignal.argtypes = [
        _DLL_CoSimHandle,
        _DLL_IO_SIGNAL_ID,
        c_uint32,
        c_void_p,
    ]
    _DLL_VeosCoSim_WriteOutgoingSignal.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetCanControllers(DsVeosCoSim_Handle handle, uint32_t* canControllersCount, const DsVeosCoSim_CanController** canControllers);
    _DLL_VeosCoSim_GetCanControllers = _dll.DsVeosCoSim_GetCanControllers
    _DLL_VeosCoSim_GetCanControllers.argtypes = [
        _DLL_CoSimHandle,
        POINTER(c_uint32),
        POINTER(POINTER(CanController)),
    ]
    _DLL_VeosCoSim_GetCanControllers.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveCanMessage(DsVeosCoSim_Handle handle, DsVeosCoSim_CanMessage* message);
    _DLL_VeosCoSim_ReceiveCanMessage = _dll.DsVeosCoSim_ReceiveCanMessage
    _DLL_VeosCoSim_ReceiveCanMessage.argtypes = [_DLL_CoSimHandle, POINTER(CanMessage)]
    _DLL_VeosCoSim_ReceiveCanMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveCanMessageContainer(DsVeosCoSim_Handle handle, DsVeosCoSim_CanMessageContainer* messageContainer);
    _DLL_VeosCoSim_ReceiveCanMessageContainer = getattr(_dll, "DsVeosCoSim_ReceiveCanMessageContainer", None)
    if _DLL_VeosCoSim_ReceiveCanMessageContainer is not None:
        _DLL_VeosCoSim_ReceiveCanMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(CanMessageContainer)]
        _DLL_VeosCoSim_ReceiveCanMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitCanMessage(DsVeosCoSim_Handle handle, const DsVeosCoSim_CanMessage* message);
    _DLL_VeosCoSim_TransmitCanMessage = _dll.DsVeosCoSim_TransmitCanMessage
    _DLL_VeosCoSim_TransmitCanMessage.argtypes = [_DLL_CoSimHandle, POINTER(CanMessage)]
    _DLL_VeosCoSim_TransmitCanMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitCanMessageContainer(DsVeosCoSim_Handle handle, const DsVeosCoSim_CanMessageContainer* messageContainer);
    _DLL_VeosCoSim_TransmitCanMessageContainer = getattr(_dll, "DsVeosCoSim_TransmitCanMessageContainer", None)
    if _DLL_VeosCoSim_TransmitCanMessageContainer is not None:
        _DLL_VeosCoSim_TransmitCanMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(CanMessageContainer)]
        _DLL_VeosCoSim_TransmitCanMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetEthControllers(DsVeosCoSim_Handle handle, uint32_t* ethControllersCount, const DsVeosCoSim_EthController** ethControllers);
    _DLL_VeosCoSim_GetEthControllers = _dll.DsVeosCoSim_GetEthControllers
    _DLL_VeosCoSim_GetEthControllers.argtypes = [
        _DLL_CoSimHandle,
        POINTER(c_uint32),
        POINTER(POINTER(EthController)),
    ]
    _DLL_VeosCoSim_GetEthControllers.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveEthMessage(DsVeosCoSim_Handle handle, DsVeosCoSim_EthMessage* message);
    _DLL_VeosCoSim_ReceiveEthMessage = _dll.DsVeosCoSim_ReceiveEthMessage
    _DLL_VeosCoSim_ReceiveEthMessage.argtypes = [_DLL_CoSimHandle, POINTER(EthMessage)]
    _DLL_VeosCoSim_ReceiveEthMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveEthMessageContainer(DsVeosCoSim_Handle handle, DsVeosCoSim_EthMessageContainer* messageContainer);
    _DLL_VeosCoSim_ReceiveEthMessageContainer = getattr(_dll, "DsVeosCoSim_ReceiveEthMessageContainer", None)
    if _DLL_VeosCoSim_ReceiveEthMessageContainer is not None:
        _DLL_VeosCoSim_ReceiveEthMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(EthMessageContainer)]
        _DLL_VeosCoSim_ReceiveEthMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitEthMessage(DsVeosCoSim_Handle handle, const DsVeosCoSim_EthMessage* message);
    _DLL_VeosCoSim_TransmitEthMessage = _dll.DsVeosCoSim_TransmitEthMessage
    _DLL_VeosCoSim_TransmitEthMessage.argtypes = [_DLL_CoSimHandle, POINTER(EthMessage)]
    _DLL_VeosCoSim_TransmitEthMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitEthMessageContainer(DsVeosCoSim_Handle handle, const DsVeosCoSim_EthMessageContainer* messageContainer);
    _DLL_VeosCoSim_TransmitEthMessageContainer = getattr(_dll, "DsVeosCoSim_TransmitEthMessageContainer", None)
    if _DLL_VeosCoSim_TransmitEthMessageContainer is not None:
        _DLL_VeosCoSim_TransmitEthMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(EthMessageContainer)]
        _DLL_VeosCoSim_TransmitEthMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetLinControllers(DsVeosCoSim_Handle handle, uint32_t* linControllersCount, const DsVeosCoSim_LinController** linControllers);
    _DLL_VeosCoSim_GetLinControllers = _dll.DsVeosCoSim_GetLinControllers
    _DLL_VeosCoSim_GetLinControllers.argtypes = [
        _DLL_CoSimHandle,
        POINTER(c_uint32),
        POINTER(POINTER(LinController)),
    ]
    _DLL_VeosCoSim_GetLinControllers.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveLinMessage(DsVeosCoSim_Handle handle, DsVeosCoSim_LinMessage* message);
    _DLL_VeosCoSim_ReceiveLinMessage = _dll.DsVeosCoSim_ReceiveLinMessage
    _DLL_VeosCoSim_ReceiveLinMessage.argtypes = [_DLL_CoSimHandle, POINTER(LinMessage)]
    _DLL_VeosCoSim_ReceiveLinMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveLinMessageContainer(DsVeosCoSim_Handle handle, DsVeosCoSim_LinMessageContainer* messageContainer);
    _DLL_VeosCoSim_ReceiveLinMessageContainer = getattr(_dll, "DsVeosCoSim_ReceiveLinMessageContainer", None)
    if _DLL_VeosCoSim_ReceiveLinMessageContainer is not None:
        _DLL_VeosCoSim_ReceiveLinMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(LinMessageContainer)]
        _DLL_VeosCoSim_ReceiveLinMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitLinMessage(DsVeosCoSim_Handle handle, const DsVeosCoSim_LinMessage* message);
    _DLL_VeosCoSim_TransmitLinMessage = _dll.DsVeosCoSim_TransmitLinMessage
    _DLL_VeosCoSim_TransmitLinMessage.argtypes = [_DLL_CoSimHandle, POINTER(LinMessage)]
    _DLL_VeosCoSim_TransmitLinMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitLinMessageContainer(DsVeosCoSim_Handle handle, const DsVeosCoSim_LinMessageContainer* messageContainer);
    _DLL_VeosCoSim_TransmitLinMessageContainer = getattr(_dll, "DsVeosCoSim_TransmitLinMessageContainer", None)
    if _DLL_VeosCoSim_TransmitLinMessageContainer is not None:
        _DLL_VeosCoSim_TransmitLinMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(LinMessageContainer)]
        _DLL_VeosCoSim_TransmitLinMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetFrControllers(DsVeosCoSim_Handle handle, uint32_t* frControllersCount, const DsVeosCoSim_FrController** frControllers);
    _DLL_VeosCoSim_GetFrControllers = getattr(_dll, "DsVeosCoSim_GetFrControllers", None)
    if _DLL_VeosCoSim_GetFrControllers is not None:
        _DLL_VeosCoSim_GetFrControllers.argtypes = [
            _DLL_CoSimHandle,
            POINTER(c_uint32),
            POINTER(POINTER(FrController)),
        ]
        _DLL_VeosCoSim_GetFrControllers.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveFrMessage(DsVeosCoSim_Handle handle, DsVeosCoSim_FrMessage* message);
    _DLL_VeosCoSim_ReceiveFrMessage = getattr(_dll, "DsVeosCoSim_ReceiveFrMessage", None)
    if _DLL_VeosCoSim_ReceiveFrMessage is not None:
        _DLL_VeosCoSim_ReceiveFrMessage.argtypes = [_DLL_CoSimHandle, POINTER(FrMessage)]
        _DLL_VeosCoSim_ReceiveFrMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ReceiveFrMessageContainer(DsVeosCoSim_Handle handle, DsVeosCoSim_FrMessageContainer* messageContainer);
    _DLL_VeosCoSim_ReceiveFrMessageContainer = getattr(_dll, "DsVeosCoSim_ReceiveFrMessageContainer", None)
    if _DLL_VeosCoSim_ReceiveFrMessageContainer is not None:
        _DLL_VeosCoSim_ReceiveFrMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(FrMessageContainer)]
        _DLL_VeosCoSim_ReceiveFrMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitFrMessage(DsVeosCoSim_Handle handle, const DsVeosCoSim_FrMessage* message);
    _DLL_VeosCoSim_TransmitFrMessage = getattr(_dll, "DsVeosCoSim_TransmitFrMessage", None)
    if _DLL_VeosCoSim_TransmitFrMessage is not None:
        _DLL_VeosCoSim_TransmitFrMessage.argtypes = [_DLL_CoSimHandle, POINTER(FrMessage)]
        _DLL_VeosCoSim_TransmitFrMessage.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TransmitFrMessageContainer(DsVeosCoSim_Handle handle, const DsVeosCoSim_FrMessageContainer* messageContainer);
    _DLL_VeosCoSim_TransmitFrMessageContainer = getattr(_dll, "DsVeosCoSim_TransmitFrMessageContainer", None)
    if _DLL_VeosCoSim_TransmitFrMessageContainer is not None:
        _DLL_VeosCoSim_TransmitFrMessageContainer.argtypes = [_DLL_CoSimHandle, POINTER(FrMessageContainer)]
        _DLL_VeosCoSim_TransmitFrMessageContainer.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_StartSimulation(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_StartSimulation = _dll.DsVeosCoSim_StartSimulation
    _DLL_VeosCoSim_StartSimulation.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_StartSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_StopSimulation(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_StopSimulation = _dll.DsVeosCoSim_StopSimulation
    _DLL_VeosCoSim_StopSimulation.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_StopSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_PauseSimulation(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_PauseSimulation = _dll.DsVeosCoSim_PauseSimulation
    _DLL_VeosCoSim_PauseSimulation.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_PauseSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_ContinueSimulation(DsVeosCoSim_Handle handle);
    _DLL_VeosCoSim_ContinueSimulation = _dll.DsVeosCoSim_ContinueSimulation
    _DLL_VeosCoSim_ContinueSimulation.argtypes = [_DLL_CoSimHandle]
    _DLL_VeosCoSim_ContinueSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_TerminateSimulation(DsVeosCoSim_Handle handle, DsVeosCoSim_TerminateReason terminateReason);
    _DLL_VeosCoSim_TerminateSimulation = _dll.DsVeosCoSim_TerminateSimulation
    _DLL_VeosCoSim_TerminateSimulation.argtypes = [
        _DLL_CoSimHandle,
        _DLL_VeosCoSim_TerminateReason,
    ]
    _DLL_VeosCoSim_TerminateSimulation.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetCurrentSimulationTime(DsVeosCoSim_Handle handle, DsVeosCoSim_SimulationTime* simulationTime);
    _DLL_VeosCoSim_GetCurrentSimulationTime = getattr(_dll, "DsVeosCoSim_GetCurrentSimulationTime", None)
    if _DLL_VeosCoSim_GetCurrentSimulationTime is not None:
        _DLL_VeosCoSim_GetCurrentSimulationTime.argtypes = [
            _DLL_CoSimHandle,
            POINTER(_DLL_CoSim_SimulationTime),
        ]
        _DLL_VeosCoSim_GetCurrentSimulationTime.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetSimulationState(DsVeosCoSim_Handle handle, DsVeosCoSim_SimulationState* simulationState);
    _DLL_VeosCoSim_GetSimulationState = getattr(_dll, "DsVeosCoSim_GetSimulationState", None)
    if _DLL_VeosCoSim_GetSimulationState is not None:
        _DLL_VeosCoSim_GetSimulationState.argtypes = [_DLL_CoSimHandle, POINTER(c_int32)]
        _DLL_VeosCoSim_GetSimulationState.restype = _DLL_VeosCoSim_Result

    # DSVEOSCOSIM_DECL DsVeosCoSim_Result DsVeosCoSim_GetRoundTripTime(DsVeosCoSim_Handle handle, int64_t* roundTripTimeInNanoseconds);
    _DLL_VeosCoSim_GetRoundTripTime = getattr(_dll, "DsVeosCoSim_GetRoundTripTime", None)
    if _DLL_VeosCoSim_GetRoundTripTime is not None:
        _DLL_VeosCoSim_GetRoundTripTime.argtypes = [_DLL_CoSimHandle, POINTER(c_int64)]
        _DLL_VeosCoSim_GetRoundTripTime.restype = _DLL_VeosCoSim_Result

    return _dll


_SEVERITY_FMT_COLORS = {
    Severity.INFO: "[\033[37;1mINFO\033[0m ]",
    Severity.WARNING: "[\033[33;1mWARN\033[0m ]",
    Severity.ERROR: "[\033[31;1mERROR\033[0m]",
    Severity.TRACE: "[\033[34;1mTRACE\033[0m]",  # Bright blue color for TRACE
    -1: "[\033[90;1m-UNK-\033[0m]",
}

_SEVERITY_FMT_NOCOLORS = {
    Severity.INFO: "[INFO]",
    Severity.WARNING: "[WARN]",
    Severity.ERROR: "[ERROR]",
    Severity.TRACE: "[TRACE]",
    -1: "[-UNK-]",
}

_DATATYPE_CTYPE_MAP = {
    Datatype.BOOL: c_bool,
    Datatype.INT8: c_int8,
    Datatype.INT16: c_int16,
    Datatype.INT32: c_int32,
    Datatype.INT64: c_int64,
    Datatype.UINT8: c_uint8,
    Datatype.UINT16: c_uint16,
    Datatype.UINT32: c_uint32,
    Datatype.UINT64: c_uint64,
    Datatype.FLOAT32: c_float,
    Datatype.FLOAT64: c_double,
}

_DATATYPE_CTYPE_PTR_MAP = {
    Datatype.BOOL: POINTER(c_bool),
    Datatype.INT8: POINTER(c_int8),
    Datatype.INT16: POINTER(c_int16),
    Datatype.INT32: POINTER(c_int32),
    Datatype.INT64: POINTER(c_int64),
    Datatype.UINT8: POINTER(c_uint8),
    Datatype.UINT16: POINTER(c_uint16),
    Datatype.UINT32: POINTER(c_uint32),
    Datatype.UINT64: POINTER(c_uint64),
    Datatype.FLOAT32: POINTER(c_float),
    Datatype.FLOAT64: POINTER(c_double),
}


def Init_dll_callbacks(config):
    dllcfg = _DLL_Callbacks()

    if config.on_start_simulation is not None:
        dllcfg.simulationStartedCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_start_simulation(sim_time)
        )

    if config.on_stop_simulation is not None:
        dllcfg.simulationStoppedCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_stop_simulation(sim_time)
        )

    if config.on_pause_simulation is not None:
        dllcfg.simulationPausedCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_pause_simulation(sim_time)
        )

    if config.on_continue_simulation is not None:
        dllcfg.simulationContinuedCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_continue_simulation(sim_time)
        )

    if config.on_begin_step_simulation is not None:
        dllcfg.simulationBeginStepCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_begin_step_simulation(sim_time)
        )

    if config.on_end_step_simulation is not None:
        dllcfg.simulationEndStepCallback = _DLL_SimulationCallback_t(
            lambda sim_time, user_data: config.on_end_step_simulation(sim_time)
        )

    if config.on_simulation_terminated is not None:
        dllcfg.simulationTerminatedCallback = _DLL_SimulationTerminatedCallback_t(
            lambda sim_time, reason, user_data: config.on_simulation_terminated(sim_time, reason)
        )

    if config.on_incoming_signal_changed is not None:
        dllcfg.incomingSignalChangedCallback = _DLL_IncomingSignalChangedCallback_t(
            lambda sim_time, io_signal, length, value, user_data: _signal_read_callback(
                config.on_incoming_signal_changed,
                sim_time,
                io_signal,
                length,
                value,
            )
        )

    if config.on_can_message_received is not None:
        dllcfg.canMessageReceivedCallback = _DLL_CanMessageReceivedCallback_t(
            lambda sim_time, controller_ptr, message_ptr, user_data: config.on_can_message_received(
                sim_time,
                cast(controller_ptr, POINTER(CanController)).contents,
                cast(message_ptr, POINTER(CanMessage)).contents,
            )
        )

    if config.on_eth_message_received is not None:
        dllcfg.ethMessageReceivedCallback = _DLL_EthMessageReceivedCallback_t(
            lambda sim_time, controller_ptr, message_ptr, user_data: config.on_eth_message_received(
                sim_time,
                cast(controller_ptr, POINTER(EthController)).contents,
                cast(message_ptr, POINTER(EthMessage)).contents,
            )
        )

    if config.on_lin_message_received is not None:
        dllcfg.linMessageReceivedCallback = _DLL_LinMessageReceivedCallback_t(
            lambda sim_time, controller_ptr, message_ptr, user_data: config.on_lin_message_received(
                sim_time,
                cast(controller_ptr, POINTER(LinController)).contents,
                cast(message_ptr, POINTER(LinMessage)).contents,
            )
        )

    return dllcfg


def _signal_read_callback(callback, sim_time, io_signal, length, value):
    signal = cast(io_signal, POINTER(Io_Signal)).contents

    valuepointer = cast(value, _DATATYPE_CTYPE_PTR_MAP.get(signal.dataType))

    arr = [valuepointer[i] for i in range(length)]
    callback(sim_time, signal, length, arr)
