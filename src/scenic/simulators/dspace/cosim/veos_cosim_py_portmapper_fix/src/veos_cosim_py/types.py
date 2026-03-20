from __future__ import annotations

import ctypes as C
from enum import IntEnum

VEOSCOSIM_MAX_NAME_LENGTH = 1024
TIME_RESOLUTION_PER_SECOND = 1_000_000_000

class Result(IntEnum):
    OK = 0
    ERROR = 1
    EMPTY = 2
    FULL = 3
    ARGUMENT = 4

class Command(IntEnum):
    NONE = 0
    START = 1
    STOP = 2
    TERMINATE = 3
    TIME_TRIGGER = 4

class ConnectionState(IntEnum):
    DISCONNECTED = 0
    CONNECTED = 1

class DataType(IntEnum):
    UNKNOWN = 0
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

class Direction(IntEnum):
    UNKNOWN = 0
    READ = 1
    WRITE = 2

class SizeKind(IntEnum):
    UNKNOWN = 0
    FIXED = 1
    VARIABLE = 2

class VeosCoSim_IoSignalInfo(C.Structure):
    _fields_ = [
        ("id", C.c_uint32),
        ("length", C.c_uint32),
        ("dataType", C.c_int32),
        ("direction", C.c_int32),
        ("sizeKind", C.c_int32),
        ("name", C.c_char * (VEOSCOSIM_MAX_NAME_LENGTH + 1)),
    ]

class VeosCoSim_GeneralInfo(C.Structure):
    _fields_ = [("sampleTime", C.c_int64)]

SCALAR_FORMATS = {
    DataType.BOOL: "?",
    DataType.INT8: "b",
    DataType.INT16: "h",
    DataType.INT32: "i",
    DataType.INT64: "q",
    DataType.UINT8: "B",
    DataType.UINT16: "H",
    DataType.UINT32: "I",
    DataType.UINT64: "Q",
    DataType.FLOAT32: "f",
    DataType.FLOAT64: "d",
}
