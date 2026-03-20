// Copyright dSPACE GmbH. All rights reserved.

#pragma once

#include <stdint.h>

#if (defined __GNUC__) && (!defined _DOXYGEN)
#ifdef VEOSCOSIM_EXPORT
#define VEOSCOSIM_API __attribute__((__visibility__("default")))
#else
#define VEOSCOSIM_API
#endif
#elif (defined _MSC_VER)
#if defined(VEOSCOSIM_EXPORT)
#define VEOSCOSIM_API __declspec(dllexport)
#elif defined(VEOSCOSIM_IMPORT)
#define VEOSCOSIM_API __declspec(dllimport)
#else
#define VEOSCOSIM_API
#endif
#else
#define VEOSCOSIM_API
#endif

#ifdef __cplusplus
#define VEOSCOSIM_EXTERN extern "C"
#else
#define VEOSCOSIM_EXTERN extern
#endif

#define VEOSCOSIM_DECL VEOSCOSIM_EXTERN VEOSCOSIM_API

#define VEOSCOSIM_TIME_RESOLUTION_PER_SECOND 1e9
#define VEOSCOSIM_MAX_NAME_LENGTH 1024u

#define VEOSCOSIM_CAN_MESSAGE_MAX_LENGTH 64u
#define VEOSCOSIM_LIN_MESSAGE_MAX_LENGTH 8u
#define VEOSCOSIM_ETH_MESSAGE_MAX_LENGTH 9018u

// CAN Transmit and receive flags
#define VEOSCOSIM_CAN_MESSAGE_FLAG_EXT 1u  // Extended identifier
#define VEOSCOSIM_CAN_MESSAGE_FLAG_BRS 2u  // Bit rate switch
#define VEOSCOSIM_CAN_MESSAGE_FLAG_FD 4u   // Flexible data rate format

// CAN Receive only flags
#define VEOSCOSIM_CAN_MESSAGE_FLAG_TX 8u     // Transmit direction
#define VEOSCOSIM_CAN_MESSAGE_FLAG_ERR 16u   // Error indicator
#define VEOSCOSIM_CAN_MESSAGE_FLAG_DROP 32u  // Queue drop

// LIN transmit and receive flags
#define VEOSCOSIM_LIN_MESSAGE_FLAG_HEADER 1u    // Header event
#define VEOSCOSIM_LIN_MESSAGE_FLAG_RESPONSE 2u  // Response data event

// LIN receive only flags
#define VEOSCOSIM_LIN_MESSAGE_FLAG_TX 8u       // Transmit direction
#define VEOSCOSIM_LIN_MESSAGE_FLAG_ERR 16u     // Error indicator
#define VEOSCOSIM_LIN_MESSAGE_FLAG_DROP 32u    // Queue drop
#define VEOSCOSIM_LIN_MESSAGE_FLAG_SNR 64u     // Slave not responding
#define VEOSCOSIM_LIN_MESSAGE_FLAG_WAKE 128u   // Wake event
#define VEOSCOSIM_LIN_MESSAGE_FLAG_SLEEP 256u  // Sleep event

// ETH Receive only flags
#define VEOSCOSIM_ETH_MESSAGE_FLAG_TX 1u    // Transmit direction
#define VEOSCOSIM_ETH_MESSAGE_FLAG_DROP 2u  // Queue drop

typedef uint32_t VeosCoSim_BusProtocol;
typedef uint32_t VeosCoSim_ChannelId;
typedef uint32_t VeosCoSim_DataType;
typedef uint32_t VeosCoSim_Direction;
typedef uint32_t VeosCoSim_IoSignalId;
typedef uint32_t VeosCoSim_SizeKind;
typedef int64_t VeosCoSim_Time;

typedef enum VeosCoSim_Command {
    VeosCoSim_Command_None,
    VeosCoSim_Command_Start,
    VeosCoSim_Command_Stop,
    VeosCoSim_Command_Terminate,
    VeosCoSim_Command_TimeTrigger
} VeosCoSim_Command;

typedef enum VeosCoSim_Result {
    VeosCoSim_Result_OK,
    VeosCoSim_Result_Error,
    VeosCoSim_Result_Empty,
    VeosCoSim_Result_Full,
    VeosCoSim_Result_Argument
} VeosCoSim_Result;

typedef enum VeosCoSim_ConnectionState {
    VeosCoSim_ConnectionState_Disconnected,
    VeosCoSim_ConnectionState_Connected,
} VeosCoSim_ConnectionState;

enum {
    VeosCoSim_BusProtocol_Unknown,
    VeosCoSim_BusProtocol_CAN,
    VeosCoSim_BusProtocol_LIN,
    VeosCoSim_BusProtocol_ETH
};

enum {
    VeosCoSim_DataType_Unknown,
    VeosCoSim_DataType_Bool,
    VeosCoSim_DataType_Int8,
    VeosCoSim_DataType_Int16,
    VeosCoSim_DataType_Int32,
    VeosCoSim_DataType_Int64,
    VeosCoSim_DataType_UInt8,
    VeosCoSim_DataType_UInt16,
    VeosCoSim_DataType_UInt32,
    VeosCoSim_DataType_UInt64,
    VeosCoSim_DataType_Float32,
    VeosCoSim_DataType_Float64
};

enum {
    VeosCoSim_Direction_Unknown,
    VeosCoSim_Direction_Read,
    VeosCoSim_Direction_Write
};

enum {
    VeosCoSim_SizeKind_Fixed = 1,
    VeosCoSim_SizeKind_Variable,
};

typedef enum VeosCoSim_Severity {
    VeosCoSim_Severity_Info,
    VeosCoSim_Severity_Warning,
    VeosCoSim_Severity_Error,
    VeosCoSim_Severity_Trace,
} VeosCoSim_Severity;

typedef struct VeosCoSim_IoSignalInfo {
    VeosCoSim_IoSignalId id;
    uint32_t length;
    VeosCoSim_DataType dataType;
    VeosCoSim_Direction direction;
    VeosCoSim_SizeKind sizeKind;
    char name[VEOSCOSIM_MAX_NAME_LENGTH + 1];
} VeosCoSim_IoSignalInfo;

typedef struct VeosCoSim_BusChannelInfo {
    VeosCoSim_ChannelId id;
    VeosCoSim_BusProtocol busProtocol;
    char controllerName[VEOSCOSIM_MAX_NAME_LENGTH + 1];
} VeosCoSim_BusChannelInfo;

typedef struct VeosCoSim_CanMessage {
    VeosCoSim_Time timestamp;
    VeosCoSim_ChannelId channelId;
    uint32_t identifier;
    uint32_t flags;
    uint32_t length;
    uint8_t data[VEOSCOSIM_CAN_MESSAGE_MAX_LENGTH];
} VeosCoSim_CanMessage;

typedef struct VeosCoSim_LinMessage {
    VeosCoSim_Time timestamp;
    VeosCoSim_ChannelId channelId;
    uint32_t identifier;
    uint32_t flags;
    uint32_t length;
    uint8_t data[VEOSCOSIM_LIN_MESSAGE_MAX_LENGTH];
} VeosCoSim_LinMessage;

typedef struct VeosCoSim_EthMessage {
    VeosCoSim_Time timestamp;
    VeosCoSim_ChannelId channelId;
    uint32_t flags;
    uint32_t length;
    uint8_t data[VEOSCOSIM_ETH_MESSAGE_MAX_LENGTH];
} VeosCoSim_EthMessage;

typedef struct VeosCoSim_GeneralInfo {
    VeosCoSim_Time sampleTime;
} VeosCoSim_GeneralInfo;

typedef void (*VeosCoSim_Callback)(VeosCoSim_Time simulationTime, void* userData);
typedef void (*VeosCoSim_IoReadCallback)(VeosCoSim_Time simulationTime, VeosCoSim_IoSignalId id, uint32_t length, const void* value, void* userData);
typedef void (*VeosCoSim_CanReceiveMessageCallback)(VeosCoSim_Time simulationTime, const VeosCoSim_CanMessage* message, void* userData);
typedef void (*VeosCoSim_LinReceiveMessageCallback)(VeosCoSim_Time simulationTime, const VeosCoSim_LinMessage* message, void* userData);
typedef void (*VeosCoSim_EthReceiveMessageCallback)(VeosCoSim_Time simulationTime, const VeosCoSim_EthMessage* message, void* userData);

typedef struct VeosCoSim_RuntimeConfiguration {
    VeosCoSim_Callback startSimulationCallback;
    VeosCoSim_Callback stopSimulationCallback;
    VeosCoSim_Callback terminateSimulationCallback;
    VeosCoSim_Callback timeTriggerCallback;
    VeosCoSim_IoReadCallback ioReadCallback;
    VeosCoSim_CanReceiveMessageCallback canMessageReceivedCallback;
    VeosCoSim_LinReceiveMessageCallback linMessageReceivedCallback;
    VeosCoSim_EthReceiveMessageCallback ethMessageReceivedCallback;
    void* userData;
} VeosCoSim_RuntimeConfiguration;

typedef void (*VeosCoSim_LogCallback)(VeosCoSim_Severity severity, const char* logMessage);

typedef struct VeosCoSim_ConnectConfiguration {
    const char* remoteIpAddress;
    const char* name;
    VeosCoSim_LogCallback logCallback;
    uint16_t remotePort;
    uint16_t localPort;
} VeosCoSim_ConnectConfiguration;

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_Connect(const char* remoteIpAddress, const char* name, VeosCoSim_LogCallback logCallback);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_Connect2(VeosCoSim_ConnectConfiguration config);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_Disconnect(void);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetConnectionState(VeosCoSim_ConnectionState* connectionState);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetLastTime(VeosCoSim_Time* lastTime);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetGeneralInfo(VeosCoSim_GeneralInfo* generalInfo);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_Run(VeosCoSim_RuntimeConfiguration config);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_StartNonBlocking(VeosCoSim_RuntimeConfiguration config);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetNextCommand(VeosCoSim_Time* simulationTime, VeosCoSim_Command* nextCommand);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_FinishCommand(void);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoGetAvailableSignals(uint32_t* count, const VeosCoSim_IoSignalInfo** ioSignalInfos);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoRead(VeosCoSim_IoSignalId id, uint32_t* length, void* value);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoWrite(VeosCoSim_IoSignalId id, uint32_t length, const void* value);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetAvailableChannels(uint32_t* count, const VeosCoSim_BusChannelInfo** channelInfos);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_CanReceiveMessage(VeosCoSim_CanMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_CanTransmitMessage(const VeosCoSim_CanMessage* message);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_LinReceiveMessage(VeosCoSim_LinMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_LinTransmitMessage(const VeosCoSim_LinMessage* message);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_EthReceiveMessage(VeosCoSim_EthMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_EthTransmitMessage(const VeosCoSim_EthMessage* message);

typedef void* VeosCoSim_Handle;

VEOSCOSIM_DECL VeosCoSim_Handle VeosCoSim_CreateMI(void);
VEOSCOSIM_DECL void VeosCoSim_DestroyMI(VeosCoSim_Handle handle);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_ConnectMI(VeosCoSim_Handle handle, const char* remoteIpAddress, const char* name, VeosCoSim_LogCallback logCallback);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_ConnectMI2(VeosCoSim_Handle handle, VeosCoSim_ConnectConfiguration config);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_DisconnectMI(VeosCoSim_Handle handle);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetConnectionStateMI(VeosCoSim_Handle handle, VeosCoSim_ConnectionState* connectionState);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetLastTimeMI(VeosCoSim_Handle handle, VeosCoSim_Time* lastTime);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetGeneralInfoMI(VeosCoSim_Handle handle, VeosCoSim_GeneralInfo* generalInfo);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_RunMI(VeosCoSim_Handle handle, VeosCoSim_RuntimeConfiguration config);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_StartNonBlockingMI(VeosCoSim_Handle handle, VeosCoSim_RuntimeConfiguration config);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetNextCommandMI(VeosCoSim_Handle handle, VeosCoSim_Time* simulationTime, VeosCoSim_Command* nextCommand);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_FinishCommandMI(VeosCoSim_Handle handle);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoGetAvailableSignalsMI(VeosCoSim_Handle handle, uint32_t* count, const VeosCoSim_IoSignalInfo** ioSignalInfos);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoReadMI(VeosCoSim_Handle handle, VeosCoSim_IoSignalId id, uint32_t* length, void* value);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_IoWriteMI(VeosCoSim_Handle handle, VeosCoSim_IoSignalId id, uint32_t length, const void* value);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_GetAvailableChannelsMI(VeosCoSim_Handle handle, uint32_t* count, const VeosCoSim_BusChannelInfo** channelInfos);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_CanReceiveMessageMI(VeosCoSim_Handle handle, VeosCoSim_CanMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_CanTransmitMessageMI(VeosCoSim_Handle handle, const VeosCoSim_CanMessage* message);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_LinReceiveMessageMI(VeosCoSim_Handle handle, VeosCoSim_LinMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_LinTransmitMessageMI(VeosCoSim_Handle handle, const VeosCoSim_LinMessage* message);

VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_EthReceiveMessageMI(VeosCoSim_Handle handle, VeosCoSim_EthMessage* message);
VEOSCOSIM_DECL VeosCoSim_Result VeosCoSim_EthTransmitMessageMI(VeosCoSim_Handle handle, const VeosCoSim_EthMessage* message);
