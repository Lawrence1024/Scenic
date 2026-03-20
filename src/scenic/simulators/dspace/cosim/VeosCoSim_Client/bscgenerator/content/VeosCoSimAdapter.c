// Copyright dSPACE GmbH. All rights reserved.

#include "VeosCoSimAdapter.h"

#if defined(HAS_ETH) && defined(OLD_ETH_IF)
#include <DSEthernetApi.h>
#endif
#include <DsMsg.h>
#include <VEOS.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#ifdef _WIN32
#define WIN32_LEAN_AND_MEAN
#include <Windows.h>
#else
#include <dlfcn.h>
#endif

typedef struct VeosCoSim_ServerCallbacks {
    VeosCoSim_LogCallback logCallback;
    VeosCoSim_Callback simulationStopRequested;
    VeosCoSim_Callback simulationTerminateRequested;
    VeosCoSim_CanReceiveMessageCallback canMessageReceived;
    VeosCoSim_LinReceiveMessageCallback linMessageReceived;
    VeosCoSim_EthReceiveMessageCallback ethMessageReceived;
} VeosCoSim_ServerCallbacks;

typedef void (*VeosCoSim_Server_Load)(uint16_t port,
                                      const char* serverName,
                                      VeosCoSim_Time sampleTime,
                                      bool clientOptional,
                                      uint32_t ioSignalsCount,
                                      const VeosCoSim_IoSignalInfo* ioSignals,
                                      uint32_t busControllersCount,
                                      const VeosCoSim_BusChannelInfo* busControllers,
                                      VeosCoSim_ServerCallbacks callbacks);

typedef void (*VeosCoSim_Server_Start)(VeosCoSim_Time simulationTime);
typedef void (*VeosCoSim_Server_Stop)(VeosCoSim_Time simulationTime);
typedef void (*VeosCoSim_Server_Step)(VeosCoSim_Time simulationTime);

typedef void (*VeosCoSim_Server_OutputSignalRead)(VeosCoSim_IoSignalId id, uint32_t* length, void* value);
typedef void (*VeosCoSim_Server_InputSignalWrite)(VeosCoSim_IoSignalId id, uint32_t length, const void* value);

typedef void (*VeosCoSim_Server_CanMessageTransmit)(const VeosCoSim_CanMessage* message);
typedef void (*VeosCoSim_Server_LinMessageTransmit)(const VeosCoSim_LinMessage* message);
typedef void (*VeosCoSim_Server_EthMessageTransmit)(const VeosCoSim_EthMessage* message);

static VeosCoSim_Server_Load g_load;

static VeosCoSim_Server_Start g_start;
static VeosCoSim_Server_Stop g_stop;
static VeosCoSim_Server_Step g_step;

static VeosCoSim_Server_OutputSignalRead g_outputSignalRead;
static VeosCoSim_Server_InputSignalWrite g_inputSignalWrite;

static VeosCoSim_Server_CanMessageTransmit g_canMessageTransmit;
static VeosCoSim_Server_LinMessageTransmit g_linMessageTransmit;
static VeosCoSim_Server_EthMessageTransmit g_ethMessageTransmit;

static VoidVoidCallback g_requestStop;
static VoidVoidCallback g_requestTerminate;

#define LOG_ERROR(...)                       \
    do {                                     \
        msg_error_printf(0, 0, __VA_ARGS__); \
        if (g_requestTerminate) {            \
            g_requestTerminate();            \
        }                                    \
                                             \
        return false;                        \
    } while (0)

#if defined(HAS_CAN) || defined(HAS_LIN)
#define BUFFER_SIZE 512
#endif

#ifdef _WIN32
#define strncpy(destination, source, size) strcpy_s(destination, size, source)
#endif

static bool g_noLoopBackMessages;

static uint32_t g_busControllersCount = 0;
static const VeosCoSim_BusChannelInfo* g_busControllers;

#ifdef HAS_CAN
typedef struct CanHwHandle {
    DsTCanCh channel;
    DsTCanMsg monitor;
    DsTCanMsg queue;
} CanHwHandle;
#endif

#ifdef HAS_LIN
typedef struct LinHwHandle {
    dslin_channel_p channel;
    dslin_node_p node;
    uint32_t clientId;
    VeosCoSim_ChannelId channelId;
} LinHwHandle;
#endif

#ifdef HAS_ETH
typedef struct EthHwHandle {
#ifdef OLD_ETH_IF
    DSTEthHandle clientId;
#else
    DsPcapSHandle* handle;
    uint8_t sourceMacAddress[6];
#endif
} EthHwHandle;
#endif

typedef union BusHwHandle {
#ifdef HAS_CAN
    CanHwHandle canHwHandle;
#endif
#ifdef HAS_LIN
    LinHwHandle linHwHandle;
#endif
#ifdef HAS_ETH
    EthHwHandle ethHwHandle;
#endif
    void* dummy;  // So that this struct will not be empty
} BusHwHandle;

#ifdef HAS_CAN
static DsTCanBoard g_canBoard = NULL;
#endif
#ifdef HAS_LIN
static dslin_board_p g_linBoard = NULL;
#endif

static BusHwHandle* g_busChannelHandles = NULL;

static void OnLogCallback(VeosCoSim_Severity severity, const char* text) {
    switch (severity) {  // NOLINT(clang-diagnostic-switch-enum, hicpp-multiway-paths-covered)
        case VeosCoSim_Severity_Info:
            msg_info_printf(0, 0, text);
            break;
        case VeosCoSim_Severity_Warning:
            msg_warning_printf(0, 0, text);
            break;
        case VeosCoSim_Severity_Error:
            msg_error_printf(0, 0, text);
            break;
        default:  // NOLINT(clang-diagnostic-covered-switch-default)
            break;
    }
}

static void OnStopRequested(VeosCoSim_Time simulationTime, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_requestStop) {
        g_requestStop();
    }
}

static void OnTerminateRequested(VeosCoSim_Time simulationTime, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_requestTerminate) {
        g_requestTerminate();
    }
}

#ifdef HAS_CAN

static uint32_t ToVeosCoSimCanFlags(const DsSCanMsg_Item* item) {
    uint32_t flags = 0;

    switch (item->Status) { // NOLINT(clang-diagnostic-switch-enum)
        case DSCAN_MSG_OVERRUN:
            flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_DROP;
            break;
        case DSCAN_MSG_LOST:
            flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_ERR;
            break;
        default:
            // GCC outputs a warning if not all enum values are handled
            break;
    }

    const DsECanMsg_Format format = item->Format;
    if (format == DSCAN_MSG_FORMAT_EXT || format == DSCAN_MSG_FORMAT_FD_EXT) {
        flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_EXT;
    }

    if (format == DSCAN_MSG_FORMAT_FD_STD || format == DSCAN_MSG_FORMAT_FD_EXT) {
        flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_FD;
    }

    if (item->Dir == DSCAN_MSG_DIR_TRANSMIT) {
        flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_TX;
    }

    if (item->Brs == DSCAN_MSG_BRS_ENABLED) {
        flags |= VEOSCOSIM_CAN_MESSAGE_FLAG_BRS;
    }

    return flags;
}

static void FromVeosCoSimCanFlags(DsSCanMsg_Item* item, uint32_t flags) {
    if (flags & VEOSCOSIM_CAN_MESSAGE_FLAG_EXT) {
        if (flags & VEOSCOSIM_CAN_MESSAGE_FLAG_FD) {
            item->Format = DSCAN_MSG_FORMAT_FD_EXT;
        } else {
            item->Format = DSCAN_MSG_FORMAT_EXT;
        }
    } else {
        if (flags & VEOSCOSIM_CAN_MESSAGE_FLAG_FD) {
            item->Format = DSCAN_MSG_FORMAT_FD_STD;
        } else {
            item->Format = DSCAN_MSG_FORMAT_STD;
        }
    }

    if (flags & VEOSCOSIM_CAN_MESSAGE_FLAG_BRS) {
        item->Brs = DSCAN_MSG_BRS_ENABLED;
    } else {
        item->Brs = DSCAN_MSG_BRS_DISABLED;
    }
}

static bool ConnectCanHwHandle(VeosCoSim_ChannelId channelId, DsTCanCh_Address addressHandle) {
    CanHwHandle* handle = &g_busChannelHandles[channelId].canHwHandle;

    if (!g_canBoard && DsCanBoard_create(&g_canBoard, addressHandle) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not create CAN board.");
    }

    if (DsCanCh_create(&handle->channel, g_canBoard, addressHandle) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not create CAN channel.");
    }

    if (DsCanCh_start(handle->channel) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not create CAN channel.");
    }

    if (DsCanMsg_createRxMonitor(&handle->monitor, handle->channel) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not create CAN monitor.");
    }

    if (DsCanMsg_setQueueSize(handle->monitor, BUFFER_SIZE) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not set queue size for CAN monitor.");
    }

    if (DsCanMsg_apply(handle->monitor) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not apply CAN monitor.");
    }

    if (DsCanMsg_start(handle->monitor) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not start CAN monitor.");
    }

    if (DsCanMsg_createTxQueue(&handle->queue, handle->channel) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not create CAN queue.");
    }

    if (DsCanMsg_setQueueSize(handle->queue, BUFFER_SIZE) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not set queue size for CAN monitor.");
    }

    if (DsCanMsg_apply(handle->queue) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not apply CAN queue.");
    }

    if (DsCanMsg_start(handle->queue) != DSCAN_NO_ERROR) {
        LOG_ERROR("Could not start CAN queue.");
    }

    return true;
}

static void FetchCanData(VeosCoSim_ChannelId channelId) {
    const CanHwHandle* hwHandle = &g_busChannelHandles[channelId].canHwHandle;

    DsSCanMsg_Item item = DSCAN_MSG_ITEM_INITIALIZER;
    while (DsCanMsg_readRxItem(hwHandle->monitor, &item) == DSCAN_NO_ERROR && item.Status != DSCAN_MSG_NO_DATA) {
        if (g_noLoopBackMessages && item.Dir == DSCAN_MSG_DIR_TRANSMIT) {
            continue;
        }

        VeosCoSim_CanMessage message = {0};
        message.channelId = channelId;
        message.identifier = item.Id;
        message.length = item.Dlc;
        message.flags = ToVeosCoSimCanFlags(&item);
        message.timestamp = (VeosCoSim_Time)(item.TimeStamp * VEOSCOSIM_TIME_RESOLUTION_PER_SECOND);
        memcpy(message.data, item.Data, item.Dlc);

        g_canMessageTransmit(&message);
    }
}

static void OnCanMessageReceived(VeosCoSim_Time simulationTime, const VeosCoSim_CanMessage* message, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_busControllersCount == 0 || message->channelId >= g_busControllersCount ||
        g_busControllers[message->channelId].busProtocol != VeosCoSim_BusProtocol_CAN) {
        return;
    }

    DsSCanMsg_Item item = DSCAN_MSG_ITEM_INITIALIZER;
    item.Status = DSCAN_MSG_NEW;
    item.Id = message->identifier;
    item.Dlc = message->length;
    FromVeosCoSimCanFlags(&item, message->flags);
    memcpy(item.Data, message->data, message->length);

    if (DsCanMsg_transmitItem(g_busChannelHandles[message->channelId].canHwHandle.queue, &item) != DSCAN_NO_ERROR) {
        VeosCoSim_CanMessage copy = {0};
        copy.timestamp = message->timestamp;
        copy.channelId = message->channelId;
        copy.identifier = message->identifier;
        copy.flags = message->flags | VEOSCOSIM_CAN_MESSAGE_FLAG_TX | VEOSCOSIM_CAN_MESSAGE_FLAG_DROP;
        copy.length = message->length;
        memcpy(copy.data, message->data, message->length);
        g_canMessageTransmit(&copy);
    }
}

#endif

#ifdef HAS_LIN

static uint32_t ToVeosCoSimLinEventFlags(const dslin_channel_event_monitor_data_t* eventItem) {
    switch (eventItem->channel_event.id) {
        case DSLIN_CHANNEL_EVENT_ID_WAKE:
            return VEOSCOSIM_LIN_MESSAGE_FLAG_WAKE;
        case DSLIN_CHANNEL_EVENT_ID_SLEEP:
            return VEOSCOSIM_LIN_MESSAGE_FLAG_SLEEP;
        case DSLIN_CHANNEL_EVENT_ID_HEADER_ERROR:
        case DSLIN_CHANNEL_EVENT_ID_BUS_ERROR:
        case DSLIN_CHANNEL_EVENT_ID_SYSTEM:
            return VEOSCOSIM_LIN_MESSAGE_FLAG_ERR;
    }

    return 0;
}

static uint32_t ToVeosCoSimLinResponseFlags(const dslin_channel_event_monitor_data_t* eventItem) {
    uint32_t flags = VEOSCOSIM_LIN_MESSAGE_FLAG_RESPONSE;
    if (eventItem->rx_data.rx_status & DSLIN_CHANNEL_RX_LOOPBACK) {
        flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_TX;
    }

    if (eventItem->rx_data.rx_status & DSLIN_CHANNEL_RX_SNR_ERR) {
        flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_SNR;
    }

    if (eventItem->rx_data.rx_status & (DSLIN_CHANNEL_RX_FRAMING_ERR | DSLIN_CHANNEL_RX_CHECKSUM_ERR)) {
        flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_ERR;
    }

    return flags;
}

static int OnLinHeaderEvent(void* pUserArg, dslin_channel_dm_rx_data_t* eventItem) {
    if (g_noLoopBackMessages && eventItem->rx_status & DSLIN_CHANNEL_RX_LOOPBACK) {
        return 0;
    }

    VeosCoSim_LinMessage message = {0};
    message.flags = VEOSCOSIM_LIN_MESSAGE_FLAG_HEADER;
    if (eventItem->rx_status & DSLIN_CHANNEL_RX_LOOPBACK) {
        message.flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_TX;
    }

    message.timestamp = (VeosCoSim_Time)(eventItem->timestamp * VEOSCOSIM_TIME_RESOLUTION_PER_SECOND);
    message.identifier = (uint32_t)eventItem->identifier;
    message.channelId = *(VeosCoSim_ChannelId*)pUserArg;
    message.length = 0;
    g_linMessageTransmit(&message);

    return 0;
}

static bool ConnectLinHwHandle(VeosCoSim_ChannelId channelId, dslin_channel_address_t addressHandle) {
    LinHwHandle* handle = &g_busChannelHandles[channelId].linHwHandle;
    handle->channelId = channelId;

    if (!g_linBoard && dslin_board_create(&g_linBoard, addressHandle) != DSLIN_OK) {
        LOG_ERROR("Could not create LIN board.");
    }

    const enum DSLIN_ERROR error = dslin_channel_create(g_linBoard, g_busControllers[channelId].controllerName, addressHandle, &handle->channel);
    if (error != DSLIN_OBJECT_REUSED && error != DSLIN_OK) {
        LOG_ERROR("Could not create LIN channel.");
    }

    if (dslin_channel_rx_monitor_init(handle->channel, DSLIN_MAX_FRAME_LENGTH, BUFFER_SIZE) != DSLIN_OK) {
        LOG_ERROR("Could not initialize LIN monitor.");
    }

    if (dslin_channel_rx_monitor_client_init(handle->channel, &handle->clientId) != DSLIN_OK) {
        LOG_ERROR("Could not initialize LIN monitor client.");
    }

    if (dslin_channel_enable(handle->channel) != DSLIN_OK) {
        LOG_ERROR("Could not enable LIN channel.");
    }

    if (dslin_node_create(handle->channel, g_busControllers[channelId].controllerName, &handle->node) != DSLIN_OK) {
        LOG_ERROR("Could not create LIN node.");
    }

    for (uint8_t id = 0x00; id <= 0x3F; id++) {
        if (dslin_node_frame_event_connect(handle->node, DSLIN_NODE_EVENT_ON_HEADER_RECEIVED, id, OnLinHeaderEvent, &handle->channelId) != DSLIN_OK) {
            LOG_ERROR("Could not connect LIN header event.");
        }
    }

    return true;
}

static void FetchLinData(VeosCoSim_ChannelId channelId) {
    const LinHwHandle* hwHandle = &g_busChannelHandles[channelId].linHwHandle;

    dslin_channel_event_monitor_data_t eventItem;
    while (dslin_channel_event_monitor_client_read(hwHandle->channel, hwHandle->clientId, &eventItem) == DSLIN_OK &&
           eventItem.data_type != DSLIN_CHANNEL_NO_DATA) {
        if (g_noLoopBackMessages && eventItem.data_type == DSLIN_CHANNEL_RX_DATA && eventItem.rx_data.rx_status & DSLIN_CHANNEL_RX_LOOPBACK) {
            continue;
        }

        VeosCoSim_LinMessage message = {0};
        message.channelId = channelId;

        if (eventItem.data_type == DSLIN_CHANNEL_EVENT_DATA) {
            message.flags = ToVeosCoSimLinEventFlags(&eventItem);
            message.identifier = eventItem.channel_event.sub_id;
            message.length = 0;
            message.timestamp = (VeosCoSim_Time)(eventItem.channel_event.timestamp * VEOSCOSIM_TIME_RESOLUTION_PER_SECOND);
        } else {
            memcpy(message.data, eventItem.rx_data.data, eventItem.rx_data.dlc);
            message.flags = ToVeosCoSimLinResponseFlags(&eventItem);
            message.identifier = eventItem.rx_data.identifier;
            message.length = eventItem.rx_data.dlc;
            message.timestamp = (VeosCoSim_Time)(eventItem.rx_data.timestamp * VEOSCOSIM_TIME_RESOLUTION_PER_SECOND);
        }

        g_linMessageTransmit(&message);
    }
}

static void OnLinMessageReceived(VeosCoSim_Time simulationTime, const VeosCoSim_LinMessage* message, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_busControllersCount == 0 || message->channelId >= g_busControllersCount ||
        g_busControllers[message->channelId].busProtocol != VeosCoSim_BusProtocol_LIN) {
        return;
    }

    if (message->flags & VEOSCOSIM_LIN_MESSAGE_FLAG_HEADER) {
        if (dslin_channel_header_send(g_busChannelHandles[message->channelId].linHwHandle.channel, (uint8_t)message->identifier) != DSLIN_OK) {
            VeosCoSim_LinMessage copy = *message;
            copy.flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_TX;
            copy.flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_DROP;
            g_linMessageTransmit(&copy);
        }
    }

    if (message->flags & VEOSCOSIM_LIN_MESSAGE_FLAG_RESPONSE) {
        dslin_channel_tx_data_t responseData = {0};
        responseData.identifier = (uint8_t)message->identifier;
        responseData.dlc = (uint8_t)message->length;
        responseData.response_delay = 0.0;
        responseData.checksum = 0;
        responseData.tx_mode = DSLIN_CHANNEL_TX_ONCE | DSLIN_CHANNEL_TX_ENHANCED_CHECKSUM;

        memcpy(responseData.data, message->data, message->length);
        if (dslin_channel_tx_response_send(g_busChannelHandles[message->channelId].linHwHandle.channel, &responseData) != DSLIN_OK) {
            VeosCoSim_LinMessage copy = *message;
            copy.flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_TX;
            copy.flags |= VEOSCOSIM_LIN_MESSAGE_FLAG_DROP;
            g_linMessageTransmit(&copy);
        }
    }

    if (message->flags & VEOSCOSIM_LIN_MESSAGE_FLAG_SLEEP) {
        dslin_node_command_sleep(g_busChannelHandles[message->channelId].linHwHandle.node);
    }

    if (message->flags & VEOSCOSIM_LIN_MESSAGE_FLAG_WAKE) {
        dslin_node_command_wakeup(g_busChannelHandles[message->channelId].linHwHandle.node);
    }
}

#endif

#ifdef HAS_ETH
#ifdef OLD_ETH_IF

static bool ConnectEthHwHandle(VeosCoSim_ChannelId channelId, uint32_t ethAddress) {
    (void)ethAddress;

    char providerName[DSETH_MAX_NAME_LENGTH];
    strncpy(providerName, DSETH_ACCESS_PROVIDER_NAME_DSPACE, sizeof providerName - 1);

    char interfaceName[DSETH_MAX_NAME_LENGTH];
    strncpy(interfaceName, DSETH_INTERFACE_NAME_UNKNOWN, sizeof interfaceName - 1);

    char interfaceSerialNumber[DSETH_MAX_NAME_LENGTH];
    strncpy(interfaceSerialNumber, "", sizeof interfaceSerialNumber - 1);

    char interfaceIdentifier[DSETH_MAX_NAME_LENGTH];
    strncpy(interfaceIdentifier, g_busControllers[channelId].controllerName, sizeof interfaceIdentifier - 1);

    EthHwHandle* handle = &g_busChannelHandles[channelId].ethHwHandle;
    if (DSETH_RegisterInterface(providerName, interfaceName, interfaceSerialNumber, interfaceIdentifier, &handle->clientId) != DSETH_ERR_NO_ERROR) {
        LOG_ERROR("Could not register ethernet interface.");
    }

    if (DSETH_ActivateInterface(handle->clientId) != DSETH_ERR_NO_ERROR) {
        LOG_ERROR("Could not activate ethernet interface.");
    }

    return true;
}

static uint32_t ToVeosCoSimEthFlags(const DSSEthRawFrame* frame) {
    if (frame->tHeader.tFrameType == DSETH_FRAME_TYPE_LOOPBACK_FRAME) {
        return VEOSCOSIM_ETH_MESSAGE_FLAG_TX;
    }

    return 0;
}

static void FetchEthData(VeosCoSim_ChannelId channelId) {
    const EthHwHandle* hwHandle = &g_busChannelHandles[channelId].ethHwHandle;

    unsigned long framesCount = 0;
    unsigned char buffer[10240];
    while (DSETH_ReadFrames(hwHandle->clientId, sizeof buffer, buffer, &framesCount) == DSETH_ERR_NO_ERROR && framesCount > 0) {
        size_t bufferPos = 0;
        for (unsigned long i = 0; i < framesCount; i++) {
            const DSSEthRawFrame* frame = (DSSEthRawFrame*)&buffer[bufferPos];

            if (!g_noLoopBackMessages || (frame->tHeader.tFrameType != DSETH_FRAME_TYPE_LOOPBACK_FRAME)) {
                VeosCoSim_EthMessage message = {0};
                message.flags = ToVeosCoSimEthFlags(frame);
                message.channelId = channelId;
                message.length = frame->tHeader.ulRawDataLength;
                message.timestamp = (VeosCoSim_Time)frame->tHeader.ui64ControllerTimestamp;
                memcpy(message.data, frame->tRawData, frame->tHeader.ulRawDataLength);

                g_ethMessageTransmit(&message);
            }

            bufferPos += frame->tHeader.ulRawDataLength + sizeof(DSSEthRawFrame);
        }
    }
}

static void OnEthMessageReceived(VeosCoSim_Time simulationTime, const VeosCoSim_EthMessage* message, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_busControllersCount == 0 || message->channelId >= g_busControllersCount ||
        g_busControllers[message->channelId].busProtocol != VeosCoSim_BusProtocol_ETH) {
        return;
    }

    unsigned long count = 1;
    DSSEthRawFrame frame = {0};
    frame.tHeader.ulRawDataLength = message->length;
    frame.tHeader.tFrameType = DSETH_FRAME_TYPE_ETHERNET_FRAME_WITHOUT_FCS;
    frame.tRawData = calloc(message->length, sizeof(DSTEthData));
    if (!frame.tRawData) {
        LOG_ERROR("Could not allocate data for ethernet frame.");
    }

    memcpy(frame.tRawData, message->data, message->length);

    if (DSETH_TransmitFrames(g_busChannelHandles[message->channelId].ethHwHandle.clientId, count, (unsigned char*)&frame, &count) != DSETH_ERR_NO_ERROR) {
        VeosCoSim_EthMessage copy = {0};
        copy.timestamp = message->timestamp;
        copy.channelId = message->channelId;
        copy.flags = message->flags | VEOSCOSIM_ETH_MESSAGE_FLAG_TX | VEOSCOSIM_ETH_MESSAGE_FLAG_DROP;
        copy.length = message->length;
        memcpy(copy.data, message->data, message->length);
        g_ethMessageTransmit(&copy);
    }

    free(frame.tRawData);
}

#else

static bool ConnectEthHwHandle(VeosCoSim_ChannelId channelId, DsEthInterfaceHandle ethAddress) {
    EthHwHandle* hwHandle = &g_busChannelHandles[channelId].ethHwHandle;
    char errorBuffer[DSPCAP_ERRBUF_SIZE] = {0};
    hwHandle->handle = DsPcap_create(ethAddress, errorBuffer);
    if (!hwHandle->handle) {
        LOG_ERROR("Could not create PCAP handle. Error message: %s", errorBuffer);
    }

    if (DsPcap_activate(hwHandle->handle) != 0) {
        LOG_ERROR("Could not activate PCAP handle.");
    }

    return true;
}

static VeosCoSim_Time TimeValToVeosCoSimTime(const DsSTimeval* timeVal) {
    return (timeVal->tv_sec * 1000000000) + ((int64_t)timeVal->tv_usec * 1000);
}

static void FetchEthData(VeosCoSim_ChannelId channelId) {
    const EthHwHandle* hwHandle = &g_busChannelHandles[channelId].ethHwHandle;
    DsPcapSPktHdr* header = NULL;
    const uint8_t* data = NULL;

    while (DsPcap_nextEx(hwHandle->handle, &header, &data) > 0) {
        bool isTx = (header->len >= 12) && memcmp(hwHandle->sourceMacAddress, data + 6, 6) == 0;

        if (g_noLoopBackMessages && isTx) {
            continue;
        }

        VeosCoSim_EthMessage message = {0};
        if (isTx) {
            message.flags = VEOSCOSIM_ETH_MESSAGE_FLAG_TX;
        }

        message.channelId = channelId;
        message.length = header->len;
        message.timestamp = TimeValToVeosCoSimTime(&header->ts);
        memcpy(message.data, data, header->len);

        g_ethMessageTransmit(&message);
    }
}

static DsSTimeval VeosCoSimTimeToTimeVal(VeosCoSim_Time time) {
    DsSTimeval timeVal = {0};
    timeVal.tv_sec = time / 1000000000;
    timeVal.tv_usec = (int)((time / 1000) % 1000000);

    return timeVal;
}

static void OnEthMessageReceived(VeosCoSim_Time simulationTime, const VeosCoSim_EthMessage* message, void* userData) {
    (void)simulationTime;
    (void)userData;

    if (g_busControllersCount == 0 || message->channelId >= g_busControllersCount ||
        g_busControllers[message->channelId].busProtocol != VeosCoSim_BusProtocol_ETH) {
        return;
    }

    EthHwHandle* hwHandle = &g_busChannelHandles[message->channelId].ethHwHandle;

    if (message->length >= 12) {
        memcpy(hwHandle->sourceMacAddress, message->data + 6, 6);
    }

    DsPcapSPktHdr header = {0};
    header.len = message->length;
    header.caplen = message->length;
    header.ts = VeosCoSimTimeToTimeVal(message->timestamp);

    if (DsPcap_inject(hwHandle->handle, &header, (uint8_t*)message->data) <= 0) {
        VeosCoSim_EthMessage copy = {0};
        copy.timestamp = message->timestamp;
        copy.channelId = message->channelId;
        copy.flags = message->flags | VEOSCOSIM_ETH_MESSAGE_FLAG_TX | VEOSCOSIM_ETH_MESSAGE_FLAG_DROP;
        copy.length = message->length;
        memcpy(copy.data, message->data, message->length);
        g_ethMessageTransmit(&copy);
    }
}

#endif
#endif

#ifdef _WIN32
#define VCS_GET_PROC_ADDRESS GetProcAddress
#else
#define VCS_GET_PROC_ADDRESS dlsym
#endif

static bool LoadVeosCoSimModule(void) {
#ifdef _WIN32
    const HMODULE hModule = LoadLibraryA("VeosCoSimAppl");
#else
    void* hModule = dlopen("./libVeosCoSimAppl.so", RTLD_LAZY);
#endif
    if (!hModule) {
        LOG_ERROR("Could not load VeosCoSim Library.");
    }

    g_load = (VeosCoSim_Server_Load)VCS_GET_PROC_ADDRESS(hModule, "VeosCoSim_Server_Load");     // NOLINT(clang-diagnostic-cast-function-type)
    g_start = (VeosCoSim_Server_Start)VCS_GET_PROC_ADDRESS(hModule, "VeosCoSim_Server_Start");  // NOLINT(clang-diagnostic-cast-function-type)
    g_stop = (VeosCoSim_Server_Stop)VCS_GET_PROC_ADDRESS(hModule, "VeosCoSim_Server_Stop");     // NOLINT(clang-diagnostic-cast-function-type)
    g_step = (VeosCoSim_Server_Step)VCS_GET_PROC_ADDRESS(hModule, "VeosCoSim_Server_Step");     // NOLINT(clang-diagnostic-cast-function-type)

    const char* ioRead = "VeosCoSim_Server_OutputSignalRead";
    g_outputSignalRead = (VeosCoSim_Server_OutputSignalRead)VCS_GET_PROC_ADDRESS(hModule, ioRead);  // NOLINT(clang-diagnostic-cast-function-type)

    const char* ioWrite = "VeosCoSim_Server_InputSignalWrite";
    g_inputSignalWrite = (VeosCoSim_Server_InputSignalWrite)VCS_GET_PROC_ADDRESS(hModule, ioWrite);  // NOLINT(clang-diagnostic-cast-function-type)

    const char* canTransmit = "VeosCoSim_Server_CanMessageTransmit";
    g_canMessageTransmit = (VeosCoSim_Server_CanMessageTransmit)VCS_GET_PROC_ADDRESS(hModule, canTransmit);  // NOLINT(clang-diagnostic-cast-function-type)

    const char* linTransmit = "VeosCoSim_Server_LinMessageTransmit";
    g_linMessageTransmit = (VeosCoSim_Server_LinMessageTransmit)VCS_GET_PROC_ADDRESS(hModule, linTransmit);  // NOLINT(clang-diagnostic-cast-function-type)

    const char* ethTransmit = "VeosCoSim_Server_EthMessageTransmit";
    g_ethMessageTransmit = (VeosCoSim_Server_EthMessageTransmit)VCS_GET_PROC_ADDRESS(hModule, ethTransmit);  // NOLINT(clang-diagnostic-cast-function-type)

    return true;
}

bool VeosCoSim_Adapter_Load(uint16_t port,
                            const char* serverName,
                            VeosCoSim_Time sampleTime,
                            bool clientOptional,
                            uint32_t ioSignalsCount,
                            const VeosCoSim_IoSignalInfo* ioSignals,
                            bool noLoopBackMessages,
                            uint32_t busControllersCount,
                            const VeosCoSim_BusChannelInfo* busControllers,
                            AddressType busAddresses[],
                            VoidVoidCallback requestTerminate,
                            VoidVoidCallback requestStop) {
    g_requestTerminate = requestTerminate;
    g_requestStop = requestStop;
    g_noLoopBackMessages = noLoopBackMessages;

    g_busControllersCount = busControllersCount;
    g_busControllers = busControllers;

    if (busControllersCount > 0) {
        g_busChannelHandles = (BusHwHandle*)calloc(busControllersCount, sizeof(BusHwHandle));
        if (!g_busChannelHandles) {
            LOG_ERROR("Could not allocate bus channel handles.");
        }
    } else {
        g_busChannelHandles = NULL;
    }

    if (!LoadVeosCoSimModule()) {
        return false;
    }

    for (uint32_t i = 0; i < busControllersCount; i++) {
        switch (g_busControllers[i].busProtocol) {
#ifdef HAS_CAN
            case VeosCoSim_BusProtocol_CAN:

                if (!ConnectCanHwHandle(i, *busAddresses[i].canAddress)) {
                    return false;
                }

                break;
#endif
#ifdef HAS_LIN
            case VeosCoSim_BusProtocol_LIN:
                if (!ConnectLinHwHandle(i, *busAddresses[i].linAddress)) {
                    return false;
                }

                break;
#endif
#ifdef HAS_ETH
            case VeosCoSim_BusProtocol_ETH:
                if (!ConnectEthHwHandle(i, *busAddresses[i].ethAddress)) {
                    return false;
                }

                break;
#endif
            default:
                break;
        }
    }

    VeosCoSim_ServerCallbacks callbacks = {0};
    callbacks.logCallback = OnLogCallback;
    callbacks.simulationStopRequested = OnStopRequested;
    callbacks.simulationTerminateRequested = OnTerminateRequested;
#ifdef HAS_CAN
    callbacks.canMessageReceived = OnCanMessageReceived;
#endif
#ifdef HAS_LIN
    callbacks.linMessageReceived = OnLinMessageReceived;
#endif
#ifdef HAS_ETH
    callbacks.ethMessageReceived = OnEthMessageReceived;
#endif

    g_load(port, serverName, sampleTime, clientOptional, ioSignalsCount, ioSignals, busControllersCount, busControllers, callbacks);
    return true;
}

void VeosCoSim_Adapter_Unload(void) {
    free(g_busChannelHandles);
    g_busChannelHandles = NULL;
}

void VeosCoSim_Adapter_Start(void) {
    g_start(g_VEOS_CurrentSimulationCounter_1ns);
}

void VeosCoSim_Adapter_Stop(void) {
    g_stop(g_VEOS_CurrentSimulationCounter_1ns);
}

void VeosCoSim_Adapter_PreStep(void) {
#ifdef HAS_CAN
    if (g_canBoard) {
        DsCanBoard_update(g_canBoard);
    }
#endif

#ifdef HAS_LIN
    if (g_linBoard) {
        dslin_board_update(g_linBoard);
    }
#endif

    for (uint32_t i = 0; i < g_busControllersCount; i++) {
        switch (g_busControllers[i].busProtocol) {
#ifdef HAS_CAN
            case VeosCoSim_BusProtocol_CAN:
                FetchCanData(i);
                break;
#endif
#ifdef HAS_LIN
            case VeosCoSim_BusProtocol_LIN:
                FetchLinData(i);
                break;
#endif
#ifdef HAS_ETH
            case VeosCoSim_BusProtocol_ETH:
                FetchEthData(i);
                break;
#endif
            default:
                break;
        }
    }
}

void VeosCoSim_Adapter_Step(void) {
    g_step(g_VEOS_CurrentSimulationCounter_1ns);
}

void VeosCoSim_Adapter_OutputSignalRead(VeosCoSim_IoSignalId id, uint32_t* length, void* value) {
    g_outputSignalRead(id, length, value);
}

void VeosCoSim_Adapter_InputSignalWrite(VeosCoSim_IoSignalId id, uint32_t length, const void* value) {
    g_inputSignalWrite(id, length, value);
}
