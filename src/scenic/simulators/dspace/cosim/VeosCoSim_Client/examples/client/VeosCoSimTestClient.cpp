// Copyright dSPACE GmbH. All rights reserved.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <string>
#include <thread>
#include <vector>

#include "ClientServerTestHelper.h"
#include "Generator.h"
#include "VeosCoSim.h"

namespace {

VeosCoSim_Handle g_handle;
uint32_t g_busControllersCount;
const VeosCoSim_BusChannelInfo* g_busControllers;
uint32_t g_ioSignalsCount;
const VeosCoSim_IoSignalInfo* g_ioSignals;

#define CHECK_RESULT(action)                  \
    do {                                      \
        VeosCoSim_Result _result = action;    \
        if (_result != VeosCoSim_Result_OK) { \
            return _result;                   \
        }                                     \
    } while (0)

std::string BusProtocolToString(VeosCoSim_BusProtocol busProtocol) {
    switch (busProtocol) {
        case VeosCoSim_BusProtocol_CAN:
            return "CAN";
        case VeosCoSim_BusProtocol_LIN:
            return "LIN";
        case VeosCoSim_BusProtocol_ETH:
            return "ETH";
        default:
            return "<unknown bus protocol>";
    }
}

std::string DataTypeToString(VeosCoSim_DataType dataType) {
    switch (dataType) {
        case VeosCoSim_DataType_Bool:
            return "bool";
        case VeosCoSim_DataType_Int8:
            return "int8";
        case VeosCoSim_DataType_Int16:
            return "int16";
        case VeosCoSim_DataType_Int32:
            return "int32";
        case VeosCoSim_DataType_Int64:
            return "int64";
        case VeosCoSim_DataType_UInt8:
            return "uint8";
        case VeosCoSim_DataType_UInt16:
            return "uint16";
        case VeosCoSim_DataType_UInt32:
            return "uint32";
        case VeosCoSim_DataType_UInt64:
            return "uint64";
        case VeosCoSim_DataType_Float32:
            return "float32";
        case VeosCoSim_DataType_Float64:
            return "float64";
        default:
            return "<unknown data type>";
    }
}

std::string DirectionToString(VeosCoSim_Direction direction) {
    switch (direction) {
        case VeosCoSim_Direction_Read:
            return "IN";
        case VeosCoSim_Direction_Write:
            return "OUT";
        default:
            return "<unknown direction>";
    }
}

const VeosCoSim_IoSignalInfo* GetIoSignalInfo(VeosCoSim_IoSignalId id) {
    if (id >= g_ioSignalsCount) {
        return nullptr;
    }

    return &g_ioSignals[id];
}

const VeosCoSim_BusChannelInfo* GetBusControllerInfo(VeosCoSim_ChannelId id) {
    if (id >= g_busControllersCount) {
        return nullptr;
    }

    return &g_busControllers[id];
}

void OnStartCallback(VeosCoSim_Time simulationTime, void* /*unused*/) {
    printf("Simulation started at %f s.\n", TimeToSeconds(simulationTime));
}

void OnStopCallback(VeosCoSim_Time simulationTime, void* /*unused*/) {
    printf("Simulation stopped at %f s.\n", TimeToSeconds(simulationTime));
}

VeosCoSim_Result SendSomeData(VeosCoSim_Time simulationTime) {
    static int64_t lastHalfSecond = -1;
    static int64_t counter = 0;
    const int64_t currentHalfSecond = simulationTime / 500000000;
    if (currentHalfSecond == lastHalfSecond) {
        return VeosCoSim_Result_OK;
    }

    lastHalfSecond = currentHalfSecond;
    counter++;

    if (IsSendingIoSignalsEnabled() && ((counter % 4) == 0)) {
        for (uint32_t i = 0; i < g_ioSignalsCount; i++) {
            const VeosCoSim_IoSignalInfo& signal = g_ioSignals[i];
            if (signal.direction != VeosCoSim_Direction_Write) {
                continue;
            }

            const uint32_t length = signal.sizeKind == VeosCoSim_SizeKind_Fixed ? signal.length : GenerateRandom(0U, signal.length);
            std::vector<uint8_t> data;
            data.resize(static_cast<size_t>(length) * GetDataTypeSize(signal.dataType));
            FillWithRandom(data.data(), data.size());
            CHECK_RESULT(VeosCoSim_IoWriteMI(g_handle, signal.id, length, data.data()));
        }
    }

    for (uint32_t i = 0; i < g_busControllersCount; i++) {
        const VeosCoSim_BusChannelInfo& controller = g_busControllers[i];
        switch (controller.busProtocol) {
            case VeosCoSim_BusProtocol_CAN: {
                if (IsSendingCanMessagesEnabled() && ((counter % 4) == 1)) {
                    VeosCoSim_CanMessage message{};
                    FillMessage(controller.id, message);
                    CHECK_RESULT(VeosCoSim_CanTransmitMessageMI(g_handle, &message));
                }

                break;
            }
            case VeosCoSim_BusProtocol_ETH: {
                if (IsSendingEthMessagesEnabled() && ((counter % 4) == 2)) {
                    VeosCoSim_EthMessage message{};
                    FillMessage(controller.id, message);
                    CHECK_RESULT(VeosCoSim_EthTransmitMessageMI(g_handle, &message));
                }

                break;
            }
            case VeosCoSim_BusProtocol_LIN: {
                if (IsSendingLinMessagesEnabled() && ((counter % 4) == 3)) {
                    VeosCoSim_LinMessage message{};
                    FillMessage(controller.id, message);
                    CHECK_RESULT(VeosCoSim_LinTransmitMessageMI(g_handle, &message));
                }

                break;
            }
            default:
                return VeosCoSim_Result_Error;
        }
    }

    return VeosCoSim_Result_OK;
}

void OnTimeTriggerCallback(VeosCoSim_Time simulationTime, void* /*unused*/) {
    SendSomeData(simulationTime);
}

void OnInputSignalChanged(VeosCoSim_Time simulationTime, VeosCoSim_IoSignalId id, uint32_t length, const void* value, void* /*unused*/) {
    const VeosCoSim_IoSignalInfo* signalInfo = GetIoSignalInfo(id);
    if (!signalInfo) {
        printf("Unknown input signal changed: %u\n", id);
        return;
    }

    LogIoData(signalInfo->name, simulationTime, signalInfo->dataType, length, value);
}

void OnCanReceiveMessageCallback(VeosCoSim_Time simulationTime, const VeosCoSim_CanMessage* message, void* /*unused*/) {
    const std::string type = (message->flags & VEOSCOSIM_CAN_MESSAGE_FLAG_FD) != 0 ? "CANFD" : "CAN";
    const VeosCoSim_BusChannelInfo* busControllerInfo = GetBusControllerInfo(message->channelId);
    const std::string controllerName = busControllerInfo ? busControllerInfo->controllerName : "<unknown bus controller>";

    LogCanMessage(controllerName, simulationTime, message->identifier, message->length, message->data, type);
}

void OnLinReceiveMessageCallback(VeosCoSim_Time simulationTime, const VeosCoSim_LinMessage* message, void* /*unused*/) {
    const VeosCoSim_BusChannelInfo* busControllerInfo = GetBusControllerInfo(message->channelId);
    const std::string controllerName = busControllerInfo ? busControllerInfo->controllerName : "<unknown bus controller>";

    LogLinMessage(controllerName, simulationTime, message->identifier, message->length, message->data);
}

void OnEthReceiveMessageCallback(VeosCoSim_Time simulationTime, const VeosCoSim_EthMessage* message, void* /*unused*/) {
    const VeosCoSim_BusChannelInfo* busControllerInfo = GetBusControllerInfo(message->channelId);
    const std::string controllerName = busControllerInfo ? busControllerInfo->controllerName : "<unknown bus controller>";
    const uint8_t* data = message->data;
    const uint32_t length = message->length;

    LogEthMessage(controllerName, simulationTime, length, data);
}

[[noreturn]] void RunCoSimulationBlocking() {
    VeosCoSim_RuntimeConfiguration config = {};
    config.startSimulationCallback = OnStartCallback;
    config.stopSimulationCallback = OnStopCallback;
    config.timeTriggerCallback = OnTimeTriggerCallback;
    config.ioReadCallback = OnInputSignalChanged;
    config.canMessageReceivedCallback = OnCanReceiveMessageCallback;
    config.linMessageReceivedCallback = OnLinReceiveMessageCallback;
    config.ethMessageReceivedCallback = OnEthReceiveMessageCallback;

    (void)VeosCoSim_RunMI(g_handle, config);
    (void)VeosCoSim_DisconnectMI(g_handle);
    exit(0);  // NOLINT(concurrency-mt-unsafe)
}

VeosCoSim_Result Connect(std::string_view host, std::string_view serverName) {
    g_handle = VeosCoSim_CreateMI();
    if (!g_handle) {
        return VeosCoSim_Result_Error;
    }

    CHECK_RESULT(VeosCoSim_ConnectMI(g_handle, host.data(), serverName.data(), LogMessage));

    printf("Successfully connected.\n\n");

    CHECK_RESULT(VeosCoSim_GetAvailableChannelsMI(g_handle, &g_busControllersCount, &g_busControllers));
    if (g_busControllersCount > 0) {
        printf("Found the following bus controllers:\n");
        for (uint32_t i = 0; i < g_busControllersCount; i++) {
            printf("  %s (id: %u, protocol: %s)\n",
                   g_busControllers[i].controllerName,
                   g_busControllers[i].id,
                   BusProtocolToString(g_busControllers[i].busProtocol).c_str());
        }

        printf("\n");
    }

    CHECK_RESULT(VeosCoSim_IoGetAvailableSignalsMI(g_handle, &g_ioSignalsCount, &g_ioSignals));
    if (g_ioSignalsCount > 0) {
        printf("Found the following IO signals:\n");
        for (uint32_t i = 0; i < g_ioSignalsCount; i++) {
            printf("  %s (id: %u, data type: %s, direction: %s, length: %u)\n",
                   g_ioSignals[i].name,
                   g_ioSignals[i].id,
                   DataTypeToString(g_ioSignals[i].dataType).c_str(),
                   DirectionToString(g_ioSignals[i].direction).c_str(),
                   g_ioSignals[i].length);
        }

        printf("\n");
    }

    return VeosCoSim_Result_OK;
}

VeosCoSim_Result HostClient(std::string_view host, std::string_view serverName) {
    CHECK_RESULT(Connect(host, serverName));

    std::thread(RunCoSimulationBlocking).detach();

    while (true) {
        switch (GetChar()) {
            case CTRL('c'):
                return VeosCoSim_DisconnectMI(g_handle);
            case '1':
                SwitchSendingIoSignals();
                break;
            case '2':
                SwitchSendingCanMessages();
                break;
            case '3':
                SwitchSendingEthMessages();
                break;
            case '4':
                SwitchSendingLinMessages();
                break;
            default:
                LogMessage(VeosCoSim_Severity_Error, "Unknown key.");
                break;
        }
    }
}

}  // namespace

int main(int argc, char** argv) {
    std::string host = "192.168.100.101";
    std::string serverName = "CoSimServerScenic";

    for (int i = 1; i < argc; i++) {
        if (strcmp(argv[i], "--host") == 0) {
            if (++i < argc) {
                host = argv[i];
            } else {
                printf("No host specified.\n");
                return 1;
            }
        }

        if (strcmp(argv[i], "--name") == 0) {
            if (++i < argc) {
                serverName = argv[i];
            } else {
                printf("No name specified.\n");
                return 1;
            }
        }
    }

    const VeosCoSim_Result result = HostClient(host, serverName);

    return result == VeosCoSim_Result_OK ? 0 : 1;
}
