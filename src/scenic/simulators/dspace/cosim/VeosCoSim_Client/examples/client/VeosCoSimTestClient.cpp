// Copyright dSPACE GmbH. All rights reserved.

#include <cstdint>
#include <cstdio>
#include <cstring>
#include <atomic>
#include <condition_variable>
#include <mutex>
#include <sstream>
#include <string>
#include <thread>
#include <vector>

#ifdef _WIN32
#include <winsock2.h>
#include <ws2tcpip.h>
#pragma comment(lib, "Ws2_32.lib")
#else
#include <arpa/inet.h>
#include <netinet/in.h>
#include <sys/socket.h>
#include <unistd.h>
#endif

#include "ClientServerTestHelper.h"
#include "Generator.h"
#include "VeosCoSim.h"

namespace {

VeosCoSim_Handle g_handle;
uint32_t g_busControllersCount;
const VeosCoSim_BusChannelInfo* g_busControllers;
uint32_t g_ioSignalsCount;
const VeosCoSim_IoSignalInfo* g_ioSignals;
std::mutex g_apiMutex;

bool g_bridgeEnabled = false;
uint16_t g_bridgePort = 0;
std::atomic<bool> g_bridgeShutdown{false};
std::thread g_bridgeThread;
std::mutex g_stepMutex;
std::condition_variable g_stepCv;
bool g_waitForStepRequest = false;
bool g_stepAvailable = false;
VeosCoSim_Time g_stepTime = 0;

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
    {
        std::lock_guard<std::mutex> lock(g_apiMutex);
        SendSomeData(simulationTime);
    }
    if (!g_bridgeEnabled) {
        return;
    }
    std::unique_lock<std::mutex> lock(g_stepMutex);
    if (!g_waitForStepRequest) {
        return;
    }
    g_stepTime = simulationTime;
    g_stepAvailable = true;
    g_stepCv.notify_all();
    g_stepCv.wait(lock, [] { return !g_waitForStepRequest || g_bridgeShutdown.load(); });
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

std::string ReadLineFromSocket(int socketFd) {
    std::string line;
    while (true) {
        char ch = 0;
#ifdef _WIN32
        const int read = recv(socketFd, &ch, 1, 0);
#else
        const int read = static_cast<int>(recv(socketFd, &ch, 1, 0));
#endif
        if (read <= 0) {
            return {};
        }
        if (ch == '\n') {
            return line;
        }
        if (ch != '\r') {
            line.push_back(ch);
        }
    }
}

void WriteLineToSocket(int socketFd, const std::string& line) {
    const std::string message = line + "\n";
#ifdef _WIN32
    (void)send(socketFd, message.c_str(), static_cast<int>(message.size()), 0);
#else
    (void)send(socketFd, message.c_str(), message.size(), 0);
#endif
}

void BridgeServerThread() {
#ifdef _WIN32
    WSADATA wsaData;
    if (WSAStartup(MAKEWORD(2, 2), &wsaData) != 0) {
        return;
    }
#endif
    const int listenFd = static_cast<int>(socket(AF_INET, SOCK_STREAM, IPPROTO_TCP));
    if (listenFd < 0) {
        return;
    }
    sockaddr_in addr{};
    addr.sin_family = AF_INET;
    addr.sin_port = htons(g_bridgePort);
    addr.sin_addr.s_addr = htonl(INADDR_LOOPBACK);
    if (bind(listenFd, reinterpret_cast<sockaddr*>(&addr), sizeof(addr)) != 0) {
        return;
    }
    if (listen(listenFd, 1) != 0) {
        return;
    }
    printf("Bridge server listening on 127.0.0.1:%u\n", g_bridgePort);

    const int clientFd = static_cast<int>(accept(listenFd, nullptr, nullptr));
    if (clientFd < 0) {
        return;
    }
    WriteLineToSocket(clientFd, "OK VeosCoSim bridge ready");

    while (!g_bridgeShutdown.load()) {
        const std::string line = ReadLineFromSocket(clientFd);
        if (line.empty()) {
            break;
        }
        if (line == "PING") {
            WriteLineToSocket(clientFd, "PONG");
            continue;
        }
        if (line == "STEP") {
            VeosCoSim_Time stepTime = 0;
            {
                std::unique_lock<std::mutex> lock(g_stepMutex);
                g_waitForStepRequest = true;
                g_stepAvailable = false;
                g_stepCv.wait(lock, [] { return g_stepAvailable || g_bridgeShutdown.load(); });
                stepTime = g_stepTime;
                g_waitForStepRequest = false;
            }
            g_stepCv.notify_all();
            WriteLineToSocket(clientFd, "STEP " + std::to_string(stepTime));
            continue;
        }
        if (line == "QUIT") {
            WriteLineToSocket(clientFd, "BYE");
            break;
        }
        WriteLineToSocket(clientFd, "ERR unknown command");
    }

#ifdef _WIN32
    closesocket(clientFd);
    closesocket(listenFd);
    WSACleanup();
#else
    close(clientFd);
    close(listenFd);
#endif
}

VeosCoSim_Result HostClient(std::string_view host, std::string_view serverName, uint16_t bridgePort) {
    CHECK_RESULT(Connect(host, serverName));
    if (bridgePort > 0) {
        g_bridgeEnabled = true;
        g_bridgePort = bridgePort;
        g_bridgeThread = std::thread(BridgeServerThread);
    }

    std::thread(RunCoSimulationBlocking).detach();

    while (true) {
        switch (GetChar()) {
            case CTRL('c'):
                g_bridgeShutdown.store(true);
                g_stepCv.notify_all();
                if (g_bridgeThread.joinable()) {
                    g_bridgeThread.join();
                }
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
    uint16_t bridgePort = 0;

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

        if (strcmp(argv[i], "--bridge-port") == 0) {
            if (++i < argc) {
                bridgePort = static_cast<uint16_t>(std::stoi(argv[i]));
            } else {
                printf("No bridge port specified.\n");
                return 1;
            }
        }
    }

    const VeosCoSim_Result result = HostClient(host, serverName, bridgePort);

    return result == VeosCoSim_Result_OK ? 0 : 1;
}
