#include "TcpEventClient.h"
#include "VeosCoSim.h"

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <sstream>
#include <string>

#include <windows.h>

namespace {
std::atomic<uint64_t> g_timeTriggerCmdCount{0};

TcpEventClient g_ipc;
bool g_ipcEnabled = false;
std::string g_ipcHost = "127.0.0.1";
unsigned short g_ipcPort = 50555;

const char* SeverityToString(VeosCoSim_Severity severity) {
    switch (severity) {
        case VeosCoSim_Severity_Info:
            return "INFO";
        case VeosCoSim_Severity_Warning:
            return "WARNING";
        case VeosCoSim_Severity_Error:
            return "ERROR";
        case VeosCoSim_Severity_Trace:
            return "TRACE";
        default:
            return "UNKNOWN";
    }
}

std::string EscapeJson(const std::string& s) {
    std::ostringstream oss;
    for (char c : s) {
        switch (c) {
            case '\\': oss << "\\\\";
                break;
            case '"': oss << "\\\"";
                break;
            case '\n': oss << "\\n";
                break;
            case '\r': oss << "\\r";
                break;
            case '\t': oss << "\\t";
                break;
            default: oss << c;
                break;
        }
    }
    return oss.str();
}

bool SendJsonLine(const std::string& json) {
    if (!g_ipcEnabled || !g_ipc.IsConnected()) {
        return false;
    }
    return g_ipc.SendLine(json + "\n");
}

void SendHello() {
    SendJsonLine("{\"event\":\"HELLO\",\"message\":\"ipc connected\"}");
}

void LogCallback(VeosCoSim_Severity severity, const char* message) {
    const char* sev = SeverityToString(severity);
    const char* msg = (message != nullptr) ? message : "";

    std::cout << "[veos/" << sev << "] " << msg << std::endl;

    if (g_ipcEnabled && g_ipc.IsConnected()) {
        std::ostringstream oss;
        oss << "{\"event\":\"LOG\",\"severity\":\""
            << EscapeJson(sev)
            << "\",\"message\":\""
            << EscapeJson(msg)
            << "\"}";
        SendJsonLine(oss.str());
    }
}

void OnStartCallback(VeosCoSim_Time simTime, void*) {
    std::ostringstream oss;
    oss << "{\"event\":\"START\",\"sim_time\":" << simTime << "}";
    SendJsonLine(oss.str());
}

void OnStopCallback(VeosCoSim_Time simTime, void*) {
    std::ostringstream oss;
    oss << "{\"event\":\"STOP\",\"sim_time\":" << simTime << "}";
    SendJsonLine(oss.str());
}

void OnTerminateCallback(VeosCoSim_Time simTime, void*) {
    std::ostringstream oss;
    oss << "{\"event\":\"TERMINATE\",\"sim_time\":" << simTime << "}";
    SendJsonLine(oss.str());
}

void OnTimeTriggerCallback(VeosCoSim_Time simTime, void*) {
    std::ostringstream oss;
    oss << "{\"event\":\"TIME_TRIGGER_CALLBACK\",\"sim_time\":" << simTime << "}";
    SendJsonLine(oss.str());
}

const char* GetArgValue(int argc, char** argv, const char* key) {
    for (int i = 1; i + 1 < argc; ++i) {
        if (std::strcmp(argv[i], key) == 0) {
            return argv[i + 1];
        }
    }
    return nullptr;
}

}  // namespace

int main(int argc, char** argv) {
    const char* host = GetArgValue(argc, argv, "--host");
    const char* name = GetArgValue(argc, argv, "--name");
    const char* ipcHost = GetArgValue(argc, argv, "--ipc-host");
    const char* ipcPort = GetArgValue(argc, argv, "--ipc-port");

    if (host == nullptr || name == nullptr) {
        std::cerr
            << "Usage: VeosCoSimTestClientIpc.exe --host <ip> --name <server_name> "
               "[--ipc-host 127.0.0.1] [--ipc-port 50555]"
            << std::endl;
        return 2;
    }

    if (ipcHost != nullptr) {
        g_ipcHost = ipcHost;
    }
    if (ipcPort != nullptr) {
        g_ipcPort = static_cast<unsigned short>(std::atoi(ipcPort));
    }
    g_ipcEnabled = true;

    std::cout << "[ipc] Connecting to listener at " << g_ipcHost << ":" << g_ipcPort << " ..." << std::endl;
    if (!g_ipc.Connect(g_ipcHost, g_ipcPort)) {
        std::cerr << "[ipc] Failed to connect to listener at " << g_ipcHost << ":" << g_ipcPort << std::endl;
        return 3;
    }
    std::cout << "[ipc] Connected to listener." << std::endl;
    SendHello();

    VeosCoSim_Handle handle = VeosCoSim_CreateMI();
    if (handle == nullptr) {
        std::cerr << "ERROR: VeosCoSim_CreateMI failed." << std::endl;
        g_ipc.Disconnect();
        return 1;
    }

    VeosCoSim_Result connectResult = VeosCoSim_ConnectMI(handle, host, name, LogCallback);
    if (connectResult != VeosCoSim_Result_OK) {
        std::cerr << "ERROR: VeosCoSim_ConnectMI failed with result="
                  << static_cast<int>(connectResult) << std::endl;
        VeosCoSim_DestroyMI(handle);
        g_ipc.Disconnect();
        return 1;
    }

    VeosCoSim_RuntimeConfiguration cfg{};
    cfg.startSimulationCallback = OnStartCallback;
    cfg.stopSimulationCallback = OnStopCallback;
    cfg.terminateSimulationCallback = OnTerminateCallback;
    cfg.timeTriggerCallback = OnTimeTriggerCallback;
    cfg.ioReadCallback = nullptr;
    cfg.canMessageReceivedCallback = nullptr;
    cfg.linMessageReceivedCallback = nullptr;
    cfg.ethMessageReceivedCallback = nullptr;
    cfg.userData = nullptr;

    VeosCoSim_Result startResult = VeosCoSim_StartNonBlockingMI(handle, cfg);
    if (startResult != VeosCoSim_Result_OK) {
        std::cerr << "ERROR: VeosCoSim_StartNonBlockingMI failed with result="
                  << static_cast<int>(startResult) << std::endl;
        VeosCoSim_DisconnectMI(handle);
        VeosCoSim_DestroyMI(handle);
        g_ipc.Disconnect();
        return 1;
    }

    std::cout << "[ipc] Entering command loop..." << std::endl;

    while (true) {
        VeosCoSim_Time simTime = 0;
        VeosCoSim_Command command = VeosCoSim_Command_None;

        VeosCoSim_Result nextResult = VeosCoSim_GetNextCommandMI(handle, &simTime, &command);

        if (nextResult == VeosCoSim_Result_Empty) {
            Sleep(1);
            continue;
        }

        if (nextResult != VeosCoSim_Result_OK) {
            std::cerr << "ERROR: VeosCoSim_GetNextCommandMI failed with result="
                      << static_cast<int>(nextResult) << std::endl;
            break;
        }

        if (command == VeosCoSim_Command_TimeTrigger) {
            const auto cmdCount = ++g_timeTriggerCmdCount;

            std::cout << "[ipc] Command TIME_TRIGGER count=" << cmdCount
                      << " sim_time=" << simTime
                      << " -> waiting for Scenic STEP before FinishCommandMI"
                      << std::endl;

            std::ostringstream oss;
            oss << "{\"event\":\"TIME_TRIGGER\",\"source\":\"command\",\"sim_time\":" << simTime
                << ",\"count\":" << cmdCount << "}";

            bool ok = g_ipc.SendAndWaitLine(oss.str(), "STEP");

            if (ok) {
                std::cout << "[ipc] Scenic STEP received for TIME_TRIGGER count="
                          << cmdCount << std::endl;
            } else {
                std::cout << "[ipc] Scenic STEP FAILED for TIME_TRIGGER count="
                          << cmdCount << std::endl;
            }
        } else if (command == VeosCoSim_Command_Start) {
            std::cout << "[ipc] Command START sim_time=" << simTime << std::endl;
            std::ostringstream oss;
            oss << "{\"event\":\"START_CMD\",\"sim_time\":" << simTime << "}";
            SendJsonLine(oss.str());
        } else if (command == VeosCoSim_Command_Stop) {
            std::cout << "[ipc] Command STOP sim_time=" << simTime << std::endl;
            std::ostringstream oss;
            oss << "{\"event\":\"STOP_CMD\",\"sim_time\":" << simTime << "}";
            SendJsonLine(oss.str());
        } else if (command == VeosCoSim_Command_Terminate) {
            std::cout << "[ipc] Command TERMINATE sim_time=" << simTime << std::endl;
            std::ostringstream oss;
            oss << "{\"event\":\"TERMINATE_CMD\",\"sim_time\":" << simTime << "}";
            SendJsonLine(oss.str());
        } else {
            std::cout << "[ipc] Command OTHER value=" << static_cast<int>(command)
                      << " sim_time=" << simTime << std::endl;
        }

        VeosCoSim_Result finishResult = VeosCoSim_FinishCommandMI(handle);
        if (finishResult != VeosCoSim_Result_OK) {
            std::cerr << "ERROR: VeosCoSim_FinishCommandMI failed with result="
                      << static_cast<int>(finishResult) << std::endl;
            break;
        }

        if (command == VeosCoSim_Command_Terminate) {
            break;
        }
    }

    VeosCoSim_DisconnectMI(handle);
    VeosCoSim_DestroyMI(handle);
    g_ipc.Disconnect();
    return 0;
}