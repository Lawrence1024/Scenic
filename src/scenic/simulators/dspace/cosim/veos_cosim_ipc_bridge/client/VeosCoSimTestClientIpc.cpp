#include "TcpEventClient.h"
#include "VeosCoSim.h"

#include <atomic>
#include <cstdint>
#include <cstdlib>
#include <cstring>
#include <iostream>
#include <map>
#include <sstream>
#include <string>

#include <windows.h>

namespace {
std::atomic<uint64_t> g_timeTriggerCmdCount{0};

TcpEventClient g_ipc;
bool g_ipcEnabled = false;
std::string g_ipcHost = "127.0.0.1";
unsigned short g_ipcPort = 50555;

// Cache of CoSim Outport (Direction_Write) signals by signal name, populated at
// connect time via VeosCoSim_IoGetAvailableSignalsMI. Used later to write commands
// that Scenic sends over the IPC reply channel.
struct OutportSpec {
    VeosCoSim_IoSignalId id;
    uint32_t length;
    VeosCoSim_DataType dataType;
};
std::map<std::string, OutportSpec> g_outportsByName;

const char* DataTypeToString(VeosCoSim_DataType dt) {
    switch (dt) {
        case VeosCoSim_DataType_Bool:    return "Bool";
        case VeosCoSim_DataType_Int8:    return "Int8";
        case VeosCoSim_DataType_Int16:   return "Int16";
        case VeosCoSim_DataType_Int32:   return "Int32";
        case VeosCoSim_DataType_Int64:   return "Int64";
        case VeosCoSim_DataType_UInt8:   return "UInt8";
        case VeosCoSim_DataType_UInt16:  return "UInt16";
        case VeosCoSim_DataType_UInt32:  return "UInt32";
        case VeosCoSim_DataType_UInt64:  return "UInt64";
        case VeosCoSim_DataType_Float32: return "Float32";
        case VeosCoSim_DataType_Float64: return "Float64";
        default: return "Unknown";
    }
}

const char* DirectionToString(VeosCoSim_Direction d) {
    switch (d) {
        case VeosCoSim_Direction_Read:  return "Read";   // signal flows from VEOS -> client (Inport)
        case VeosCoSim_Direction_Write: return "Write";  // signal flows from client -> VEOS (Outport)
        default: return "Unknown";
    }
}

// Narrow JSON parser for the specific shape of STEP replies:
//   {"reply":"STEP","outports":{"name1":value1,"name2":value2,...}}
// Extracts the outports object body. No nested objects supported.
// Returns the body substring (without surrounding braces), or empty string if
// the reply isn't a JSON STEP or outports key is absent.
std::string ExtractOutportsBody(const std::string& reply) {
    auto keyPos = reply.find("\"outports\"");
    if (keyPos == std::string::npos) {
        return std::string();
    }
    auto openBrace = reply.find('{', keyPos);
    if (openBrace == std::string::npos) {
        return std::string();
    }
    // Walk until matching close brace.
    int depth = 1;
    size_t i = openBrace + 1;
    while (i < reply.size() && depth > 0) {
        char c = reply[i];
        if (c == '{') ++depth;
        else if (c == '}') --depth;
        if (depth == 0) break;
        ++i;
    }
    if (depth != 0) return std::string();
    return reply.substr(openBrace + 1, i - openBrace - 1);
}

// Parse "name":number pairs out of the body and write each to the corresponding
// Outport via VeosCoSim_IoWriteMI. Returns the number of successful writes.
int WriteOutportsFromBody(VeosCoSim_Handle handle, const std::string& body) {
    int wrote = 0;
    size_t p = 0;
    while (p < body.size()) {
        // Skip whitespace and commas.
        while (p < body.size() && (body[p] == ' ' || body[p] == ',' || body[p] == '\t' || body[p] == '\n' || body[p] == '\r')) ++p;
        if (p >= body.size()) break;
        if (body[p] != '"') break;  // malformed
        ++p;
        size_t nameEnd = body.find('"', p);
        if (nameEnd == std::string::npos) break;
        std::string name = body.substr(p, nameEnd - p);
        p = nameEnd + 1;
        // Skip `:` and whitespace.
        while (p < body.size() && (body[p] == ' ' || body[p] == ':' || body[p] == '\t')) ++p;
        // Read value token until comma, close brace, or whitespace.
        size_t valEnd = p;
        while (valEnd < body.size()
               && body[valEnd] != ','
               && body[valEnd] != ' '
               && body[valEnd] != '\t'
               && body[valEnd] != '\n'
               && body[valEnd] != '\r') {
            ++valEnd;
        }
        std::string valueStr = body.substr(p, valEnd - p);
        p = valEnd;

        auto it = g_outportsByName.find(name);
        if (it == g_outportsByName.end()) {
            std::cerr << "[ipc]   outport WRITE SKIP unknown name='" << name << "'" << std::endl;
            continue;
        }
        const OutportSpec& spec = it->second;
        // Phase 1: only scalar Float64 outports. Arrays will come later if needed.
        if (spec.length != 1 || spec.dataType != VeosCoSim_DataType_Float64) {
            std::cerr << "[ipc]   outport WRITE SKIP (non-scalar-Float64) name='" << name
                      << "' length=" << spec.length
                      << " dataType=" << DataTypeToString(spec.dataType) << std::endl;
            continue;
        }
        double val = std::atof(valueStr.c_str());
        VeosCoSim_Result r = VeosCoSim_IoWriteMI(handle, spec.id, 1, &val);
        if (r != VeosCoSim_Result_OK) {
            std::cerr << "[ipc]   outport WRITE FAIL name='" << name
                      << "' value=" << val
                      << " result=" << static_cast<int>(r) << std::endl;
            continue;
        }
        ++wrote;
    }
    return wrote;
}

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

    // ----- Enumerate all IO signals exposed by the CoSim server -----
    // Goal: build a cache of Outport (Direction_Write) signals by name so we can later
    // call VeosCoSim_IoWriteMI(id, ...) when Scenic sends command values over IPC.
    // Also log every signal to stdout and over IPC (SIGNAL_INFO events) so run.log
    // captures the full catalog.
    {
        uint32_t sigCount = 0;
        const VeosCoSim_IoSignalInfo* sigInfos = nullptr;
        VeosCoSim_Result sigResult =
            VeosCoSim_IoGetAvailableSignalsMI(handle, &sigCount, &sigInfos);
        if (sigResult != VeosCoSim_Result_OK) {
            std::cerr << "[ipc] VeosCoSim_IoGetAvailableSignalsMI FAILED result="
                      << static_cast<int>(sigResult) << std::endl;
            std::ostringstream oss;
            oss << "{\"event\":\"SIGNAL_ENUM_ERROR\",\"result\":"
                << static_cast<int>(sigResult) << "}";
            SendJsonLine(oss.str());
        } else {
            std::cout << "[ipc] IoGetAvailableSignals: count=" << sigCount << std::endl;
            {
                std::ostringstream oss;
                oss << "{\"event\":\"SIGNAL_ENUM_BEGIN\",\"count\":" << sigCount << "}";
                SendJsonLine(oss.str());
            }
            uint32_t outportCount = 0;
            uint32_t inportCount = 0;
            for (uint32_t i = 0; i < sigCount; ++i) {
                const auto& s = sigInfos[i];
                const char* dir = DirectionToString(s.direction);
                const char* dt  = DataTypeToString(s.dataType);

                // Console log (one line per signal).
                std::cout << "[ipc]   signal id=" << s.id
                          << "  dir=" << dir
                          << "  type=" << dt
                          << "  length=" << s.length
                          << "  name=\"" << s.name << "\""
                          << std::endl;

                // Emit as an IPC JSON event so the Python listener can record it in run.log.
                std::ostringstream oss;
                oss << "{\"event\":\"SIGNAL_INFO\""
                    << ",\"id\":" << s.id
                    << ",\"direction\":\"" << dir << "\""
                    << ",\"dataType\":\""   << dt  << "\""
                    << ",\"length\":"       << s.length
                    << ",\"name\":\""       << EscapeJson(s.name) << "\""
                    << "}";
                SendJsonLine(oss.str());

                // Cache Write-direction signals for future outport writes.
                if (s.direction == VeosCoSim_Direction_Write) {
                    OutportSpec spec{ s.id, s.length, s.dataType };
                    g_outportsByName[std::string(s.name)] = spec;
                    ++outportCount;
                } else if (s.direction == VeosCoSim_Direction_Read) {
                    ++inportCount;
                }
            }
            std::cout << "[ipc] IoGetAvailableSignals: cached " << outportCount
                      << " outports, " << inportCount << " inports." << std::endl;
            std::ostringstream oss;
            oss << "{\"event\":\"SIGNAL_ENUM_END\",\"outports\":" << outportCount
                << ",\"inports\":" << inportCount << "}";
            SendJsonLine(oss.str());
        }
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

            std::ostringstream oss;
            oss << "{\"event\":\"TIME_TRIGGER\",\"source\":\"command\",\"sim_time\":" << simTime
                << ",\"count\":" << cmdCount << "}";

            // Send the TIME_TRIGGER event, then read a single line reply. The reply
            // may be either legacy plain "STEP" or a JSON envelope like
            //   {"reply":"STEP","outports":{"throttle_cmd":1.0, ...}}
            // In the JSON case, parse the outports and write each one to VEOS via
            // VeosCoSim_IoWriteMI BEFORE calling FinishCommandMI.
            bool sentOk = g_ipc.SendLine(oss.str() + "\n");
            std::string reply;
            bool receivedOk = sentOk && g_ipc.ReceiveLine(reply);

            if (!sentOk || !receivedOk) {
                std::cout << "[ipc] Scenic STEP FAILED for TIME_TRIGGER count="
                          << cmdCount << " (sent=" << sentOk
                          << " received=" << receivedOk << ")" << std::endl;
            } else {
                int wrote = 0;
                if (reply != "STEP") {
                    // JSON envelope — parse and write outports.
                    std::string body = ExtractOutportsBody(reply);
                    if (!body.empty()) {
                        wrote = WriteOutportsFromBody(handle, body);
                    }
                }
                // Light log: print once per tick only if we wrote something, or every
                // 1000 triggers for a heartbeat.
                if (wrote > 0 || (cmdCount % 1000) == 0) {
                    std::cout << "[ipc] TIME_TRIGGER count=" << cmdCount
                              << " sim_time=" << simTime
                              << " wrote_outports=" << wrote << std::endl;
                }
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