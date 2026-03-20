#include <cstdint>
#include <cstdio>
#include "VeosCoSim.h"

struct BridgeHandle {
    VeosCoSim_Handle inner{nullptr};
};

static void LogMessage(VeosCoSim_Severity severity, const char* logMessage) {
    const char* sev = "INFO";
    switch (severity) {
        case VeosCoSim_Severity_Warning: sev = "WARN"; break;
        case VeosCoSim_Severity_Error: sev = "ERROR"; break;
        case VeosCoSim_Severity_Trace: sev = "TRACE"; break;
        case VeosCoSim_Severity_Info:
        default: sev = "INFO"; break;
    }
    std::fprintf(stderr, "[veos/%s] %s\n", sev, logMessage ? logMessage : "");
    std::fflush(stderr);
}

extern "C" {
__declspec(dllexport) void* vcp_create() {
    auto* h = new BridgeHandle();
    h->inner = VeosCoSim_CreateMI();
    if (!h->inner) {
        delete h;
        return nullptr;
    }
    return h;
}

__declspec(dllexport) void vcp_destroy(void* raw) {
    if (!raw) return;
    auto* h = static_cast<BridgeHandle*>(raw);
    if (h->inner) {
        VeosCoSim_DestroyMI(h->inner);
        h->inner = nullptr;
    }
    delete h;
}

__declspec(dllexport) int32_t vcp_connect(void* raw, const char* remote_ip, const char* server_name) {
    if (!raw || !remote_ip || !server_name) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_ConnectMI(h->inner, remote_ip, server_name, LogMessage);
}

__declspec(dllexport) int32_t vcp_connect2(void* raw, const char* remote_ip, const char* server_name, uint16_t remote_port, uint16_t local_port) {
    if (!raw || !remote_ip) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    VeosCoSim_ConnectConfiguration cfg{};
    cfg.remoteIpAddress = remote_ip;
    cfg.name = server_name;
    cfg.logCallback = LogMessage;
    cfg.remotePort = remote_port;
    cfg.localPort = local_port;
    return VeosCoSim_ConnectMI2(h->inner, cfg);
}

__declspec(dllexport) int32_t vcp_disconnect(void* raw) {
    if (!raw) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_DisconnectMI(h->inner);
}

__declspec(dllexport) int32_t vcp_start_nonblocking(void* raw) {
    if (!raw) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    VeosCoSim_RuntimeConfiguration cfg{};
    return VeosCoSim_StartNonBlockingMI(h->inner, cfg);
}

__declspec(dllexport) int32_t vcp_get_next_command(void* raw, int64_t* simulation_time, int32_t* next_command) {
    if (!raw || !simulation_time || !next_command) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    VeosCoSim_Command cmd = VeosCoSim_Command_None;
    auto res = VeosCoSim_GetNextCommandMI(h->inner, simulation_time, &cmd);
    *next_command = static_cast<int32_t>(cmd);
    return res;
}

__declspec(dllexport) int32_t vcp_finish_command(void* raw) {
    if (!raw) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_FinishCommandMI(h->inner);
}

__declspec(dllexport) int32_t vcp_io_get_available_signals(void* raw, uint32_t* count, const VeosCoSim_IoSignalInfo** infos) {
    if (!raw || !count || !infos) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_IoGetAvailableSignalsMI(h->inner, count, infos);
}

__declspec(dllexport) int32_t vcp_io_read(void* raw, uint32_t id, uint32_t* length, void* value) {
    if (!raw || !length || !value) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_IoReadMI(h->inner, id, length, value);
}

__declspec(dllexport) int32_t vcp_io_write(void* raw, uint32_t id, uint32_t length, const void* value) {
    if (!raw || (!value && length > 0)) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_IoWriteMI(h->inner, id, length, value);
}

__declspec(dllexport) int32_t vcp_get_general_info(void* raw, VeosCoSim_GeneralInfo* info) {
    if (!raw || !info) return VeosCoSim_Result_Argument;
    auto* h = static_cast<BridgeHandle*>(raw);
    return VeosCoSim_GetGeneralInfoMI(h->inner, info);
}
}
