#include <array>
#include <iostream>
#include <string>
#include <unordered_map>
#include <vector>

#include "DsVeosCoSim/DsVeosCoSim.h"

struct SignalSpec {
    const char* name;
    const char* portName;
    uint32_t length;
};

struct SignalBinding {
    DsVeosCoSim_IoSignalId id{};
    uint32_t length{};
    std::string portName;
};

struct SignalRegistry {
    std::unordered_map<std::string, SignalBinding> incomingByName;
    std::unordered_map<std::string, SignalBinding> outgoingByName;
    std::unordered_map<std::string, std::vector<std::string>> incomingByPort;
    std::unordered_map<std::string, std::vector<std::string>> outgoingByPort;
};

struct AppContext {
    DsVeosCoSim_Handle handle{};
    SignalRegistry signals;
    std::vector<double> fellowVelocityX{std::vector<double>(30, 0.0)};
    std::vector<double> fellowVelocityKmH{std::vector<double>(30, 0.0)};
};

// Keep this list in sync with cosim_server_config.json.
static constexpr std::array<SignalSpec, 16> kIncomingSignalSpecs{{
    {"Pos_x_Vehicle_CoorSys_E_m", "Ego_State", 1},
    {"Pos_y_Vehicle_CoorSys_E_m", "Ego_State", 1},
    {"Pos_z_Vehicle_CoorSys_E_m", "Ego_State", 1},
    {"Angle_Yaw_Vehicle_CoorSys_E_deg", "Ego_State", 1},
    {"v_x_Vehicle_CoG_m_s", "Ego_State", 1},
    {"v_y_Vehicle_CoG_m_s", "Ego_State", 1},
    {"pos_x_V_m", "Fellow_States", 30},
    {"pos_y_V_m", "Fellow_States", 30},
    {"pos_z_V_m", "Fellow_States", 30},
    {"angle_yaw_deg", "Fellow_States", 30},
    {"velocity_x_m_s", "Fellow_States", 30},
    {"velocity_y_m_s", "Fellow_States", 30},
    {"velocity_z_m_s", "Fellow_States", 30},
    {"acceleration_x_m_s2", "Fellow_States", 30},
    {"acceleration_y_m_s2", "Fellow_States", 30},
    {"acceleration_z_m_s2", "Fellow_States", 30},
}};

// Keep this list in sync with cosim_server_config.json.
static constexpr std::array<SignalSpec, 8> kOutgoingSignalSpecs{{
    {"throttle_cmd", "Ego_Control", 1},
    {"brake_cmd_front", "Ego_Control", 1},
    {"brake_cmd_rear", "Ego_Control", 1},
    {"steering_cmd_deg", "Ego_Control", 1},
    {"gear_cmd", "Ego_Control", 1},
    {"Pos_ClutchPedal", "Ego_Control", 1},
    {"v_fellows_external_km_h", "Fellow_Control", 30},
    {"d_fellows_external_m", "Fellow_Control", 30},
}};

template <size_t N>
bool BuildSignalBindings(const DsVeosCoSim_IoSignal* discoveredSignals,
                         uint32_t discoveredCount,
                         const std::array<SignalSpec, N>& expectedSpecs,
                         std::unordered_map<std::string, SignalBinding>& byName,
                         std::unordered_map<std::string, std::vector<std::string>>& byPort,
                         const char* directionLabel) {
    // Build a map of discovered signals by their full name (PortName/SignalName format)
    std::unordered_map<std::string, const DsVeosCoSim_IoSignal*> discoveredByFullName;
    for (uint32_t i = 0; i < discoveredCount; ++i) {
        discoveredByFullName[discoveredSignals[i].name] = &discoveredSignals[i];
    }

    for (const SignalSpec& spec : expectedSpecs) {
        // Construct the expected full name in PortName/SignalName format
        std::string expectedFullName = std::string(spec.portName) + "/" + spec.name;
        
        const auto it = discoveredByFullName.find(expectedFullName);
        if (it == discoveredByFullName.end()) {
            std::cout << "ERROR Missing " << directionLabel << " signal '" << expectedFullName << "'.\n";
            return false;
        }

        const DsVeosCoSim_IoSignal& signal = *it->second;
        if (signal.length != spec.length) {
            std::cout << "ERROR Length mismatch for " << directionLabel << " signal '" << spec.name << "'. "
                      << "Expected " << spec.length << ", got " << signal.length << ".\n";
            return false;
        }

        byName[spec.name] = SignalBinding{signal.id, spec.length, spec.portName};
        byPort[spec.portName].push_back(spec.name);
    }

    return true;
}

bool ReadIncomingDouble(AppContext& app, const char* signalName, double& value) {
    const auto it = app.signals.incomingByName.find(signalName);
    if (it == app.signals.incomingByName.end()) {
        std::cout << "ERROR Unknown incoming signal '" << signalName << "'.\n";
        return false;
    }

    uint32_t length{};
    if (DsVeosCoSim_ReadIncomingSignal(app.handle, it->second.id, &length, &value) != DsVeosCoSim_Result_Ok) {
        return false;
    }

    return length == it->second.length;
}

bool ReadIncomingArray(AppContext& app, const char* signalName, std::vector<double>& values) {
    const auto it = app.signals.incomingByName.find(signalName);
    if (it == app.signals.incomingByName.end()) {
        std::cout << "ERROR Unknown incoming signal '" << signalName << "'.\n";
        return false;
    }

    values.assign(it->second.length, 0.0);
    uint32_t length{};
    if (DsVeosCoSim_ReadIncomingSignal(app.handle, it->second.id, &length, values.data()) != DsVeosCoSim_Result_Ok) {
        return false;
    }

    return length == it->second.length;
}

bool WriteOutgoingDouble(AppContext& app, const char* signalName, double value) {
    const auto it = app.signals.outgoingByName.find(signalName);
    if (it == app.signals.outgoingByName.end()) {
        std::cout << "ERROR Unknown outgoing signal '" << signalName << "'.\n";
        return false;
    }

    return DsVeosCoSim_WriteOutgoingSignal(app.handle, it->second.id, it->second.length, &value) ==
           DsVeosCoSim_Result_Ok;
}

bool WriteOutgoingArray(AppContext& app, const char* signalName, const std::vector<double>& values) {
    const auto it = app.signals.outgoingByName.find(signalName);
    if (it == app.signals.outgoingByName.end()) {
        std::cout << "ERROR Unknown outgoing signal '" << signalName << "'.\n";
        return false;
    }

    if (values.size() != it->second.length) {
        std::cout << "ERROR Invalid outgoing vector length for signal '" << signalName << "'.\n";
        return false;
    }

    return DsVeosCoSim_WriteOutgoingSignal(app.handle, it->second.id, it->second.length, values.data()) ==
           DsVeosCoSim_Result_Ok;
}

void OnLogCallback(DsVeosCoSim_Severity severity, const char* logMessage) {
    switch (severity) {
        case DsVeosCoSim_Severity_Error:
            std::cout << "ERROR " << logMessage << "\n";
            break;
        case DsVeosCoSim_Severity_Warning:
            std::cout << "WARN  " << logMessage << "\n";
            break;
        case DsVeosCoSim_Severity_Info:
            std::cout << "INFO  " << logMessage << "\n";
            break;
        case DsVeosCoSim_Severity_Trace:
            std::cout << "TRACE " << logMessage << "\n";
            break;
    }
}

// Called after every interval specified in the StepSize parameter of the Example.json file
void OnEndStep(DsVeosCoSim_SimulationTime simulationTime, void* userData) {
    auto* app = static_cast<AppContext*>(userData);

    double egoVelocityX{};
    if (!ReadIncomingDouble(*app, "v_x_Vehicle_CoG_m_s", egoVelocityX)) {
        DsVeosCoSim_Disconnect(app->handle);
        return;
    }
    std::cout << "Ego velocity x: " << egoVelocityX << " m/s at "
              << DSVEOSCOSIM_SIMULATION_TIME_TO_SECONDS(simulationTime) << " s.\n";

    if (!ReadIncomingArray(*app, "velocity_x_m_s", app->fellowVelocityX)) {
        DsVeosCoSim_Disconnect(app->handle);
        return;
    }

    if ((simulationTime % 100) == 0) {  // Only every 10 milliseconds
        for (size_t i = 0; i < app->fellowVelocityX.size(); ++i) {
            app->fellowVelocityKmH[i] = app->fellowVelocityX[i] * 3.6;
        }

        const double throttleCmd = egoVelocityX * 0.05;
        if (!WriteOutgoingDouble(*app, "throttle_cmd", throttleCmd) ||
            !WriteOutgoingArray(*app, "v_fellows_external_km_h", app->fellowVelocityKmH)) {
            DsVeosCoSim_Disconnect(app->handle);
            return;
        }
    }
}

// Called when the simulation starts
void OnStarted(DsVeosCoSim_SimulationTime simulationTime, void* userData) {
    std::cout << "Simulation started at " << DSVEOSCOSIM_SIMULATION_TIME_TO_SECONDS(simulationTime) << " s.\n";
}

// Called when the simulation stops
void OnStopped(DsVeosCoSim_SimulationTime simulationTime, void* userData) {
    std::cout << "Simulation stopped at " << DSVEOSCOSIM_SIMULATION_TIME_TO_SECONDS(simulationTime) << " s.\n";
}

// Called when the simulation pauses
void OnPaused(DsVeosCoSim_SimulationTime simulationTime, void* userData) {
    std::cout << "Simulation paused at " << DSVEOSCOSIM_SIMULATION_TIME_TO_SECONDS(simulationTime) << " s.\n";
}

// Called when the simulation continues
void OnContinued(DsVeosCoSim_SimulationTime simulationTime, void* userData) {
    std::cout << "Simulation continued at " << DSVEOSCOSIM_SIMULATION_TIME_TO_SECONDS(simulationTime) << " s.\n";
}

int main() {
    // Step 1: Enable logging
    DsVeosCoSim_SetLogCallback(OnLogCallback);

    // Step 2: Create a handle for the client
    DsVeosCoSim_Handle handle = DsVeosCoSim_Create();
    if (handle == nullptr) {
        return 1;
    }

    // Step 3: Connect to the Example server that is running in VEOS
    DsVeosCoSim_ConnectConfig connectConfig{};
    connectConfig.serverName = "CoSimExample";
    connectConfig.remoteIpAddress = "127.0.0.1";
    // connectConfig.remotePort = 51535;
    if (DsVeosCoSim_Connect(handle, connectConfig) != DsVeosCoSim_Result_Ok) {
        DsVeosCoSim_Destroy(handle);
        return 1;
    }

        uint32_t canControllersCount{};
        const DsVeosCoSim_CanController* canControllers{};
        if (DsVeosCoSim_GetCanControllers(handle, &canControllersCount, &canControllers) != DsVeosCoSim_Result_Ok) {
            DsVeosCoSim_Disconnect(handle);
            DsVeosCoSim_Destroy(handle);
            return 1;
    }

    for (uint32_t i = 0; i < canControllersCount; i++) {
            std::cout << "Found CAN controller '" << canControllers[i].name << "'\n";
    }

    uint32_t outgoingSignalsCount{};
    const DsVeosCoSim_IoSignal* outgoingSignals{};
    if (DsVeosCoSim_GetOutgoingSignals(handle, &outgoingSignalsCount, &outgoingSignals) != DsVeosCoSim_Result_Ok) {
            DsVeosCoSim_Disconnect(handle);
            DsVeosCoSim_Destroy(handle);
            return 1;
    }

    for (uint32_t i = 0; i < outgoingSignalsCount; i++) {
            std::cout << "Found outgoing signal '" << outgoingSignals[i].name << "'\n";
    }

        uint32_t incomingSignalsCount{};
        const DsVeosCoSim_IoSignal* incomingSignals{};
        if (DsVeosCoSim_GetIncomingSignals(handle, &incomingSignalsCount, &incomingSignals) != DsVeosCoSim_Result_Ok) {
            DsVeosCoSim_Disconnect(handle);
            DsVeosCoSim_Destroy(handle);
            return 1;
    }

    for (uint32_t i = 0; i < incomingSignalsCount; i++) {
            std::cout << "Found incoming signal '" << incomingSignals[i].name << "'\n";
    }

    AppContext appContext{};
    appContext.handle = handle;
    if (!BuildSignalBindings(incomingSignals,
                             incomingSignalsCount,
                             kIncomingSignalSpecs,
                             appContext.signals.incomingByName,
                             appContext.signals.incomingByPort,
                             "incoming") ||
        !BuildSignalBindings(outgoingSignals,
                             outgoingSignalsCount,
                             kOutgoingSignalSpecs,
                             appContext.signals.outgoingByName,
                             appContext.signals.outgoingByPort,
                             "outgoing")) {
        DsVeosCoSim_Disconnect(handle);
        DsVeosCoSim_Destroy(handle);
        return 1;
    }

    std::cout << "Incoming signals grouped by port:\n";
    for (const auto& [portName, signalNames] : appContext.signals.incomingByPort) {
        std::cout << "  " << portName << ": ";
        for (const std::string& signalName : signalNames) {
            std::cout << signalName << " ";
        }
        std::cout << "\n";
    }

    std::cout << "Outgoing signals grouped by port:\n";
    for (const auto& [portName, signalNames] : appContext.signals.outgoingByPort) {
        std::cout << "  " << portName << ": ";
        for (const std::string& signalName : signalNames) {
            std::cout << signalName << " ";
        }
        std::cout << "\n";
    }

    // Step 4: Run a callback-based co-simulation
    DsVeosCoSim_Callbacks callbacks{};
    callbacks.simulationEndStepCallback = OnEndStep;
    callbacks.simulationStartedCallback = OnStarted;
    callbacks.simulationStoppedCallback = OnStopped;
    callbacks.simulationPausedCallback = OnPaused;
    callbacks.simulationContinuedCallback = OnContinued;

    callbacks.userData = &appContext;
    if (DsVeosCoSim_RunCallbackBasedCoSimulation(handle, callbacks) != DsVeosCoSim_Result_Disconnected) {
        DsVeosCoSim_Disconnect(handle);
        DsVeosCoSim_Destroy(handle);
    }

    DsVeosCoSim_Destroy(handle);
    return 0; 
}