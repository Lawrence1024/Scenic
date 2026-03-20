// Bindings for dSPACE VeosCoSim MI API (VeosCoSimAppl.dll).

#include <pybind11/pybind11.h>
#include <pybind11/stl.h>

#include <cstdio>
#include <stdexcept>
#include <string>

#include "VeosCoSim.h"

namespace py = pybind11;

namespace {

void default_log(VeosCoSim_Severity severity, const char* message) {
    const char* level = "INFO";
    switch (severity) {
        case VeosCoSim_Severity_Error:
            level = "ERROR";
            break;
        case VeosCoSim_Severity_Warning:
            level = "WARN";
            break;
        case VeosCoSim_Severity_Trace:
            level = "TRACE";
            break;
        default:
            break;
    }
    std::fprintf(stderr, "[VeosCoSim:%s] %s\n", level, message ? message : "");
    std::fflush(stderr);
}

}  // namespace

class CoSimClient {
public:
    CoSimClient() = default;

    ~CoSimClient() { release_handle(); }

    CoSimClient(const CoSimClient&) = delete;
    CoSimClient& operator=(const CoSimClient&) = delete;

    void connect(const std::string& host, const std::string& server_name) {
        if (handle_) {
            throw std::runtime_error("CoSimClient: already connected; disconnect first.");
        }
        handle_ = VeosCoSim_CreateMI();
        if (!handle_) {
            throw std::runtime_error("CoSimClient: VeosCoSim_CreateMI returned null");
        }
        const VeosCoSim_Result r =
            VeosCoSim_ConnectMI(handle_, host.c_str(), server_name.c_str(), default_log);
        if (r != VeosCoSim_Result_OK) {
            VeosCoSim_DestroyMI(handle_);
            handle_ = nullptr;
            throw std::runtime_error(
                "CoSimClient: VeosCoSim_ConnectMI failed (VeosCoSim_Result="
                + std::to_string(static_cast<int>(r)) + ")");
        }
    }

    void connect2(
        const std::string& host,
        const std::string& server_name,
        uint16_t remote_port,
        uint16_t local_port) {
        if (handle_) {
            throw std::runtime_error("CoSimClient: already connected; disconnect first.");
        }
        handle_ = VeosCoSim_CreateMI();
        if (!handle_) {
            throw std::runtime_error("CoSimClient: VeosCoSim_CreateMI returned null");
        }

        VeosCoSim_ConnectConfiguration cfg{};
        cfg.remoteIpAddress = host.c_str();
        cfg.name = server_name.c_str();
        cfg.logCallback = default_log;
        cfg.remotePort = remote_port;
        cfg.localPort = local_port;

        const VeosCoSim_Result r = VeosCoSim_ConnectMI2(handle_, cfg);
        if (r != VeosCoSim_Result_OK) {
            VeosCoSim_DestroyMI(handle_);
            handle_ = nullptr;
            throw std::runtime_error(
                "CoSimClient: VeosCoSim_ConnectMI2 failed (VeosCoSim_Result="
                + std::to_string(static_cast<int>(r)) + ")");
        }
    }

    void disconnect() { release_handle(); }

    py::list io_signals() const {
        require_handle();
        uint32_t count = 0;
        const VeosCoSim_IoSignalInfo* infos = nullptr;
        const VeosCoSim_Result r =
            VeosCoSim_IoGetAvailableSignalsMI(handle_, &count, &infos);
        if (r != VeosCoSim_Result_OK) {
            throw std::runtime_error("CoSimClient: VeosCoSim_IoGetAvailableSignalsMI failed");
        }
        py::list out;
        for (uint32_t i = 0; i < count; ++i) {
            const auto& s = infos[i];
            py::dict d;
            d["id"] = s.id;
            d["length"] = s.length;
            d["data_type"] = s.dataType;
            d["direction"] = s.direction;
            d["size_kind"] = s.sizeKind;
            d["name"] = std::string(s.name);
            out.append(d);
        }
        return out;
    }

    py::list channels() const {
        require_handle();
        uint32_t count = 0;
        const VeosCoSim_BusChannelInfo* infos = nullptr;
        const VeosCoSim_Result r =
            VeosCoSim_GetAvailableChannelsMI(handle_, &count, &infos);
        if (r != VeosCoSim_Result_OK) {
            throw std::runtime_error("CoSimClient: VeosCoSim_GetAvailableChannelsMI failed");
        }
        py::list out;
        for (uint32_t i = 0; i < count; ++i) {
            const auto& c = infos[i];
            py::dict d;
            d["id"] = c.id;
            d["bus_protocol"] = c.busProtocol;
            d["controller_name"] = std::string(c.controllerName);
            out.append(d);
        }
        return out;
    }

    /// Blocking co-simulation loop. See README: return value may be Error even on success.
    int run(py::object time_trigger, py::object on_start = py::none(), py::object on_stop = py::none()) {
        require_handle();
        if (!time_trigger || time_trigger.is_none()) {
            throw std::runtime_error("CoSimClient.run: time_trigger callback is required");
        }

        on_time_trigger_ = std::move(time_trigger);
        on_start_ = std::move(on_start);
        on_stop_ = std::move(on_stop);

        VeosCoSim_RuntimeConfiguration cfg{};
        cfg.userData = this;
        cfg.timeTriggerCallback = &CoSimClient::time_trigger_trampoline;

        if (on_start_ && !on_start_.is_none()) {
            cfg.startSimulationCallback = &CoSimClient::start_trampoline;
        }
        if (on_stop_ && !on_stop_.is_none()) {
            cfg.stopSimulationCallback = &CoSimClient::stop_trampoline;
        }

        VeosCoSim_Result r = VeosCoSim_Result_Error;
        {
            py::gil_scoped_release release;
            r = VeosCoSim_RunMI(handle_, cfg);
            VeosCoSim_DisconnectMI(handle_);
            VeosCoSim_DestroyMI(handle_);
            handle_ = nullptr;
        }

        on_time_trigger_ = py::none();
        on_start_ = py::none();
        on_stop_ = py::none();

        return static_cast<int>(r);
    }

private:
    VeosCoSim_Handle handle_{nullptr};
    py::object on_time_trigger_;
    py::object on_start_;
    py::object on_stop_;

    void require_handle() const {
        if (!handle_) {
            throw std::runtime_error("CoSimClient: not connected");
        }
    }

    void release_handle() {
        if (!handle_) {
            return;
        }
        VeosCoSim_DisconnectMI(handle_);
        VeosCoSim_DestroyMI(handle_);
        handle_ = nullptr;
    }

    static void start_trampoline(VeosCoSim_Time t, void* user_data) {
        auto* self = static_cast<CoSimClient*>(user_data);
        self->invoke_callback(t, self->on_start_);
    }

    static void stop_trampoline(VeosCoSim_Time t, void* user_data) {
        auto* self = static_cast<CoSimClient*>(user_data);
        self->invoke_callback(t, self->on_stop_);
    }

    static void time_trigger_trampoline(VeosCoSim_Time t, void* user_data) {
        auto* self = static_cast<CoSimClient*>(user_data);
        self->invoke_callback(t, self->on_time_trigger_);
    }

    void invoke_callback(VeosCoSim_Time t, const py::object& fn) {
        py::gil_scoped_acquire gil;
        try {
            if (fn && !fn.is_none()) {
                fn(t);
            }
        } catch (py::error_already_set&) {
            PyErr_Print();
        }
    }
};

PYBIND11_MODULE(_veos_cosim, m) {
    m.doc() = "VeosCoSim client bindings (VeosCoSimAppl). Windows x64.";
    m.attr("__version__") = "0.2.0";

    py::class_<CoSimClient>(m, "CoSimClient")
        .def(py::init<>(), "Create a client; call connect() before run().")
        .def(
            "connect",
            &CoSimClient::connect,
            py::arg("host"),
            py::arg("server_name"),
            "Connect to a running VEOS co-simulation server (VeosCoSim_ConnectMI).")
        .def(
            "connect2",
            &CoSimClient::connect2,
            py::arg("host"),
            py::arg("server_name"),
            py::arg("remote_port") = 0,
            py::arg("local_port") = 0,
            "Connect using VeosCoSim_ConnectMI2 with optional explicit remote/local ports.")
        .def("disconnect", &CoSimClient::disconnect)
        .def("io_signals", &CoSimClient::io_signals, "Queries IO metadata (VeosCoSim_IoGetAvailableSignalsMI).")
        .def("channels", &CoSimClient::channels, "Queries bus channels (VeosCoSim_GetAvailableChannelsMI).")
        .def(
            "run",
            &CoSimClient::run,
            py::arg("time_trigger"),
            py::arg("on_start") = py::none(),
            py::arg("on_stop") = py::none(),
            R"pbdoc(
            Start the blocking co-simulation loop (VeosCoSim_RunMI).

            **time_trigger** receives one argument: ``sim_time_ns`` (``int``), simulation time in
            nanoseconds (``1e9`` per simulated second). It is invoked every simulation step, last
            for that step (dSPACE documentation).

            **on_start** / **on_stop** are optional callbacks with the same ``sim_time_ns`` signature.

            **Return value:** ``int`` ``VeosCoSim_Result`` from ``VeosCoSim_RunMI``. Per dSPACE docs,
            this may be ``Error`` even on normal runs—do not rely on it alone.

            After ``run`` returns, the client is disconnected and must ``connect`` again to reuse.
            )pbdoc");
}
