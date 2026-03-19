// Copyright dSPACE GmbH. All rights reserved.

#include "ClientServerTestHelper.h"

#include <iomanip>
#include <sstream>

#include "VeosCoSim.h"

#ifdef _WIN32
#include <conio.h>
#else
#include <termios.h>
#include <unistd.h>
#endif

namespace {

bool g_sendIoData;
bool g_sendCanMessages;
bool g_sendEthMessages;
bool g_sendLinMessages;

void PrintStatus(bool value, std::string_view what) {
    printf("%s sending %s.\n", value ? "Enabled" : "Disabled", what.data());
}

}  // namespace

int GetChar() {
#ifdef _WIN32
    return _getch();
#else
    int ch;
    struct termios oldt;
    struct termios newt;

    tcgetattr(STDIN_FILENO, &oldt);
    newt = oldt;
    newt.c_lflag &= ~(ICANON | ECHO);

    tcsetattr(STDIN_FILENO, TCSANOW, &newt);

    ch = getchar();

    tcsetattr(STDIN_FILENO, TCSANOW, &oldt);

    return ch;
#endif
}

int GetDataTypeSize(VeosCoSim_DataType dataType) {
    switch (dataType) {
        case VeosCoSim_DataType_Bool:
        case VeosCoSim_DataType_Int8:
        case VeosCoSim_DataType_UInt8:
            return 1;
        case VeosCoSim_DataType_Int16:
        case VeosCoSim_DataType_UInt16:
            return 2;
        case VeosCoSim_DataType_Int32:
        case VeosCoSim_DataType_UInt32:
        case VeosCoSim_DataType_Float32:
            return 4;
        case VeosCoSim_DataType_Int64:
        case VeosCoSim_DataType_UInt64:
        case VeosCoSim_DataType_Float64:
            return 8;
        default:  // NOLINT(clang-diagnostic-covered-switch-default)
            return 0;
    }
}

double TimeToSeconds(VeosCoSim_Time simulationTime) {
    return static_cast<double>(simulationTime) / VEOSCOSIM_TIME_RESOLUTION_PER_SECOND;
}

std::string DataTypeValueToString(const void* value, uint32_t index, VeosCoSim_DataType dataType) {
    switch (dataType) {
        case VeosCoSim_DataType_Bool:
            return std::to_string(static_cast<const uint8_t*>(value)[index]);
        case VeosCoSim_DataType_Int8:
            return std::to_string(static_cast<const int8_t*>(value)[index]);
        case VeosCoSim_DataType_Int16:
            return std::to_string(static_cast<const int16_t*>(value)[index]);
        case VeosCoSim_DataType_Int32:
            return std::to_string(static_cast<const int32_t*>(value)[index]);
        case VeosCoSim_DataType_Int64:
            return std::to_string(static_cast<const int64_t*>(value)[index]);
        case VeosCoSim_DataType_UInt8:
            return std::to_string(static_cast<const uint8_t*>(value)[index]);
        case VeosCoSim_DataType_UInt16:
            return std::to_string(static_cast<const uint16_t*>(value)[index]);
        case VeosCoSim_DataType_UInt32:
            return std::to_string(static_cast<const uint32_t*>(value)[index]);
        case VeosCoSim_DataType_UInt64:
            return std::to_string(static_cast<const uint64_t*>(value)[index]);
        case VeosCoSim_DataType_Float32:
            return std::to_string(static_cast<const float*>(value)[index]);
        case VeosCoSim_DataType_Float64:
            return std::to_string(static_cast<const double*>(value)[index]);
        default:
            return "";
    }
}

std::string ValueToString(const void* value, uint32_t length, VeosCoSim_DataType dataType) {
    if (length == 0) {
        return "";
    }

    std::stringstream ss;
    for (uint32_t i = 0; i < length; i++) {
        if (i > 0) {
            ss << " ";
        }

        ss << DataTypeValueToString(value, i, dataType);
    }

    return ss.str();
}

std::string DataToString(const uint8_t* data, uint32_t dataLength, char separator) {
    std::ostringstream oss;
    oss << std::hex << std::setfill('0');
    for (uint32_t i = 0; i < dataLength; i++) {
        oss << std::setw(2) << static_cast<int>(data[i]);
        if ((i < dataLength - 1) && separator != 0) {
            oss << separator;
        }
    }

    return oss.str();
}

std::string SeverityToString(VeosCoSim_Severity severity) {
    switch (severity) {
        case VeosCoSim_Severity_Error:
            return "ERROR";
        case VeosCoSim_Severity_Warning:
            return "WARNING";
        case VeosCoSim_Severity_Info:
            return "INFO";
        case VeosCoSim_Severity_Trace:
            return "TRACE";
    }

    return "<unknown severity>";
}

void LogMessage(VeosCoSim_Severity severity, const char* message) {
    printf("%-7s %s\n", SeverityToString(severity).c_str(), message);
}

void LogIoData(const std::string& signalName, VeosCoSim_Time simulationTime, VeosCoSim_DataType dataType, uint32_t length, const void* value) {
    printf("%f,%s,IN,%u,%s\n", TimeToSeconds(simulationTime), signalName.c_str(), length, ValueToString(value, length, dataType).c_str());
}

void LogCanMessage(const std::string& controllerName,
                   VeosCoSim_Time simulationTime,
                   uint32_t id,
                   uint32_t length,
                   const uint8_t* data,
                   const std::string& type) {
    printf("%f,%s,%u,Rx,%u,%s,%s,\n", TimeToSeconds(simulationTime), controllerName.c_str(), id, length, DataToString(data, length, '-').c_str(), type.c_str());
}

void LogEthMessage(const std::string& controllerName, VeosCoSim_Time simulationTime, uint32_t length, const uint8_t* data) {
    if (length >= 14) {
        const std::string macAddress1 = DataToString(data, 6, ':');
        const std::string macAddress2 = DataToString(data + 6, 6, ':');
        const std::string ethernetType = DataToString(data + 12, 2, 0);
        printf("%f,%s,%s-%s,Rx,%u,%s,ETH,%s\n",
               TimeToSeconds(simulationTime),
               controllerName.c_str(),
               macAddress2.c_str(),
               macAddress1.c_str(),
               length,
               DataToString(data + 14, length - 14, '-').c_str(),
               ethernetType.c_str());
    } else {
        printf("%f,%s,Rx,%u,%s,ETH,\n", TimeToSeconds(simulationTime), controllerName.c_str(), length, DataToString(data, length, '-').c_str());
    }
}

void LogLinMessage(const std::string& controllerName, VeosCoSim_Time simulationTime, uint32_t id, uint32_t length, const uint8_t* data) {
    printf("%f,%s,%u,Rx,%u,%s,LIN,\n", TimeToSeconds(simulationTime), controllerName.c_str(), id, length, DataToString(data, length, '-').c_str());
}

void SwitchSendingIoSignals() {
    g_sendIoData = !g_sendIoData;
    PrintStatus(g_sendIoData, "IO data");
}

void SwitchSendingCanMessages() {
    g_sendCanMessages = !g_sendCanMessages;
    PrintStatus(g_sendCanMessages, "CAN messages");
}

void SwitchSendingEthMessages() {
    g_sendEthMessages = !g_sendEthMessages;
    PrintStatus(g_sendEthMessages, "ETH messages");
}

void SwitchSendingLinMessages() {
    g_sendLinMessages = !g_sendLinMessages;
    PrintStatus(g_sendLinMessages, "LIN messages");
}

bool IsSendingIoSignalsEnabled() {
    return g_sendIoData;
}

bool IsSendingCanMessagesEnabled() {
    return g_sendCanMessages;
}

bool IsSendingEthMessagesEnabled() {
    return g_sendEthMessages;
}

bool IsSendingLinMessagesEnabled() {
    return g_sendLinMessages;
}
