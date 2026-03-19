// Copyright dSPACE GmbH. All rights reserved.

#pragma once

#include <string>

#include "VeosCoSim.h"

#ifndef CTRL
#define CTRL(c) ((c)&037)
#endif

int GetChar();

int GetDataTypeSize(VeosCoSim_DataType dataType);

double TimeToSeconds(VeosCoSim_Time simulationTime);

void LogMessage(VeosCoSim_Severity severity, const char* message);

void LogIoData(const std::string& signalName, VeosCoSim_Time simulationTime, VeosCoSim_DataType dataType, uint32_t length, const void* value);
void LogCanMessage(const std::string& controllerName, VeosCoSim_Time simulationTime, uint32_t id, uint32_t length, const uint8_t* data, const std::string& type);
void LogEthMessage(const std::string& controllerName, VeosCoSim_Time simulationTime, uint32_t length, const uint8_t* data);
void LogLinMessage(const std::string& controllerName, VeosCoSim_Time simulationTime, uint32_t id, uint32_t length, const uint8_t* data);

void SwitchSendingIoSignals();
void SwitchSendingCanMessages();
void SwitchSendingEthMessages();
void SwitchSendingLinMessages();

bool IsSendingIoSignalsEnabled();
bool IsSendingCanMessagesEnabled();
bool IsSendingEthMessagesEnabled();
bool IsSendingLinMessagesEnabled();
