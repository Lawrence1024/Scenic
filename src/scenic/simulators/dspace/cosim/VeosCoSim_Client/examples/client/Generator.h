// Copyright dSPACE GmbH. All rights reserved.

#pragma once

#include <cstdint>
#include <cstdlib>
#include "VeosCoSim.h"

int Random(int min, int max);

void FillWithRandom(uint8_t* data, size_t length);

template <typename T>
T GenerateRandom(T min, T max) {
    return static_cast<T>(Random(static_cast<int>(min), static_cast<int>(max)));
}

uint8_t GenerateU8();
uint32_t GenerateU32();

void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_CanMessage& message);
void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_EthMessage& message);
void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_LinMessage& message);
