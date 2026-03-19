// Copyright dSPACE GmbH. All rights reserved.

#include "Generator.h"

int Random(int min, int max) {
    static bool first = true;
    if (first) {
        srand(42);  // NOLINT(cert-msc51-cpp)
        first = false;
    }

    const int diff = max + 1 - min;

    return min + rand() % diff;  // NOLINT(concurrency-mt-unsafe)
}

void FillWithRandom(uint8_t* data, size_t length) {
    for (size_t i = 0; i < length; i++) {
        data[i] = GenerateU8();
    }
}

uint8_t GenerateU8() {
    return GenerateRandom(static_cast<uint8_t>(0U), static_cast<uint8_t>(UINT8_MAX));
}

uint32_t GenerateU32() {
    return GenerateRandom(0U, 123456789U);
}

void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_CanMessage& message) {
    message.timestamp = 0;
    message.channelId = channelId;
    message.identifier = GenerateU32();
    message.flags = VEOSCOSIM_CAN_MESSAGE_FLAG_FD | VEOSCOSIM_CAN_MESSAGE_FLAG_EXT;
    message.length = GenerateRandom(1U, 32U);
    FillWithRandom(message.data, message.length);
}

void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_EthMessage& message) {
    message.timestamp = 0;
    message.channelId = channelId;
    message.flags = 0;
    message.length = 64U;
    FillWithRandom(message.data, message.length);
}

void FillMessage(VeosCoSim_ChannelId channelId, VeosCoSim_LinMessage& message) {
    message.timestamp = 0;
    message.channelId = channelId;
    message.identifier = GenerateRandom(0U, 63U);
    message.flags = VEOSCOSIM_LIN_MESSAGE_FLAG_HEADER | VEOSCOSIM_LIN_MESSAGE_FLAG_RESPONSE;
    message.length = GenerateRandom(1U, 8U);
    FillWithRandom(message.data, message.length);
}
