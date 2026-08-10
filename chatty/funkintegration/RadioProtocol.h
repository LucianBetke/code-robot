#ifndef RADIO_PROTOCOL_H
#define RADIO_PROTOCOL_H

#include <Arduino.h>

namespace RadioProtocol
{
    static const uint8_t VERSION = 1;
    static const uint8_t PAYLOAD_SIZE = 26;
    static const uint8_t MAX_FRAGMENT_COUNT = 5;
    static const uint8_t MAX_LINE_LENGTH =
        PAYLOAD_SIZE * MAX_FRAGMENT_COUNT;
    static const byte ADDRESS[6] = "IGOR1";

    struct __attribute__((packed)) Packet
    {
        uint8_t version;
        uint16_t messageId;
        uint8_t fragmentIndex;
        uint8_t fragmentCount;
        uint8_t payloadLength;
        char payload[PAYLOAD_SIZE];
    };

    static_assert(sizeof(Packet) == 32, "nRF24 payload must be 32 bytes");
}

#endif
