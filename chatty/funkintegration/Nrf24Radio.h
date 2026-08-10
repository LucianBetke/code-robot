#ifndef NRF24_RADIO_H
#define NRF24_RADIO_H

#include <Arduino.h>
#include <RF24.h>
#include "RadioProtocol.h"

class Nrf24Radio
{
public:
    Nrf24Radio();
    bool begin();
    bool send(const RadioProtocol::Packet& packet);
    bool isReady() const;
    uint32_t sentPackets() const;
    uint32_t failedPackets() const;
    uint16_t maxSendMicros() const;

private:
    RF24 _radio;
    bool _ready;
    uint32_t _sentPackets;
    uint32_t _failedPackets;
    uint16_t _maxSendMicros;
};

#endif
