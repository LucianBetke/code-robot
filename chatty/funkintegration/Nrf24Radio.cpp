#include "Nrf24Radio.h"

Nrf24Radio::Nrf24Radio()
    : _radio(7, 8),
      _ready(false),
      _sentPackets(0),
      _failedPackets(0),
      _maxSendMicros(0)
{}

bool Nrf24Radio::begin()
{
    _ready = _radio.begin();
    if (!_ready) return false;

    _radio.setPALevel(RF24_PA_LOW);
    _radio.setDataRate(RF24_1MBPS);
    _radio.setChannel(76);
    _radio.setPayloadSize(sizeof(RadioProtocol::Packet));
    _radio.setRetries(1, 2);
    _radio.openWritingPipe(RadioProtocol::ADDRESS);
    _radio.stopListening();
    return true;
}

bool Nrf24Radio::send(const RadioProtocol::Packet& packet)
{
    if (!_ready)
    {
        ++_failedPackets;
        return false;
    }

    const uint32_t started = micros();
    const bool ok = _radio.write(&packet, sizeof(packet));
    const uint32_t elapsed = (uint32_t)(micros() - started);
    if (elapsed > 0xFFFFUL) _maxSendMicros = 0xFFFF;
    else if ((uint16_t)elapsed > _maxSendMicros)
        _maxSendMicros = (uint16_t)elapsed;
    if (ok) ++_sentPackets;
    else ++_failedPackets;
    return ok;
}

bool Nrf24Radio::isReady() const { return _ready; }
uint32_t Nrf24Radio::sentPackets() const { return _sentPackets; }
uint32_t Nrf24Radio::failedPackets() const { return _failedPackets; }
uint16_t Nrf24Radio::maxSendMicros() const { return _maxSendMicros; }
