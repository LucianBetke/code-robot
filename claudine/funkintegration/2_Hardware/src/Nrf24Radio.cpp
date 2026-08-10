// ============================================================
// File: Nrf24Radio.cpp
// ============================================================

#include "src/Nrf24Radio.h"

namespace
{
    // CE an D7, CSN an D8.
    const uint8_t RADIO_CE_PIN = 7;
    const uint8_t RADIO_CSN_PIN = 8;

    // Auto-Retransmit-Delay-Stufe und Anzahl Wiederholungen.
    //
    // Rechnung, nicht gemessen:
    //   ein 32-Byte-Paket bei 1 MBit/s dauert mit Adresse,
    //   PCF und CRC rund 0,33 ms, dazu ACK und Umschaltzeiten
    //   rund 0,25 ms. Ein Versuch kostet also grob 0,6 ms.
    //   Stufe 1 entspricht 500 us Wartezeit vor dem naechsten
    //   Versuch.
    //
    //   3 Versuche  = 3 * 0,6 ms + 2 * 0,5 ms  ~ 2,8 ms
    //
    // Das ist die schlechteste Blockierdauer eines send().
    // Ob sie tragbar ist, muss maxSendMicros() am Fahrzeug
    // zeigen. Solange nicht gemessen wurde, bleiben die Werte
    // wie in der Arbeitsanweisung vorgegeben.
    const uint8_t RADIO_RETRY_DELAY = 1;
    const uint8_t RADIO_RETRY_COUNT = 2;

    const uint8_t RADIO_CHANNEL = 76;
}

Nrf24Radio::Nrf24Radio()
    : _radio(RADIO_CE_PIN, RADIO_CSN_PIN),
    _ready(false),
    _sentPackets(0),
    _failedPackets(0),
    _maxSendMicros(0)
{
}

bool Nrf24Radio::begin()
{
    _ready = _radio.begin();

    if (!_ready)
    {
        return false;
    }

    _radio.setPALevel(RF24_PA_LOW);
    _radio.setDataRate(RF24_1MBPS);
    _radio.setChannel(RADIO_CHANNEL);

    // Feste Paketgroesse: der Empfaenger liest immer 32 Byte.
    _radio.setPayloadSize(sizeof(RadioProtocol::Packet));

    _radio.setRetries(
        RADIO_RETRY_DELAY,
        RADIO_RETRY_COUNT);

    // openWritingPipe setzt zugleich RX_ADDR_P0, das der
    // Auto-Ack braucht.
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

    const uint32_t start = micros();

    const bool ok =
        _radio.write(&packet, sizeof(packet));

    const uint32_t elapsed =
        (uint32_t)(micros() - start);

    if (elapsed > 0xFFFFUL)
    {
        _maxSendMicros = 0xFFFF;
    }
    else if ((uint16_t)elapsed > _maxSendMicros)
    {
        _maxSendMicros = (uint16_t)elapsed;
    }

    if (ok)
    {
        ++_sentPackets;
    }
    else
    {
        ++_failedPackets;
    }

    return ok;
}

bool Nrf24Radio::isReady() const
{
    return _ready;
}

uint32_t Nrf24Radio::sentPackets() const
{
    return _sentPackets;
}

uint32_t Nrf24Radio::failedPackets() const
{
    return _failedPackets;
}

uint16_t Nrf24Radio::maxSendMicros() const
{
    return _maxSendMicros;
}

void Nrf24Radio::resetMaxSendMicros()
{
    _maxSendMicros = 0;
}
