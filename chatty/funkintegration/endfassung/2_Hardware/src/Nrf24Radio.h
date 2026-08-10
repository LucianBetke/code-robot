// ============================================================
// File: Nrf24Radio.h
// Zweck:
//  - Kapselung des nRF24L01+ am vorderen Nano
//  - Initialisierung, Senden eines Pakets, Fehlerzaehler
//  - Kennt keine Telemetrieinhalte
//
// Pinbelegung, fest aus der Arbeitsanweisung:
//   CE   D7
//   CSN  D8
//   MOSI D11
//   MISO D12
//   SCK  D13
//
// Hinweis zu D13: dort haengt zusaetzlich die Bordkontroll-LED
// des Nano. Sie liegt als Last am SPI-Takt. Bei Uebertragungs-
// problemen ist das der erste Verdaechtige.
// ============================================================

#ifndef NRF24_RADIO_H
#define NRF24_RADIO_H

#include <Arduino.h>
#include <RF24.h>

#include "src/RadioProtocol.h"

class Nrf24Radio
{
public:
    Nrf24Radio();

    bool begin();

    // Sendet genau ein Paket. Blockiert fuer die Dauer der
    // Uebertragung inklusive Auto-Retransmit.
    bool send(const RadioProtocol::Packet& packet);

    bool isReady() const;

    uint32_t sentPackets() const;
    uint32_t failedPackets() const;

    // Laengste bisher gemessene Dauer eines send()-Aufrufs.
    // Das ist der Messwert, mit dem sich entscheiden laesst,
    // ob die Retry-Einstellung fuer die Regelung tragbar ist.
    uint16_t maxSendMicros() const;
    void resetMaxSendMicros();

private:
    RF24 _radio;

    bool _ready;

    uint32_t _sentPackets;
    uint32_t _failedPackets;

    uint16_t _maxSendMicros;
};

#endif // NRF24_RADIO_H

