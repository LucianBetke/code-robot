#ifndef RADIO_TELEMETRY_SENDER_H
#define RADIO_TELEMETRY_SENDER_H

#include <Arduino.h>
#include "RadioProtocol.h"

class Nrf24Radio;

class RadioTelemetrySender
{
public:
    explicit RadioTelemetrySender(Nrf24Radio& radio);

    void beginLine();
    void addLineByte(char value);
    void commitLine();
    void update();

    bool busy() const;
    uint32_t sentLines() const;
    uint32_t droppedLines() const;
    uint8_t ringPeak() const;

private:
    // Maximaler gleichzeitig erzeugter Frame-Stoss nach Datentypgrenzen:
    // WHEELS 102 + CNTF 64 + ODOM 70 + 3 Laengenbytes = 239 Byte.
    static const uint8_t RING_SIZE = 240;

    static uint8_t nextPos(uint8_t pos);
    void rollbackLine();
    bool startNextMessage();
    void finishMessage(bool success);

    Nrf24Radio& _radio;
    char _ring[RING_SIZE];

    uint8_t _writePos;
    uint8_t _lenPos;
    uint8_t _pending;
    bool _lineOpen;
    bool _lineOverflow;

    uint8_t _readPos;
    uint8_t _committed;

    bool _sending;
    uint8_t _dataPos;
    uint8_t _lineLength;
    uint8_t _fragmentIndex;
    uint8_t _fragmentCount;
    uint16_t _messageId;

    uint32_t _sentLines;
    uint32_t _droppedLines;
    uint8_t _ringPeak;
};

#endif
