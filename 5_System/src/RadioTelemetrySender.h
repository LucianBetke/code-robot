// ============================================================
// File: RadioTelemetrySender.h
// Zweck:
//  - Nimmt vollstaendige Telemetriezeilen entgegen
//  - Zerlegt sie in Funkpakete
//  - Uebergibt je Aufruf von update() genau ein Paket an das
//    Funkmodul
//
// Warum ein Ringpuffer und kein einzelner Zeilenslot:
//
// Der vordere Nano gibt mehrere Telemetriezeilen unmittelbar
// hintereinander aus. Beim Abschluss eines Frames sind das
// #WHEELS, #CNTF und #ODOM in einem einzigen Durchlauf von
// handleIncomingLines(). Haelt der Sender nur eine Zeile, dann
// wird die erste Zeile gesendet und alle weiteren desselben
// Durchlaufs werden verworfen. Der Verlust ist dann nicht
// zufaellig, sondern trifft immer dieselben Zeilentypen: #ODOM
// und #CNTF kaemen nie beim Empfaenger an.
//
// Der Ring nimmt den Stoss eines Schleifendurchlaufs auf und
// wird zwischen zwei Frames wieder leer gefahren.
//
// Die Zeile wird byteweise direkt in den Ring geschrieben. Es
// gibt bewusst keinen zweiten Sammelpuffer, weil auf 2 KB SRAM
// keine zwei Kopien derselben Zeile gehalten werden sollen.
//
// Ringformat: je Zeile ein Laengenbyte, dahinter die Nutzbytes.
// ============================================================

#ifndef RADIO_TELEMETRY_SENDER_H
#define RADIO_TELEMETRY_SENDER_H

#include <Arduino.h>

#include "src/RadioProtocol.h"

// Nur Referenz noetig. Die Vorwaertsdeklaration haelt RF24.h
// aus allen Uebersetzungseinheiten heraus, die lediglich
// Telemetrie ausgeben.
class Nrf24Radio;

class RadioTelemetrySender
{
public:
    explicit RadioTelemetrySender(Nrf24Radio& radio);

    // ----- Eingang, byteweise -----
    //
    // beginLine() eroeffnet eine Zeile, addLineByte() haengt an,
    // commitLine() schliesst ab. Passt die Zeile nicht in den
    // Ring oder ist sie laenger als MAX_LINE_LENGTH, wird sie
    // beim Abschluss vollstaendig verworfen und gezaehlt. Es
    // gelangt nie eine halbe Zeile in den Funk.
    void beginLine();
    void addLineByte(char value);
    void commitLine();

    // Sendet hoechstens ein Fragment.
    void update();

    bool busy() const;

    uint32_t sentLines() const;
    uint32_t droppedLines() const;

    // Hoechststand der Ringbelegung. Zeigt beim Test, ob der
    // Ring gross genug gewaehlt ist.
    uint8_t ringPeak() const;

private:
    // Datentyp-Maxima des groessten gleichzeitig erzeugbaren Stosses:
    // #WHEELS 102 + #CNTF 64 + #ODOM 70 + 3 Laengenbytes
    // ergeben 239 Byte. Ein Byte Reserve bleibt frei.
    static const uint8_t RING_SIZE = 240;

    static uint8_t nextPos(uint8_t pos);

    void rollbackLine();
    bool startNextMessage();
    void finishMessage(bool success);

    Nrf24Radio& _radio;

    char _ring[RING_SIZE];

    // Schreibseite
    uint8_t _writePos;
    uint8_t _lenPos;
    uint8_t _pending;        // Bytes der offenen Zeile inkl. Laengenbyte
    bool    _lineOpen;
    bool    _lineOverflow;

    // Leseseite
    uint8_t _readPos;        // Laengenbyte der aeltesten fertigen Zeile
    uint8_t _committed;      // Bytes fertiger Zeilen inkl. Laengenbytes

    // Sendezustand
    bool     _sending;
    uint8_t  _dataPos;
    uint8_t  _lineLength;
    uint8_t  _fragmentIndex;
    uint8_t  _fragmentCount;
    uint16_t _messageId;

    // Zaehler
    uint32_t _sentLines;
    uint32_t _droppedLines;
    uint8_t  _ringPeak;
};

#endif // RADIO_TELEMETRY_SENDER_H
