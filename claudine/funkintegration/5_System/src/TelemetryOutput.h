// ============================================================
// File: TelemetryOutput.h
// Zweck:
//  - Zentraler Uebergabepunkt fuer PC-Telemetrie
//  - Leitet jedes Byte unveraendert an Serial weiter
//  - Gibt Zeilen, die mit # beginnen, zusaetzlich an den
//    Funksender
//
// Warum nur #-Zeilen:
//
// Serial ist am vorderen Nano zugleich PC-Schnittstelle und
// UART zum hinteren Nano. Die Protokollzeilen VSOL, VIST und
// der Handshake duerfen nicht in den Funk gelangen. Sie laufen
// deshalb weiter direkt ueber Serial und nicht ueber dieses
// Objekt.
//
// Das Objekt haelt selbst keinen Zeilenpuffer. Die Bytes gehen
// direkt in den Ring des Senders.
// ============================================================

#ifndef TELEMETRY_OUTPUT_H
#define TELEMETRY_OUTPUT_H

#include <Arduino.h>

class RadioTelemetrySender;

class TelemetryOutput : public Print
{
public:
    // radioSender darf 0 sein. Dann wirkt das Objekt wie eine
    // reine Weiterleitung auf Serial.
    TelemetryOutput(
        Print& serialOutput,
        RadioTelemetrySender* radioSender);

    virtual size_t write(uint8_t value);

    using Print::write;

private:
    Print& _serialOutput;
    RadioTelemetrySender* _radioSender;

    bool _atLineStart;
    bool _capture;
};

#endif // TELEMETRY_OUTPUT_H
