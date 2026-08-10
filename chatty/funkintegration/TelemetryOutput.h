#ifndef TELEMETRY_OUTPUT_H
#define TELEMETRY_OUTPUT_H

#include <Arduino.h>

class RadioTelemetrySender;

class TelemetryOutput : public Print
{
public:
    TelemetryOutput(Print& serialOutput, RadioTelemetrySender* radioSender);
    virtual size_t write(uint8_t value);
    using Print::write;

private:
    Print& _serialOutput;
    RadioTelemetrySender* _radioSender;
    bool _atLineStart;
    bool _capture;
};

#endif
