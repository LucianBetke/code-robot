#include "TelemetryOutput.h"
#include "RadioTelemetrySender.h"

TelemetryOutput::TelemetryOutput(
    Print& serialOutput,
    RadioTelemetrySender* radioSender)
    : _serialOutput(serialOutput), _radioSender(radioSender),
      _atLineStart(true), _capture(false)
{}

size_t TelemetryOutput::write(uint8_t value)
{
    const size_t written = _serialOutput.write(value);
    if (_radioSender == 0) return written;
    if (value == '\r') return written;

    if (value == '\n')
    {
        if (_capture) _radioSender->commitLine();
        _capture = false;
        _atLineStart = true;
        return written;
    }

    if (_atLineStart)
    {
        _atLineStart = false;
        if (value == '#')
        {
            _capture = true;
            _radioSender->beginLine();
        }
    }

    if (_capture) _radioSender->addLineByte((char)value);
    return written;
}
