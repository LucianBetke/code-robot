// UartLink.cpp
#include "UartLink.h"

#include <string.h>
#include <avr/pgmspace.h>

static bool equalsProgmem(const char* text, PGM_P pattern)
{
    return strcmp_P(text, pattern) == 0;
}

UartLink::UartLink(Stream& link, bool initiator)
    : _link(link),
    _initiator(initiator),
    _connected(false),
    _lastPing(0),
    _lastSeen(0),
    _idx(0),
    _lineHead(0),
    _lineTail(0),
    _lineCount(0)
{}

void UartLink::begin()
{
    _connected = false;
    _lastPing = 0;
    _lastSeen = millis();
    _idx = 0;

    _lineHead = 0;
    _lineTail = 0;
    _lineCount = 0;

    _buf[0] = '\0';

    for (uint8_t i = 0; i < LINE_QUEUE_DEPTH; i++)
    {
        _lineQueue[i][0] = '\0';
    }
}

void UartLink::update()
{
    const uint32_t now = millis();

    // ===== RX =====
    while (_link.available())
    {
        const char c = _link.read();

        if (c == '\r')
        {
            continue;
        }

        if (c == '\n')
        {
            _buf[_idx] = '\0';
            _lastSeen = now;

            if (!_initiator && equalsProgmem(_buf, PSTR("PING")))
            {
                _link.println(F("PONG"));
            }
            else if (_initiator &&
                equalsProgmem(_buf, PSTR("PONG")) &&
                !_connected)
            {
                _link.println(F("ACK"));
                _connected = true;
            }
            else if (!_initiator &&
                equalsProgmem(_buf, PSTR("ACK")))
            {
                _connected = true;
            }
            else if (!equalsProgmem(_buf, PSTR("KA")) &&
                _buf[0] != '#')
            {
                enqueueLine(_buf);
            }

            _idx = 0;
        }
        else if (_idx < sizeof(_buf) - 1)
        {
            _buf[_idx++] = c;
        }
        else
        {
            // Ueberlange oder beschaedigte Zeile verwerfen.
            _idx = 0;
        }
    }

    // ===== TX =====
    if ((uint32_t)(now - _lastPing) > PING_INTERVAL)
    {
        _lastPing = now;

        if (!_connected)
        {
            if (_initiator)
            {
                _link.println(F("PING"));
            }
        }
        else
        {
            // KA darf nicht entfernt werden.
            _link.println(F("KA"));
        }
    }

    // ===== Timeout =====
    if (_connected &&
        (uint32_t)(now - _lastSeen) > TIMEOUT)
    {
        _connected = false;
    }
}

void UartLink::sendLine(const char* msg)
{
    if (_connected)
    {
        _link.println(msg);
    }
}

bool UartLink::availableLine() const
{
    return _lineCount > 0;
}

const char* UartLink::getLine()
{
    if (_lineCount == 0)
    {
        return "";
    }

    const char* line = _lineQueue[_lineHead];

    _lineHead++;

    if (_lineHead >= LINE_QUEUE_DEPTH)
    {
        _lineHead = 0;
    }

    _lineCount--;

    return line;
}

bool UartLink::isConnected() const
{
    return _connected;
}

void UartLink::enqueueLine(const char* line)
{
    if (_lineCount >= LINE_QUEUE_DEPTH)
    {
        // Bereits gepufferte Protokollzeilen bleiben erhalten.
        // Die neue Zeile wird verworfen.
        return;
    }

    strncpy(
        _lineQueue[_lineTail],
        line,
        LINE_LENGTH - 1
    );

    _lineQueue[_lineTail][LINE_LENGTH - 1] = '\0';

    _lineTail++;

    if (_lineTail >= LINE_QUEUE_DEPTH)
    {
        _lineTail = 0;
    }

    _lineCount++;
}