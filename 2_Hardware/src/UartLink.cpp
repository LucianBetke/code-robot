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
    _lineAvailable(false)
{
}

void UartLink::begin()
{
    _connected = false;
    _lastPing = 0;
    _lastSeen = millis();
    _idx = 0;
    _lineAvailable = false;
    _line[0] = '\0';
}

void UartLink::update()
{
    const uint32_t now = millis();

    // ===== RX =====
    while (_link.available())
    {
        const char c = _link.read();

        if (c == '\r') continue;

        if (c == '\n')
        {
            _buf[_idx] = '\0';
            _lastSeen = now;

            if (!_initiator && equalsProgmem(_buf, PSTR("PING")))
                _link.println(F("PONG"));

            else if (_initiator && equalsProgmem(_buf, PSTR("PONG")) && !_connected)
            {
                _link.println(F("ACK"));
                _connected = true;
            }

            else if (!_initiator && equalsProgmem(_buf, PSTR("ACK")))
                _connected = true;

            else if (!equalsProgmem(_buf, PSTR("KA")) && _buf[0] != '#')
            {
                strncpy(_line, _buf, sizeof(_line) - 1);
                _line[sizeof(_line) - 1] = '\0';
                _lineAvailable = true;
            }

            _idx = 0;
        }
        else if (_idx < sizeof(_buf) - 1)
            _buf[_idx++] = c;
        else
            _idx = 0;
    }

    // ===== TX =====
    if (now - _lastPing > PING_INTERVAL)
    {
        _lastPing = now;

        if (!_connected)
        {
            if (_initiator) _link.println(F("PING"));
        }
        else
            _link.println(F("KA"));
    }

    // ===== Timeout =====
    if (_connected && (now - _lastSeen > TIMEOUT)) _connected = false;
}

void UartLink::sendLine(const char* msg)
{
    if (_connected) _link.println(msg);
}

bool UartLink::availableLine() const { return _lineAvailable; }

const char* UartLink::getLine()
{
    _lineAvailable = false;
    return _line;
}

bool UartLink::isConnected() const { return _connected; }