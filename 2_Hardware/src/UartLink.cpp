#include "UartLink.h"
#include <string.h>

UartLink::UartLink(Stream& link, bool initiator)
    : _link(link), _initiator(initiator),
    _connected(false),
    _lastPing(0),
    _lastSeen(0),
    _idx(0),
    _lineAvailable(false) {
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
    uint32_t now = millis();
    // ===== RX =====
    while (_link.available())
    {
        char c = _link.read();
        if (c == '\r') continue;
        if (c == '\n')
        {
            _buf[_idx] = '\0';
            _lastSeen = now;
            if (!_initiator && strcmp(_buf, "PING") == 0)
            {
                _link.println("PONG");
            }
            else if (_initiator && strcmp(_buf, "PONG") == 0 && !_connected)
            {
                _link.println("ACK");
                _connected = true;
            }
            else if (!_initiator && strcmp(_buf, "ACK") == 0)
            {
                _connected = true;
            }
            else if (_initiator && _buf[0] == '#')
            {
                // Debug-Zeile von hinten — an PC weiterleiten
                _link.println(_buf);
            }
            else if (strcmp(_buf, "KA") != 0 && _buf[0] != '#')
            {
                // 👉 Nutzdaten speichern
                strncpy(_line, _buf, sizeof(_line) - 1);
                _line[sizeof(_line) - 1] = '\0';
                _lineAvailable = true;
            }
            _idx = 0;
        }
        else if (_idx < sizeof(_buf) - 1)
        {
            _buf[_idx++] = c;
        }
        else
        {
            _idx = 0; // Overflow-Schutz
        }
    }
    // ===== TX =====
    if (now - _lastPing > PING_INTERVAL)
    {
        _lastPing = now;
        if (!_connected)
        {
            if (_initiator) _link.println("PING");
        }
        else
        {
            _link.println("KA");
        }
    }
    // ===== Timeout =====
    if (_connected && (now - _lastSeen > TIMEOUT))
    {
        _connected = false;
    }
}

void UartLink::sendLine(const char* msg)
{
    if (_connected) _link.println(msg);
}

bool UartLink::availableLine() const
{
    return _lineAvailable;
}

const char* UartLink::getLine()
{
    _lineAvailable = false;
    return _line;
}

bool UartLink::isConnected() const
{
    return _connected;
}