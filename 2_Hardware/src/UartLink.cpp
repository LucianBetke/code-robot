// UartLink.cpp
#include "UartLink.h"
#include <string.h>

UartLink::UartLink(uint8_t rx, uint8_t tx)
    : _link(rx, tx),
    _connected(false),
    _lastPing(0),
    _lastSeen(0)
{
}

void UartLink::begin(unsigned long baud)
{
    _link.begin(baud);
    _handshake();   // blockierend, bis Verbindung steht
}

void UartLink::_handshake()
{
    _connected = false;
    Serial.println("Handshake startet...");

    while (!_connected)
    {
        if (millis() - _lastPing > PING_INTERVAL)
        {
            _lastPing = millis();
            _link.println("PING");
        }

        if (_link.available())
        {
            size_t n = _link.readBytesUntil('\n', _buf, sizeof(_buf) - 1);
            _buf[n] = '\0';

            if (n > 0 && _buf[n - 1] == '\r')
                _buf[n - 1] = '\0';

            if (strcmp(_buf, "PING") == 0)
                _link.println("PONG");
            else if (strcmp(_buf, "PONG") == 0)
            {
                _connected = true;
                _lastSeen = millis();
                _lastPing = millis();
                Serial.println("Handshake OK");
            }
        }
    }
}

void UartLink::update()
{
    if (_link.available())
    {
        size_t n = _link.readBytesUntil('\n', _buf, sizeof(_buf) - 1);
        _buf[n] = '\0';

        if (n > 0 && _buf[n - 1] == '\r')
            _buf[n - 1] = '\0';

        _lastSeen = millis();

        if (strcmp(_buf, "PING") == 0)
            _link.println("PONG");
    }

    if (millis() - _lastSeen > TIMEOUT)
    {
        Serial.println("Verbindung verloren -> Reconnect");
        _handshake();
    }
}

bool UartLink::isConnected() const
{
    return _connected;
}
