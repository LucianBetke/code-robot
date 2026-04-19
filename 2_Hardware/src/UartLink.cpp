#include "UartLink.h"
#include <string.h>

UartLink::UartLink(Stream& link, bool initiator)
    : _link(link),
    _initiator(initiator),
    _connected(false),
    _lastPing(0),
    _lastSeen(0),
    _idx(0)
{
}

void UartLink::begin(unsigned long baud)
{
    if (&_link == &Serial)
    {
        Serial.begin(baud);
    }

    _connected = false;
    _lastPing = 0;
    _lastSeen = millis();
    _idx = 0;
}

void UartLink::update()
{
    uint32_t now = millis();

    // =========================
    // RX (nicht blockierend)
    // =========================
    while (_link.available())
    {
        char c = _link.read();

        if (c == '\r') continue;

        if (c == '\n')
        {
            _buf[_idx] = '\0';

            _lastSeen = now;

            // =========================
            // PROTOKOLL (entscheidend!)
            // =========================

            // 🔴 Responder reagiert auf PING
            if (!_initiator && strcmp(_buf, "PING") == 0)
            {
                _link.println("PONG");
            }
            // 🔵 Initiator reagiert auf PONG
            else if (_initiator && strcmp(_buf, "PONG") == 0)
            {
                if (!_connected)
                {
                    _link.println("ACK");
                    _connected = true;
                }
            }
            // 🔴 Responder wird durch ACK verbunden
            else if (!_initiator && strcmp(_buf, "ACK") == 0)
            {
                _connected = true;
            }
            // KA → nur Lebenszeichen
            else if (strcmp(_buf, "KA") == 0)
            {
                // nichts tun
            }

            _idx = 0;
        }
        else
        {
            if (_idx < sizeof(_buf) - 1)
            {
                _buf[_idx++] = c;
            }
            else
            {
                // Overflow-Schutz
                _idx = 0;
            }
        }
    }

    // =========================
    // TX
    // =========================
    if (_initiator && (now - _lastPing > PING_INTERVAL))
    {
        _lastPing = now;

        if (!_connected)
        {
            _link.println("PING");
        }
        else
        {
            _link.println("KA");
        }
    }

    // =========================
    // Timeout
    // =========================
    if (_connected && (now - _lastSeen > TIMEOUT))
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

bool UartLink::isConnected() const
{
    return _connected;
}