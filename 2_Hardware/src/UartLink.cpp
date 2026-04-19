#include "UartLink.h"
#include <string.h>

UartLink::UartLink(Stream& link, bool initiator)
    : _link(link),
    _initiator(initiator),
    _connected(false),
    _lastPing(0),
    _lastSeen(0),
    _gotPong(false),
    _gotAck(false)
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
    _gotPong = false;
    _gotAck = false;
}

void UartLink::update()
{
    // =========================
    // Empfangen
    // =========================
    while (_link.available())
    {
        size_t n = _link.readBytesUntil('\n', _buf, sizeof(_buf) - 1);
        _buf[n] = '\0';

        // Debug ignorieren
        if (_buf[0] == '#')
            continue;

        if (n > 0 && _buf[n - 1] == '\r')
            _buf[n - 1] = '\0';

        // ✅ NUR hier: Verbindung lebt
        _lastSeen = millis();

        if (strcmp(_buf, "PING") == 0)
        {
            _link.println("PONG");
        }
        else if (strcmp(_buf, "PONG") == 0)
        {
            _gotPong = true;
            _link.println("ACK");
            _connected = true;
        }
        else if (strcmp(_buf, "ACK") == 0)
        {
            _gotAck = true;
            _connected = true;
        }
        else if (strcmp(_buf, "KA") == 0)
        {
            // Keepalive empfangen → reicht schon für _lastSeen
        }
    }

    // =========================
    // Verbindung bestätigen
    // =========================
    if (!_connected && _gotPong && _gotAck)
    {
        _connected = true;
    }

    // =========================
    // Senden
    // =========================
    if (_initiator)
    {
        if (millis() - _lastPing > PING_INTERVAL)
        {
            _lastPing = millis();

            if (!_connected)
            {
                // Handshake
                _link.println("PING");
            }
            else
            {
                // Keepalive
                _link.println("KA");

                // ❌ WICHTIG:
                // KEIN _lastSeen hier!
            }
        }
    }

    // =========================
    // Timeout (jetzt wirksam)
    // =========================
    if (_connected && millis() - _lastSeen > TIMEOUT)
    {
        _connected = false;
        _gotPong = false;
        _gotAck = false;
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