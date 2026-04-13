// UartLink.cpp

#include "UartLink.h"
#include <string.h>

UartLink::UartLink(Stream& link)
    : _link(link),
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
    _lastSeen = 0;
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
        
        // NEU: Wenn die Zeile mit '#' beginnt, sofort ignorieren
        if (_buf[0] == '#') {
            continue;
        }

        if (n > 0 && _buf[n - 1] == '\r')
        {
            _buf[n - 1] = '\0';
        }

        _lastSeen = millis();

        if (strcmp(_buf, "PING") == 0)
        {
            _link.println("PONG");
        }
        else if (strcmp(_buf, "PONG") == 0)
        {
            _gotPong = true;
            _link.println("ACK");
    //        _connected = true; // NEU: Sobald PONG da ist, sind wir verbunden
        }
        else if (strcmp(_buf, "ACK") == 0)
        {
            _gotAck = true;
            _connected = true; // NEU: Sobald PONG da ist, sind wir verbunden
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
    // PING senden
    // =========================
    if (!_connected)
    {
        if (millis() - _lastPing > PING_INTERVAL)
        {
            _lastPing = millis();
            _link.println("PING");
        }
    }

    // =========================
    // Timeout
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