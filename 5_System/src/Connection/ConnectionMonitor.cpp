// ConnectionMonitor.cpp
#include "ConnectionMonitor.h"

ConnectionMonitor::ConnectionMonitor(UartLink& uart, uint8_t ledPin)
    : _uart(uart), _ledPin(ledPin), _lastState(false), _lastOk(0)
{
}

void ConnectionMonitor::waitForConnection()
{
    Serial.println("#Warte auf Handshake...");

    while (!_uart.isConnected())
    {
        _uart.update();
        update();   // eigene Logik
    }

    Serial.println("#Handshake1 OK");
}

void ConnectionMonitor::begin(bool wait)
{
    // 🔥 Hardware-Init
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, HIGH);   // wartet auf Verbindung

    // 🔥 Zustand
    _lastState = false;
    _lastOk = millis();

    // 🔥 optional blockierend
    if (wait)
    {
        waitForConnection();
    }
}

void ConnectionMonitor::update()
{
    bool now = _uart.isConnected();

    // 🔴 Verbindung verloren
    if (!now && _lastState)
    {
        Serial.println("#DISCONNECTED");
        digitalWrite(_ledPin, HIGH);
    }

    // 🟢 Verbindung neu da
    if (now && !_lastState)
    {
        Serial.println("#CONNECTED");
        digitalWrite(_ledPin, LOW);
    }

    _lastState = now;
}