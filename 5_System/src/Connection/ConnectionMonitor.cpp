// ConnectionMonitor.cpp
#include "ConnectionMonitor.h"

ConnectionMonitor::ConnectionMonitor(UartLink& uart, uint8_t ledPin)
    : _uart(uart),
    _ledPin(ledPin),
    _lastState(false),
    _lastOk(0)
{
}

void ConnectionMonitor::waitForConnection()
{
    Serial.println(F("#WAIT"));

    while (!_uart.isConnected())
    {
        _uart.update();
        update();
    }

    Serial.println(F("#HS1"));
}

void ConnectionMonitor::begin(bool wait)
{
    pinMode(_ledPin, OUTPUT);
    digitalWrite(_ledPin, HIGH);

    _lastState = false;
    _lastOk = millis();

    if (wait) waitForConnection();
}

void ConnectionMonitor::update()
{
    const bool now = _uart.isConnected();

    if (!now && _lastState)
    {
        Serial.println(F("#DIS"));
        digitalWrite(_ledPin, HIGH);
    }

    if (now && !_lastState)
    {
        Serial.println(F("#CON"));
        digitalWrite(_ledPin, LOW);
    }

    _lastState = now;
}