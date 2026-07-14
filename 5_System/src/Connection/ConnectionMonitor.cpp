// ConnectionMonitor.cpp
#include "ConnectionMonitor.h"

ConnectionMonitor::ConnectionMonitor(
    UartLink& uart,
    uint8_t ledPin
)
    : _uart(uart),
    _ledPin(ledPin),
    _lastState(false),
    _useLed(true)
{}

void ConnectionMonitor::writeLed(uint8_t level)
{
    if (_useLed)
    {
        digitalWrite(_ledPin, level);
    }
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

void ConnectionMonitor::begin(
    bool wait,
    bool useLed
)
{
    _useLed = useLed;
    _lastState = false;

    if (_useLed)
    {
        pinMode(_ledPin, OUTPUT);
        digitalWrite(_ledPin, HIGH);
    }

    if (wait)
    {
        waitForConnection();
    }
}

void ConnectionMonitor::update()
{
    const bool now = _uart.isConnected();

    if (!now && _lastState)
    {
        Serial.println(F("#DIS"));
        writeLed(HIGH);
    }

    if (now && !_lastState)
    {
        Serial.println(F("#CON"));
        writeLed(LOW);
    }

    _lastState = now;
}