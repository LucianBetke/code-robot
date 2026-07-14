// ConnectionMonitor.h
#pragma once

#include <Arduino.h>
#include "src/UartLink.h"

class ConnectionMonitor
{
public:
    ConnectionMonitor(
        UartLink& uart,
        uint8_t ledPin
    );

    void begin(
        bool wait,
        bool useLed = true
    );

    void update();
    void waitForConnection();

private:
    UartLink& _uart;
    uint8_t _ledPin;

    bool _lastState;
    bool _useLed;

    void writeLed(uint8_t level);
};