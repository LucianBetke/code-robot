// ConnectionMonitor.h
#pragma once

#include <Arduino.h>
#include "src/UartLink.h"

class ConnectionMonitor
{
public:
    ConnectionMonitor(UartLink& uart, uint8_t ledPin);

    void begin(bool wait);
    void update();
    void waitForConnection();

private:
    UartLink& _uart;
    uint8_t _ledPin;

    bool _lastState;
    uint32_t _lastOk;
};