// ConnectionMonitor.h
#pragma once

#include <Arduino.h>
#include "../2_Hardware/src/UartLink.h"

class ConnectionMonitor
{
public:
    ConnectionMonitor(UartLink& uart, uint8_t ledPin);

    void begin();
    void update();

private:
    UartLink& _uart;
    uint8_t _ledPin;

    bool _lastState;
    uint32_t _lastOk;
};