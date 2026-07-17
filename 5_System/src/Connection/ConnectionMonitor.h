// ConnectionMonitor.h
#pragma once

#include <Arduino.h>
#include "src/UartLink.h"

class ConnectionMonitor
{
public:
    // Variante MIT Status-LED (z. B. vorne).
    ConnectionMonitor(
        UartLink& uart,
        uint8_t ledPin
    );

    // Variante OHNE Status-LED (z. B. hinten).
    // Es wird kein Pin belegt oder geschaltet.
    explicit ConnectionMonitor(
        UartLink& uart
    );

    // Ob die LED benutzt wird, entscheidet allein der Konstruktor.
    void begin(
        bool wait
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
