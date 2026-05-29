// RearApp.h
#ifndef REAR_APP_H
#define REAR_APP_H

#include <Arduino.h>

#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"

class RearApp
{
public:
    RearApp();

    void begin(void (*syncCallback)());
    void update(uint32_t now);

    void onSyncPulseFromIsr();

private:
    UartLink uart;
    ConnectionMonitor conn;

    uint32_t lastVsolMs;
    uint16_t lastVsolFrameId;

    bool rearSollActive;
    volatile bool syncFlag;

    void updateCommunication();

    void stopRearWheels();
    void updateConnectionSafety(uint32_t now);
    void updateVsolTimeout(uint32_t now);

    void handleIncomingVsol(uint32_t now);
    void handleSyncVist();
};

#endif // REAR_APP_H