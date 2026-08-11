// RearApp.h
#ifndef REAR_APP_H
#define REAR_APP_H

#include <Arduino.h>

#include "src/UartLink.h"
#include "src/ConnectionMonitor.h"
#include "src/UltrasonicManager.h"

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
    UltrasonicManager ultrasonic;

    uint32_t lastVsolMs;
    uint16_t lastVsolFrameId;
    uint32_t lastUsSendMs;

    bool rearSollActive;
    volatile bool syncFlag;

    // SYNC-Diagnose: zaehlt jeden per ISR empfangenen
    // Impuls und gibt ihn einmal pro Sekunde per UART aus.
    volatile uint16_t syncPulseCount;
    uint32_t lastSyncDiagMs;

    void updateCommunication();

    void stopRearWheels();
    void updateConnectionSafety(uint32_t now);
    void resetByWatchdog();
    void updateVsolTimeout(uint32_t now);

    void handleIncomingVsol(uint32_t now);
    void handleSyncVist(uint32_t now);

    void updateUltrasonic(uint32_t now);
    void sendUltrasonicSnapshot(uint32_t now);

    void updateSyncDiagnostics(uint32_t now);

    // Startsperre bei zu leerem Akku.
    void checkBatteryOrHalt();
    void haltOnLowBattery(uint16_t battMv);
};

#endif // REAR_APP_H
