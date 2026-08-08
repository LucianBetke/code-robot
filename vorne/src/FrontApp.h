// FrontApp.h
#ifndef FRONT_APP_H
#define FRONT_APP_H

#include <Arduino.h>

#include "CommandScript.h"

#include "src/CommProtocol.h"
#include "src/VehicleController.h"
#include "src/MecanumOdometer.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"
#include "src/RearFrameClient.h"
#include "src/FrameScheduler.h"
#include "src/TelemetryPrinter.h"
#include "src/UltrasonicManager.h"

class FrontApp
{
public:
    FrontApp();

    void begin();
    void update(uint32_t now);

    bool isConnected() const;

private:
    VehicleController vehicle;
    MecanumOdometer odometer;

    UartLink uart;
    ConnectionMonitor conn;
    CommandRunner commandRunner;

    RearFrameClient rearFrameClient;
    FrameScheduler frameScheduler;
    TelemetryPrinter printer;

    // Die beiden seitlichen HC-SR04 misst der vordere Nano
    // selbst. Der Frontwert kommt weiterhin per UART.
    UltrasonicManager ultrasonic;

    bool _odomResetPending;

    UsMessage _lastUs;
    uint32_t _lastUsReceivedMs;
    bool _hasUs;

    void updateCommunication();
    void updateConnectionSafety(uint32_t now);
    void updateUltrasonic(uint32_t now);

    void handleIncomingLines(uint32_t now);
    bool handleUltrasonicLine(
        const char* line,
        uint32_t now);

    void updateFrameTimeout(uint32_t now);
    void tryRequestFrame(uint32_t now);
    void updateCommandRunner(uint32_t now);
    void updateLogRaster(uint32_t now);
    void updateVehicleAndFrontControl(uint32_t now);
    void updateRearStopSequence(uint32_t now);

    void resetByWatchdog();

    void applyFrontWheelSoll();
    void updateVehicleIst();
    void updateOdometerFromCompletedFrame();

    RearFrameRequest makeRearFrameRequest(
        uint32_t frameTime,
        bool resetPi);

    void requestRearFrame(
        uint32_t now,
        uint32_t frameTime,
        bool resetPi);

    void requestStartFrameForNewCommand(uint32_t now);
};

#endif // FRONT_APP_H