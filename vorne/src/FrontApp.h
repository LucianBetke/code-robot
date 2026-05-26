// FrontApp.h
#ifndef FRONT_APP_H
#define FRONT_APP_H

#include <Arduino.h>

#include "CommandScript.h"

#include "src/VehicleController.h"
#include "src/MecanumOdometer.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"
#include "src/RearFrameClient.h"
#include "src/FrameScheduler.h"
#include "src/Printer.h"

// ============================================================
// FrontApp
//
// Konkrete Anwendung fuer den Front-Nano.
//
// Wichtig:
//  - Kein Regler.
//  - Keine allgemeine Systemklasse.
//  - Front-spezifische Ablaufsteuerung.
//
// Die .ino darf den Ablauf sichtbar behalten.
// Deshalb sind die Hauptmodule hier bewusst oeffentlich.
// ============================================================

class FrontApp
{
public:
    FrontApp();

    // --------------------------------------------------------
    // Sichtbare Hauptmodule fuer setup()
    // --------------------------------------------------------

    VehicleController vehicle;
    MecanumOdometer odometer;

    UartLink uart;
    ConnectionMonitor conn;
    CommandRunner commandRunner;

    RearFrameClient rearFrameClient;
    FrameScheduler frameScheduler;
    Printer printer;

    // --------------------------------------------------------
    // Sichtbare loop()-Schritte fuer vorne.ino
    // --------------------------------------------------------

    void updateCommunication();
    void updateConnectionSafety(uint32_t now);
    void handleIncomingLines(uint32_t now);
    void updateFrameTimeout(uint32_t now);
    void tryRequestFrame(uint32_t now);
    void updateCommandRunner(uint32_t now);
    void updateLogRaster(uint32_t now);
    void updateVehicleAndFrontControl(uint32_t now);
    void updateRearStopSequence(uint32_t now);

    bool isConnected() const;

private:
    bool _odomResetPending;

    void resetByWatchdog();

    void applyFrontWheelSoll();
    void updateVehicleIst();
    void updateOdometerFromCompletedFrame();

    RearFrameRequest makeRearFrameRequest(uint32_t frameTime, bool resetPi);
    void requestRearFrame(uint32_t now, uint32_t frameTime, bool resetPi);
    void requestStartFrameForNewCommand(uint32_t now);
};

#endif // FRONT_APP_H