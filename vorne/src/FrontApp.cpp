// ============================================================
// File: FrontApp.cpp
// Zweck:
//  - Ablaufsteuerung fuer den Front-Nano
//  - UART-Kommunikation mit Rear-Nano
//  - Frame-Anforderung / VIST-Auswertung
//  - Odometrie-Update
//  - Verbindung von CommandRunner, VehicleController, Control und Printer
// ============================================================

#include "FrontApp.h"

#include <avr/wdt.h>

#include "src/Hardware.h"
#include "src/Control.h"
#include "src/CommProtocol.h"
#include "src/ScaleUtils.h"

namespace
{
    const uint16_t REAR_FRAME_TIMEOUT_MS = 300;
}

// ============================================================
// Konstruktor
// ============================================================

FrontApp::FrontApp()
    : vehicle(),
    odometer(),
    uart(Serial, true),
    conn(uart, 13),
    commandRunner(vehicle, odometer, CommandScript::get, CommandScript::size),
    rearFrameClient(),
    frameScheduler(),
    printer(),
    _odomResetPending(false)
{
}

// ============================================================
// Kommunikation / Verbindung
// ============================================================

void FrontApp::updateCommunication()
{
    uart.update();
    conn.update();
}

bool FrontApp::isConnected() const
{
    return uart.isConnected();
}

void FrontApp::updateConnectionSafety(uint32_t now)
{
    (void)now;

    if (!uart.isConnected())
    {
        vehicle.stop();
        control_stopAll();

        rearFrameClient.clearWaiting();
        rearFrameClient.cancelStopSequence();

        frameScheduler.stop();
        _odomResetPending = false;
    }
}

// ============================================================
// Eingehende UART-Zeilen
// ============================================================

void FrontApp::handleIncomingLines(uint32_t now)
{
    if (!uart.availableLine())
    {
        return;
    }

    const char* line = uart.getLine();

    if (rearFrameClient.handleVsolOkLine(line, now))
    {
        hardware_requestVist();
        return;
    }

    VistMessage vist = {};

    if (rearFrameClient.handleVistLine(line, vist))
    {
        updateOdometerFromCompletedFrame();
        updateVehicleIst();

        const RearPendingFrame& frame = rearFrameClient.frame();

        printer.printCompletedFrame(
            vehicle,
            frame,
            (float)rearFrameClient.hiLiIstCms(),
            (float)rearFrameClient.hiReIstCms(),
            rearFrameClient.hiLiPwm(),
            rearFrameClient.hiRePwm()
        );

        printer.printOdom2(
            commandRunner.activeCmdpId(),
            frame.t,
            odometer
        );

        return;
    }
}

// ============================================================
// Timeout fuer Rear-Frame
// ============================================================

void FrontApp::updateFrameTimeout(uint32_t now)
{
    if (!rearFrameClient.isBusy())
    {
        return;
    }

    const uint32_t requestMs = rearFrameClient.requestMs();

    if (requestMs == 0)
    {
        return;
    }

    if ((uint32_t)(now - requestMs) <= REAR_FRAME_TIMEOUT_MS)
    {
        return;
    }

#if PRINTER_ENABLE_ERRORS
    Serial.println(F("#ERROR,RearFrameTimeout"));
#endif

    rearFrameClient.clearWaiting();
    rearFrameClient.armStopSequence();

    frameScheduler.stop();
    _odomResetPending = false;
}

// ============================================================
// Fahrzeug / Front-Regelung
// ============================================================

void FrontApp::applyFrontWheelSoll()
{
    control_setSoll(Li, vehicle.getWheelSoll(VoLi));
    control_setSoll(Re, vehicle.getWheelSoll(VoRe));
}

void FrontApp::updateVehicleIst()
{
    vehicle.updateIst(
        speed[Re].cms(),                         // VoRe
        speed[Li].cms(),                         // VoLi
        (float)rearFrameClient.hiLiIstCms(),     // HiLi
        (float)rearFrameClient.hiReIstCms()      // HiRe
    );
}

void FrontApp::updateVehicleAndFrontControl(uint32_t now)
{
    vehicle.update(now);
    applyFrontWheelSoll();
    control_update(now);
}

// ============================================================
// RearFrameRequest aufbauen
// ============================================================

RearFrameRequest FrontApp::makeRearFrameRequest(uint32_t frameTime, bool resetPi)
{
    RearFrameRequest request = {};

    request.frameTimeMs = frameTime;
    request.resetPi = resetPi;

    request.voLi_s_cms = scaleRoundToInt16(vehicle.getWheelSoll(VoLi));
    request.voLi_i_cms = speed[Li].cmsInt();
    request.voLi_pwm = rad[Li].lastPwm();
    request.voLiCnt = (int32_t)speed[Li].counts_total();

    request.voRe_s_cms = scaleRoundToInt16(vehicle.getWheelSoll(VoRe));
    request.voRe_i_cms = speed[Re].cmsInt();
    request.voRe_pwm = rad[Re].lastPwm();
    request.voReCnt = (int32_t)speed[Re].counts_total();

    request.hiLi_s_cms = scaleRoundToInt16(vehicle.getWheelSoll(HiLi));
    request.hiRe_s_cms = scaleRoundToInt16(vehicle.getWheelSoll(HiRe));

    return request;
}

void FrontApp::requestRearFrame(uint32_t now, uint32_t frameTime, bool resetPi)
{
    if (!uart.isConnected())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    RearFrameRequest request = makeRearFrameRequest(frameTime, resetPi);

    rearFrameClient.requestFrame(
        Serial,
        now,
        request
    );
}

void FrontApp::requestStartFrameForNewCommand(uint32_t now)
{
    if (rearFrameClient.isBusy())
    {
        return;
    }

    if (!commandRunner.consumeStartFramePending())
    {
        return;
    }

    // Wichtig:
    // Ab jetzt darf der CommandRunner den Weg noch NICHT pruefen.
    // Erst der erste VIST-Frame setzt den neuen Odometrie-Nullpunkt.
    _odomResetPending = true;

    rearFrameClient.cancelStopSequence();
    frameScheduler.start(now);

    requestRearFrame(now, 0, true);
}

void FrontApp::tryRequestFrame(uint32_t now)
{
    if (!commandRunner.hasActivePathCommand())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    uint32_t frameTime = 0;

    if (!frameScheduler.due(now, frameTime))
    {
        return;
    }

    requestRearFrame(now, frameTime, false);
}

// ============================================================
// CommandRunner / Messraster / Stop-Sequenz
// ============================================================

void FrontApp::updateCommandRunner(uint32_t now)
{
    // --------------------------------------------------------
    // Zentrale Korrektur:
    //
    // Nach dem Start eines neuen CMDP wartet FrontApp auf den
    // ersten VIST-Frame. Dieser Frame setzt in
    // updateOdometerFromCompletedFrame() den Odometrie-Nullpunkt.
    //
    // Solange _odomResetPending true ist, darf der CommandRunner
    // KEINE Wegpruefung machen. Sonst benutzt er noch den alten
    // Odometerstand vom vorherigen CMDP und beendet den neuen
    // Befehl sofort.
    // --------------------------------------------------------
    if (_odomResetPending)
    {
        return;
    }

    const bool wasActivePath = commandRunner.hasActivePathCommand();

    commandRunner.update(now);

    const bool isActivePath = commandRunner.hasActivePathCommand();

    if (wasActivePath && !isActivePath)
    {
        frameScheduler.stop();
        rearFrameClient.armStopSequence();
    }
}

void FrontApp::updateLogRaster(uint32_t now)
{
    requestStartFrameForNewCommand(now);
}

void FrontApp::updateRearStopSequence(uint32_t now)
{
    if (commandRunner.hasActivePathCommand())
    {
        return;
    }

    rearFrameClient.updateStopSequence(
        Serial,
        now,
        uart.isConnected(),
        VEHICLE_DT_MS
    );
}

// ============================================================
// Odometrie
// ============================================================

void FrontApp::updateOdometerFromCompletedFrame()
{
    const RearPendingFrame& frame = rearFrameClient.frame();

    if (_odomResetPending)
    {
        odometer.reset(
            frame.voReCnt,
            frame.voLiCnt,
            frame.hiLiCnt,
            frame.hiReCnt
        );

        _odomResetPending = false;
        return;
    }

    odometer.update(
        frame.voReCnt,
        frame.voLiCnt,
        frame.hiLiCnt,
        frame.hiReCnt
    );
}

// ============================================================
// Watchdog-Reset
// ============================================================

void FrontApp::resetByWatchdog()
{
    wdt_enable(WDTO_15MS);

    while (true)
    {
    }
}