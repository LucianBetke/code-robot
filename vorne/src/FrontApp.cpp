// ============================================================
// File: FrontApp.cpp
// ============================================================

#include "FrontApp.h"

#include <avr/wdt.h>

#include "src/Hardware.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/PrinterConfig.h"

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
    _odomResetPending(true)
{
}

// ============================================================
// Sichtbare loop()-Schritte
// ============================================================

void FrontApp::updateCommunication()
{
    uart.update();
    conn.update();
}

void FrontApp::updateConnectionSafety(uint32_t now)
{
    (void)now;

    static bool prevConnected = false;
    const bool nowConnected = uart.isConnected();

    if (prevConnected && !nowConnected)
    {
        resetByWatchdog();
    }

    prevConnected = nowConnected;
}

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

    if (rearFrameClient.handleVistLine(line))
    {
        updateVehicleIst();
        updateOdometerFromCompletedFrame();

        printer.printCompletedFrame(
            vehicle,
            rearFrameClient.frame(),
            rearFrameClient.hiLiIst(),
            rearFrameClient.hiReIst(),
            rearFrameClient.hiLiPwm(),
            rearFrameClient.hiRePwm()
        );

#if PRINTER_ENABLE_ODOM
        if (commandRunner.hasActivePathCommand())
        {
            Serial.print(F("#ODOM2,"));
            Serial.print((unsigned int)commandRunner.activeCmdpId());
            Serial.print(',');
            Serial.print((unsigned long)rearFrameClient.frame().t);
            Serial.print(',');
            Serial.print(odometer.absCm(), 2);
            Serial.print(',');
            Serial.print(odometer.xCm(), 2);
            Serial.print(',');
            Serial.print(odometer.yCm(), 2);
            Serial.print(',');
            Serial.println(odometer.phiDeg(), 2);
        }
#endif
    }
}

void FrontApp::updateFrameTimeout(uint32_t now)
{
    if (rearFrameClient.isBusy() &&
        now - rearFrameClient.requestMs() > 2 * VEHICLE_DT_MS)
    {
        resetByWatchdog();
    }
}

void FrontApp::tryRequestFrame(uint32_t now)
{
    if (!commandRunner.isActive())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    uint32_t frameTime = 0;

    if (frameScheduler.due(now, frameTime))
    {
        requestRearFrame(now, frameTime, false);
    }
}

void FrontApp::updateCommandRunner(uint32_t now)
{
    // Solange noch VSOL_OK oder VIST offen ist, darf der CommandRunner
    // nicht zum naechsten Befehl springen. Sonst geht der letzte Frame
    // eines Befehls verloren.

    if (!uart.isConnected())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    const bool wasActive = commandRunner.isActive();

    commandRunner.update(now);

    const bool isActive = commandRunner.isActive();

    if (isActive && commandRunner.consumeStartFramePending())
    {
        requestStartFrameForNewCommand(now);
    }

    if (wasActive && !isActive && commandRunner.isFinished())
    {
        rearFrameClient.armStopSequence();
    }
}

void FrontApp::updateLogRaster(uint32_t now)
{
    RearPendingFrame& frame = rearFrameClient.frame();

    if (!commandRunner.isActive())
    {
        frameScheduler.stop();

        if (!rearFrameClient.isBusy())
        {
            frame.hasFrontSnapshot = false;
        }
    }

    if (commandRunner.isActive())
    {
        rearFrameClient.cancelStopSequence();

        // Fallback, falls ein aktiver Befehl ohne Startframe laufen sollte.
        if (!frameScheduler.isRunning())
        {
            frameScheduler.start(now);
        }
    }
}

void FrontApp::updateVehicleAndFrontControl(uint32_t now)
{
    updateVehicleIst();

    vehicle.update(now);

    applyFrontWheelSoll();

    control_update(now);
}

void FrontApp::updateRearStopSequence(uint32_t now)
{
    const bool ready =
        uart.isConnected() &&
        !commandRunner.isActive() &&
        commandRunner.isFinished();

    rearFrameClient.updateStopSequence(
        Serial,
        now,
        ready,
        VEHICLE_DT_MS
    );
}

bool FrontApp::isConnected() const
{
    return uart.isConnected();
}

// ============================================================
// Private Hilfsfunktionen
// ============================================================

void FrontApp::resetByWatchdog()
{
    wdt_enable(WDTO_15MS);

    while (1)
    {
    }
}

void FrontApp::applyFrontWheelSoll()
{
    rad[Li].setSoll(commandRunner.getWheelSoll(VoLi));
    rad[Re].setSoll(commandRunner.getWheelSoll(VoRe));
}

void FrontApp::updateVehicleIst()
{
    vehicle.updateIst(
        speed[Re].mps(),
        speed[Li].mps(),
        rearFrameClient.hiLiIst(),
        rearFrameClient.hiReIst()
    );
}

void FrontApp::updateOdometerFromCompletedFrame()
{
    const RearPendingFrame& frame = rearFrameClient.frame();

    if (_odomResetPending || !odometer.isPrimed())
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

RearFrameRequest FrontApp::makeRearFrameRequest(uint32_t frameTime, bool resetPi)
{
    RearFrameRequest request = {};

    request.frameTimeMs = frameTime;
    request.resetPi = resetPi;

    request.voLi_s = commandRunner.getWheelSoll(VoLi);
    request.voLi_i = speed[Li].mps();
    request.voLi_pwm = rad[Li].lastPwm();
    request.voLiCnt = (int32_t)speed[Li].counts_total();

    request.voRe_s = commandRunner.getWheelSoll(VoRe);
    request.voRe_i = speed[Re].mps();
    request.voRe_pwm = rad[Re].lastPwm();
    request.voReCnt = (int32_t)speed[Re].counts_total();

    request.hiLi_s = commandRunner.getWheelSoll(HiLi);
    request.hiRe_s = commandRunner.getWheelSoll(HiRe);

    return request;
}

void FrontApp::requestRearFrame(uint32_t now, uint32_t frameTime, bool resetPi)
{
    const RearFrameRequest request = makeRearFrameRequest(frameTime, resetPi);

    rearFrameClient.requestFrame(
        Serial,
        now,
        request
    );

    // VIST wird erst nach VSOL_OK,<frameId> angefordert.
}

void FrontApp::requestStartFrameForNewCommand(uint32_t now)
{
    if (!uart.isConnected())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    // Neuer Fahrabschnitt:
    // Odometrie wird beim ersten vollstaendigen Frame dieses Befehls
    // auf die dann vorhandenen vier Encoderstaende genullt.
    _odomResetPending = true;

    // Neuer echter Fahrabschnitt:
    // Front-PI einmalig zuruecksetzen.
    // Rear-PI wird ueber resetPi=true im VSOL-Startframe zurueckgesetzt.
    control_resetPiStates();

    applyFrontWheelSoll();

    rearFrameClient.clearWaiting();
    rearFrameClient.clearFrame();

    frameScheduler.start(now);

    // Startframe:
    // t = 0, neue Sollwerte, resetPi=true fuer Rear.
    requestRearFrame(now, 0, true);
}