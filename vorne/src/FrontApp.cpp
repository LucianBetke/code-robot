// ============================================================
// File: FrontApp.cpp
// ============================================================

#include "FrontApp.h"

#include <avr/wdt.h>

#include "src/Hardware.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/PrinterConfig.h"

namespace
{
    int16_t cmsToInt16Rounded(float value_cms)
    {
        if (value_cms >= 0.0f)
        {
            return (int16_t)(value_cms + 0.5f);
        }

        return (int16_t)(value_cms - 0.5f);
    }
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

#if PRINTER_ENABLE_WHEELS || PRINTER_ENABLE_CHASSIS || PRINTER_ENABLE_COUNTS
        printer.printCompletedFrame(
            vehicle,
            rearFrameClient.frame(),
            rearFrameClient.hiLiIstCms(),
            rearFrameClient.hiReIstCms(),
            rearFrameClient.hiLiPwm(),
            rearFrameClient.hiRePwm()
        );
#endif

        if (commandRunner.hasActivePathCommand())
        {
            printer.printOdom2(
                commandRunner.activeCmdpId(),
                rearFrameClient.frame().t,
                odometer
            );
        }
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
    if (!commandRunner.hasActivePathCommand())
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
    if (!uart.isConnected())
    {
        return;
    }

    if (rearFrameClient.isBusy())
    {
        return;
    }

    const bool wasActive = commandRunner.isActive();
    const bool pathWasActive = commandRunner.hasActivePathCommand();

    commandRunner.update(now);

    const bool isActive = commandRunner.isActive();
    const bool pathIsActive = commandRunner.hasActivePathCommand();

    if (pathWasActive && !pathIsActive)
    {
        applyFrontWheelSoll();
        rearFrameClient.sendStop(Serial, now);
    }

    if (pathIsActive &&
        commandRunner.consumeStartFramePending())
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

    const bool pathActive = commandRunner.hasActivePathCommand();

    if (!pathActive)
    {
        frameScheduler.stop();

        if (!rearFrameClient.isBusy())
        {
            frame.hasFrontSnapshot = false;
        }
    }

    if (pathActive)
    {
        rearFrameClient.cancelStopSequence();

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
        speed[Re].cms(),
        speed[Li].cms(),
        (float)rearFrameClient.hiLiIstCms(),
        (float)rearFrameClient.hiReIstCms()
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

    request.voLi_s_cms = cmsToInt16Rounded(commandRunner.getWheelSoll(VoLi));
    request.voLi_i_cms = speed[Li].cmsInt();
    request.voLi_pwm = rad[Li].lastPwm();
    request.voLiCnt = (int32_t)speed[Li].counts_total();

    request.voRe_s_cms = cmsToInt16Rounded(commandRunner.getWheelSoll(VoRe));
    request.voRe_i_cms = speed[Re].cmsInt();
    request.voRe_pwm = rad[Re].lastPwm();
    request.voReCnt = (int32_t)speed[Re].counts_total();

    request.hiLi_s_cms = cmsToInt16Rounded(commandRunner.getWheelSoll(HiLi));
    request.hiRe_s_cms = cmsToInt16Rounded(commandRunner.getWheelSoll(HiRe));

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

    _odomResetPending = true;

    speed_reset_all();

    control_resetPiStates();

    applyFrontWheelSoll();

    rearFrameClient.clearWaiting();
    rearFrameClient.clearFrame();

    frameScheduler.start(now);

    requestRearFrame(now, 0, true);
}