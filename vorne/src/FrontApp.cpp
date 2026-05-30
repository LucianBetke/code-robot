// ============================================================
// File: FrontApp.cpp
// ============================================================

#include "FrontApp.h"

#include <avr/wdt.h>

#include "src/Hardware.h"
#include "src/HardwarePins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"
#include "src/VehicleControlConfig.h"
#include "src/TelemetryPrinterConfig.h"
#include "src/ScaleUtils.h"
#include "src/WheelValues.h"

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
// Initialisierung und Hauptzyklus
// ============================================================

void FrontApp::begin()
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsFront::PINS);
    radControl_begin(ConfigFront::CONFIG);
    wheelMeasurement_reset_all();

    vehicle.begin(ConfigVehicleFront::CONFIG);

    commandRunner.begin();
    rearFrameClient.begin();
    frameScheduler.begin(VEHICLE_DT_MS);

    uart.begin();
    conn.begin(true);

    printer.printInfo(vehicle, ConfigFront::CONFIG);
}

void FrontApp::update(uint32_t now)
{
    updateCommunication();
    handleIncomingLines(now);
    updateFrameTimeout(now);
    updateConnectionSafety(now);

    if (isConnected())
    {
        updateVehicleAndFrontControl(now);

        // Erst Messframe pruefen.
        // Danach darf der CommandRunner den aktiven Befehl beenden.
        tryRequestFrame(now);

        updateCommandRunner(now);
        updateLogRaster(now);
        updateRearStopSequence(now);
    }
}

// ============================================================
// Interne loop()-Schritte
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
        vehicle.stop();

        radControl_stopAll();
        radControl_resetPiStates();
        wheelMeasurement_reset_all();

        rearFrameClient.clearWaiting();
        rearFrameClient.clearFrame();
        rearFrameClient.cancelStopSequence();

        frameScheduler.stop();

        _odomResetPending = true;
    }

    if (!prevConnected && nowConnected)
    {
        vehicle.stop();

        radControl_stopAll();
        radControl_resetPiStates();
        wheelMeasurement_reset_all();

        commandRunner.begin();
        rearFrameClient.begin();
        frameScheduler.begin(VEHICLE_DT_MS);

        _odomResetPending = true;
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

#if PRINTER_ENABLE_ODOM
        if (commandRunner.hasActivePathCommand())
        {
            printer.printOdom2(
                commandRunner.activeCmdpId(),
                rearFrameClient.frame().t,
                odometer
            );
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

    radControl_update(now);
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
    WheelSpeedCms wheelIst = {};

    // Uebersetzung lokale Front-Radordnung -> Fahrzeug-Radordnung:
    //   Front lokal Re = VoRe
    //   Front lokal Li = VoLi
    //   Rear kommt bereits als HiLi / HiRe vom RearFrameClient.
    wheelIst.v[VoRe] = wheelMeasurements[Re].cms();
    wheelIst.v[VoLi] = wheelMeasurements[Li].cms();
    wheelIst.v[HiLi] = (float)rearFrameClient.hiLiIstCms();
    wheelIst.v[HiRe] = (float)rearFrameClient.hiReIstCms();

    vehicle.updateIst(wheelIst);
}

void FrontApp::updateOdometerFromCompletedFrame()
{
    const RearPendingFrame& frame = rearFrameClient.frame();

    WheelCounts counts = {};

    // Uebersetzung Frame-Counts -> Fahrzeug-Radordnung:
    //   v[VoRe] = vorne rechts
    //   v[VoLi] = vorne links
    //   v[HiLi] = hinten links
    //   v[HiRe] = hinten rechts
    counts.v[VoRe] = frame.voReCnt;
    counts.v[VoLi] = frame.voLiCnt;
    counts.v[HiLi] = frame.hiLiCnt;
    counts.v[HiRe] = frame.hiReCnt;

    if (_odomResetPending || !odometer.isPrimed())
    {
        odometer.reset(counts);

        _odomResetPending = false;
        return;
    }

    odometer.update(counts);
}

RearFrameRequest FrontApp::makeRearFrameRequest(uint32_t frameTime, bool resetPi)
{
    RearFrameRequest request = {};

    request.frameTimeMs = frameTime;
    request.resetPi = resetPi;

    request.voLi_s_cms = scaleRoundToInt16(commandRunner.getWheelSoll(VoLi));
    request.voLi_i_cms = wheelMeasurements[Li].cmsInt();
    request.voLi_pwm = rad[Li].lastPwm();
    request.voLiCnt = (int32_t)wheelMeasurements[Li].counts_total();

    request.voRe_s_cms = scaleRoundToInt16(commandRunner.getWheelSoll(VoRe));
    request.voRe_i_cms = wheelMeasurements[Re].cmsInt();
    request.voRe_pwm = rad[Re].lastPwm();
    request.voReCnt = (int32_t)wheelMeasurements[Re].counts_total();

    request.hiLi_s_cms = scaleRoundToInt16(commandRunner.getWheelSoll(HiLi));
    request.hiRe_s_cms = scaleRoundToInt16(commandRunner.getWheelSoll(HiRe));

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

    wheelMeasurement_reset_all();

    radControl_resetPiStates();

    applyFrontWheelSoll();

    rearFrameClient.clearWaiting();
    rearFrameClient.clearFrame();

    frameScheduler.start(now);

    requestRearFrame(now, 0, true);
}