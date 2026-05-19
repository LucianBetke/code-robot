// RearApp.cpp
#include "RearApp.h"

#include "src/CommProtocol.h"
#include "src/CommUtils.h"
#include "src/Control.h"
#include "src/ControlConfig.h"

RearApp::RearApp()
    : uart(Serial, false),
    conn(uart, 13),
    lastVsolMs(0),
    lastVsolFrameId(0),
    rearSollActive(false),
    syncFlag(false)
{
}

void RearApp::begin()
{
    uart.begin();
    conn.begin(false);
}

void RearApp::updateCommunication()
{
    uart.update();
    conn.update();
}

void RearApp::stopRearWheels()
{
    rad[Li].setSoll(0.0f);
    rad[Re].setSoll(0.0f);
    rearSollActive = false;
}

void RearApp::updateConnectionSafety(uint32_t now)
{
    (void)now;

    if (!uart.isConnected())
    {
        stopRearWheels();
    }
}

void RearApp::updateVsolTimeout(uint32_t now)
{
    if (rearSollActive &&
        lastVsolMs > 0 &&
        now - lastVsolMs > 2 * VEHICLE_DT_MS)
    {
        stopRearWheels();
    }
}

void RearApp::handleIncomingVsol(uint32_t now)
{
    if (!uart.availableLine()) return;

    const char* line = uart.getLine();

    VsolMessage vsol = {};

    if (parseVsolLine(line, vsol))
    {
        lastVsolFrameId = vsol.frameId;

        float vSollLi = int100ToFloat(vsol.hiLiSoll);
        float vSollRe = int100ToFloat(vsol.hiReSoll);

        if (vsol.resetPi)
        {
            control_resetPiStates();
        }

        rad[Li].setSoll(vSollLi);
        rad[Re].setSoll(vSollRe);

        lastVsolMs = now;
        rearSollActive = (vsol.hiLiSoll != 0 || vsol.hiReSoll != 0);

        if (uart.isConnected())
        {
            printVsolOk(Serial, lastVsolFrameId);
        }
    }
}

void RearApp::onSyncPulseFromIsr()
{
    syncFlag = true;
}

void RearApp::handleSyncVist()
{
    if (!syncFlag) return;

    syncFlag = false;

    const int16_t vIstLi = floatToInt100(speed[Li].mps());
    const int16_t vIstRe = floatToInt100(speed[Re].mps());

    const int16_t pwmLi = rad[Li].lastPwm();
    const int16_t pwmRe = rad[Re].lastPwm();

    const int32_t cntLi = (int32_t)speed[Li].counts_total();
    const int32_t cntRe = (int32_t)speed[Re].counts_total();

    if (uart.isConnected())
    {
        printVist(
            Serial,
            lastVsolFrameId,
            vIstLi,
            vIstRe,
            pwmLi,
            pwmRe,
            cntLi,
            cntRe
        );
    }
}