// RearApp.cpp
#include "RearApp.h"

#include "src/CommProtocol.h"
#include "src/Control.h"
#include "src/ControlConfig.h"

namespace
{
    float cmsToMps(int16_t value_cms)
    {
        return (float)value_cms * 0.01f;
    }

    int16_t mpsToCmsRounded(float value_mps)
    {
        const float value_cms = value_mps * 100.0f;

        if (value_cms >= 0.0f)
        {
            return (int16_t)(value_cms + 0.5f);
        }

        return (int16_t)(value_cms - 0.5f);
    }
}

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

        const float vSollLi = cmsToMps(vsol.hiLiSoll);
        const float vSollRe = cmsToMps(vsol.hiReSoll);

        if (vsol.resetPi)
        {
            // Neuer CMDP-Fahrabschnitt:
            // PI-Zustaende und SpeedWeg-Filter muessen beide zurueckgesetzt werden,
            // damit alte Istgeschwindigkeiten nicht in den neuen Abschnitt laufen.
            control_resetPiStates();
            speed_reset_all();
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

    const int16_t vIstLi = mpsToCmsRounded(speed[Li].mps());
    const int16_t vIstRe = mpsToCmsRounded(speed[Re].mps());

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