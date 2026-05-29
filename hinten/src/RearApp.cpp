// RearApp.cpp
#include "RearApp.h"

#include <avr/wdt.h>

#include "src/CommProtocol.h"
#include "src/Hardware.h"
#include "src/HardwarePins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"

static const uint8_t REAR_SYNC_INPUT_PIN = 3;

RearApp::RearApp()
    : uart(Serial, false),
    conn(uart, 13),
    lastVsolMs(0),
    lastVsolFrameId(0),
    rearSollActive(false),
    syncFlag(false)
{
}

void RearApp::begin(void (*syncCallback)())
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsRear::PINS);
    radControl_begin(ConfigRear::CONFIG);
    wheelMeasurement_reset_all();

    uart.begin();
    conn.begin(false);

    hardware_enableMotors();

    pinMode(REAR_SYNC_INPUT_PIN, INPUT);
    attachInterrupt(digitalPinToInterrupt(REAR_SYNC_INPUT_PIN), syncCallback, RISING);
}

void RearApp::update(uint32_t now)
{
    updateCommunication();
    updateConnectionSafety(now);
    updateVsolTimeout(now);
    handleIncomingVsol(now);

    radControl_update(now);

    handleSyncVist();
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

        const float vSollLiCms = (float)vsol.hiLiSoll;
        const float vSollReCms = (float)vsol.hiReSoll;

        if (vsol.resetPi)
        {
            radControl_resetPiStates();
            wheelMeasurement_reset_all();
        }

        rad[Li].setSoll(vSollLiCms);
        rad[Re].setSoll(vSollReCms);

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

    const int16_t vIstLiCms = wheelMeasurements[Li].cmsInt();
    const int16_t vIstReCms = wheelMeasurements[Re].cmsInt();

    const int16_t pwmLi = rad[Li].lastPwm();
    const int16_t pwmRe = rad[Re].lastPwm();

    const int32_t cntLi = (int32_t)wheelMeasurements[Li].counts_total();
    const int32_t cntRe = (int32_t)wheelMeasurements[Re].counts_total();

    if (uart.isConnected())
    {
        printVist(
            Serial,
            lastVsolFrameId,
            vIstLiCms,
            vIstReCms,
            pwmLi,
            pwmRe,
            cntLi,
            cntRe
        );
    }
}