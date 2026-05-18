// hinten.ino
#include <avr/wdt.h>
#include "src/CommProtocol.h"
#include "src/Hardware.h"
#include "src/hardware_pins.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"

UartLink uart(Serial, false);
ConnectionMonitor conn(uart, 13);

static uint32_t lastVsolMs = 0;

// Merkt sich die letzte empfangene VSOL-Frame-ID.
// Diese ID wird bei VIST wieder zurueckgesendet.
static uint16_t g_lastVsolFrameId = 0;

// Merkt sich, ob hinten zuletzt einen echten Fahr-Sollwert bekommen hat.
// VSOL,<id>,<resetPi>,0,0 bedeutet: kein aktiver Fahrbefehl mehr.
static bool g_rearSollActive = false;

volatile bool g_syncFlag = false;

void syncISR()
{
    g_syncFlag = true;
}

// ============================================================
// Hilfsfunktionen
// ============================================================

static void stopRearWheels()
{
    rad[Li].setSoll(0.0f);
    rad[Re].setSoll(0.0f);
    g_rearSollActive = false;
}

// ============================================================
// loop()-Teilfunktionen
// ============================================================

static void updateRearConnectionSafety(uint32_t now)
{
    (void)now;

    if (!uart.isConnected())
    {
        stopRearWheels();
    }
}

static void updateVsolTimeout(uint32_t now)
{
    if (g_rearSollActive &&
        lastVsolMs > 0 &&
        now - lastVsolMs > 2 * VEHICLE_DT_MS)
    {
        stopRearWheels();
    }
}

static void handleIncomingVsol(uint32_t now)
{
    if (!uart.availableLine()) return;

    const char* line = uart.getLine();

    VsolMessage vsol = {};

    if (parseVsolLine(line, vsol))
    {
        g_lastVsolFrameId = vsol.frameId;

        float vSollLi = int100ToFloat(vsol.hiLiSoll);
        float vSollRe = int100ToFloat(vsol.hiReSoll);

        if (vsol.resetPi)
        {
            control_resetPiStates();
        }

        rad[Li].setSoll(vSollLi);
        rad[Re].setSoll(vSollRe);

        lastVsolMs = now;
        g_rearSollActive = (vsol.hiLiSoll != 0 || vsol.hiReSoll != 0);

        if (uart.isConnected()) printVsolOk(Serial, g_lastVsolFrameId);
    }
}

static void handleSyncVist()
{
    if (!g_syncFlag) return;

    g_syncFlag = false;

    int16_t vIstLi = floatToInt100(speed[Li].mps());
    int16_t vIstRe = floatToInt100(speed[Re].mps());

    int16_t pwm2 = rad[Li].lastPwm();
    int16_t pwm3 = rad[Re].lastPwm();

    if (uart.isConnected())
    {
        printVist(Serial, g_lastVsolFrameId, vIstLi, vIstRe, pwm2, pwm3);
    }
}

// ============================================================
// setup()
// ============================================================

void setup()
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsRear::PINS);
    control_begin(ConfigRear::CONFIG);
    speed_reset_all();

    uart.begin();
    conn.begin(false);

    hardware_enableMotors();

    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), syncISR, RISING);
}

// ============================================================
// loop()
// ============================================================

void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    updateRearConnectionSafety(now);
    updateVsolTimeout(now);
    handleIncomingVsol(now);

    control_update(now);

    handleSyncVist();
}