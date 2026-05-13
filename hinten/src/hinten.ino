// hinten.ino
#include <avr/wdt.h>
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

// Merkt sich, ob hinten zuletzt einen echten Fahr-Sollwert bekommen hat.
// VSOL,0,0 bedeutet: kein aktiver Fahrbefehl mehr.
static bool g_rearSollActive = false;

volatile bool g_syncFlag = false;

void syncISR()
{
    g_syncFlag = true;
}

static void stopRearWheels()
{
    rad[Li].setSoll(0.0f);
    rad[Re].setSoll(0.0f);
    g_rearSollActive = false;
}

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

void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    // --------------------------------------------------------
    // Verbindung weg:
    // Nicht resetten, sondern Motor-Sollwerte sicher auf 0.
    // --------------------------------------------------------

    if (!uart.isConnected())
    {
        stopRearWheels();
    }

    // --------------------------------------------------------
    // VSOL-Timeout:
    //
    // Alt:
    // Fehlendes VSOL fuehrte zum Watchdog-Reset.
    //
    // Neu:
    // Fehlendes VSOL ist im Stillstand erlaubt.
    // Nur wenn vorher ein echter Fahr-Sollwert aktiv war,
    // wird bei Timeout hinten sicher gestoppt.
    // --------------------------------------------------------

    if (g_rearSollActive && lastVsolMs > 0 && now - lastVsolMs > 2 * VEHICLE_DT_MS)
    {
        stopRearWheels();
    }

    // --------------------------------------------------------
    // Eingehende Sollwerte vom vorderen Nano
    // --------------------------------------------------------

    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        int16_t v2_i;
        int16_t v3_i;

        if (sscanf(line, "VSOL,%hd,%hd", &v2_i, &v3_i) == 2)
        {
            float vSollLi = int100ToFloat(v2_i);
            float vSollRe = int100ToFloat(v3_i);

            rad[Li].setSoll(vSollLi);
            rad[Re].setSoll(vSollRe);

            lastVsolMs = now;

            g_rearSollActive = (v2_i != 0 || v3_i != 0);
        }
    }

    // --------------------------------------------------------
    // Hinterachse lokal regeln
    // --------------------------------------------------------

    control_update(now);

    // --------------------------------------------------------
    // Sync-Puls vom vorderen Nano:
    // Hintere Istwerte und PWM-Werte zuruecksenden.
    // --------------------------------------------------------

    if (g_syncFlag)
    {
        g_syncFlag = false;

        int16_t vIstLi = floatToInt100(speed[Li].mps());
        int16_t vIstRe = floatToInt100(speed[Re].mps());

        int16_t pwm2 = rad[Li].lastPwm();
        int16_t pwm3 = rad[Re].lastPwm();

        char bufVist[32];
        snprintf(bufVist, sizeof(bufVist), "VIST,%d,%d,%d,%d", vIstLi, vIstRe, pwm2, pwm3);

        uart.sendLine(bufVist);
    }
}