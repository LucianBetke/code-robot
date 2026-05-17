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
// Fester Integer-Parser fuer UART-Telegramme
//
// Neues Format:
//   VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
//
// resetPi:
//   0 = normaler Sollwert innerhalb desselben CMDT
//   1 = neuer CMDT-Fahrabschnitt, PI-Zustaende hinten loeschen
// ============================================================

static void skipSpaces(const char*& p)
{
    while (*p == ' ' || *p == '\t') p++;
}

static bool expectChar(const char*& p, char expected)
{
    if (*p != expected) return false;

    p++;
    return true;
}

static bool expectText(const char*& p, const char* text)
{
    while (*text != '\0')
    {
        if (*p != *text) return false;

        p++;
        text++;
    }

    return true;
}

static bool parseUInt16(const char*& p, uint16_t& out)
{
    skipSpaces(p);

    if (*p < '0' || *p > '9') return false;

    uint32_t value = 0;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10UL + (uint32_t)(*p - '0');

        if (value > 65535UL) return false;

        p++;
    }

    out = (uint16_t)value;
    return true;
}

static bool parseInt16(const char*& p, int16_t& out)
{
    skipSpaces(p);

    bool negative = false;

    if (*p == '-') { negative = true; p++; }

    if (*p < '0' || *p > '9') return false;

    int32_t value = 0;
    const int32_t limit = negative ? 32768L : 32767L;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10L + (int32_t)(*p - '0');

        if (value > limit) return false;

        p++;
    }

    if (negative) value = -value;

    out = (int16_t)value;
    return true;
}

static bool parseVsolLine(
    const char* line,
    uint16_t& frameId,
    bool& resetPi,
    int16_t& v2,
    int16_t& v3)
{
    if (!line) return false;

    const char* p = line;

    uint16_t frameIdTmp = 0;
    uint16_t resetTmp = 0;
    int16_t v2Tmp = 0;
    int16_t v3Tmp = 0;

    if (!expectText(p, "VSOL,")) return false;

    if (!parseUInt16(p, frameIdTmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseUInt16(p, resetTmp)) return false;
    if (resetTmp > 1) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, v2Tmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, v3Tmp)) return false;

    skipSpaces(p);

    if (*p != '\0') return false;

    frameId = frameIdTmp;
    resetPi = (resetTmp != 0);
    v2 = v2Tmp;
    v3 = v3Tmp;

    return true;
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

static void sendVsolOk(uint16_t frameId)
{
    if (!uart.isConnected()) return;

    Serial.print(F("VSOL_OK,"));
    Serial.println((unsigned int)frameId);
}

static void sendVist(uint16_t frameId, int16_t vIstLi, int16_t vIstRe, int16_t pwm2, int16_t pwm3)
{
    if (!uart.isConnected()) return;

    Serial.print(F("VIST,"));
    Serial.print((unsigned int)frameId);
    Serial.print(',');
    Serial.print((int)vIstLi);
    Serial.print(',');
    Serial.print((int)vIstRe);
    Serial.print(',');
    Serial.print((int)pwm2);
    Serial.print(',');
    Serial.println((int)pwm3);
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

    if (!uart.isConnected()) stopRearWheels();

    // --------------------------------------------------------
    // VSOL-Timeout:
    //
    // Fehlendes VSOL ist im Stillstand erlaubt.
    // Nur wenn vorher ein echter Fahr-Sollwert aktiv war,
    // wird bei Timeout hinten sicher gestoppt.
    // --------------------------------------------------------

    if (g_rearSollActive && lastVsolMs > 0 && now - lastVsolMs > 2 * VEHICLE_DT_MS)
        stopRearWheels();

    // --------------------------------------------------------
    // Eingehende Sollwerte vom vorderen Nano
    //
    // Format:
    // VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
    //
    // Antwort:
    // VSOL_OK,<frameId>
    // --------------------------------------------------------

    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        uint16_t frameIdRx = 0;
        bool resetPi = false;
        int16_t v2_i = 0;
        int16_t v3_i = 0;

        if (parseVsolLine(line, frameIdRx, resetPi, v2_i, v3_i))
        {
            g_lastVsolFrameId = frameIdRx;

            float vSollLi = int100ToFloat(v2_i);
            float vSollRe = int100ToFloat(v3_i);

            // Neuer CMDT-Fahrabschnitt:
            // PI-Zustaende hinten loeschen, aber keinen Stop erzeugen.
            // Danach werden die neuen Sollwerte gesetzt.
            if (resetPi) control_resetPiStates();

            rad[Li].setSoll(vSollLi);
            rad[Re].setSoll(vSollRe);

            lastVsolMs = now;
            g_rearSollActive = (v2_i != 0 || v3_i != 0);

            sendVsolOk(g_lastVsolFrameId);
        }
    }

    // --------------------------------------------------------
    // Hinterachse lokal regeln
    // --------------------------------------------------------

    control_update(now);

    // --------------------------------------------------------
    // Sync-Puls vom vorderen Nano:
    // Hintere Istwerte und PWM-Werte zuruecksenden.
    //
    // Format:
    // VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>
    // --------------------------------------------------------

    if (g_syncFlag)
    {
        g_syncFlag = false;

        int16_t vIstLi = floatToInt100(speed[Li].mps());
        int16_t vIstRe = floatToInt100(speed[Re].mps());

        int16_t pwm2 = rad[Li].lastPwm();
        int16_t pwm3 = rad[Re].lastPwm();

        sendVist(g_lastVsolFrameId, vIstLi, vIstRe, pwm2, pwm3);
    }
}