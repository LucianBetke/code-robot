// vorne.ino
#include <avr/wdt.h>
#include "CommandScript.h"
#include "src/Hardware.h"
#include "src/hardware_pins.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"
#include "src/Printer.h"

// ============================================================
// Globale Variablen
// ============================================================

static float g_v2_ist = 0.0f;
static float g_v3_ist = 0.0f;
static int16_t g_pwm2 = 0;
static int16_t g_pwm3 = 0;

static uint32_t g_startMs = 0;
static bool g_timerStarted = false;

static uint32_t g_nextFrameMs = 0;
static uint32_t g_requestMs = 0;
static uint32_t g_lastVsolSendMs = 0;

static bool g_waitingRear = false;

// ============================================================
// Gemeinsamer Log-Datensatz fuer einen Zeitpunkt
// ============================================================

struct PendingFrame
{
    uint32_t t;

    float voLi_s;
    float voLi_i;
    int16_t voLi_pwm;

    float voRe_s;
    float voRe_i;
    int16_t voRe_pwm;

    float hiLi_s;
    float hiRe_s;

    bool valid;
};

static PendingFrame g_frame =
{
    0,

    0.0f,
    0.0f,
    0,

    0.0f,
    0.0f,
    0,

    0.0f,
    0.0f,

    false
};

// ============================================================
// Globale Objekte
// ============================================================

VehicleController vehicle;
UartLink uart(Serial, true);
ConnectionMonitor conn(uart, 13);
CommandParser parser;

CommandRunner commandRunner(vehicle, uart, parser,
    CommandScript::get, CommandScript::size);

Printer printer;

// ============================================================
// Hilfsfunktionen
// ============================================================

static void resetByWatchdog()
{
    wdt_enable(WDTO_15MS);
    while (1) {}
}

static bool timeReached(uint32_t now, uint32_t target)
{
    return (int32_t)(now - target) >= 0;
}

static void printFloatCell(float v)
{
    Serial.print('\t');
    Serial.print(v, 2);
}

static void printIntCell(int16_t v)
{
    Serial.print('\t');
    Serial.print(v);
}

static void sendRearSoll(uint32_t now)
{
    int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
    int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));

    char bufVsoll[32];
    snprintf(bufVsoll, sizeof(bufVsoll), "VSOL,%d,%d", v2_i, v3_i);

    uart.sendLine(bufVsoll);
    g_lastVsolSendMs = now;
}

static void printCompletedFrame(float hiLi_i, float hiRe_i, int16_t hiLi_pwm, int16_t hiRe_pwm)
{
#ifdef PRINTER_MODE_RAEDER
    Serial.print('#');
    Serial.print(g_frame.t);

    printFloatCell(g_frame.voLi_s);
    printFloatCell(g_frame.voLi_i);
    printIntCell(g_frame.voLi_pwm);

    printFloatCell(g_frame.voRe_s);
    printFloatCell(g_frame.voRe_i);
    printIntCell(g_frame.voRe_pwm);

    printFloatCell(g_frame.hiLi_s);
    printFloatCell(hiLi_i);
    printIntCell(hiLi_pwm);

    printFloatCell(g_frame.hiRe_s);
    printFloatCell(hiRe_i);
    printIntCell(hiRe_pwm);

    Serial.println();
#endif

#ifdef PRINTER_MODE_CHASSIS
    Serial.print('#');
    Serial.print(g_frame.t);

    printFloatCell(g_frame.voLi_s);
    printFloatCell(g_frame.voLi_i);

    printFloatCell(g_frame.voRe_s);
    printFloatCell(g_frame.voRe_i);

    printFloatCell(g_frame.hiLi_s);
    printFloatCell(hiLi_i);

    printFloatCell(g_frame.hiRe_s);
    printFloatCell(hiRe_i);

    Serial.println();
#endif
}

static void requestRearFrame(uint32_t now, uint32_t frameTime)
{
    g_frame.t = frameTime;

    g_frame.voLi_s = commandRunner.getWheelSoll(VoLi);
    g_frame.voLi_i = speed[Li].mps();
    g_frame.voLi_pwm = rad[VoLi].lastPwm();

    g_frame.voRe_s = commandRunner.getWheelSoll(VoRe);
    g_frame.voRe_i = speed[Re].mps();
    g_frame.voRe_pwm = rad[VoRe].lastPwm();

    g_frame.hiLi_s = commandRunner.getWheelSoll(HiLi);
    g_frame.hiRe_s = commandRunner.getWheelSoll(HiRe);

    g_frame.valid = true;
    g_waitingRear = true;
    g_requestMs = now;

    sendRearSoll(now);
    hardware_requestVist();
}

// ============================================================
// setup()
// ============================================================

void setup()
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsFront::PINS);
    control_begin(ConfigFront::CONFIG);
    speed_reset_all();

    vehicle.begin(
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f
    );

    commandRunner.begin();

    uart.begin();
    conn.begin(true);

    printer.printHeader(vehicle, ConfigFront::CONFIG);
}

// ============================================================
// loop()
// ============================================================

void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    // --------------------------------------------------------
    // DISCONNECT erkennen -> Reset
    // --------------------------------------------------------
    static bool prevConnected = false;
    bool nowConnected = uart.isConnected();

    if (prevConnected && !nowConnected)
    {
        resetByWatchdog();
    }

    prevConnected = nowConnected;

    // --------------------------------------------------------
    // Eingehende VIST-Daten auswerten
    // --------------------------------------------------------
    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        int16_t v2_i;
        int16_t v3_i;
        int16_t pwm2;
        int16_t pwm3;

        if (sscanf(line, "VIST,%hd,%hd,%hd,%hd", &v2_i, &v3_i, &pwm2, &pwm3) == 4)
        {
            g_v2_ist = int100ToFloat(v2_i);
            g_v3_ist = int100ToFloat(v3_i);
            g_pwm2 = pwm2;
            g_pwm3 = pwm3;

            if (g_waitingRear && g_frame.valid)
            {
                printCompletedFrame(g_v2_ist, g_v3_ist, g_pwm2, g_pwm3);

                g_waitingRear = false;
                g_frame.valid = false;
            }
        }
    }

    // --------------------------------------------------------
    // Timeout: VIST-Antwort fehlt
    // --------------------------------------------------------
    if (g_waitingRear && now - g_requestMs > 2 * VEHICLE_DT_MS)
    {
        resetByWatchdog();
    }

    // --------------------------------------------------------
    // CommandRunner nur bei Verbindung aktualisieren
    // --------------------------------------------------------
    if (uart.isConnected())
    {
        commandRunner.update(now);
    }

    // --------------------------------------------------------
    // Start/Stop des Log-Zeitrasters
    // --------------------------------------------------------
    if (!commandRunner.isActive())
    {
        g_timerStarted = false;
        g_waitingRear = false;
        g_frame.valid = false;
    }

    if (commandRunner.isActive())
    {
        if (!g_timerStarted)
        {
            g_startMs = now;
            g_nextFrameMs = now;
            g_timerStarted = true;

            g_waitingRear = false;
            g_frame.valid = false;
        }
    }

    // --------------------------------------------------------
    // Vehicle-Istwerte aktualisieren
    // --------------------------------------------------------
    vehicle.updateIst(
        speed[Re].mps(),
        speed[Li].mps(),
        g_v2_ist,
        g_v3_ist
    );

    vehicle.update(now);

    // --------------------------------------------------------
    // Vorderachse lokal regeln
    // --------------------------------------------------------
    rad[VoLi].setSoll(commandRunner.getWheelSoll(VoLi));
    rad[VoRe].setSoll(commandRunner.getWheelSoll(VoRe));

    control_update(now);

    // --------------------------------------------------------
    // Wenn kein aktives Kommando laeuft:
    // Rear trotzdem mit VSOL versorgen, damit hinten kein
    // VSOL-Timeout ausloest.
    // --------------------------------------------------------
    if (uart.isConnected() && !commandRunner.isActive())
    {
        if (g_lastVsolSendMs == 0)
        {
            g_lastVsolSendMs = now;
        }

        if (now - g_lastVsolSendMs >= VEHICLE_DT_MS)
        {
            sendRearSoll(now);
        }
    }

    // --------------------------------------------------------
    // Aktiver Messframe:
    // Frontwerte speichern, VSOL senden, Sync-Puls ausloesen.
    // Gedruckt wird NICHT hier, sondern erst bei VIST-Empfang.
    // --------------------------------------------------------
    if (commandRunner.isActive() && !g_waitingRear)
    {
        if (timeReached(now, g_nextFrameMs))
        {
            uint32_t frameTime = g_nextFrameMs - g_startMs;

            requestRearFrame(now, frameTime);

            g_nextFrameMs += VEHICLE_DT_MS;
        }
    }
}