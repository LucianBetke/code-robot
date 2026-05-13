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

// Nach Befehlsende werden nur noch wenige Stop-Telegramme gesendet.
static const uint8_t STOP_SEND_MAX = 2;
static uint8_t g_stopSendCount = 0;

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

static void sendRearSoll(uint32_t now)
{
    int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
    int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));

    char bufVsoll[32];
    snprintf(bufVsoll, sizeof(bufVsoll), "VSOL,%d,%d", v2_i, v3_i);

    uart.sendLine(bufVsoll);
    g_lastVsolSendMs = now;
}

static void sendRearStop(uint32_t now)
{
    uart.sendLine("VSOL,0,0");
    g_lastVsolSendMs = now;
}

static void printCompletedFrame(float hiLi_i, float hiRe_i, int16_t hiLi_pwm, int16_t hiRe_pwm)
{
#ifdef PRINTER_MODE_CHASSIS
    printer.printFrame(
        vehicle,
        g_frame.t,
        g_frame.voLi_i,
        g_frame.voRe_i,
        hiLi_i,
        hiRe_i
    );
#endif

#ifdef PRINTER_MODE_RAEDER
    printer.printFrame(
        g_frame.t,

        g_frame.voLi_s,
        g_frame.voLi_i,
        g_frame.voLi_pwm,

        g_frame.voRe_s,
        g_frame.voRe_i,
        g_frame.voRe_pwm,

        g_frame.hiLi_s,
        hiLi_i,
        hiLi_pwm,

        g_frame.hiRe_s,
        hiRe_i,
        hiRe_pwm
    );
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

static void tryRequestFrame(uint32_t now)
{
    if (!commandRunner.isActive())
    {
        return;
    }

    if (g_waitingRear)
    {
        return;
    }

    if (!g_timerStarted)
    {
        return;
    }

    if (timeReached(now, g_nextFrameMs))
    {
        uint32_t frameTime = g_nextFrameMs - g_startMs;

        requestRearFrame(now, frameTime);

        g_nextFrameMs += VEHICLE_DT_MS;
    }
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

            vehicle.updateIst(
                speed[Re].mps(),
                speed[Li].mps(),
                g_v2_ist,
                g_v3_ist
            );

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
    // Vor commandRunner.update() pruefen, ob ein Frame faellig ist.
    // Dadurch wird bei 2000 ms noch der letzte Frame des alten Befehls
    // angefordert, bevor der CommandRunner den Befehl beendet.
    // --------------------------------------------------------

    if (uart.isConnected())
    {
        tryRequestFrame(now);
    }

    // --------------------------------------------------------
    // CommandRunner aktualisieren
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

        if (!g_waitingRear)
        {
            g_frame.valid = false;
        }
    }

    if (commandRunner.isActive())
    {
        g_stopSendCount = 0;

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
    // Kein aktives Kommando:
    //
    // Hinten bekommt noch 3 Stop-Telegramme im 100-ms-Takt.
    // Danach wird nicht mehr endlos VSOL,0,0 gesendet.
    //
    // Nicht senden, solange noch ein Messframe auf VIST wartet.
    // --------------------------------------------------------

    if (uart.isConnected() && !commandRunner.isActive() && !g_waitingRear)
    {
        if (g_lastVsolSendMs == 0)
        {
            g_lastVsolSendMs = now;
        }

        if (g_stopSendCount < STOP_SEND_MAX)
        {
            if (now - g_lastVsolSendMs >= VEHICLE_DT_MS)
            {
                sendRearStop(now);
                g_stopSendCount++;
            }
        }
    }

    // --------------------------------------------------------
    // Aktiver Messframe nach commandRunner.update():
    // wichtig fuer den ersten Frame bei 0 ms.
    // Die Endmessung bei 2000 ms wird oben vor commandRunner.update()
    // abgefangen.
    // --------------------------------------------------------

    if (uart.isConnected())
    {
        tryRequestFrame(now);
    }
}