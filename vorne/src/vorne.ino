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

// Front wartet zuerst auf VSOL_OK und danach auf VIST.
static bool g_waitingVsolOk = false;
static bool g_waitingVist = false;

// Frame-ID 0 bleibt reserviert fuer "ungueltig / keine ID".
static uint16_t g_nextFrameId = 1;

// Nach Befehlsende werden drei Stop-Telegramme gesendet.
// Wichtig: erst nach dem letzten fertigen Messframe.
// Und nur, wenn das ganze Script fertig ist.
static const uint8_t STOP_SEND_MAX = 3;
static uint8_t g_stopSendCount = 0;
static bool g_stopSequenceArmed = false;

// ============================================================
// Gemeinsamer Log-Datensatz fuer einen Zeitpunkt
// ============================================================

struct PendingFrame
{
    uint16_t frameId;
    uint32_t t;

    float voLi_s; float voLi_i; int16_t voLi_pwm;
    float voRe_s; float voRe_i; int16_t voRe_pwm;
    float hiLi_s; float hiRe_s;

    bool hasFrontSnapshot;
};

static PendingFrame g_frame = {};

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
// Fester Integer-Parser fuer UART-Telegramme
// Ersetzt sscanf() fuer:
//   VSOL_OK,<frameId>
//   VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>
// ============================================================

static void skipSpaces(const char*& p)
{
    while (*p == ' ' || *p == '\t')
    {
        p++;
    }
}

static bool expectChar(const char*& p, char expected)
{
    if (*p != expected)
    {
        return false;
    }

    p++;
    return true;
}

static bool expectText(const char*& p, const char* text)
{
    while (*text != '\0')
    {
        if (*p != *text)
        {
            return false;
        }

        p++;
        text++;
    }

    return true;
}

static bool parseUInt16(const char*& p, uint16_t& out)
{
    skipSpaces(p);

    if (*p < '0' || *p > '9')
    {
        return false;
    }

    uint32_t value = 0;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10UL + (uint32_t)(*p - '0');

        if (value > 65535UL)
        {
            return false;
        }

        p++;
    }

    out = (uint16_t)value;
    return true;
}

static bool parseInt16(const char*& p, int16_t& out)
{
    skipSpaces(p);

    bool negative = false;

    if (*p == '-')
    {
        negative = true;
        p++;
    }

    if (*p < '0' || *p > '9')
    {
        return false;
    }

    int32_t value = 0;
    const int32_t limit = negative ? 32768L : 32767L;

    while (*p >= '0' && *p <= '9')
    {
        value = value * 10L + (int32_t)(*p - '0');

        if (value > limit)
        {
            return false;
        }

        p++;
    }

    if (negative)
    {
        value = -value;
    }

    out = (int16_t)value;
    return true;
}

static bool parseVsolOkLine(const char* line, uint16_t& frameId)
{
    if (!line)
    {
        return false;
    }

    const char* p = line;
    uint16_t frameIdTmp = 0;

    if (!expectText(p, "VSOL_OK,")) return false;
    if (!parseUInt16(p, frameIdTmp)) return false;

    skipSpaces(p);

    if (*p != '\0')
    {
        return false;
    }

    frameId = frameIdTmp;

    return true;
}

static bool parseVistLine(
    const char* line,
    uint16_t& frameId,
    int16_t& v2,
    int16_t& v3,
    int16_t& pwm2,
    int16_t& pwm3)
{
    if (!line)
    {
        return false;
    }

    const char* p = line;

    uint16_t frameIdTmp = 0;
    int16_t v2Tmp = 0;
    int16_t v3Tmp = 0;
    int16_t pwm2Tmp = 0;
    int16_t pwm3Tmp = 0;

    if (!expectText(p, "VIST,")) return false;

    if (!parseUInt16(p, frameIdTmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, v2Tmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, v3Tmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, pwm2Tmp)) return false;
    skipSpaces(p);
    if (!expectChar(p, ',')) return false;

    if (!parseInt16(p, pwm3Tmp)) return false;

    skipSpaces(p);

    if (*p != '\0')
    {
        return false;
    }

    frameId = frameIdTmp;
    v2 = v2Tmp;
    v3 = v3Tmp;
    pwm2 = pwm2Tmp;
    pwm3 = pwm3Tmp;

    return true;
}

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

static uint16_t nextFrameId()
{
    uint16_t id = g_nextFrameId++;

    if (g_nextFrameId == 0)
    {
        g_nextFrameId = 1;
    }

    return id;
}

static void sendRearSoll(uint32_t now, uint16_t frameId)
{
    int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
    int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));

    char bufVsoll[36];
    snprintf(bufVsoll, sizeof(bufVsoll), "VSOL,%u,%d,%d",
        (unsigned int)frameId,
        v2_i,
        v3_i
    );

    uart.sendLine(bufVsoll);
    g_lastVsolSendMs = now;
}

static void sendRearStop(uint32_t now)
{
    uint16_t frameId = nextFrameId();

    char bufVsoll[24];
    snprintf(bufVsoll, sizeof(bufVsoll), "VSOL,%u,0,0",
        (unsigned int)frameId
    );

    uart.sendLine(bufVsoll);
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
    uint16_t frameId = nextFrameId();

    g_frame.frameId = frameId;
    g_frame.t = frameTime;

    g_frame.voLi_s = commandRunner.getWheelSoll(VoLi);
    g_frame.voLi_i = speed[Li].mps();
    g_frame.voLi_pwm = rad[VoLi].lastPwm();

    g_frame.voRe_s = commandRunner.getWheelSoll(VoRe);
    g_frame.voRe_i = speed[Re].mps();
    g_frame.voRe_pwm = rad[VoRe].lastPwm();

    g_frame.hiLi_s = commandRunner.getWheelSoll(HiLi);
    g_frame.hiRe_s = commandRunner.getWheelSoll(HiRe);

    g_frame.hasFrontSnapshot = true;

    g_waitingVsolOk = true;
    g_waitingVist = false;
    g_requestMs = now;

    sendRearSoll(now, frameId);

    // VIST wird erst nach VSOL_OK,<frameId> angefordert.
}

static void tryRequestFrame(uint32_t now)
{
    if (!commandRunner.isActive()) return;
    if (g_waitingVsolOk) return;
    if (g_waitingVist) return;
    if (!g_timerStarted) return;

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
    // Eingehende Daten auswerten:
    //
    // 1. VSOL_OK,<frameId>
    // 2. VIST,<frameId>,<hiLiIst>,<hiReIst>,<hiLiPwm>,<hiRePwm>
    // --------------------------------------------------------

    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        // ----------------------------------------------------
        // VSOL_OK auswerten
        // ----------------------------------------------------

        uint16_t frameIdOk = 0;

        if (parseVsolOkLine(line, frameIdOk))
        {
            if (g_waitingVsolOk &&
                g_frame.hasFrontSnapshot &&
                frameIdOk == g_frame.frameId)
            {
                g_waitingVsolOk = false;
                g_waitingVist = true;
                g_requestMs = now;

                hardware_requestVist();
            }
        }

        // ----------------------------------------------------
        // VIST auswerten
        // ----------------------------------------------------

        uint16_t frameIdVist = 0;
        int16_t v2_i = 0;
        int16_t v3_i = 0;
        int16_t pwm2 = 0;
        int16_t pwm3 = 0;

        if (parseVistLine(line, frameIdVist, v2_i, v3_i, pwm2, pwm3))
        {
            if (g_waitingVist &&
                g_frame.hasFrontSnapshot &&
                frameIdVist == g_frame.frameId)
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

                printCompletedFrame(g_v2_ist, g_v3_ist, g_pwm2, g_pwm3);

                g_waitingVist = false;
                g_frame.hasFrontSnapshot = false;
            }
        }
    }

    // --------------------------------------------------------
    // Timeout:
    //
    // Front wartet entweder auf VSOL_OK oder auf VIST.
    // Wenn beides zu lange ausbleibt, ist die Kopplung gestoert.
    // --------------------------------------------------------

    if ((g_waitingVsolOk || g_waitingVist) && now - g_requestMs > 2 * VEHICLE_DT_MS)
    {
        resetByWatchdog();
    }

    // --------------------------------------------------------
    // Vor commandRunner.update() pruefen, ob ein Frame faellig ist.
    //
    // Dadurch wird bei 2000 ms bzw. 3000 ms noch der letzte Frame
    // des aktuellen Befehls angefordert, bevor der CommandRunner
    // den Befehl beendet.
    // --------------------------------------------------------

    if (uart.isConnected())
    {
        tryRequestFrame(now);
    }

    // --------------------------------------------------------
    // CommandRunner aktualisieren
    //
    // Wichtig:
    // Solange noch VSOL_OK oder VIST offen ist, darf der CommandRunner
    // NICHT zum naechsten Befehl springen.
    //
    // Sonst geht der letzte Frame eines Befehls verloren.
    // --------------------------------------------------------

    if (uart.isConnected() && !g_waitingVsolOk && !g_waitingVist)
    {
        bool wasActive = commandRunner.isActive();

        commandRunner.update(now);

        bool isActive = commandRunner.isActive();

        // Stop-Sequenz nur dann vormerken, wenn das ganze Script fertig ist.
        // Nicht zwischen zwei direkt aufeinanderfolgenden CMDT-Befehlen.
        if (wasActive && !isActive && commandRunner.isFinished())
        {
            g_stopSequenceArmed = true;
            g_stopSendCount = 0;
        }
    }

    // --------------------------------------------------------
    // Start/Stop des Log-Zeitrasters
    // --------------------------------------------------------

    if (!commandRunner.isActive())
    {
        g_timerStarted = false;

        if (!g_waitingVsolOk && !g_waitingVist)
        {
            g_frame.hasFrontSnapshot = false;
        }
    }

    if (commandRunner.isActive())
    {
        g_stopSequenceArmed = false;
        g_stopSendCount = 0;

        if (!g_timerStarted)
        {
            g_startMs = now;
            g_nextFrameMs = now;
            g_timerStarted = true;

            g_waitingVsolOk = false;
            g_waitingVist = false;
            g_frame.hasFrontSnapshot = false;
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
    // Stop-Sequenz fuer hinten:
    //
    // Nur wenn das ganze Script fertig ist.
    // Nicht zwischen zwei Befehlen.
    // Nicht senden, solange noch VSOL_OK oder VIST offen ist.
    // --------------------------------------------------------

    if (uart.isConnected() &&
        !commandRunner.isActive() &&
        commandRunner.isFinished() &&
        !g_waitingVsolOk &&
        !g_waitingVist &&
        g_stopSequenceArmed)
    {
        if (g_stopSendCount < STOP_SEND_MAX)
        {
            if (g_stopSendCount == 0 || now - g_lastVsolSendMs >= VEHICLE_DT_MS)
            {
                sendRearStop(now);
                g_stopSendCount++;
            }
        }

        if (g_stopSendCount >= STOP_SEND_MAX)
        {
            g_stopSequenceArmed = false;
        }
    }

    // --------------------------------------------------------
    // Aktiver Messframe nach commandRunner.update():
    // wichtig fuer den ersten Frame bei 0 ms.
    //
    // Die Endmessung bei 2000/3000 ms wird oben vor
    // commandRunner.update() abgefangen.
    // --------------------------------------------------------

    if (uart.isConnected())
    {
        tryRequestFrame(now);
    }
}