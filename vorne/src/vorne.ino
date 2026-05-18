// vorne.ino
#include <avr/wdt.h>
#include "CommandScript.h"
#include "src/CommProtocol.h"
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

    if (g_nextFrameId == 0) g_nextFrameId = 1;

    return id;
}

// VSOL-Format:
//   VSOL,<frameId>,<resetPi>,<hiLiSoll>,<hiReSoll>
//
// resetPi = 1:
//   Hinten soll die PI-Zustaende hart loeschen.
//
// resetPi = 0:
//   Hinten soll keinen globalen PI-Reset ausfuehren.
//   Die lokale Logik in Rad::setSoll() entscheidet dann:
//   Stop -> reset(), Start/Richtungswechsel -> presetOutput(),
//   gleiche Richtung -> Integrator behalten.
static void sendVsolLine(uint16_t frameId, bool resetPi, int16_t v2, int16_t v3)
{
    if (!uart.isConnected()) return;

    Serial.print(F("VSOL,"));
    Serial.print((unsigned int)frameId);
    Serial.print(',');
    Serial.print(resetPi ? 1 : 0);
    Serial.print(',');
    Serial.print((int)v2);
    Serial.print(',');
    Serial.println((int)v3);
}

static void sendRearSoll(uint32_t now, uint16_t frameId, bool resetPi)
{
    int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
    int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));

    sendVsolLine(frameId, resetPi, v2_i, v3_i);

    g_lastVsolSendMs = now;
}

static void sendRearStop(uint32_t now)
{
    uint16_t frameId = nextFrameId();

    // resetPi=false reicht hier.
    // Bei Sollwert 0 fuehrt Rad::setSoll(0) hinten ohnehin einen harten Stop
    // mit Regler-Reset aus.
    sendVsolLine(frameId, false, 0, 0);

    g_lastVsolSendMs = now;
}

static void applyFrontWheelSoll()
{
    rad[Li].setSoll(commandRunner.getWheelSoll(VoLi));
    rad[Re].setSoll(commandRunner.getWheelSoll(VoRe));
}

static void startCommandLogRaster(uint32_t now)
{
    g_startMs = now;
    g_nextFrameMs = now + VEHICLE_DT_MS;
    g_timerStarted = true;
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

static void requestRearFrame(uint32_t now, uint32_t frameTime, bool resetPi)
{
    uint16_t frameId = nextFrameId();

    g_frame.frameId = frameId;
    g_frame.t = frameTime;

    g_frame.voLi_s = commandRunner.getWheelSoll(VoLi);
    g_frame.voLi_i = speed[Li].mps();
    g_frame.voLi_pwm = rad[Li].lastPwm();

    g_frame.voRe_s = commandRunner.getWheelSoll(VoRe);
    g_frame.voRe_i = speed[Re].mps();
    g_frame.voRe_pwm = rad[Re].lastPwm();

    g_frame.hiLi_s = commandRunner.getWheelSoll(HiLi);
    g_frame.hiRe_s = commandRunner.getWheelSoll(HiRe);

    g_frame.hasFrontSnapshot = true;

    g_waitingVsolOk = true;
    g_waitingVist = false;
    g_requestMs = now;

    sendRearSoll(now, frameId, resetPi);

    // VIST wird erst nach VSOL_OK,<frameId> angefordert.
}

static void requestStartFrameForNewCommand(uint32_t now)
{
    if (!uart.isConnected()) return;
    if (g_waitingVsolOk) return;
    if (g_waitingVist) return;

    // Neuer CMDT-Fahrabschnitt:
    // Kein globaler PI-Hard-Reset mehr.
    // Rad::setSoll() entscheidet lokal:
    // - gleicher Vortrieb: Integrator behalten
    // - Stop: reset() + Bremse
    // - Start/Richtungswechsel: presetOutput()
    //
    // Dadurch bleibt bei Uebergaengen wie 30 -> 20 der PWM-Arbeitspunkt erhalten.
    applyFrontWheelSoll();

    g_waitingVsolOk = false;
    g_waitingVist = false;
    g_frame.hasFrontSnapshot = false;

    startCommandLogRaster(now);

    // Echter Startframe des neuen Befehls:
    // t = 0, neue Sollwerte, aktuelle Istwerte, aktuelle PWM-Zustaende.
    //
    // resetPi=false:
    // Hinten behaelt bei gleicher Richtung den Integrator.
    // Bei Stop oder Richtungswechsel reagiert Rad::setSoll() selbst.
    requestRearFrame(now, 0, false);
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

        // Normales Messframe innerhalb desselben CMDT:
        // resetPi=false, damit hinten den Integrator nicht staendig loescht.
        requestRearFrame(now, frameTime, false);

        g_nextFrameMs += VEHICLE_DT_MS;
    }
}

// ============================================================
// loop()-Teilfunktionen
// ============================================================

static void updateConnectionSafety(uint32_t now)
{
    (void)now;

    static bool prevConnected = false;
    bool nowConnected = uart.isConnected();

    if (prevConnected && !nowConnected) resetByWatchdog();

    prevConnected = nowConnected;
}

static void handleIncomingLines(uint32_t now)
{
    if (!uart.availableLine()) return;

    const char* line = uart.getLine();

    // --------------------------------------------------------
    // VSOL_OK auswerten
    // --------------------------------------------------------

    VsolOkMessage vsolOk = {};

    if (parseVsolOkLine(line, vsolOk))
    {
        if (g_waitingVsolOk &&
            g_frame.hasFrontSnapshot &&
            vsolOk.frameId == g_frame.frameId)
        {
            g_waitingVsolOk = false;
            g_waitingVist = true;
            g_requestMs = now;

            hardware_requestVist();
        }
    }

    // --------------------------------------------------------
    // VIST auswerten
    // --------------------------------------------------------

    VistMessage vist = {};

    if (parseVistLine(line, vist))
    {
        if (g_waitingVist &&
            g_frame.hasFrontSnapshot &&
            vist.frameId == g_frame.frameId)
        {
            g_v2_ist = int100ToFloat(vist.hiLiIst);
            g_v3_ist = int100ToFloat(vist.hiReIst);
            g_pwm2 = vist.hiLiPwm;
            g_pwm3 = vist.hiRePwm;

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

static void updateFrameTimeout(uint32_t now)
{
    if ((g_waitingVsolOk || g_waitingVist) &&
        now - g_requestMs > 2 * VEHICLE_DT_MS)
    {
        resetByWatchdog();
    }
}

static void updateCommandRunner(uint32_t now)
{
    // Wichtig:
    // Solange noch VSOL_OK oder VIST offen ist, darf der CommandRunner
    // NICHT zum naechsten Befehl springen.
    //
    // Sonst geht der letzte Frame eines Befehls verloren.

    if (!uart.isConnected()) return;
    if (g_waitingVsolOk) return;
    if (g_waitingVist) return;

    bool wasActive = commandRunner.isActive();

    commandRunner.update(now);

    bool isActive = commandRunner.isActive();

    // Neuer CMDT-Befehl wurde gestartet.
    // Jetzt wird sofort ein echter Messframe mit t = 0 angefordert.
    // Dieses Frame sendet gleichzeitig die neuen hinteren Sollwerte.
    // resetPi bleibt false, damit bei gleicher Richtung der Integrator
    // nicht hart geloescht wird.
    if (isActive && commandRunner.consumeStartFramePending())
    {
        requestStartFrameForNewCommand(now);
    }

    // Stop-Sequenz nur dann vormerken, wenn das ganze Script fertig ist.
    // Nicht zwischen zwei direkt aufeinanderfolgenden CMDT-Befehlen.
    if (wasActive && !isActive && commandRunner.isFinished())
    {
        g_stopSequenceArmed = true;
        g_stopSendCount = 0;
    }
}

static void updateLogRaster()
{
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

        // Normalerweise wird das Raster schon im Startframe gesetzt.
        // Das hier bleibt als Fallback, falls ein aktiver Befehl ohne
        // Startframe laufen sollte.
        if (!g_timerStarted) startCommandLogRaster(millis());
    }
}

static void updateVehicleAndFrontControl(uint32_t now)
{
    vehicle.updateIst(
        speed[Re].mps(),
        speed[Li].mps(),
        g_v2_ist,
        g_v3_ist
    );

    vehicle.update(now);

    applyFrontWheelSoll();

    control_update(now);
}

static void updateRearStopSequence(uint32_t now)
{
    // Stop-Sequenz fuer hinten:
    //
    // Nur wenn das ganze Script fertig ist.
    // Nicht zwischen zwei Befehlen.
    // Nicht senden, solange noch VSOL_OK oder VIST offen ist.

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

    updateConnectionSafety(now);
    handleIncomingLines(now);
    updateFrameTimeout(now);

    // Vor commandRunner.update() pruefen, ob ein Frame faellig ist.
    //
    // Dadurch wird bei 2000 ms bzw. 3000 ms noch der letzte Frame
    // des aktuellen Befehls angefordert, bevor der CommandRunner
    // den Befehl beendet.
    if (uart.isConnected()) tryRequestFrame(now);

    updateCommandRunner(now);
    updateLogRaster();
    updateVehicleAndFrontControl(now);
    updateRearStopSequence(now);

    // Aktiver Messframe nach commandRunner.update():
    //
    // Der Startframe eines neuen Befehls wird jetzt bewusst bei
    // t = 0 ms angefordert. Danach laufen die normalen Messframes
    // weiter bei 100, 200, 300 ... ms.
    // Die Endmessung bei 2000/3000 ms wird oben vor
    // commandRunner.update() abgefangen.
    if (uart.isConnected()) tryRequestFrame(now);
}