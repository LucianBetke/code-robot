// vorne.ino
#include <avr/wdt.h>
#include "CommandScript.h"
#include "src/Hardware.h"
#include "src/hardware_pins.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"
#include "src/RearFrameClient.h"
#include "src/Printer.h"

// ============================================================
// Globale Variablen
// ============================================================

static uint32_t g_startMs = 0;
static bool g_timerStarted = false;

static uint32_t g_nextFrameMs = 0;

// ============================================================
// Globale Objekte
// ============================================================

VehicleController vehicle;
UartLink uart(Serial, true);
ConnectionMonitor conn(uart, 13);
CommandParser parser;

CommandRunner commandRunner(vehicle, uart, parser,
    CommandScript::get, CommandScript::size);

RearFrameClient rearFrameClient;
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

static RearFrameRequest makeRearFrameRequest(uint32_t frameTime, bool resetPi)
{
    RearFrameRequest request = {};

    request.frameTimeMs = frameTime;
    request.resetPi = resetPi;

    request.voLi_s = commandRunner.getWheelSoll(VoLi);
    request.voLi_i = speed[Li].mps();
    request.voLi_pwm = rad[Li].lastPwm();

    request.voRe_s = commandRunner.getWheelSoll(VoRe);
    request.voRe_i = speed[Re].mps();
    request.voRe_pwm = rad[Re].lastPwm();

    request.hiLi_s = commandRunner.getWheelSoll(HiLi);
    request.hiRe_s = commandRunner.getWheelSoll(HiRe);

    return request;
}

static void requestRearFrame(uint32_t now, uint32_t frameTime, bool resetPi)
{
    const RearFrameRequest request = makeRearFrameRequest(frameTime, resetPi);

    rearFrameClient.requestFrame(
        Serial,
        now,
        request
    );

    // VIST wird erst nach VSOL_OK,<frameId> angefordert.
}

static void requestStartFrameForNewCommand(uint32_t now)
{
    if (!uart.isConnected()) return;
    if (rearFrameClient.isBusy()) return;

    // Neuer CMDT-Fahrabschnitt:
    // Kein globaler PI-Hard-Reset mehr.
    // Rad::setSoll() entscheidet lokal:
    // - gleicher Vortrieb: Integrator behalten
    // - Stop: reset() + Bremse
    // - Start/Richtungswechsel: presetOutput()
    //
    // Dadurch bleibt bei Uebergaengen wie 30 -> 20 der PWM-Arbeitspunkt erhalten.
    applyFrontWheelSoll();

    rearFrameClient.clearWaiting();
    rearFrameClient.clearFrame();

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
    if (rearFrameClient.isBusy()) return;
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

static void updateVehicleIst()
{
    vehicle.updateIst(
        speed[Re].mps(),
        speed[Li].mps(),
        rearFrameClient.hiLiIst(),
        rearFrameClient.hiReIst()
    );
}

static void handleIncomingLines(uint32_t now)
{
    if (!uart.availableLine()) return;

    const char* line = uart.getLine();

    // --------------------------------------------------------
    // VSOL_OK auswerten
    // --------------------------------------------------------

    if (rearFrameClient.handleVsolOkLine(line, now))
    {
        hardware_requestVist();
        return;
    }

    // --------------------------------------------------------
    // VIST auswerten
    // --------------------------------------------------------

    if (rearFrameClient.handleVistLine(line))
    {
        updateVehicleIst();

        printer.printCompletedFrame(
            vehicle,
            rearFrameClient.frame(),
            rearFrameClient.hiLiIst(),
            rearFrameClient.hiReIst(),
            rearFrameClient.hiLiPwm(),
            rearFrameClient.hiRePwm()
        );
    }
}

static void updateFrameTimeout(uint32_t now)
{
    if (rearFrameClient.isBusy() &&
        now - rearFrameClient.requestMs() > 2 * VEHICLE_DT_MS)
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
    if (rearFrameClient.isBusy()) return;

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
        rearFrameClient.armStopSequence();
    }
}

static void updateLogRaster()
{
    RearPendingFrame& frame = rearFrameClient.frame();

    if (!commandRunner.isActive())
    {
        g_timerStarted = false;

        if (!rearFrameClient.isBusy())
        {
            frame.hasFrontSnapshot = false;
        }
    }

    if (commandRunner.isActive())
    {
        rearFrameClient.cancelStopSequence();

        // Normalerweise wird das Raster schon im Startframe gesetzt.
        // Das hier bleibt als Fallback, falls ein aktiver Befehl ohne
        // Startframe laufen sollte.
        if (!g_timerStarted) startCommandLogRaster(millis());
    }
}

static void updateVehicleAndFrontControl(uint32_t now)
{
    updateVehicleIst();

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

    const bool ready =
        uart.isConnected() &&
        !commandRunner.isActive() &&
        commandRunner.isFinished();

    rearFrameClient.updateStopSequence(
        Serial,
        now,
        ready,
        VEHICLE_DT_MS
    );
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
    rearFrameClient.begin();

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