// RearApp.cpp
#include "RearApp.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "src/BatteryGuard.h"
#include "src/CommProtocol.h"
#include "src/Hardware.h"
#include "src/HardwarePins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"
#include "src/StatusLed.h"

static const uint16_t
ULTRASONIC_SEND_INTERVAL_MS = 100;

static const uint16_t
SYNC_DIAG_INTERVAL_MS = 1000;

// Status-LED an D11/D12:
//   Dauerlicht - Startpruefung bestanden, Motoren freigegeben
//   Blinken    - Akku zu leer, Motoren gesperrt
//
// Gleichmaessiges Blinken, damit es sich vom Dauerlicht auf
// den ersten Blick unterscheidet. Nach jeweils vier Blinks
// geht eine Meldung ueber die serielle Schnittstelle raus,
// also etwa alle zwei Sekunden.
static const uint16_t
LOW_BATTERY_BLINK_ON_MS = 250;

static const uint16_t
LOW_BATTERY_BLINK_OFF_MS = 250;

static const uint8_t
LOW_BATTERY_BLINKS_PER_MESSAGE = 4;

// Der ConnectionMonitor des vorderen Nano laeuft ohne LED:
// dort ist D13 vom Funk belegt (SCK).

RearApp::RearApp()
    : uart(Serial, false),
    conn(uart, PinsRear::CONNECTION_LED_PIN),
    ultrasonic(),
    lastVsolMs(0),
    lastVsolFrameId(0),
    lastUsSendMs(0),
    rearSollActive(false),
    syncFlag(false),
    syncPulseCount(0),
    lastSyncDiagMs(0)
{}

void RearApp::begin(
    void (*syncCallback)())
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(
        PinsRear::PINS
    );

    // Ab hier ist der STBY-Pin Ausgang und liegt LOW, beide
    // Achsen sind also stromlos. Die Akkupruefung laeuft
    // bewusst vor conn.begin(): dort wird blockierend auf den
    // vorderen Nano gewartet, und bei leerem Akku soll die
    // Meldung sofort kommen und nicht erst nach dem Handshake.
    checkBatteryOrHalt();

    radControl_begin(
        ConfigRear::CONFIG
    );

    wheelMeasurement_reset_all();

    uart.begin();

    // Im Setup blockierend auf die Verbindung warten.
    conn.begin(true);

    // Nur der vordere Sensor haengt noch hier. Links und
    // rechts misst der vordere Nano selbst.
    ultrasonic.begin(
        PinsRear::US_FRONT_TRIGGER_PIN,
        PinsRear::US_FRONT_ECHO_PIN,

        ULTRASONIC_NO_PIN,
        ULTRASONIC_NO_PIN,

        ULTRASONIC_NO_PIN,
        ULTRASONIC_NO_PIN
    );

    hardware_enableMotors();

    // D2 = INT0 bleibt der vorhandene Sync-Eingang.
    pinMode(
        PinsRear::SYNC_INPUT_PIN,
        INPUT
    );

    attachInterrupt(
        digitalPinToInterrupt(
            PinsRear::SYNC_INPUT_PIN
        ),
        syncCallback,
        RISING
    );
}

void RearApp::checkBatteryOrHalt()
{
    statusLed_begin(
        PinsRear::STATUS_LED_ANODE_PIN,
        PinsRear::STATUS_LED_CATHODE_PIN
    );

    batteryGuard_begin(
        PinsRear::BATTERY_SENSE_PIN
    );

    uint16_t battMv = 0;

    if (!batteryGuard_isStartAllowed(battMv))
    {
        // Kehrt nicht zurueck.
        haltOnLowBattery(battMv);
    }

    // Dauerlicht. Die Motoren werden weiter unten in begin()
    // freigegeben, sobald die Verbindung steht.
    statusLed_on();

    // '#'-Praefix: der vordere Nano ignoriert diese Zeile
    // (siehe UartLink::update, _buf[0] == '#').
    Serial.print(F("#BATT,OK,"));
    Serial.println(battMv);
}

void RearApp::haltOnLowBattery(uint16_t battMv)
{
    // STBY bleibt LOW. Weil der Standby-Eingang der vorderen
    // Treiber an derselben Leitung haengt, sind alle vier
    // Raeder stromlos - auch wenn der vordere Nano normal
    // hochlaeuft und Fahrbefehle erzeugt.
    hardware_disableMotors();

    // Bewusst kein Zugriff auf rad[]: radControl_begin()
    // laeuft erst nach dieser Pruefung. Noetig ist es auch
    // nicht - die Regelung wird nie erreicht.

    // Bewusst Endlosschleife: nach erkannter Unterspannung
    // soll der Roboter erst nach Akkuwechsel und Reset wieder
    // anlaufen. Der Watchdog ist in begin() abgeschaltet.
    while (1)
    {
        Serial.print(F("#BATT,LOW,"));
        Serial.println(battMv);

        statusLed_blinkBlocking(
            LOW_BATTERY_BLINKS_PER_MESSAGE,
            LOW_BATTERY_BLINK_ON_MS,
            LOW_BATTERY_BLINK_OFF_MS
        );
    }
}

void RearApp::update(uint32_t now)
{
    updateCommunication();
    updateConnectionSafety(now);
    updateVsolTimeout(now);
    handleIncomingVsol(now);

    radControl_update(now);

    updateUltrasonic(now);

    handleSyncVist(now);

    sendUltrasonicSnapshot(now);

    updateSyncDiagnostics(now);
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

void RearApp::updateConnectionSafety(
    uint32_t now)
{
    (void)now;

    static bool prevConnected = true;

    const bool nowConnected =
        uart.isConnected();

    if (prevConnected &&
        !nowConnected)
    {
        stopRearWheels();
        resetByWatchdog();
    }

    prevConnected = nowConnected;
}

void RearApp::resetByWatchdog()
{
    wdt_enable(WDTO_15MS);

    while (1)
    {
    }
}

void RearApp::updateVsolTimeout(
    uint32_t now)
{
    if (rearSollActive &&
        lastVsolMs > 0 &&
        now - lastVsolMs >
        2 * VEHICLE_DT_MS)
    {
        stopRearWheels();
    }
}

void RearApp::handleIncomingVsol(
    uint32_t now)
{
    if (!uart.availableLine())
    {
        return;
    }

    const char* line =
        uart.getLine();

    VsolMessage vsol = {};

    if (!parseVsolLine(
        line,
        vsol))
    {
        return;
    }

    lastVsolFrameId =
        vsol.frameId;

    const float vSollLiCms =
        (float)vsol.hiLiSoll;

    const float vSollReCms =
        (float)vsol.hiReSoll;

    if (vsol.resetPi)
    {
        radControl_resetPiStates();
        wheelMeasurement_reset_all();
    }

    rad[Li].setSoll(vSollLiCms);
    rad[Re].setSoll(vSollReCms);

    lastVsolMs = now;

    const bool wasRearSollActive =
        rearSollActive;

    // Bei den derzeit erlaubten CMDP-Befehlen hat
    // mindestens eines der beiden Hinterräder einen
    // Sollwert ungleich null.
    rearSollActive =
        vsol.hiLiSoll != 0 ||
        vsol.hiReSoll != 0;

    if (rearSollActive &&
        !wasRearSollActive)
    {
        // Das 100-ms-Ausgaberaster startet mit
        // dem neuen Fahrbefehl ebenfalls neu.
        // Dadurch wird nicht sofort ein leerer
        // Start-Snapshot übertragen.
        lastUsSendMs = now;
    }

    if (uart.isConnected())
    {
        printVsolOk(
            Serial,
            lastVsolFrameId
        );
    }
}

void RearApp::onSyncPulseFromIsr()
{
    syncPulseCount++;
    syncFlag = true;
}

void RearApp::handleSyncVist(uint32_t now)
{
    if (!syncFlag)
    {
        return;
    }

    syncFlag = false;

    // Der Sync-Impuls ist der gemeinsame Zeitnullpunkt
    // beider Nanos. Hinten misst in der ersten Haelfte des
    // Frames, vorne in der zweiten - so ueberschneiden sich
    // die Echofenster nicht.
    ultrasonic.requestMeasurement(now);

    const int16_t vIstLiCms =
        wheelMeasurements[Li].cmsInt();

    const int16_t vIstReCms =
        wheelMeasurements[Re].cmsInt();

    const int16_t pwmLi =
        rad[Li].lastPwm();

    const int16_t pwmRe =
        rad[Re].lastPwm();

    const int32_t cntLi =
        (int32_t)
        wheelMeasurements[Li]
        .counts_total();

    const int32_t cntRe =
        (int32_t)
        wheelMeasurements[Re]
        .counts_total();

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

void RearApp::updateUltrasonic(
    uint32_t now)
{
    // Ultraschall folgt dem Fahrzustand des
    // hinteren Nanos.
    ultrasonic.setEnabled(
        rearSollActive
    );

    ultrasonic.update(now);
}

void RearApp::sendUltrasonicSnapshot(
    uint32_t now)
{
    // Keine Messung aktiv:
    // kein US-Telegramm senden.
    if (!ultrasonic.isEnabled())
    {
        return;
    }

    if (!uart.isConnected())
    {
        return;
    }

    if ((uint32_t)(
        now -
        lastUsSendMs
        ) < ULTRASONIC_SEND_INTERVAL_MS)
    {
        return;
    }

    lastUsSendMs = now;

    UltrasonicSnapshot snapshot = {};

    ultrasonic.makeSnapshot(
        now,
        snapshot
    );

    printUs(
        Serial,
        snapshot.sequence,
        snapshot.frontMm,
        snapshot.leftMm,
        snapshot.rightMm,
        snapshot.validMask,
        snapshot.frontAgeMs,
        snapshot.leftAgeMs,
        snapshot.rightAgeMs
    );
}

void RearApp::updateSyncDiagnostics(
    uint32_t now)
{
    if ((uint32_t)(
        now -
        lastSyncDiagMs
        ) < SYNC_DIAG_INTERVAL_MS)
    {
        return;
    }

    lastSyncDiagMs = now;

    // Zaehler atomar auslesen und zuruecksetzen, damit
    // die ISR nicht mitten in der Auswertung zuschlaegt.
    const uint8_t savedSreg = SREG;
    cli();

    const uint16_t count = syncPulseCount;
    syncPulseCount = 0;

    SREG = savedSreg;

    // '#'-Praefix: der vordere Nano ignoriert diese Zeile
    // (siehe UartLink::update, _buf[0] == '#').
    Serial.print(F("#SYNC,"));
    Serial.println(count);
}
