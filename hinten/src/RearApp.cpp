// RearApp.cpp
#include "RearApp.h"

#include <avr/interrupt.h>
#include <avr/wdt.h>

#include "src/CommProtocol.h"
#include "src/Hardware.h"
#include "src/HardwarePins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"

static const uint8_t REAR_SYNC_INPUT_PIN = 2;

static const uint16_t
ULTRASONIC_SEND_INTERVAL_MS = 100;

static const uint16_t
SYNC_DIAG_INTERVAL_MS = 1000;

// Die seitlichen HC-SR04 sitzen jetzt am vorderen Nano.
// Dadurch ist D13 hinten frei und traegt die Verbindungs-LED,
// die vorher vorne lief. Vorne wird D13 fuer den Funk (SCK)
// gebraucht, sein ConnectionMonitor laeuft deshalb ohne LED.

RearApp::RearApp()
    : uart(Serial, false),
    conn(uart, 13),
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
        REAR_SYNC_INPUT_PIN,
        INPUT
    );

    attachInterrupt(
        digitalPinToInterrupt(
            REAR_SYNC_INPUT_PIN
        ),
        syncCallback,
        RISING
    );
}

void RearApp::update(uint32_t now)
{
    updateCommunication();
    updateConnectionSafety(now);
    updateVsolTimeout(now);
    handleIncomingVsol(now);

    radControl_update(now);

    updateUltrasonic(now);

    handleSyncVist();

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

void RearApp::handleSyncVist()
{
    if (!syncFlag)
    {
        return;
    }

    syncFlag = false;

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
