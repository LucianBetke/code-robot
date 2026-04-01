// ============================================================
// File: hinten.ino
// Zweck:
//  - Testprogramm Hinterachse
//  - Räder laufen 5 Sekunden mit Sollgeschwindigkeit
//  - Danach Stop
//  - Logging läuft weiter
// ============================================================

#include "Hardware.h"
#include "Control.h"
#include "hardware_pins.h"
#include "Print.h"

constexpr float V_SOLL_GERADE = 0.30f;

uint32_t startTime;
bool stopped = false;

void setup()
{
    Serial.begin(115200);
    Serial.println("SETUP");

    // Hardware initialisieren (Pins, Encoder, Motoren)
    hardware_begin(true);

    // Control / Regler initialisieren
    control_begin();

    // Motor-Treiber aktivieren
    hardware_enableMotors();

    // Sollgeschwindigkeit setzen
    control_setSoll(Li, V_SOLL_GERADE);
    control_setSoll(Re, V_SOLL_GERADE);

    startTime = millis();
}

void loop()
{
    uint32_t now = millis();

    // --------------------------------------------------------
    // 5 Sekunden laufen lassen, dann stoppen
    // --------------------------------------------------------
    if (!stopped && (now - startTime >= 5000))
    {
        control_setSoll(Li, 0.0f);
        control_setSoll(Re, 0.0f);
        stopped = true;
        Serial.println("STOP");
    }

    // --------------------------------------------------------
    // Regelung aktualisieren
    // --------------------------------------------------------
    control_update(now);

    // --------------------------------------------------------
    // Logging / Debug-Ausgabe
    // --------------------------------------------------------
    if (!stopped)
    {
        print_update(now);
    }
}