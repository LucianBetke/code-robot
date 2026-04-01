// hinten.ino
#include "Hardware.h"
#include "Control.h"
#include "Print.h"

constexpr float V_SOLL_GERADE = 0.30f;

uint32_t startTime;
bool stopped = false;

void setup()
{
    Serial.begin(115200);

    hardware_begin(true);
    control_begin();
    print_begin();

    hardware_enableMotors();

    control_setSoll(Li, V_SOLL_GERADE);
    control_setSoll(Re, V_SOLL_GERADE);

    startTime = millis();
}

void loop()
{
    uint32_t now = millis();

    // Nach 5 Sekunden stoppen
    if (!stopped && (now - startTime >= 5000))
    {
        control_setSoll(Li, 0.0f);
        control_setSoll(Re, 0.0f);
        stopped = true;
        Serial.println("STOP");
    }

    // Regelung läuft immer weiter
    control_update(now);

    // Logging nur während der Messphase
    if (!stopped)
    {
        print_update(now);
    }
}