// hinten.ino
#include "Hardware.h"
#include "Control.h"

constexpr float V_SOLL_GERADE = 0.30f;

uint32_t startTime;
bool stopped = false;

void setup()
{
    hardware_begin(true);
    control_begin();

    control_setSoll(Li, V_SOLL_GERADE);
    control_setSoll(Re, V_SOLL_GERADE);

    startTime = millis();
}

void loop()
{
    uint32_t now = millis();

    if (!stopped && (now - startTime > 5000))
    {
        control_setSoll(Li, 0.0f);
        control_setSoll(Re, 0.0f);
        stopped = true;
    }

    control_update(now);
}