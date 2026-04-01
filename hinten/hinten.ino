// hinten.ino
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
    
    hardware_begin(true);
    control_begin();

    control_setSoll(Li, V_SOLL_GERADE);
    control_setSoll(Re, V_SOLL_GERADE);

    startTime = millis();
}

void loop()
{
    static uint32_t last = 0;
    uint32_t now = millis();

    control_update(now);
    print_update(now);     // Logging

    
}