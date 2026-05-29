// hinten.ino
#include <avr/wdt.h>

#include "RearApp.h"

#include "src/Hardware.h"
#include "src/hardware_pins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"

RearApp app;

void syncISR()
{
    app.onSyncPulseFromIsr();
}

void setup()
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsRear::PINS);
    control_begin(ConfigRear::CONFIG);
    speed_reset_all();

    app.begin();

    hardware_enableMotors();

    pinMode(3, INPUT);
    attachInterrupt(digitalPinToInterrupt(3), syncISR, RISING);
}

void loop()
{
    uint32_t now = millis();

    app.updateCommunication();
    app.updateConnectionSafety(now);
    app.updateVsolTimeout(now);
    app.handleIncomingVsol(now);

    control_update(now);

    app.handleSyncVist();
}