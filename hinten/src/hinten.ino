// hinten.ino
#include "RearApp.h"

RearApp app;

void syncISR()
{
    app.onSyncPulseFromIsr();
}

void setup()
{
    app.begin(syncISR);
}

void loop()
{
    app.update(millis());
}