// vorne.ino

// SPI und RF24 muessen hier eingebunden werden, damit der
// Arduino-Build die Bibliotheken findet. Die eigentliche
// Nutzung liegt in 2_Hardware/src/Nrf24Radio.cpp.
#include <SPI.h>
#include <RF24.h>

#include "FrontApp.h"

FrontApp app;

void setup()
{
    app.begin();
}

void loop()
{
    app.update(millis());
}
