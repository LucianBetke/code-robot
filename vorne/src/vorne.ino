// vorne.ino
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