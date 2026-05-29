// vorne.ino
#include <avr/wdt.h>

#include "FrontApp.h"

#include "src/Hardware.h"
#include "src/HardwarePins.h"
#include "src/RadControl.h"
#include "src/RadControlConfig.h"

FrontApp app;

void setup()
{
    wdt_disable();

    Serial.begin(115200);

    hardware_begin(PinsFront::PINS);
    control_begin(ConfigFront::CONFIG);
    speed_reset_all();

    app.vehicle.begin(
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f
    );

    app.commandRunner.begin();
    app.rearFrameClient.begin();
    app.frameScheduler.begin(VEHICLE_DT_MS);

    app.uart.begin();
    app.conn.begin(true);

    app.printer.printInfo(app.vehicle, ConfigFront::CONFIG);
}

void loop()
{
    uint32_t now = millis();

    app.updateCommunication();
    app.handleIncomingLines(now);
    app.updateFrameTimeout(now);
    app.updateConnectionSafety(now);

    if (app.isConnected())
    {
        app.updateVehicleAndFrontControl(now);

        // Erst Messframe pruefen.
        // Danach darf der CommandRunner den aktiven Befehl beenden.
        app.tryRequestFrame(now);

        app.updateCommandRunner(now);
        app.updateLogRaster(now);
        app.updateRearStopSequence(now);
    }
}