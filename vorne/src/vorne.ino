/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/
#include "CommandScript.h"
#include "Control.h"
#include "Hardware.h"
#include "../1_Common/src/globals.h"
#include "../2_Hardware/src/UartLink.h"
#include "../4_Vehicle/src/VehicleController.h"
#include "../5_System/src/Connection/ConnectionMonitor.h"
#include "../5_System/src/Parser/CommandParser.h"
VehicleController vehicle;

UartLink uart(Serial, true);   // Initiator
ConnectionMonitor conn(uart, 13);
CommandParser parser;

void setup()
{
    Serial.begin(115200);
    hardware_begin();
    control_begin();

    uart.begin();
    conn.begin(true);   
}

void loop()
{
    static bool done = false;
    uart.update();
    conn.update();

    static uint8_t i = 0;

    if (!done && i < CommandScript::size())
    {
        const char* line = CommandScript::get(i);
        TimeCommand cmd;

        // zerlegt "CMDT(...)" in vx, vy, wz und Zeit
        parser.parseTimeCommand(line, cmd);
        // berechnet aus vx, vy, wz v0-v3
        vehicle.cmd(cmd.vx, cmd.vy, cmd.wz);
        // im loop nur einmal durchlaufen
        done = true;
        // überträgt Fahrzeug-Sollwert auf linkes Vorderrad
		rad[Li].setSoll(vehicle.getWheelSoll(VoLi));
    }
    control_update(millis());
}