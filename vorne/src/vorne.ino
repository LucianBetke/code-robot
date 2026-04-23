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
    hardware_enableMotors();
    control_begin();

/*    uart.begin();
    conn.begin(true); */  
}

void loop()
{
    static bool done = false;

    if (!done)
    {
        const char* line = CommandScript::get(0);
        TimeCommand cmd;

        Serial.print("Script: ");
        Serial.println(line);

        if (parser.parseTimeCommand(line, cmd))
        {
            Serial.println("Parser OK");

            Serial.print("vx: "); Serial.println(cmd.vx);
            Serial.print("vy: "); Serial.println(cmd.vy);
            Serial.print("wz: "); Serial.println(cmd.wz);

            vehicle.cmd(cmd.vx, cmd.vy, cmd.wz);

            float vLi = vehicle.getWheelSoll(VoLi);
            float vRe = vehicle.getWheelSoll(VoRe);

            Serial.print("VoLi: "); Serial.println(vLi);
            Serial.print("VoRe: "); Serial.println(vRe);

            rad[Li].setSoll(vLi);
            rad[Re].setSoll(vRe);
        }
        else
        {
            Serial.println("Parser FEHLER");
        }

        done = true;
    }

    control_update(millis());
}