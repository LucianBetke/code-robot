/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/
#include "CommandScript.h"
#include "Control.h"
#include "Hardware.h"
#include "../1_Common/src/globals.h"
#include "../1_Common/src/CommUtils.h"
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

    uart.begin();
    conn.begin(true);   
}

void loop()
{
    static bool active = false;
    static bool done = false;
    static uint32_t startTime = 0;
    static float duration = 0.0f;

    if (active)
    {
        if (millis() - startTime >= (uint32_t)duration)
        {
            rad[Li].setSoll(0);
            rad[Re].setSoll(0);
            active = false;
            done = true;   // 🔥 entscheidend
        }
    }
    else if (!done)
    {
        const char* line = CommandScript::get(0);
        TimeCommand cmd;

        if (parser.parseTimeCommand(line, cmd))
        {
            vehicle.cmd(cmd.vx, cmd.vy, cmd.wz);

            rad[Li].setSoll(vehicle.getWheelSoll(VoLi));
            rad[Re].setSoll(vehicle.getWheelSoll(VoRe));
            // Werte für hinteren Nano vorbereiten
            int16_t v2_i = floatToInt100(vehicle.getWheelSoll(HiLi));
            int16_t v3_i = floatToInt100(vehicle.getWheelSoll(HiRe));

            char buffer[32];
            snprintf(buffer, sizeof(buffer), "VSOL,%d,%d", v2_i, v3_i);

            uart.sendLine(buffer);

            duration = cmd.t * 1000.0f;
            startTime = millis();
            active = true;
        }
    }

    control_update(millis());
}