/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/

#include "CommandScript.h"
#include "Control.h"
#include "Hardware.h"

#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"

VehicleController vehicle;

UartLink uart(Serial, true);   // Initiator
ConnectionMonitor conn(uart, 13);
CommandParser parser;
CommandRunner commandRunner(vehicle, uart, parser);

void setup()
{
    Serial.begin(115200);

    hardware_begin();
    hardware_enableMotors();
    speed_reset_all();

    commandRunner.begin();

    uart.begin();
    conn.begin(true);
}

void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    static bool active = false;
    static bool done = false;
    static uint32_t startTime = 0;
    static uint32_t lastSend = 0;

    if (!done && !active)
    {
        vehicle.cmd(0.20f, 0.0f, 0.0f);   // vx, vy, wz
        startTime = now;
        active = true;
    }

    if (active && (now - startTime >= 2000))
    {
        vehicle.cmd(0.0f, 0.0f, 0.0f);

        rad[Li].setSoll(0.0f);
        rad[Re].setSoll(0.0f);

        uart.sendLine("VSOL,0,0");

        active = false;
        done = true;
    }

    if (active)
    {
        rad[Li].setSoll(vehicle.getWheelSoll(VoLi));
        rad[Re].setSoll(vehicle.getWheelSoll(VoRe));
    }

    if (now - lastSend >= 100)
    {
        lastSend = now;

        int16_t v2_i = floatToInt100(vehicle.getWheelSoll(HiLi));
        int16_t v3_i = floatToInt100(vehicle.getWheelSoll(HiRe));

        char buf[32];
        snprintf(buf, sizeof(buf), "VSOL,%d,%d", v2_i, v3_i);

        uart.sendLine(buf);
    }

    control_update(now);
}