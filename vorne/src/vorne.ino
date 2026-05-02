/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Dr. Faust
*/
#include "CommandScript.h"
#include "src/Hardware.h"
#include "src/hardware_pins.h"
#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"
#include "src/CommandRunner/CommandRunner.h"

VehicleController vehicle;
UartLink uart(Serial, true);
ConnectionMonitor conn(uart, 13);
CommandParser parser;
CommandRunner commandRunner(vehicle, uart, parser,
    CommandScript::get, CommandScript::size);

void setup()
{
    Serial.begin(115200);
    hardware_begin(PinsFront::PINS);
    hardware_enableMotors();
    control_begin(ConfigFront::CONFIG);
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
    commandRunner.update(now);

    rad[VoLi].setSoll(commandRunner.getWheelSoll(VoLi));
    rad[VoRe].setSoll(commandRunner.getWheelSoll(VoRe));

    static uint32_t lastSend = 0;
    if (now - lastSend >= 100)
    {
        lastSend = now;
        int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
        int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));
        char buf[32];
        snprintf(buf, sizeof(buf), "VSOL,%d,%d", v2_i, v3_i);
        uart.sendLine(buf);
    }
    control_update(now);
}