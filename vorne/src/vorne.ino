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
#include "src/Printer.h"

// globale Variablen
static float g_v2_ist = 0.0f;
static float g_v3_ist = 0.0f;
static int16_t g_pwm2 = 0;
static int16_t g_pwm3 = 0;
static uint32_t g_startMs = 0;
static bool g_timerStarted = false;

// globale Objekte
VehicleController vehicle;
UartLink uart(Serial, true);
ConnectionMonitor conn(uart, 13);
CommandParser parser;
CommandRunner commandRunner(vehicle, uart, parser,
    CommandScript::get, CommandScript::size);
Printer printer;

void setup()
{
    Serial.begin(115200);
    hardware_begin(PinsFront::PINS);
    hardware_enableMotors();
    control_begin(ConfigFront::CONFIG);
    speed_reset_all();

    vehicle.begin(
        0.0f, 0.0f,   // Kp_vx, Ki_vx
        0.0f, 0.0f,   // Kp_vy, Ki_vy
        0.0f, 0.0f    // Kp_wz, Ki_wz
    );

    commandRunner.begin();
    uart.begin();
    conn.begin(true);
    printer.printHeader();
}


void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    if (uart.isConnected())
        commandRunner.update(now);

    if (commandRunner.isActive() && !g_timerStarted)
    {
        g_startMs = now;
        g_timerStarted = true;
    }

    if (uart.availableLine())
    {
        const char* line = uart.getLine();
        int16_t v2_i, v3_i;
        if (sscanf(line, "VIST,%hd,%hd,%hd,%hd", &v2_i, &v3_i, &g_pwm2, &g_pwm3) == 4)
        {
            g_v2_ist = int100ToFloat(v2_i);
            g_v3_ist = int100ToFloat(v3_i);
        }
    }

    vehicle.updateIst(
        speed[Re].mps(),
        speed[Li].mps(),
        g_v2_ist,
        g_v3_ist
    );
    vehicle.update(now);

    rad[VoLi].setSoll(commandRunner.getWheelSoll(VoLi));
    rad[VoRe].setSoll(commandRunner.getWheelSoll(VoRe));

    static uint32_t lastSend = 0;
    if (now - lastSend >= VEHICLE_DT_MS)
    {
        lastSend = now;
        int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
        int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));
        char buf[32];
        snprintf(buf, sizeof(buf), "VSOL,%d,%d", v2_i, v3_i);
        uart.sendLine(buf);
    }

    control_update(now);

    static uint32_t lastDbg = 0;
    if (!commandRunner.isFinished() && now - lastDbg >= VEHICLE_DT_MS)
    {
        lastDbg = now;
        uint32_t t = g_timerStarted ? (now - g_startMs) : 0;
        printer.printWheels(vehicle, g_v2_ist, g_v3_ist, g_pwm2, g_pwm3, t);
    }
}