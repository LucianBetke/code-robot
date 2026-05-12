// vorne.ino
#include <avr/wdt.h>
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
static uint32_t lastVistMs = 0;
static uint32_t lastDbg = 0;

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
    wdt_disable();
    Serial.begin(115200);
    hardware_begin(PinsFront::PINS);
    control_begin(ConfigFront::CONFIG);
    speed_reset_all();
    vehicle.begin(
        0.0f, 0.0f,
        0.0f, 0.0f,
        0.0f, 0.0f
    );
    commandRunner.begin();
    uart.begin();
    conn.begin(true);
    printer.printHeader(vehicle, ConfigFront::CONFIG);
}

void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    // DISCONNECT erkennen → Reset
    static bool prevConnected = false;
    bool nowConnected = uart.isConnected();
    if (prevConnected && !nowConnected)
    {
        wdt_enable(WDTO_15MS);
        while (1) {}
    }
    prevConnected = nowConnected;

    // VIST-Timeout → Reset
    if (lastVistMs > 0 && now - lastVistMs > 2 * VEHICLE_DT_MS)
    {
        wdt_enable(WDTO_15MS);
        while (1) {}
    }

    if (uart.isConnected())
        commandRunner.update(now);

    if (!commandRunner.isActive())
    {
        g_timerStarted = false;
        lastDbg = 0;
    }

    if (commandRunner.isActive())
    {
        if (!g_timerStarted)
        {
            g_startMs = now;
            g_timerStarted = true;
        }
    }

    if (uart.availableLine())
    {
        const char* line = uart.getLine();
        int16_t v2_i, v3_i;
        if (sscanf(line, "VIST,%hd,%hd,%hd,%hd", &v2_i, &v3_i, &g_pwm2, &g_pwm3) == 4)
        {
            g_v2_ist = int100ToFloat(v2_i);
            g_v3_ist = int100ToFloat(v3_i);
            lastVistMs = now;
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

    static uint32_t lastVsolSend = 0;
    if (lastVsolSend == 0) lastVsolSend = now;
    if (now - lastVsolSend >= VEHICLE_DT_MS)
    {
        lastVsolSend += VEHICLE_DT_MS;
        int16_t v2_i = floatToInt100(commandRunner.getWheelSoll(HiLi));
        int16_t v3_i = floatToInt100(commandRunner.getWheelSoll(HiRe));
        char bufVsoll[32];
        snprintf(bufVsoll, sizeof(bufVsoll), "VSOL,%d,%d", v2_i, v3_i);
        uart.sendLine(bufVsoll);
        hardware_requestVist();
    }

    control_update(now);

    if (g_timerStarted && now - lastDbg >= VEHICLE_DT_MS)
    {
        if (lastDbg == 0) lastDbg = now;
        else lastDbg += VEHICLE_DT_MS;
        uint32_t t = lastDbg - g_startMs;
#ifdef PRINTER_MODE_CHASSIS
        printer.printWheels(vehicle, g_v2_ist, g_v3_ist, t);
#endif
#ifdef PRINTER_MODE_RAEDER
        printer.printWheels(vehicle, g_v2_ist, g_v3_ist, g_pwm2, g_pwm3, t);
#endif
    }
}