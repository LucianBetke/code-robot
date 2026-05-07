// hinten.ino

#include "src/Hardware.h"
#include "src/hardware_pins.h"

#include "src/Control.h"
#include "src/ControlConfig.h"
#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"

UartLink uart(Serial, false);   // Responder
ConnectionMonitor conn(uart, 13);

void setup()
{
    Serial.begin(115200);

    hardware_begin(PinsRear::PINS);
    hardware_enableMotors();

    control_begin(ConfigRear::CONFIG);

    speed_reset_all();

    uart.begin();
    conn.begin(false);
}
void loop()
{
    uint32_t now = millis();

    uart.update();
    conn.update();

    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        int16_t v2_i;
        int16_t v3_i;

        if (sscanf(line, "VSOL,%hd,%hd", &v2_i, &v3_i) == 2)
        {
            float v2 = int100ToFloat(v2_i);
            float v3 = int100ToFloat(v3_i);

            rad[Li].setSoll(v2);
            rad[Re].setSoll(v3);
        }
    }

    // VIST senden: v2_ist, v3_ist alle 100ms
    static uint32_t lastVist = 0;
    if (now - lastVist >= VEHICLE_DT_MS)
    {
        lastVist = now;
        int16_t v2_i = floatToInt100(speed[Li].mps());
        int16_t v3_i = floatToInt100(speed[Re].mps());
        int16_t pwm2 = rad[Li].lastPwm();
        int16_t pwm3 = rad[Re].lastPwm();
        char buf[32];
        snprintf(buf, sizeof(buf), "VIST,%d,%d,%d,%d", v2_i, v3_i, pwm2, pwm3);
        uart.sendLine(buf);
    }

    control_update(now);
}

//void loop()
//{
//    uint32_t now = millis();
//
//    // TEST: nur Re dreht
//    rad[Re].setSoll(0.0f);
//    rad[Li].setSoll(0.2f);
//
//    control_update(now);
//}
