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
        int16_t v2_i, v3_i;
        if (sscanf(line, "VSOL,%hd,%hd", &v2_i, &v3_i) == 2)
        {
            Serial.print(F("#VSOL@"));
            Serial.print(now);
            Serial.print(F(" v2="));
            Serial.print(v2_i);
            Serial.print(F(" v3="));
            Serial.println(v3_i);

            float v2 = int100ToFloat(v2_i);
            float v3 = int100ToFloat(v3_i);
            rad[Li].setSoll(v2);
            rad[Re].setSoll(v3);

            // Sofort VIST zurücksenden
            int16_t vi2 = floatToInt100(speed[Li].mps());
            int16_t vi3 = floatToInt100(speed[Re].mps());
            int16_t pwm2 = rad[Li].lastPwm();
            int16_t pwm3 = rad[Re].lastPwm();
            char buf[32];
            snprintf(buf, sizeof(buf), "VIST,%d,%d,%d,%d", vi2, vi3, pwm2, pwm3);
            uart.sendLine(buf);
        }
    }

    control_update(now);
}