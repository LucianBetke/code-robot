// hinten.ino

#include "Control.h"
#include "Hardware.h"

#include "src/CommUtils.h"
#include "src/UartLink.h"
#include "src/Connection/ConnectionMonitor.h"

UartLink uart(Serial, false);   // Responder
ConnectionMonitor conn(uart, 13);

void setup()
{
    Serial.begin(115200);

    hardware_begin();
    hardware_enableMotors();
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

    control_update(now);
}