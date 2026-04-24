// hinten.ino
#include "Hardware.h"
#include "Control.h"
#include "../1_Common/src/globals.h"
#include "../1_Common/src/CommUtils.h"
#include "../2_Hardware/src/UartLink.h"
#include "../5_System/src/Connection/ConnectionMonitor.h"


UartLink uart(Serial, false);   // Responder
ConnectionMonitor conn(uart, 13);

void setup()
{
    Serial.begin(115200);
    hardware_begin();
    hardware_enableMotors();
    control_begin();

    uart.begin();
    conn.begin(false);   // ❗ nicht blockieren
}

void loop()
{
    uart.update();
    conn.update();

    if (uart.availableLine())
    {
        const char* line = uart.getLine();

        int16_t v2_i, v3_i;

        if (sscanf(line, "VSOL,%hd,%hd", &v2_i, &v3_i) == 2)
        {
            float v2 = int100ToFloat(v2_i);
            float v3 = int100ToFloat(v3_i);

            rad[Li].setSoll(v2);
            rad[Re].setSoll(v3);
        }
    }

    control_update(millis());
}