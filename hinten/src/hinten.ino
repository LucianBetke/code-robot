// hinten.ino
#include "../2_Hardware/src/UartLink.h"
#include "../5_System/src/Connection/ConnectionMonitor.h"

UartLink uart(Serial, false);   // Responder
ConnectionMonitor conn(uart, 13);

void setup()
{
    Serial.begin(115200);

    uart.begin();
    conn.begin(false);   // ❗ nicht blockieren
}

void loop()
{
    uart.update();
    conn.update();
}