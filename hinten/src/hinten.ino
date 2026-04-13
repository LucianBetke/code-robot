// hinten.ino

#include "../2_Hardware/src/UartLink.h"

UartLink uart(Serial);

void setup()
{
    Serial.begin(115200);
    uart.begin(115200);

    // blockierend warten bis Handshake vollständig (PING/PONG/ACK)
    while (!uart.isConnected())
    {
        uart.update();
    }

    // keine Ausgabe gewünscht → nichts hier
}

void loop()
{
}