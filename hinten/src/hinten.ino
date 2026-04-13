// hinten.ino

#include "../2_Hardware/src/UartLink.h"


UartLink uart(Serial, false);  // kein PING senden

void setup()
{
    Serial.begin(115200);
    uart.begin(115200);
    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(1000);

    // blockierend warten bis Handshake vollständig (PING/PONG/ACK)
    while (!uart.isConnected())
    {
        uart.update();
    }

    digitalWrite(13, LOW);

}

void loop()
{
}