/*
 Name:      vorne.ino
 Created:   11.03.2026 22:17:58
 Author:    Acer
*/

#include "../1_Common/src/globals.h"
#include "../4_Vehicle/src/VehicleController.h"
#include "../2_Hardware/src/UartLink.h"

VehicleController vehicle;

UartLink uart(Serial, true);   // initiator

void setup()
{
    Serial.begin(115200);
    uart.begin(115200);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);
    delay(1000);

    Serial.println("Warte auf Handshake...");

    while (!uart.isConnected())
    {
        uart.update();
    }

    
    digitalWrite(13, LOW);
    Serial.println("#Handshake OK");
}

void loop()
{
}
