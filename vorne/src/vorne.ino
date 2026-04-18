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
    Serial.println("#Handshake1 OK");
}

void loop()
{
    static uint32_t lastOk = 0;
    static bool lastState = false;

    uart.update();

    bool now = uart.isConnected();

    // 🔴 Verbindung verloren
    if (!now && lastState)
    {
        Serial.println("#DISCONNECTED");
    }

    // 🟢 Verbindung neu da
    if (now && !lastState)
    {
        Serial.println("#CONNECTED");
    }

    // 🟢 läuft stabil
    if (now && millis() - lastOk > 1000)
    {
        Serial.println("#OK");
        lastOk = millis();
    }

    lastState = now;
}
