// hinten.ino

#include "../2_Hardware/src/UartLink.h"

UartLink uart(Serial, false);   // ❗ kein Initiator

void setup()
{
    Serial.begin(115200);
    uart.begin(115200);

    pinMode(13, OUTPUT);
    digitalWrite(13, HIGH);   // wartet auf Verbindung

    Serial.println("Rear wartet auf Verbindung...");

    // 🔴 bewusst blockierend (für deinen aktuellen Test richtig)
    while (!uart.isConnected())
    {
        uart.update();
    }

    digitalWrite(13, LOW);   // Verbindung steht
    Serial.println("#Handshake OK (Rear)");
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