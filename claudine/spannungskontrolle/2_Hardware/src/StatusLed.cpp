// ============================================================
// File: StatusLed.cpp
// Zweck:
//  - Status-LED zwischen zwei GPIOs schalten
// ============================================================

#include <Arduino.h>

#include "StatusLed.h"

// Keine Pinnummern als Vorbelegung: die gueltigen Werte
// kommen aus HardwarePins.h ueber statusLed_begin().
static uint8_t s_anodePin = 0;
static uint8_t s_cathodePin = 0;

void statusLed_begin(
    uint8_t anodePin,
    uint8_t cathodePin)
{
    s_anodePin = anodePin;
    s_cathodePin = cathodePin;

    pinMode(s_anodePin, OUTPUT);
    pinMode(s_cathodePin, OUTPUT);

    statusLed_off();
}

void statusLed_on()
{
    // Reihenfolge bewusst: erst die Masseseite festlegen,
    // dann einspeisen.
    digitalWrite(s_cathodePin, LOW);
    digitalWrite(s_anodePin, HIGH);
}

void statusLed_off()
{
    // Beide Seiten LOW: keine Spannungsdifferenz, kein Strom.
    digitalWrite(s_anodePin, LOW);
    digitalWrite(s_cathodePin, LOW);
}

void statusLed_blinkBlocking(
    uint8_t count,
    uint16_t onMs,
    uint16_t offMs)
{
    for (uint8_t i = 0; i < count; i++)
    {
        statusLed_on();
        delay(onMs);

        statusLed_off();
        delay(offMs);
    }
}
