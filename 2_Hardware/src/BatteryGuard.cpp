// ============================================================
// File: BatteryGuard.cpp
// Zweck:
//  - Akkuspannung gegen die interne 1,1-V-Referenz messen
//  - Startfreigabe anhand einer festen Schwelle entscheiden
// ============================================================

#include <Arduino.h>

#include "BatteryGuard.h"

// Keine Pinnummer als Vorbelegung: der gueltige Wert kommt
// aus HardwarePins.h ueber batteryGuard_begin().
static uint8_t s_sensePin = 0;

void batteryGuard_begin(uint8_t sensePin)
{
    s_sensePin = sensePin;

    // Kein anderer Programmteil benutzt den ADC. Die Referenz
    // darf deshalb dauerhaft auf 1,1 V stehen bleiben.
    analogReference(INTERNAL);

    // Nach dem Umschalten braucht die Referenz einige
    // Millisekunden, bis sie steht.
    delay(10);

    for (uint8_t i = 0; i < BATTERY_DISCARD_SAMPLES; i++)
    {
        analogRead(s_sensePin);
    }
}

uint16_t batteryGuard_readMillivolts()
{
    uint32_t sum = 0;

    for (uint8_t i = 0; i < BATTERY_AVERAGE_SAMPLES; i++)
    {
        sum += (uint32_t)analogRead(s_sensePin);
        delay(1);
    }

    const uint32_t adc =
        sum / BATTERY_AVERAGE_SAMPLES;

    // Bewusst nur eine einzige Division am Ende, damit sich
    // keine Rundungsfehler aufaddieren.
    // adc <= 1023, der Zwischenwert bleibt unter 2^27.
    const uint32_t mv =
        (adc *
            (uint32_t)BATTERY_REF_MV *
            (uint32_t)BATTERY_DIVIDER_NUM) /
        ((uint32_t)BATTERY_DIVIDER_DEN * 1023UL);

    return (uint16_t)mv;
}

bool batteryGuard_isStartAllowed(uint16_t& measuredMv)
{
    measuredMv = batteryGuard_readMillivolts();

    return measuredMv >= BATTERY_MIN_START_MV;
}
