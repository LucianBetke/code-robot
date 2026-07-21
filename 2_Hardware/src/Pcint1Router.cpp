// ============================================================
// File: Pcint1Router.cpp
// Zweck:
//  - Einzige Definition von ISR(PCINT1_vect)
//  - PINC genau einmal lesen
//  - Snapshot an Encoder und Ultraschall verteilen
// ============================================================

#include <Arduino.h>
#include <avr/interrupt.h>

#include "Encoder.h"
#include "UltrasonicEchoCapture.h"

ISR(PCINT1_vect)
{
    const uint8_t pinc = PINC;

    PCINT1_Dispatcher::isr(pinc);
    UltrasonicEchoCapture::isrPcint1(pinc);
}