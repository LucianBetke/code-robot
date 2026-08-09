// ============================================================
// File: Hardware.cpp
// Zweck:
//  - Globale Hardware-Objekte bereitstellen
//  - Encoder und Motoren initialisieren
//  - STBY-/Sync-Pin steuern
// ============================================================

#include <Arduino.h>

#include "Hardware.h"
#include "Encoder.h"
#include "Motor.h"
#include "HardwarePins.h"

Enc enc[WHEEL_COUNT];
Motor motor[WHEEL_COUNT];

// Dieser Pin ist board-abhaengig:
// vorne: Sync-Ausgang fuer VIST-Anforderung
// hinten: STBY-Ausgang fuer beide Motortreiber
static uint8_t s_boardControlPin = 0;

void hardware_begin(const HardwarePinSet& pins, bool resetEnc)
{
    s_boardControlPin = pins.boardControlPin;

    enc[Li].begin(pins.encLiA, pins.encLiB, resetEnc);
    enc[Re].begin(pins.encReA, pins.encReB, resetEnc);

    motor[Li].begin(pins.motorLi1, pins.motorLi2, enc[Li]);
    motor[Re].begin(pins.motorRe1, pins.motorRe2, enc[Re]);

    pinMode(s_boardControlPin, OUTPUT);

    motor[Li].init();
    motor[Re].init();
}

void hardware_enableMotors()
{
    digitalWrite(s_boardControlPin, HIGH);
}

void hardware_disableMotors()
{
    digitalWrite(s_boardControlPin, LOW);
}

void hardware_requestVist()
{
    digitalWrite(s_boardControlPin, HIGH);
    digitalWrite(s_boardControlPin, LOW);
}