// ============================================================
// File: Hardware.cpp
// für vorne
// Zweck:
//  - Definition der physischen Hardware-Objekte (Encoder, Motoren)
//  - Zentrale Hardware-Initialisierung (Pins/IO, Treiber-Enable, etc.)
// Hinweis:
//  - Reset/Begin von Control-/Regel-Objekten (SpeedWeg, Regler)
//    gehört NICHT hierhin, sondern nach Control.cpp (speed_reset_all()).
// ============================================================
#include <Arduino.h>
#include "src/globals.h"
#include "src/Encoder.h"
#include "src/Motor.h"
#include "Hardware.h"
#include "src/hardware_pins.h"

// ============================================================
// --- Encoder ---
// ============================================================
Enc enc[WHEEL_COUNT] =
{
    Enc(PinsFront::ENC_Li_PIN_A, PinsFront::ENC_Li_PIN_B),   // Li
    Enc(PinsFront::ENC_Re_PIN_A, PinsFront::ENC_Re_PIN_B)    // Re
};

// ============================================================
// --- Motoren ---
// ============================================================
Motor motor[WHEEL_COUNT] =
{
    Motor(PinsFront::M_Li_BIN1, PinsFront::M_Li_BIN2, enc[Li]),
    Motor(PinsFront::M_Re_AIN1, PinsFront::M_Re_AIN2, enc[Re])
};

// ============================================================
// --- zentrale Hardware-Init (Option A) ---
// ============================================================
void hardware_begin(bool /*resetEnc*/)
{
    pinMode(PinsFront::STBY_PIN, OUTPUT);
    digitalWrite(PinsFront::STBY_PIN, LOW);  // Treiber AUS (sicherer Start)

    motor[Li].init();
    motor[Re].init();

    enc[Li].begin();
    enc[Re].begin();

    // --------------------------------------------------------
    // Hier wäre der richtige Ort für weitere echte IO-Init:
    //  - pinMode(...) für zusätzliche Pins
    //  - I2C/SPI begin
    //  - Interrupt-Setup, falls das rein hardwareseitig ist
    // --------------------------------------------------------
}

void hardware_enableMotors()
{
    digitalWrite(PinsFront::STBY_PIN, HIGH);
}