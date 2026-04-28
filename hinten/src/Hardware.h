// Hardware.h
#pragma once

#include <Arduino.h>

// --- Encoder ---
extern Enc enc[WHEEL_COUNT];

// --- Motoren ---
extern Motor motor[WHEEL_COUNT];

// zentrale Init-Funktion: Pins/IO + optional Encoder reset
void hardware_begin(bool resetEnc = true);
void hardware_enableMotors();
