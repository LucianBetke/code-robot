// Hardware.h (vorne)
#ifndef VORNE_HARDWARE_H
#define VORNE_HARDWARE_H

#pragma once

#include <Arduino.h>
#include "src/globals.h"
#include "src/Encoder.h"
#include "src/Motor.h"

// --- Encoder ---
extern Enc enc[WHEEL_COUNT];

// --- Motoren ---
extern Motor motor[WHEEL_COUNT];

// zentrale Init-Funktion: Pins/IO + optional Encoder reset
void hardware_begin(bool resetEnc = true);
void hardware_enableMotors();

#endif // VORNE_HARDWARE_H