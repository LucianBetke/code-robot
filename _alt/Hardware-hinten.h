// Hardware.h
#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>
#include "src/globals.h"

class Enc;
class Motor;

// --- Encoder ---
extern Enc enc[WHEEL_COUNT];

// --- Motoren ---
extern Motor motor[WHEEL_COUNT];

// zentrale Init-Funktion: Pins/IO + optional Encoder reset
void hardware_begin(bool resetEnc = true);
void hardware_enableMotors();

#endif // HARDWARE_H