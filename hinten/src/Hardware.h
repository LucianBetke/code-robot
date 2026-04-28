// Hardware.h (hinten)
#ifndef HINTEN_HARDWARE_H
#define HINTEN_HARDWARE_H

#pragma once

#include <Arduino.h>
#include "src/globals.h"

class Enc;
class Motor;

// --- Encoder ---
extern Enc enc[WHEEL_COUNT];

// --- Motoren ---
extern Motor motor[WHEEL_COUNT];

void hardware_begin(bool resetEnc = true);
void hardware_enableMotors();

#endif // HINTEN_HARDWARE_H