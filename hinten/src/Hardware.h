// Hardware.h
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