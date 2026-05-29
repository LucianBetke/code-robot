// ============================================================
// File: Hardware.h
// Zweck:
//  - Zentrale Hardware-Schnittstelle fuer vorne und hinten
//  - Globale Motor- und Encoder-Objekte bereitstellen
//  - Hardware-Pins initialisieren
// ============================================================

#ifndef HARDWARE_H
#define HARDWARE_H

#include <Arduino.h>

#include "src/RobotConfig.h"
#include "src/HardwarePins.h"

class Enc;
class Motor;

extern Enc enc[WHEEL_COUNT];
extern Motor motor[WHEEL_COUNT];

void hardware_begin(const HardwarePinSet& pins, bool resetEnc = true);

void hardware_enableMotors();
void hardware_disableMotors();

void hardware_requestVist();

#endif // HARDWARE_H