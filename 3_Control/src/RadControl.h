// ============================================================
// File: RadControl.h (3_Control)
// Zweck:
//  - Gemeinsame Radregelungs-Schnittstelle fuer vorne und hinten
//  - Einheit fuer Radgeschwindigkeit: cm/s
// ============================================================

#ifndef RAD_CONTROL_H
#define RAD_CONTROL_H

#include <Arduino.h>
#include "src/RobotConfig.h"
#include "src/WheelMeasurement.h"
#include "src/PIRegler.h"
#include "src/Rad.h"
#include "RadControlConfig.h"

extern WheelMeasurement wheelMeasurements[WHEEL_COUNT];

extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

void radControl_begin(const RadControlConfig& cfg);

void wheelMeasurement_reset_all();

void control_resetPiStates();

void radControl_update(uint32_t nowMs);

void control_setSoll(uint8_t wheel, float v_cms);

void control_stopAll();

#endif // RAD_CONTROL_H