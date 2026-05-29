// ============================================================
// File: RadControl.h
// Zweck:
//  - Gemeinsame Radregelungs-Schnittstelle fuer vorne und hinten
//  - Lokale Radmessung, PI-Regler und Radobjekte bereitstellen
//  - Apps geben ihre Konfiguration via radControl_begin(cfg) mit
// ============================================================

#ifndef RAD_CONTROL_H
#define RAD_CONTROL_H

#include <Arduino.h>

#include "src/RobotConfig.h"
#include "src/WheelMeasurement.h"
#include "src/PIRegler.h"
#include "src/Rad.h"
#include "RadControlConfig.h"

// ------------------------------------------------------------
// Radmessung pro lokalem Nano-Rad
// ------------------------------------------------------------
extern WheelMeasurement wheelMeasurements[WHEEL_COUNT];

// ------------------------------------------------------------
// PI-Regler und Radobjekte pro lokalem Nano-Rad
// ------------------------------------------------------------
extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

// ------------------------------------------------------------
// Initialisierung
// ------------------------------------------------------------
void radControl_begin(const RadControlConfig& cfg);

// ------------------------------------------------------------
// Messung / Regler / Radsteuerung
// ------------------------------------------------------------
void wheelMeasurement_reset_all();

void radControl_resetPiStates();

void radControl_update(uint32_t nowMs);

void radControl_setSoll(uint8_t wheel, float v_cms);

void radControl_stopAll();

#endif // RAD_CONTROL_H