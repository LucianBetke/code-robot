// ============================================================
// File: Control.h (3_Control)
// Zweck:
//  - Gemeinsame Steuerungs-Schnittstelle fuer vorne und hinten
//  - Apps geben ihre Konfiguration via control_begin(cfg) mit
// ============================================================

#ifndef CONTROL_H
#define CONTROL_H


#include <Arduino.h>
#include "src/globals.h"
#include "src/SpeedWeg.h"
#include "src/PIRegler.h"
#include "src/Rad.h"
#include "ControlConfig.h"

// --- SpeedWeg (Geschwindigkeit/Weg pro Rad) ---
extern SpeedWeg speed[WHEEL_COUNT];

// --- Regler / Rad ---
extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

// ============================================================
// Initialisierung
//
// Die App ruft control_begin(cfg) im setup() auf und uebergibt
// ihre gewuenschte Konfiguration (z.B. ConfigFront::CONFIG
// oder ConfigRear::CONFIG).
// ============================================================
void control_begin(const ControlConfig& cfg);

// SpeedWeg fuer alle Raeder zuruecksetzen
void speed_reset_all();

// PI-Zustaende zuruecksetzen, ohne Sollwerte oder Motorzustand zu aendern.
// Wichtig fuer neue CMDT-Fahrabschnitte.
void control_resetPiStates();

// Regelschleife pro Loop-Durchlauf aufrufen
void control_update(uint32_t nowMs);

// Sollwert eines bestimmten Rades setzen
void control_setSoll(uint8_t wheel, float v);

// Alle Raeder stoppen
void control_stopAll();

#endif // CONTROL_H