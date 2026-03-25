// Control.h
#pragma once

#include "SpeedWeg.h"
#include "PIRegler.h"
#include "Rad.h"
#include "Achse_Hi.h"   // wenn Achse_Hi wirklich Teil deiner Control-Ebene ist
#include "VehicleTypes.h"
#include "VehicleController.h"
#include "Mixer2W.h"

// --- SpeedWeg ---
extern SpeedWeg speed[WHEEL_COUNT];

// --- Regler / Rad ---
extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

// --- Hinterachse (Control-Abstraktion) ---
extern Achse_Hi achse_Hi;

// Init/Reset für Control-Objekte (ohne Pins)
void control_begin(bool resetEnc);

// --- Mixer (Command -> AxleRef) ---
extern Mixer2W mixer2W;

// optional (wenn du Command im Control halten willst)
extern VehicleCmd cmdVehicle;

extern VehicleController vehicle;

// ============================================================
// SideBinding: deterministische SIDE -> {Motor, Encoder, SpeedWeg, Rad}
// Ort: Control.h (weil es Control-Objekte bündelt)
// ============================================================

class Enc;
class Motor;

struct SideBinding {
    Motor* m;
    Enc* e;
    SpeedWeg* s;
    Rad* r;
};

// liefert Referenzen auf die existierenden globalen Objekte aus Control/Hardware
SideBinding bindSide(Side side);