// Control.h
#pragma once

#include "globals.h"
#include "SpeedWeg.h"
#include "PIRegler.h"
#include "Rad.h"

// --- SpeedWeg ---
extern SpeedWeg speed[WHEEL_COUNT];

// --- Regler / Rad ---
extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

// Init/Reset
void control_begin();
void control_update(uint32_t nowMs);

// NEU:
void control_setSoll(uint8_t wheel, float v);
void control_stopAll();