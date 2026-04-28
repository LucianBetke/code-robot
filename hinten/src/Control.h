// Control.h
#pragma once

#include "src/SpeedWeg.h"
#include "src/PIRegler.h"
#include "src/Rad.h"

// --- SpeedWeg ---
extern SpeedWeg speed[WHEEL_COUNT];

// --- Regler / Rad ---
extern PIRegler regler[WHEEL_COUNT];
extern Rad rad[WHEEL_COUNT];

// Init/Reset
void speed_reset_all();
void control_update(uint32_t nowMs);

// NEU:
void control_setSoll(uint8_t wheel, float v);
void control_stopAll();