// TestTypes.h
#pragma once
#include <Arduino.h>
// ============================================================
// --- Testauswahl (Option C: switch MODE) ---
// ============================================================
enum class TestMode : uint8_t {
    ENC_HAND,           //Von Handdrehung
    DEADBAND,           // Ab PWM startet Motor (Re/Li)
    PI_SPRUNG,          // Ermittlung K-Werte (Re/Li)
    REGEL              // Regelung eines Rades (Re/Li)
};

enum class Side : uint8_t {
    RE,
    LI,
    BOTH
};

