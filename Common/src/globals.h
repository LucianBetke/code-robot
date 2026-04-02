// ============================================================
// File: globals.h
// Zweck:
//  - Zentrale Definitionen für Logik, Geometrie und Zeit
//  - KEINE Hardware-Pins!
// ============================================================

#pragma once
#include <Arduino.h>

// ============================================================
// Wheel Index Definition
// ============================================================
enum Wheel : uint8_t {
    Li = 0,
    Re = 1,
    WHEEL_COUNT
};

// ============================================================
// Wheel Names (Debug / Printer)
// ============================================================
constexpr const char* WHEEL_NAME[WHEEL_COUNT] = {
    "Li",
    "Re"
};

// ============================================================
// ==== Zeitbegrenzungen ====
// ============================================================

/** Minimaler Messabstand für Drehzahl (ms) */
constexpr uint32_t SPEED_MIN_DT_MS = 5;

/** Regelperiode pro Rad (ms) */
constexpr uint16_t RAD_REGEL_DT_MS = 20;

/** Debug-Ausgabe Intervall */
constexpr uint16_t DBG_INTERVAL_MS = 20;

// ============================================================
// ==== Rad- / Encoder-Geometrie ====
// ============================================================

/** Encoder-Pulse pro Umdrehung (x1) */
constexpr uint16_t ENC_PPR = 330;

/** Encoder-Counts pro Umdrehung (4x) */
constexpr int32_t COUNTS_PER_REV = int32_t(ENC_PPR) * 4;

/** Raddurchmesser (mm) */
constexpr float RAD_DURCHMESSER_MM = 60.0f;

/** Radumfang (mm) */
constexpr float RAD_UMFANG_MM = RAD_DURCHMESSER_MM * PI;

/** Achsabstand (mm) */
constexpr float ACHS_ABSTAND_MM = 228.0f;

// ============================================================
// ==== Umrechnungsfaktoren ====
// ============================================================

/** Radumfang in Meter */
constexpr float RAD_UMFANG_M = RAD_UMFANG_MM / 1000.0f;

/** Meter pro Tick */
constexpr float METER_PRO_TICK = RAD_UMFANG_M / float(COUNTS_PER_REV);

/** Achsabstand in Meter */
constexpr float ACHS_ABSTAND_M = ACHS_ABSTAND_MM / 1000.0f;

constexpr int16_t MAX_PWM = 255;
constexpr int16_t SLEW_LIMIT_PWM = 255;