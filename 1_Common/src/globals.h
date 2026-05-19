// ============================================================
// File: globals.h
// Zweck:
//  - Zentrale Definitionen für Logik, Geometrie und Zeit
//  - KEINE Hardware-Pins!
// ============================================================

#ifndef GLOBALS_H
#define GLOBALS_H

#pragma once

#include <Arduino.h>

// ============================================================
// Wheel Index – Fahrzeug (alle 4 Räder)
// Für Vehicle Controller / Mecanum / Fahrzeuglogik
// Feste Reihenfolge:
//   0 = Vorne Rechts
//   1 = Vorne Links
//   2 = Hinten Links
//   3 = Hinten Rechts
// ============================================================

enum WheelVehicle : uint8_t
{
    VoRe = 0,
    VoLi = 1,
    HiLi = 2,
    HiRe = 3,
    WHEEL_VEHICLE_COUNT
};

constexpr const char* WHEEL_VEHICLE_NAME[WHEEL_VEHICLE_COUNT] =
{
    "VoRe",
    "VoLi",
    "HiLi",
    "HiRe"
};

// ============================================================
// Wheel Index – Lokal (pro Nano: links / rechts)
// Für Motor, Encoder, PI, Rad
// ============================================================

enum Wheel : uint8_t
{
    Li = 0,
    Re = 1,
    WHEEL_COUNT
};

constexpr const char* WHEEL_NAME[WHEEL_COUNT] =
{
    "Li",
    "Re"
};

// ============================================================
// ==== Zeitkonstanten ====
// ============================================================

/** Stillstands-Timeout für Drehzahlmessung (ms) */
constexpr uint16_t SPEED_TIMEOUT_MS = 60;

/** Minimaler Messabstand für Drehzahl (ms) */
constexpr uint32_t SPEED_MIN_DT_MS = 5;

/** Regelperiode pro Rad (ms) */
constexpr uint16_t RAD_REGEL_DT_MS = 20;

/** Regelperiode Fahrzeug + UART VSOL/VIST (ms) */
constexpr uint16_t VEHICLE_DT_MS = 100;

/** Debug-Ausgabe Intervall */
constexpr uint16_t DBG_INTERVAL_MS = 20;

// ============================================================
// ==== Encoder / Rad-Geometrie ====
// ============================================================

/** Encoder-Pulse pro Umdrehung (x1) */
constexpr uint16_t ENC_PPR = 330;

/** Encoder-Counts pro Umdrehung (4x) */
constexpr int32_t COUNTS_PER_REV = int32_t(ENC_PPR) * 4;

/** Raddurchmesser (mm) */
constexpr float RAD_DURCHMESSER_MM = 60.0f;

/** Radumfang (mm) */
constexpr float RAD_UMFANG_MM = RAD_DURCHMESSER_MM * PI;

/** Radumfang in Meter */
constexpr float RAD_UMFANG_M = RAD_UMFANG_MM / 1000.0f;

/** Meter pro Tick */
constexpr float METER_PRO_TICK = RAD_UMFANG_M / float(COUNTS_PER_REV);

// ============================================================
// ==== Fahrzeug-Geometrie ====
// ============================================================

/**
 * Lx = halber Radstand:
 * Abstand vom Fahrzeugmittelpunkt zur Vorder-/Hinterradlinie.
 */
constexpr float MECANUM_LX_MM = 45.5f;

/**
 * Ly = halbe Spurweite:
 * Abstand vom Fahrzeugmittelpunkt zur linken/rechten Radlinie.
 */
constexpr float MECANUM_LY_MM = 104.5f;

/** Radstand vorne ↔ hinten (mm), nur als abgeleiteter Kontrollwert */
constexpr float WHEEL_BASE_MM = MECANUM_LX_MM * 2.0f;

/** Spurweite links ↔ rechts (mm), nur als abgeleiteter Kontrollwert */
constexpr float TRACK_WIDTH_MM = MECANUM_LY_MM * 2.0f;

/** Mecanum-Faktor in mm */
constexpr float MECANUM_K_MM = MECANUM_LX_MM + MECANUM_LY_MM;

/** Lx in Meter */
constexpr float MECANUM_LX_M = MECANUM_LX_MM / 1000.0f;

/** Ly in Meter */
constexpr float MECANUM_LY_M = MECANUM_LY_MM / 1000.0f;

/**
 * Mecanum-Faktor:
 * k = Lx + Ly
 *
 * Einheit: Meter
 *
 * Wichtig:
 * vx, vy und Radgeschwindigkeiten sind aktuell intern in m/s.
 * wz muss hier in rad/s vorliegen.
 */
constexpr float MECANUM_K = MECANUM_LX_M + MECANUM_LY_M;

// ============================================================
// ==== PWM / Limits ====
// ============================================================

constexpr int16_t MAX_PWM = 255;
constexpr int16_t SLEW_LIMIT_PWM = 255;

// ============================================================
// ==== Fahrzeug / Geschwindigkeit ====
// ============================================================

/** Minimale sinnvoll regelbare Radgeschwindigkeit (m/s) */
constexpr float V_WHEEL_MIN = 0.17f;

/** Maximale Radgeschwindigkeit (m/s) */
constexpr float V_WHEEL_MAX = 0.50f;

#endif // GLOBALS_H