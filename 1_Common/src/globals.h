// ============================================================
// File: globals.h
// Zweck:
//  - Zentrale Definitionen fuer Logik, Geometrie und Zeit
//  - KEINE Hardware-Pins!
// ============================================================

#ifndef GLOBALS_H
#define GLOBALS_H

#include <Arduino.h>

// ============================================================
// Wheel Index – Fahrzeug (alle 4 Raeder)
// Fuer Vehicle Controller / Mecanum / Fahrzeuglogik
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

// ============================================================
// Wheel Index – Lokal (pro Nano: links / rechts)
// Fuer Motor, Encoder, PI, Rad
// ============================================================

enum Wheel : uint8_t
{
    Li = 0,
    Re = 1,
    WHEEL_COUNT
};

// ============================================================
// ==== Zeitkonstanten ====
// ============================================================

constexpr uint16_t SPEED_TIMEOUT_MS = 60;
constexpr uint32_t SPEED_MIN_DT_MS = 5;
constexpr uint16_t RAD_REGEL_DT_MS = 20;
constexpr uint16_t VEHICLE_DT_MS = 80;
constexpr uint16_t CMDP_SETTLE_MS = 100;
constexpr uint16_t DBG_INTERVAL_MS = 20;

// ============================================================
// ==== Encoder / Rad-Geometrie ====
// ============================================================

constexpr uint16_t ENC_PPR = 330;
constexpr int32_t COUNTS_PER_REV = int32_t(ENC_PPR) * 4;

constexpr float RAD_DURCHMESSER_MM = 60.0f;
constexpr float RAD_UMFANG_MM = RAD_DURCHMESSER_MM * PI;

// Hauptsystem fuer Geschwindigkeit: cm/s
constexpr float RAD_UMFANG_CM = RAD_UMFANG_MM * 0.1f;
constexpr float CM_PRO_TICK = RAD_UMFANG_CM / float(COUNTS_PER_REV);

// Geometriewerte in Meter bleiben nur als Hilfs-/Kompatibilitaetswerte erhalten.
// Die aktive Geschwindigkeitskette arbeitet mit cm/s.
constexpr float RAD_UMFANG_M = RAD_UMFANG_MM / 1000.0f;
constexpr float METER_PRO_TICK = RAD_UMFANG_M / float(COUNTS_PER_REV);

// ============================================================
// ==== Fahrzeug-Geometrie ====
// ============================================================

constexpr float MECANUM_LX_MM = 45.5f;
constexpr float MECANUM_LY_MM = 104.5f;

constexpr float WHEEL_BASE_MM = MECANUM_LX_MM * 2.0f;
constexpr float TRACK_WIDTH_MM = MECANUM_LY_MM * 2.0f;

constexpr float MECANUM_K_MM = MECANUM_LX_MM + MECANUM_LY_MM;

// Hauptsystem fuer Fahrzeuggeschwindigkeit: cm/s
constexpr float MECANUM_LX_CM = MECANUM_LX_MM * 0.1f;
constexpr float MECANUM_LY_CM = MECANUM_LY_MM * 0.1f;
constexpr float MECANUM_K_CM = MECANUM_K_MM * 0.1f;

// Meterwerte bleiben fuer Alt-/Kompatibilitaetscode erhalten.
constexpr float MECANUM_LX_M = MECANUM_LX_MM / 1000.0f;
constexpr float MECANUM_LY_M = MECANUM_LY_MM / 1000.0f;
constexpr float MECANUM_K = MECANUM_LX_M + MECANUM_LY_M;

// ============================================================
// ==== PWM / Limits ====
// ============================================================

constexpr int16_t MAX_PWM = 255;
constexpr int16_t SLEW_LIMIT_PWM = 255;

// ============================================================
// ==== Fahrzeug / Geschwindigkeit ====
// ============================================================
//
// Ab jetzt gilt fuer Translation und Radgeschwindigkeit:
//   Einheit = cm/s
//
// Beispiel:
//   30.0f bedeutet 30 cm/s
//   0.30 m/s wird nicht mehr als interne Einheit benutzt.
// ============================================================

constexpr float V_WHEEL_MIN = 17.0f;
constexpr float V_WHEEL_MAX = 50.0f;

#endif // GLOBALS_H