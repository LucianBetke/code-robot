// ============================================================
// File: globals.h
// Zweck:
//  - Zentrale Definitionen für Pins, Konstanten und Geometrie
//  - Gemeinsame Parameter für Motor, Encoder und Umrechnung
// Abhängigkeiten: Arduino.h (liefert PI)
// ============================================================

#pragma once
#include <Arduino.h>   // enthält PI-Konstante

// ============================================================
// Wheel Index Definition (Robot Wheel Order)
// Reihenfolge:
// 0 = vorne rechts
// 1 = vorne links
// 2 = hinten links
// 3 = hinten rechts
// ============================================================
enum Wheel: uint8_t {
    Li = 0,
    Re = 1,
    WHEEL_COUNT
};
// ============================================================
// Wheel Names (für Debug / Printer)
// ============================================================

constexpr const char* WHEEL_NAME[WHEEL_COUNT] = {
    "Li",
    "Re"
};

// ============================================================
// ==== Motor-Pins (DRV8833, 2-Pin-Modus) ==== Hinterachse
// ============================================================

/** @brief Rechter Hinterrad-Motor: IN1 (PWM) */
constexpr uint8_t M_Re_AIN1 = 9;
/** @brief Rechter Hinterrad-Motor: IN2 (PWM) */
constexpr uint8_t M_Re_AIN2 = 10;
/** @brief Linker Hinterrad-Motor: IN1 (PWM) */
constexpr uint8_t M_Li_BIN1 = 6;
/** @brief Linker Hinterrad-Motor: IN2 (PWM) */
constexpr uint8_t M_Li_BIN2 = 5;
/** @brief DRV8833 Standby-Pin */
constexpr uint8_t STBY_PIN = 8;

//// ============================================================
//// ==== Encoder-Pins (Port C / PCINT1) ==== Vorderachse
//// ============================================================
//
///** @brief Rechter Encoder: Kanal A */
//constexpr uint8_t ENC_Re_PIN_A = A0;
///** @brief Rechter Encoder: Kanal B */
//constexpr uint8_t ENC_Re_PIN_B = A1;
///** @brief Linker Encoder: Kanal A */
//constexpr uint8_t ENC_Li_PIN_A = A2;
///** @brief Linker Encoder: Kanal B */
//constexpr uint8_t ENC_Li_PIN_B = A3;

// ============================================================
// ==== Encoder-Pins (Port C / PCINT1) ==== Hinterachse
// ============================================================

/** @brief Rechter Encoder: Kanal A */
constexpr uint8_t ENC_Re_PIN_A = A2;
/** @brief Rechter Encoder: Kanal B */
constexpr uint8_t ENC_Re_PIN_B = A3;
/** @brief Linker Encoder: Kanal A */
constexpr uint8_t ENC_Li_PIN_A = A0;
/** @brief Linker Encoder: Kanal B */
constexpr uint8_t ENC_Li_PIN_B = A1;
// ============================================================
// ==== Zeitbegrenzungen ====
// ============================================================

/** @brief Minimaler Messabstand für Drehzahl (ms) */
constexpr uint32_t SPEED_MIN_DT_MS = 5;
/** @brief Regelperiode pro Rad (ms) */
constexpr uint16_t RAD_REGEL_DT_MS = 20;
/** @brief Fensterzeit der Achs-Kopplung (TickCoupler) (ms) */
constexpr uint16_t HI_COUPLER_WINDOW_MS = 50;
// Messzeit Ausgabe der Messungen
constexpr uint16_t DBG_INTERVAL_MS = 20;

// ============================================================
// ==== Rad- / Encoder-Geometrie ====
// ============================================================

/** @brief Encoder-Pulse pro Umdrehung (x1-Auswertung) */
constexpr uint16_t ENC_PPR = 330;
/** @brief Encoder-Counts pro Umdrehung (4x-Auswertung → 1320) */
constexpr int32_t  COUNTS_PER_REV = int32_t(ENC_PPR) * 4;
/** @brief Raddurchmesser in Millimetern */
constexpr float RAD_DURCHMESSER_MM = 60.0f;
/** @brief Radumfang in Millimetern */
constexpr float RAD_UMFANG_MM = RAD_DURCHMESSER_MM * PI;
constexpr float ACHS_ABSTAND_MM = 228.0f;


// ============================================================
// ==== Umrechnungsfaktoren ====
// ============================================================

/** @brief Encoder-Counts pro Millimeter */
constexpr float COUNTS_PRO_MM = float(COUNTS_PER_REV) / RAD_UMFANG_MM;
/** @brief Radumfang in Metern */
constexpr float RAD_UMFANG_M = RAD_UMFANG_MM / 1000.0f;

constexpr float METER_PRO_TICK = RAD_UMFANG_M / float(COUNTS_PER_REV);
constexpr float ACHS_ABSTAND_M = ACHS_ABSTAND_MM / 1000.0f;
