// ControlParams.h
#pragma once
#include <Arduino.h>
#include "globals.h"
static_assert(ACHS_ABSTAND_M > 0.0f, "ACHS_ABSTAND_M muss > 0 sein");
// Sollwert
constexpr float V_SOLL_GERADE = 0.30f;
// Omega-Referenz (2-Rad hinten): uOmega=1 => vR=+V_SOLL_GERADE, vL=-V_SOLL_GERADE
constexpr float OMEGA_REF = (2.0f * V_SOLL_GERADE) / ACHS_ABSTAND_M;

// ============================================================
// Hardware-Polung Hinterachse (Encoder-/Motor-Kette)
// +1: Vorwärts zählt/ wirkt positiv -1: invertiert
// Reihenfolge: R, L (passt zu AchseRegel::setSigns(signR, signL))
// ============================================================
constexpr int8_t HI_SIGN_R = +1;
constexpr int8_t HI_SIGN_L = +1;

// ============================================================
// Control-Parameter (Regler, Limits, Sollwerte, Trim)
// ============================================================

// Slewrate / PWM
constexpr int16_t SLEW_LIMIT_PWM = 255;
constexpr int16_t MAX_PWM = 255;

// Hinterachse
constexpr int16_t DEAD_PWM[WHEEL_COUNT] = {
    80,  // 2 = Li
    60   // 3 = Re
};

//// Vorderachse
//constexpr int16_t DEAD_PWM[WHEEL_COUNT] = {
//    80,  // 2 = Li
//    80   // 3 = Re
//};

struct PIParam {
    float Kp;
    float Ki;
};

//// Vorderachse
//constexpr PIParam PI_PARAMS[WHEEL_COUNT] = {
//    {2.0f, 11.0f},   // 2 = Li
//    {2.0f, 10.0f}    // 3 = Re
//};

// Hinterachse
constexpr PIParam PI_PARAMS[WHEEL_COUNT] = {
    {2.0f, 6.0f},   // 2 = Li
    {2.0f, 5.5f}    // 3 = Re
};