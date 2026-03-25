// ControlParams.h
#pragma once
#include <Arduino.h>

static_assert(ACHS_ABSTAND_M > 0.0f, "ACHS_ABSTAND_M muss > 0 sein");

// Sollwert
constexpr float V_SOLL_GERADE = 0.30f;

constexpr float ACHS_ABSTAND_M = 0.18f;// noch überprüfen
// Omega-Referenz
constexpr float OMEGA_REF = (2.0f * V_SOLL_GERADE) / ACHS_ABSTAND_M;

// Limits
constexpr int16_t SLEW_LIMIT_PWM = 255;
constexpr int16_t MAX_PWM = 255;

struct PIParam {
    float Kp;
    float Ki;
};
