// control_rear.h
#pragma once
#include "ControlParams.h"

// Vorzeichen (falls nötig später anpassen)
constexpr int8_t SIGN_R = +1;
constexpr int8_t SIGN_L = +1;

// Deadband
constexpr int16_t  DEAD_PWM_LI = 80;
constexpr int16_t DEAD_PWM_RE = 60;

// PI-Parameter
constexpr PIParam PI_LI = { 2.0f, 6.0f };
constexpr PIParam PI_RE = { 2.0f, 5.5f };