// ControlParams.h
#pragma once
#include <Arduino.h>
#include "globals.h"

// Limits
constexpr int16_t SLEW_LIMIT_PWM = 255;
constexpr int16_t MAX_PWM = 255;

struct PIParam {
    float Kp;
    float Ki;
};
