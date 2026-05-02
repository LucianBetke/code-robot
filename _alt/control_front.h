// control_front.h
#pragma once
//#include "src/globals.h"
//#include "src/ControlTypes.h"

constexpr int16_t DEAD_PWM[WHEEL_COUNT] = {
    80,  // Li
    60   // Re
};

constexpr PIParam PI_PARAMS[WHEEL_COUNT] = {
    {2.0f, 6.0f},   // Li
    {2.0f, 5.5f}    // Re
};