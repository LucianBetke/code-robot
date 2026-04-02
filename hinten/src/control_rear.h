// control_rear.h
#pragma once
#include <globals.h>
#include <ControlTypes.h>


constexpr int16_t DEAD_PWM[WHEEL_COUNT] = {
    80,  // Li
    60   // Re
};

constexpr PIParam PI_PARAMS[WHEEL_COUNT] = {
    {2.0f, 6.0f},   // Li
    {2.0f, 5.5f}    // Re
};