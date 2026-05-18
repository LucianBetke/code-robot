// CommUtils.h
// brauche ich die noch?????
#pragma once
#include <stdint.h>

inline int16_t floatToInt100(float v)
{
    return (int16_t)(v * 100.0f + (v >= 0.0f ? 0.5f : -0.5f));
}

inline float int100ToFloat(int16_t v)
{
    return (float)v / 100.0f;
}