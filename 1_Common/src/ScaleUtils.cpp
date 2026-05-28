// ============================================================
// File: ScaleUtils.cpp
// ============================================================

#include "ScaleUtils.h"

int16_t scaleRoundToInt16(float value)
{
    if (value >= 0.0f)
    {
        return (int16_t)(value + 0.5f);
    }

    return (int16_t)(value - 0.5f);
}

long scaleFloatToInt100(float value)
{
    if (value >= 0.0f)
    {
        return (long)(value * 100.0f + 0.5f);
    }

    return (long)(value * 100.0f - 0.5f);
}

long scaleFloatToInt1000(float value)
{
    if (value >= 0.0f)
    {
        return (long)(value * 1000.0f + 0.5f);
    }

    return (long)(value * 1000.0f - 0.5f);
}