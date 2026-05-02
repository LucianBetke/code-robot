// ============================================================
// VehicleController.cpp
// ============================================================
#include "VehicleController.h"
#include <math.h>

void VehicleController::cmd(float vx, float vy, float wz)
{
    _vx = vx;
    _vy = vy;
    _wz = wz;
    update();
}

void VehicleController::update()
{
    float v[WHEEL_VEHICLE_COUNT];
    v[VoRe] = _vx - _vy - MECANUM_K * _wz;
    v[VoLi] = _vx + _vy + MECANUM_K * _wz;
    v[HiLi] = _vx - _vy + MECANUM_K * _wz;
    v[HiRe] = _vx + _vy - MECANUM_K * _wz;

    float maxVal = 0.0f;
    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
    {
        float a = fabsf(v[i]);
        if (a > maxVal) maxVal = a;
    }
    if (maxVal > V_WHEEL_MAX && maxVal > 0.0001f)
    {
        float scale = V_WHEEL_MAX / maxVal;
        for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
            v[i] *= scale;
    }
    for (int i = 0; i < WHEEL_VEHICLE_COUNT; i++)
        _wheelSoll[i] = v[i];
}

float VehicleController::getWheelSoll(WheelVehicle w) const
{
    return _wheelSoll[w];
}